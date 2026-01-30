import gdb

def get_errno():
    # Casting to function pointer ensures we call the thread-local errno location correctly
    return int(gdb.parse_and_eval("*((int * (*) (void)) __errno_location) ()"))

class ReaddirReturnBreakpoint(gdb.FinishBreakpoint):
    """Inner-most hook: Captures the result of readdir."""
    def __init__(self):
        super().__init__(internal=True)

    def stop(self):
        try:
            response = gdb.parse_and_eval("((struct dirent *)$x0).d_name").string()
            response = f"\"{response}\""
        except gdb.error:
            response = "(null)"
        errno = get_errno()
        print(f"    [+] readdir {response}, errno={errno}")
        return False

class ReaddirBreakpoint(gdb.Breakpoint):
    """Middle-hook: Watches calls to readdir within Directory::Load."""
    def __init__(self):
        super().__init__("readdir", gdb.BP_BREAKPOINT, internal=True)

    def stop(self):
        ReaddirReturnBreakpoint()
        return False

class DirectoryLoadExitBreakpoint(gdb.FinishBreakpoint):
    """Cleans up the readdir hook when Directory::Load returns."""
    def __init__(self, readdir_bp):
        super().__init__(internal=True)
        self.readdir_bp = readdir_bp

    def stop(self):
        if self.readdir_bp.is_valid():
            self.readdir_bp.delete()
        return False

class DirectoryLoadBreakpoint(gdb.Breakpoint):
    """Triggered inside the target library search."""
    def __init__(self):
        super().__init__("cmsys::Directory::Load", gdb.BP_BREAKPOINT, internal=True)

    def stop(self):
        print(f"  >>> Entering Directory::Load")
        rbp = ReaddirBreakpoint()
        DirectoryLoadExitBreakpoint(rbp)
        return False

class CheckDirectoryExitBreakpoint(gdb.FinishBreakpoint):
    """Outer-most cleanup: Removes the Load hook when the library check finishes."""
    def __init__(self, load_bp):
        super().__init__(internal=True)
        self.load_bp = load_bp

    def stop(self):
        print(f"<<< Leaving CheckDirectoryForName scope.")
        if self.load_bp.is_valid():
            self.load_bp.delete()
        return False

class FindLibraryNameBreakpoint(gdb.Breakpoint):
    """The Master Filter."""
    def __init__(self):
        super().__init__("cmFindLibraryHelper::CheckDirectoryForName", gdb.BP_BREAKPOINT)

    def stop(self):
        # Let this throw if 'name' or 'Raw' isn't in scope/valid
        name_str = gdb.parse_and_eval("name.Raw.c_str()").string()
        
        if "rcutils" in name_str:
            print(f"\n[TARGET DETECTED] Processing: {name_str}")
            lbp = DirectoryLoadBreakpoint()
            CheckDirectoryExitBreakpoint(lbp)
            
        return False


FindLibraryNameBreakpoint()


try:
    gdb.execute("run")
except gdb.error:
    gdb.execute("continue")
