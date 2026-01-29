import gdb

def get_errno():
    try:
        return int(gdb.parse_and_eval("*(int*)__errno_location()"))
    except:
        return None

class State:
    active = False
    count = 0


class ReaddirReturnBreakpoint(gdb.FinishBreakpoint):
    """Catch when readdir returns."""
    def __init__(self, num):
        super().__init__(internal=True)
        self.num = num

    def stop(self):
        if not State.active:
            return False
        errno = get_errno()
        ret = int(gdb.parse_and_eval("$x0"))
        print(f"readdir #{self.num}: ret={hex(ret)}, errno={errno}")
        return False

    def out_of_scope(self):
        pass


class ReaddirBreakpoint(gdb.Breakpoint):
    """Break on readdir entry, set up finish breakpoint."""
    def __init__(self):
        super().__init__("readdir", gdb.BP_BREAKPOINT)

    def stop(self):
        if not State.active:
            return False
        State.count += 1
        ReaddirReturnBreakpoint(State.count)
        return False


class DirectoryLoadExitBreakpoint(gdb.FinishBreakpoint):
    """Deactivate tracking when Directory::Load returns."""
    def __init__(self):
        super().__init__(internal=True)

    def stop(self):
        print(f"<<< Directory::Load returned, {State.count} readdir calls")
        State.active = False
        return False

    def out_of_scope(self):
        State.active = False


class DirectoryLoadBreakpoint(gdb.Breakpoint):
    """Activate tracking when Directory::Load is entered."""
    def __init__(self):
        super().__init__("cmsys::Directory::Load", gdb.BP_BREAKPOINT)

    def stop(self):
        print(f">>> Entering Directory::Load")
        State.active = True
        State.count = 0
        DirectoryLoadExitBreakpoint()
        return False


ReaddirBreakpoint()
DirectoryLoadBreakpoint()
print("[INIT] Tracking readdir inside Directory::Load")

# Try run (local), fall back to continue (remote)
try:
    gdb.execute("run")
except gdb.error:
    try:
        gdb.execute("continue")
    except gdb.error as e:
        print(f"Note: {e}")
