import gdb

def get_errno():
    try:
        return int(gdb.parse_and_eval("*(int*)__errno_location()"))
    except:
        return None


class ReaddirReturnBreakpoint(gdb.FinishBreakpoint):
    def __init__(self,):
        super().__init__(internal=True)

    def stop(self):
        errno = get_errno()
        ret = int(gdb.parse_and_eval("$x0"))
        print(f"readdir ret={hex(ret)}, errno={errno}")
        return False

    def out_of_scope(self):
        pass


class ReaddirBreakpoint(gdb.Breakpoint):
    def __init__(self):
        super().__init__("readdir", gdb.BP_BREAKPOINT)

    def stop(self):
        ReaddirReturnBreakpoint()
        return False


class DirectoryLoadExitBreakpoint(gdb.FinishBreakpoint):
    def __init__(self):
        super().__init__(internal=True)

    def stop(self):
        print(f"<<< Directory::Load returned")

        global readdir_bp
        readdir_bp.delete()
        readdir_bp = None

        return False


class DirectoryLoadBreakpoint(gdb.Breakpoint):
    def __init__(self):
        super().__init__(
            "cmsys::Directory::Load",
            gdb.BP_BREAKPOINT,
            internal=True
        )

    def stop(self):
        print(f">>> Entering Directory::Load")
        DirectoryLoadExitBreakpoint()

        global readdir_bp
        readdir_bp = ReaddirBreakpoint()

        return False


readdir_bp = None

DirectoryLoadBreakpoint()

# Try run (local), fall back to continue (remote)
try:
    gdb.execute("run")
except gdb.error:
    gdb.execute("continue")
