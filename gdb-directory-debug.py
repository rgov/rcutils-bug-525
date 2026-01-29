import gdb

def get_errno():
    try:
        return int(gdb.parse_and_eval("*(int*)__errno_location()"))
    except:
        return None

count = 0

class ReaddirReturnBreakpoint(gdb.FinishBreakpoint):
    """Catch when readdir returns."""
    def __init__(self, num):
        super().__init__(internal=True)
        self.num = num

    def stop(self):
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
        global count
        count += 1
        ReaddirReturnBreakpoint(count)
        return False

ReaddirBreakpoint()
print("[INIT] Breakpoint set on readdir()")

# Try run (local), fall back to continue (remote)
try:
    gdb.execute("run")
except gdb.error:
    try:
        gdb.execute("continue")
    except gdb.error as e:
        print(f"Note: {e}")
