import gdb
import struct
import subprocess

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

_host_pid_cache = None


def read_ptr(addr):
    """Read a single 8-byte pointer from the inferior's memory."""
    mem = gdb.selected_inferior().read_memory(addr, 8)
    return struct.unpack_from("<Q", bytes(mem))[0]


def get_errno():
    """Read the thread-local errno from the inferior."""
    return int(gdb.parse_and_eval("*((int * (*) (void)) __errno_location) ()"))


def get_vector_state(files_impl_addr, sizeof_string):
    """Read vector _M_start/_M_finish/_M_end_of_storage, compute size and capacity."""
    start = read_ptr(files_impl_addr)
    finish = read_ptr(files_impl_addr + 8)
    end = read_ptr(files_impl_addr + 16)
    if sizeof_string > 0 and start != 0:
        size = (finish - start) // sizeof_string
        cap = (end - start) // sizeof_string
    else:
        size = 0
        cap = 0
    return start, finish, end, size, cap


def get_host_pid():
    """Get the host-side PID of the cmake-build container (cached)."""
    global _host_pid_cache
    if _host_pid_cache is not None:
        return _host_pid_cache
    try:
        result = subprocess.run(
            ["docker", "inspect", "--format", "{{.State.Pid}}", "cmake-build"],
            capture_output=True, text=True, timeout=5,
        )
        if result.returncode == 0:
            pid = int(result.stdout.strip())
            if pid > 0:
                _host_pid_cache = pid
                return _host_pid_cache
    except Exception as e:
        print(f"  [MAPS] WARNING: docker inspect failed: {e}")
    return None


def dump_maps(label):
    """Dump /proc/<pid>/maps from the host filesystem."""
    pid = get_host_pid()
    if pid is None:
        print(f"  [MAPS] === {label} === (unavailable -- no host PID)")
        return
    maps_path = f"/proc/{pid}/maps"
    content = None
    try:
        with open(maps_path, "r") as f:
            content = f.read()
    except PermissionError:
        try:
            result = subprocess.run(
                ["sudo", "cat", maps_path],
                capture_output=True, text=True, timeout=5,
            )
            if result.returncode == 0:
                content = result.stdout
        except Exception as e:
            print(f"  [MAPS] === {label} === (permission denied, sudo failed: {e})")
            return
    except Exception as e:
        print(f"  [MAPS] === {label} === (error: {e})")
        return
    if content is None:
        print(f"  [MAPS] === {label} === (could not read)")
        return
    print(f"  [MAPS] === {label} ===")
    for line in content.splitlines():
        print(f"  {line}")
    print(f"  [MAPS] === end ===")


def get_sbrk(ctx):
    """Call sbrk(0) in the inferior.  Temporarily disables the sbrk bp."""
    sbrk_bp = ctx.sbrk_bp if ctx else None
    if sbrk_bp and sbrk_bp.is_valid():
        sbrk_bp.enabled = False
    try:
        val = gdb.parse_and_eval("(unsigned long long)sbrk(0)")
        return int(val)
    except gdb.error as e:
        print(f"  [SBRK] WARNING: sbrk(0) probe failed: {e}")
        return None
    finally:
        if sbrk_bp and sbrk_bp.is_valid():
            sbrk_bp.enabled = True


# ---------------------------------------------------------------------------
# Shared context for a single Directory::Load invocation
# ---------------------------------------------------------------------------

class LoadContext:
    def __init__(self, files_impl_addr, sizeof_string):
        self.files_impl_addr = files_impl_addr   # addr of vector's 3 internal ptrs
        self.sizeof_string = sizeof_string        # sizeof(std::string) on target
        self.prev_capacity = 0
        self.iteration = 0
        # Populated after breakpoint creation
        self.sbrk_bp = None
        self.mmap_bp = None
        self.brk_bp = None


# ---------------------------------------------------------------------------
# mmap breakpoints (NEW)
# ---------------------------------------------------------------------------

class MmapReturnBreakpoint(gdb.FinishBreakpoint):
    def __init__(self):
        super().__init__(internal=True)

    def stop(self):
        try:
            ret = int(gdb.parse_and_eval("$x0"))
        except gdb.error:
            ret = 0
        errno_val = get_errno()
        if ret == 0xffffffffffffffff:
            print(f"  [MMAP]      = 0x{ret:x} (MAP_FAILED), errno={errno_val}")
        else:
            print(f"  [MMAP]      = 0x{ret:x}, errno={errno_val}")
        return False


class MmapBreakpoint(gdb.Breakpoint):
    def __init__(self):
        super().__init__("mmap", gdb.BP_BREAKPOINT, internal=True)

    def stop(self):
        try:
            addr = int(gdb.parse_and_eval("$x0"))
            length = int(gdb.parse_and_eval("$x1"))
            prot = int(gdb.parse_and_eval("$x2"))
            flags = int(gdb.parse_and_eval("$x3"))
            fd_raw = int(gdb.parse_and_eval("$x4")) & 0xffffffff
            fd = fd_raw - 0x100000000 if fd_raw >= 0x80000000 else fd_raw
            offset = int(gdb.parse_and_eval("$x5"))
            print(f"  [MMAP] mmap(addr=0x{addr:x}, len={length}, prot=0x{prot:x}, "
                  f"flags=0x{flags:x}, fd={fd}, off={offset})")
        except gdb.error as e:
            print(f"  [MMAP] mmap(<error reading args: {e}>)")
        try:
            MmapReturnBreakpoint()
        except gdb.error:
            pass
        return False


# ---------------------------------------------------------------------------
# sbrk breakpoints (NEW)
# ---------------------------------------------------------------------------

class SbrkReturnBreakpoint(gdb.FinishBreakpoint):
    def __init__(self):
        super().__init__(internal=True)

    def stop(self):
        try:
            ret = int(gdb.parse_and_eval("$x0"))
        except gdb.error:
            ret = 0
        errno_val = get_errno()
        if ret == 0xffffffffffffffff:
            print(f"  [SBRK]      = 0x{ret:x} (failed), errno={errno_val}")
        else:
            print(f"  [SBRK]      = 0x{ret:x}, errno={errno_val}")
        return False


class SbrkBreakpoint(gdb.Breakpoint):
    def __init__(self):
        super().__init__("sbrk", gdb.BP_BREAKPOINT, internal=True)

    def stop(self):
        try:
            inc_raw = int(gdb.parse_and_eval("$x0"))
            if inc_raw >= 0x8000000000000000:
                inc = inc_raw - 0x10000000000000000
            else:
                inc = inc_raw
            print(f"  [SBRK] sbrk(inc={inc})")
        except gdb.error as e:
            print(f"  [SBRK] sbrk(<error reading args: {e}>)")
        try:
            SbrkReturnBreakpoint()
        except gdb.error:
            pass
        return False


# ---------------------------------------------------------------------------
# brk breakpoints (NEW)
# ---------------------------------------------------------------------------

class BrkReturnBreakpoint(gdb.FinishBreakpoint):
    def __init__(self):
        super().__init__(internal=True)

    def stop(self):
        try:
            ret = int(gdb.parse_and_eval("$x0"))
        except gdb.error:
            ret = 0
        errno_val = get_errno()
        print(f"  [BRK]  brk returned 0x{ret:x}, errno={errno_val}")
        return False


class BrkBreakpoint(gdb.Breakpoint):
    def __init__(self):
        super().__init__("brk", gdb.BP_BREAKPOINT, internal=True)

    def stop(self):
        try:
            addr = int(gdb.parse_and_eval("$x0"))
            print(f"  [BRK]  brk(addr=0x{addr:x})")
        except gdb.error as e:
            print(f"  [BRK]  brk(<error reading args: {e}>)")
        try:
            BrkReturnBreakpoint()
        except gdb.error:
            pass
        return False


# ---------------------------------------------------------------------------
# readdir breakpoints (enhanced)
# ---------------------------------------------------------------------------

class ReaddirReturnBreakpoint(gdb.FinishBreakpoint):
    """Captures the result of readdir."""
    def __init__(self):
        super().__init__(internal=True)

    def stop(self):
        try:
            response = gdb.parse_and_eval("((struct dirent *)$x0).d_name").string()
            response = f"\"{response}\""
        except gdb.error:
            response = "(null)"
        errno_val = get_errno()
        print(f"    [+] readdir {response}, errno={errno_val}")
        return False


class ReaddirBreakpoint(gdb.Breakpoint):
    """Watches calls to readdir within Directory::Load."""
    def __init__(self, ctx):
        super().__init__("readdir", gdb.BP_BREAKPOINT, internal=True)
        self.ctx = ctx

    def stop(self):
        ctx = self.ctx
        if ctx and ctx.files_impl_addr:
            try:
                start, finish, end, size, cap = get_vector_state(
                    ctx.files_impl_addr, ctx.sizeof_string)
                print(f"  [iter {ctx.iteration}] vec: data=0x{start:x} size={size} "
                      f"cap={cap} end=0x{end:x}")
                if cap != ctx.prev_capacity:
                    sbrk_val = get_sbrk(ctx)
                    sbrk_str = f"0x{sbrk_val:x}" if sbrk_val is not None else "?"
                    print(f"    *** REALLOC: cap {ctx.prev_capacity} -> {cap}, "
                          f"sbrk(0)={sbrk_str}")
                    dump_maps(f"after-realloc-iter-{ctx.iteration}")
                    ctx.prev_capacity = cap
                ctx.iteration += 1
            except Exception as e:
                print(f"  [iter {ctx.iteration}] vec: <error: {e}>")
                ctx.iteration += 1
        ReaddirReturnBreakpoint()
        return False


# ---------------------------------------------------------------------------
# Directory::Load breakpoints (enhanced)
# ---------------------------------------------------------------------------

class DirectoryLoadExitBreakpoint(gdb.FinishBreakpoint):
    """Cleans up all inner hooks when Directory::Load returns."""
    def __init__(self, inner_bps, ctx):
        super().__init__(internal=True)
        self.inner_bps = inner_bps
        self.ctx = ctx

    def stop(self):
        ctx = self.ctx
        if ctx and ctx.files_impl_addr:
            try:
                start, finish, end, size, cap = get_vector_state(
                    ctx.files_impl_addr, ctx.sizeof_string)
                errno_val = get_errno()
                print(f"  <<< Exiting Directory::Load: {size} entries, "
                      f"final cap={cap}, final errno={errno_val}")
            except Exception as e:
                print(f"  <<< Exiting Directory::Load (error reading final state: {e})")
            sbrk_val = get_sbrk(ctx)
            if sbrk_val is not None:
                print(f"  [FINAL] sbrk(0)=0x{sbrk_val:x}")
            dump_maps("final")
        else:
            print(f"  <<< Exiting Directory::Load")
        for bp in self.inner_bps:
            try:
                if bp.is_valid():
                    bp.delete()
            except Exception:
                pass
        return False


class DirectoryLoadBreakpoint(gdb.Breakpoint):
    """Triggered inside the target library search."""
    def __init__(self):
        super().__init__("cmsys::Directory::Load", gdb.BP_BREAKPOINT, internal=True)

    def stop(self):
        # Log the directory path being scanned
        try:
            dir_path = gdb.parse_and_eval("name._M_dataplus._M_p").string()
            print(f"  >>> Entering Directory::Load(\"{dir_path}\")")
        except gdb.error:
            print(f"  >>> Entering Directory::Load")

        # Capture vector address for raw-memory reads in later breakpoints.
        # this->Internal is a raw pointer to DirectoryInternals.
        # DirectoryInternals::Files (std::vector<std::string>) is the first
        # member, so the vector's _M_impl starts at the same address.
        ctx = None
        try:
            internal_ptr = int(gdb.parse_and_eval("this->Internal"))
            # Try several ways to resolve sizeof(std::string) -- the exact
            # type name varies across GDB versions and debug-info formats.
            sizeof_str = None
            for expr in ["sizeof(std::string)",
                         "sizeof(std::__cxx11::basic_string<char>)",
                         "sizeof(this->Internal->Files[0])"]:
                try:
                    sizeof_str = int(gdb.parse_and_eval(expr))
                    break
                except gdb.error:
                    continue
            if sizeof_str is None:
                sizeof_str = 32  # aarch64 libstdc++ default
                print(f"  [INIT] WARNING: sizeof(string) fallback={sizeof_str}")
            ctx = LoadContext(internal_ptr, sizeof_str)
            print(f"  [INIT] Internal=0x{internal_ptr:x}, sizeof(string)={sizeof_str}")
        except gdb.error as e:
            print(f"  [INIT] WARNING: Could not capture vector info: {e}")
            ctx = LoadContext(0, 0)

        dump_maps("initial")

        # Create all inner breakpoints
        readdir_bp = ReaddirBreakpoint(ctx)
        mmap_bp = MmapBreakpoint()
        sbrk_bp = SbrkBreakpoint()
        brk_bp = BrkBreakpoint()

        # Store refs so get_sbrk() can disable the sbrk bp during inferior calls
        ctx.sbrk_bp = sbrk_bp
        ctx.mmap_bp = mmap_bp
        ctx.brk_bp = brk_bp

        inner_bps = [readdir_bp, mmap_bp, sbrk_bp, brk_bp]
        DirectoryLoadExitBreakpoint(inner_bps, ctx)
        return False


# ---------------------------------------------------------------------------
# Outer scope breakpoints (unchanged)
# ---------------------------------------------------------------------------

class CheckDirectoryExitBreakpoint(gdb.FinishBreakpoint):
    """Removes the Load hook when the library check finishes."""
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
        name_str = gdb.parse_and_eval("name.Raw._M_dataplus._M_p").string()

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
