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
# Malloc state introspection helpers
# ---------------------------------------------------------------------------

def dump_malloc_params():
    """Dump all fields of glibc's mp_ (malloc_par) struct."""
    fields = [
        ("trim_threshold",  "unsigned long"),
        ("top_pad",         "unsigned long"),
        ("mmap_threshold",  "unsigned long"),
        ("arena_test",      "unsigned long"),
        ("arena_max",       "unsigned long"),
        ("n_mmaps",         "int"),
        ("n_mmaps_max",     "int"),
        ("max_n_mmaps",     "int"),
        ("no_dyn_threshold","int"),
        ("mmapped_mem",     "unsigned long"),
        ("max_mmapped_mem", "unsigned long"),
        ("sbrk_base",       "unsigned long"),
        ("thp_pagesize",    "unsigned long"),
        ("hp_pagesize",     "unsigned long"),
        ("hp_flags",        "int"),
    ]
    print("  [MALLOC] === mp_ (malloc parameters) ===")
    for name, cast in fields:
        try:
            val = int(gdb.parse_and_eval(f"({cast})mp_.{name}"))
            if "long" in cast:
                print(f"    {name:24s} = {val} (0x{val:x})")
            else:
                print(f"    {name:24s} = {val}")
        except gdb.error:
            print(f"    {name:24s} = <unavailable>")


def dump_arena_state():
    """Dump main_arena fields showing heap fragmentation state."""
    print("  [MALLOC] === main_arena ===")
    fields = [
        ("system_mem",     "unsigned long"),
        ("max_system_mem", "unsigned long"),
    ]
    for name, cast in fields:
        try:
            val = int(gdb.parse_and_eval(f"({cast})main_arena.{name}"))
            print(f"    {name:24s} = {val} (0x{val:x})")
        except gdb.error:
            print(f"    {name:24s} = <unavailable>")

    # Top chunk: address and size
    try:
        top_addr = int(gdb.parse_and_eval("(unsigned long)main_arena.top"))
        top_size = int(gdb.parse_and_eval(
            "((unsigned long)main_arena.top->mchunk_size) & ~7"))
        print(f"    {'top_chunk_addr':24s} = 0x{top_addr:x}")
        print(f"    {'top_chunk_size':24s} = {top_size} (0x{top_size:x})")
    except gdb.error as e:
        print(f"    top_chunk: <error: {e}>")

    # Flags (contiguous bit)
    try:
        flags = int(gdb.parse_and_eval("main_arena.flags"))
        contiguous = "yes" if (flags & 2) == 0 else "no"
        print(f"    {'flags':24s} = 0x{flags:x} (contiguous={contiguous})")
    except gdb.error:
        pass


def _disable_inner_bps(ctx):
    """Temporarily disable mmap/sbrk/brk breakpoints to avoid triggering
    them during inferior function calls.  Returns list of disabled bps."""
    disabled = []
    if ctx is None:
        return disabled
    for bp in [ctx.sbrk_bp, ctx.mmap_bp, ctx.brk_bp]:
        if bp and bp.is_valid() and bp.enabled:
            bp.enabled = False
            disabled.append(bp)
    return disabled


def _reenable_bps(disabled):
    """Re-enable breakpoints that were disabled by _disable_inner_bps."""
    for bp in disabled:
        if bp.is_valid():
            bp.enabled = True


def dump_malloc_info(ctx=None):
    """Call malloc_info(0, stderr) in the inferior for full XML dump."""
    disabled = _disable_inner_bps(ctx)
    try:
        gdb.execute("call (int)malloc_info(0, stderr)")
        print("  [MALLOC] malloc_info() output sent to container stderr")
    except gdb.error as e:
        print(f"  [MALLOC] malloc_info() failed: {e}")
    finally:
        _reenable_bps(disabled)


def dump_mallinfo2(ctx=None):
    """Call mallinfo2() and print the returned struct."""
    print("  [MALLOC] === mallinfo2 ===")
    fields = [
        "arena",      # non-mmapped space from system
        "ordblks",    # number of free chunks
        "smblks",     # number of fastbin blocks
        "hblks",      # number of mmapped regions
        "hblkhd",     # space in mmapped regions
        "fsmblks",    # space in freed fastbin blocks
        "uordblks",   # total allocated space
        "fordblks",   # total free space
        "keepcost",   # top-most releasable space
    ]
    disabled = _disable_inner_bps(ctx)
    try:
        gdb.execute("set $mi = (struct mallinfo2)mallinfo2()")
        for f in fields:
            try:
                val = int(gdb.parse_and_eval(f"$mi.{f}"))
                print(f"    {f:16s} = {val:>12d} (0x{val:x})")
            except gdb.error:
                print(f"    {f:16s} = <unavailable>")
    except gdb.error as e:
        # mallinfo2 may not exist in glibc 2.35; try mallinfo
        print(f"    mallinfo2 failed ({e}), trying mallinfo...")
        try:
            gdb.execute("set $mi = (struct mallinfo)mallinfo()")
            for f in fields:
                try:
                    val = int(gdb.parse_and_eval(f"$mi.{f}"))
                    print(f"    {f:16s} = {val:>12d} (0x{val:x})")
                except gdb.error:
                    pass
        except gdb.error as e2:
            print(f"    mallinfo also failed: {e2}")
    finally:
        _reenable_bps(disabled)


def dump_glibc_version(ctx=None):
    """Print the glibc version string from the inferior."""
    disabled = _disable_inner_bps(ctx)
    try:
        ver = gdb.parse_and_eval(
            '(const char *)gnu_get_libc_version()').string()
        print(f"  [MALLOC] glibc version = {ver}")
    except gdb.error as e:
        print(f"  [MALLOC] glibc version: <error: {e}>")
    finally:
        _reenable_bps(disabled)


def dump_env_and_limits(ctx=None):
    """Dump malloc-relevant environment variables and resource limits."""
    disabled = _disable_inner_bps(ctx)
    try:
        # Environment variables that affect malloc behavior
        for var in ["GLIBC_TUNABLES", "MALLOC_ARENA_MAX",
                    "MALLOC_MMAP_THRESHOLD_", "MALLOC_TOP_PAD_",
                    "MALLOC_TRIM_THRESHOLD_", "MALLOC_MMAP_MAX_"]:
            try:
                val = gdb.parse_and_eval(f'(const char *)getenv("{var}")')
                s = val.string() if int(val) != 0 else "(unset)"
                print(f"  [ENV] {var} = {s}")
            except gdb.error:
                print(f"  [ENV] {var} = <unavailable>")

        # Resource limits — read from /proc/<pid>/limits on the host
        pid = get_host_pid()
        if pid is not None:
            limits_path = f"/proc/{pid}/limits"
            try:
                with open(limits_path, "r") as f:
                    for line in f:
                        if any(k in line for k in
                               ["Max address space", "Max data size",
                                "Max resident set"]):
                            print(f"  [RLIMIT] {line.rstrip()}")
            except Exception as e:
                print(f"  [RLIMIT] <error reading {limits_path}: {e}>")
        else:
            print("  [RLIMIT] (unavailable -- no host PID)")
    finally:
        _reenable_bps(disabled)


def dump_proc_status():
    """Dump memory-related fields from /proc/self/status via the host."""
    pid = get_host_pid()
    if pid is None:
        print("  [VMSTAT] (unavailable -- no host PID)")
        return
    status_path = f"/proc/{pid}/status"
    try:
        with open(status_path, "r") as f:
            for line in f:
                if any(line.startswith(k) for k in
                       ["VmSize:", "VmPeak:", "VmRSS:", "VmData:",
                        "VmStk:", "VmLib:", "VmSwap:"]):
                    print(f"  [VMSTAT] {line.rstrip()}")
    except Exception as e:
        print(f"  [VMSTAT] <error reading {status_path}: {e}>")


_malloc_state_first_call = True


def dump_malloc_state(label, ctx=None):
    """Dump comprehensive malloc state (mp_, arena, mallinfo, malloc_info).
    Pass ctx (LoadContext) when inner breakpoints are active so they get
    temporarily disabled during inferior function calls.
    Static context (version, env, limits) is only dumped on the first call."""
    global _malloc_state_first_call
    print(f"  [MALLOC] === malloc state dump: {label} ===")
    if _malloc_state_first_call:
        dump_glibc_version(ctx)
        dump_env_and_limits(ctx)
        _malloc_state_first_call = False
    dump_proc_status()
    dump_malloc_params()
    dump_arena_state()
    dump_mallinfo2(ctx)
    dump_malloc_info(ctx)
    print(f"  [MALLOC] === end malloc state dump ===")


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
                    dump_malloc_state(f"after-realloc-cap-{ctx.prev_capacity}-to-{cap}", ctx)
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
        dump_malloc_state("on-entry")

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
