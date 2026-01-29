import gdb
import re

def hexdump(data, base_addr, bytes_per_line=16):
    """Format bytes as hex dump with address and ASCII."""
    lines = []
    for offset in range(0, len(data), bytes_per_line):
        chunk = data[offset:offset + bytes_per_line]
        hex_part = ' '.join(f'{b:02x}' for b in chunk)
        ascii_part = ''.join(chr(b) if 32 <= b < 127 else '.' for b in chunk)
        lines.append(f"  {base_addr + offset:016x}  {hex_part:<{bytes_per_line * 3}} |{ascii_part}|")
    return '\n'.join(lines)


def get_files_data_ptr():
    """Get the current data pointer for this->Internal->Files. Returns None if not in right context."""
    try:
        files = gdb.parse_and_eval("this->Internal->Files")
        impl = files["_M_impl"]
        return int(impl["_M_start"])
    except gdb.error:
        # Expected when not in a Directory method context
        return None


def get_string(val):
    """Extract string from std::string value."""
    return val["_M_dataplus"]["_M_p"].string()


# Global state for tracking
class State:
    enabled = False
    iteration = 0
    last_known_ptr = None
    loop_iteration = 0


class FilesAccessBreakpoint(gdb.Breakpoint):
    """Generic breakpoint to log any access to this->Internal->Files."""

    def __init__(self, loc, description):
        super().__init__(loc, gdb.BP_BREAKPOINT)
        self.description = description
        self.loc = loc

    def stop(self):
        if not State.enabled:
            return False

        ptr = get_files_data_ptr()
        ptr_str = hex(ptr) if ptr else "NULL"

        # Check for pointer change
        change_note = ""
        if State.last_known_ptr is not None and ptr != State.last_known_ptr:
            change_note = f" [CHANGED from {hex(State.last_known_ptr)}!]"
        if ptr:
            State.last_known_ptr = ptr

        print(f"[ACCESS] {self.loc}: {self.description} -> data={ptr_str}{change_note}")
        return False


class DirectoryLoadBreakpoint(gdb.Breakpoint):
    """Breakpoint on Directory::Load loop - detailed logging with hex dump."""

    def __init__(self):
        super().__init__("Directory.cxx:236", gdb.BP_BREAKPOINT)

    def stop(self):
        if not State.enabled:
            return False

        State.iteration += 1

        d_name = gdb.parse_and_eval("d->d_name").string()

        files = gdb.parse_and_eval("this->Internal->Files")
        impl = files["_M_impl"]
        start = impl["_M_start"]
        finish = impl["_M_finish"]
        end_storage = impl["_M_end_of_storage"]

        data_ptr = int(start)
        size = int(finish - start)
        capacity = int(end_storage - start)

        # Check for pointer change
        change_note = ""
        if State.last_known_ptr is not None and data_ptr != State.last_known_ptr:
            change_note = f" [CHANGED from {hex(State.last_known_ptr)}!]"
        if data_ptr:
            State.last_known_ptr = data_ptr

        print(f"\n=== Directory::Load iteration {State.iteration} ===")
        print(f"d->d_name: {d_name}")
        print(f"Files.data(): {hex(data_ptr)}{change_note}")
        print(f"Files.size(): {size}")
        print(f"Files.capacity(): {capacity}")

        # Dump vector contents from internal structure
        if size > 0:
            print("Files contents:")
            for i in range(size):
                elem = start[i]
                s = elem["_M_dataplus"]["_M_p"].string()
                print(f"  [{i}]: {s}")

        # Hex dump of entire buffer (including uninitialized capacity)
        if capacity > 0 and data_ptr != 0:
            elem_size = start.dereference().type.sizeof
            buffer_size = capacity * elem_size
            mem = gdb.selected_inferior().read_memory(data_ptr, buffer_size)
            print(f"Buffer hex dump ({buffer_size} bytes, sizeof(std::string)={elem_size}):")
            print(hexdump(bytes(mem), data_ptr))

        return False


class DirectoryLoadReturnBreakpoint(gdb.Breakpoint):
    """Breakpoint after d.Load() call in GetDirectoryContent to log return value."""

    def __init__(self):
        super().__init__("cmGlobalGenerator.cxx:3001", gdb.BP_BREAKPOINT)

    def stop(self):
        if not State.enabled:
            return False

        n = gdb.parse_and_eval("n")
        print(f"[LOAD] d.Load() returned true, n={n} files")
        return False


class DirectoryLoadFailedBreakpoint(gdb.Breakpoint):
    """Breakpoint when Load() returns false - after the if block."""

    def __init__(self):
        super().__init__("cmGlobalGenerator.cxx:3009", gdb.BP_BREAKPOINT)

    def stop(self):
        if not State.enabled:
            return False
        print(f"[LOAD] GetDirectoryContent completed")
        return False


class FileLoopBreakpoint(gdb.Breakpoint):
    """Breakpoint at the file iteration loop in CheckDirectoryForName."""

    def __init__(self):
        super().__init__("cmFindLibraryCommand.cxx:441", gdb.BP_BREAKPOINT)

    def stop(self):
        if not State.enabled:
            return False

        State.loop_iteration += 1
        origName = gdb.parse_and_eval("origName")
        name_str = get_string(origName)
        print(f"[LOOP {State.loop_iteration}] Checking file: {name_str}")
        return False


class RegexMatchBreakpoint(gdb.Breakpoint):
    """Breakpoint after regex match to log if file matched."""

    def __init__(self):
        super().__init__("cmFindLibraryCommand.cxx:442", gdb.BP_BREAKPOINT)

    def stop(self):
        if not State.enabled:
            return False

        origName = gdb.parse_and_eval("origName")
        name_str = get_string(origName)
        print(f"  -> REGEX MATCHED: {name_str}")
        return False


class FileExistsBreakpoint(gdb.Breakpoint):
    """Breakpoint when file exists check passes."""

    def __init__(self):
        super().__init__("cmFindLibraryCommand.cxx:445", gdb.BP_BREAKPOINT)

    def stop(self):
        if not State.enabled:
            return False

        test_path = gdb.parse_and_eval("this->TestPath")
        path_str = get_string(test_path)
        print(f"  -> FILE EXISTS: {path_str}")
        return False


class BestPathUpdateBreakpoint(gdb.Breakpoint):
    """Breakpoint when BestPath gets updated."""

    def __init__(self):
        super().__init__("cmFindLibraryCommand.cxx:463", gdb.BP_BREAKPOINT)

    def stop(self):
        if not State.enabled:
            return False
        print(f"  -> BESTPATH UPDATED (line 463 hit)")
        return False


class CheckDirectoryEntryBreakpoint(gdb.Breakpoint):
    """Entry breakpoint for cmFindLibraryHelper::CheckDirectoryForName - enables logging if rcutils."""

    def __init__(self):
        super().__init__("cmFindLibraryHelper::CheckDirectoryForName", gdb.BP_BREAKPOINT)
        self.pattern = re.compile(r'.*rcutils.*')

    def stop(self):
        name = gdb.parse_and_eval("name")
        raw = name["Raw"]
        raw_str = raw["_M_dataplus"]["_M_p"].string()

        if self.pattern.match(raw_str):
            path = gdb.parse_and_eval("path")
            path_str = path["_M_dataplus"]["_M_p"].string()
            print(f"\n>>> Entering CheckDirectoryForName: name.Raw={raw_str}, path={path_str}")
            State.enabled = True
            State.iteration = 0
            State.loop_iteration = 0
            State.last_known_ptr = None
            CheckDirectoryExitBreakpoint()

        return False


class CheckDirectoryExitBreakpoint(gdb.FinishBreakpoint):
    """Exit breakpoint to disable logging when CheckDirectoryForName returns."""

    def __init__(self):
        super().__init__(internal=True)

    def stop(self):
        ret = gdb.parse_and_eval("$x0")  # Return value on aarch64
        print(f"\n<<< Exiting CheckDirectoryForName, return={int(ret)}")
        print("Backtrace at exit:")
        gdb.execute("backtrace 20")
        print(f"Active breakpoints: {len(gdb.breakpoints())}")
        for bp in gdb.breakpoints():
            print(f"  BP {bp.number}: {bp.location} enabled={bp.enabled}")
        State.enabled = False
        return False

    def out_of_scope(self):
        print("<<< CheckDirectoryForName went out of scope (longjmp/exception?)")
        State.enabled = False


# Create breakpoints for all Files access points in Directory.cxx:
FilesAccessBreakpoint("Directory.cxx:57", ".size() in GetNumberOfFiles")
FilesAccessBreakpoint("Directory.cxx:62", ".size() in GetFile bounds check")
FilesAccessBreakpoint("Directory.cxx:65", "operator[] in GetFile")
FilesAccessBreakpoint("Directory.cxx:76", ".clear()")
FilesAccessBreakpoint("Directory.cxx:133", ".push_back() [Windows]")

class DirectoryLoadReturnValueBreakpoint(gdb.FinishBreakpoint):
    """Track the actual return value of Directory::Load."""

    def __init__(self, frame):
        super().__init__(frame, internal=True)

    def stop(self):
        if not State.enabled:
            return False
        ret = gdb.parse_and_eval("$x0")
        print(f"\n[LOAD] Directory::Load returning {int(ret)}")
        print(f"Final state: last_known_ptr={hex(State.last_known_ptr) if State.last_known_ptr else 'None'}, iterations={State.iteration}")
        return False


class DirectoryLoadEntryBreakpoint(gdb.Breakpoint):
    """Track entry to Directory::Load to set up return value tracking."""

    def __init__(self):
        super().__init__("Directory.cxx:220", gdb.BP_BREAKPOINT)

    def stop(self):
        if not State.enabled:
            return False
        print(f"\n[LOAD] Entering Directory::Load")
        DirectoryLoadReturnValueBreakpoint(gdb.selected_frame())
        return False


# Detailed Directory::Load logging
DirectoryLoadEntryBreakpoint()
DirectoryLoadBreakpoint()

# Load return value tracking
DirectoryLoadReturnBreakpoint()
DirectoryLoadFailedBreakpoint()

# File search loop tracking
FileLoopBreakpoint()
RegexMatchBreakpoint()
FileExistsBreakpoint()
BestPathUpdateBreakpoint()

# Trigger breakpoint
CheckDirectoryEntryBreakpoint()

# Continue only if attached to a target
try:
    gdb.execute("continue")
except gdb.error as e:
    print(f"Note: {e} (will continue when target is attached)")
