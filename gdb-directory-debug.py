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


class DirectoryLoadBreakpoint(gdb.Breakpoint):
    """Breakpoint on Directory::Load loop - only active when triggered by rcutils search."""

    def __init__(self):
        super().__init__("Directory.cxx:236", gdb.BP_BREAKPOINT)
        self.enabled = False  # Start disabled
        self.iteration = 0

    def stop(self):
        if not self.enabled:
            return False

        self.iteration += 1

        # Get d->d_name (fresh each time)
        try:
            d_name = gdb.parse_and_eval("d->d_name").string()
        except Exception as e:
            d_name = f"<error: {e}>"

        # Get vector info from internal structure (methods may be inlined)
        try:
            files = gdb.parse_and_eval("this->Internal->Files")
            impl = files["_M_impl"]
            start = impl["_M_start"]
            finish = impl["_M_finish"]
            end_storage = impl["_M_end_of_storage"]

            data_ptr = int(start)
            size = int(finish - start)
            capacity = int(end_storage - start)
        except Exception as e:
            print(f"[{self.iteration}] Error reading vector: {e}")
            return False

        print(f"\n=== Directory::Load iteration {self.iteration} ===")
        print(f"d->d_name: {d_name}")
        print(f"Files.data(): {hex(data_ptr)}")
        print(f"Files.size(): {size}")
        print(f"Files.capacity(): {capacity}")

        # Dump vector contents from internal structure
        if size > 0:
            print("Files contents:")
            for i in range(size):  # Limit to 50 entries
                try:
                    elem = start[i]
                    # std::string internal: _M_dataplus._M_p points to char data
                    s = elem["_M_dataplus"]["_M_p"].string()
                    print(f"  [{i}]: {s}")
                except Exception as e:
                    print(f"  [{i}]: <error: {e}>")

        # Hex dump of entire buffer (including uninitialized capacity)
        if capacity > 0 and data_ptr != 0:
            try:
                # Calculate buffer size: capacity * sizeof(std::string)
                elem_size = start.dereference().type.sizeof
                buffer_size = capacity * elem_size
                mem = gdb.selected_inferior().read_memory(data_ptr, buffer_size)
                print(f"Buffer hex dump ({buffer_size} bytes, sizeof(std::string)={elem_size}):")
                print(hexdump(bytes(mem), data_ptr))
            except Exception as e:
                print(f"Hex dump error: {e}")

        return False  # Continue execution


class CheckDirectoryEntryBreakpoint(gdb.Breakpoint):
    """Entry breakpoint for cmFindLibraryHelper::CheckDirectoryForName - enables logging if rcutils."""

    def __init__(self, dir_bp):
        super().__init__("cmFindLibraryHelper::CheckDirectoryForName", gdb.BP_BREAKPOINT)
        self.dir_bp = dir_bp
        self.pattern = re.compile(r'.*rcutils.*')

    def stop(self):
        # Check if the library name (name.Raw) matches rcutils
        try:
            name = gdb.parse_and_eval("name")
            raw = name["Raw"]
            raw_str = raw["_M_dataplus"]["_M_p"].string()

            if self.pattern.match(raw_str):
                path = gdb.parse_and_eval("path")
                path_str = path["_M_dataplus"]["_M_p"].string()
                print(f"\n>>> Entering CheckDirectoryForName: name.Raw={raw_str}, path={path_str}")
                self.dir_bp.enabled = True
                self.dir_bp.iteration = 0
                # Set a finish breakpoint to disable when we return
                CheckDirectoryExitBreakpoint(self.dir_bp)
        except Exception as e:
            print(f"CheckDirectoryEntry error: {e}")

        return False  # Continue execution


class CheckDirectoryExitBreakpoint(gdb.FinishBreakpoint):
    """Exit breakpoint to disable Directory::Load logging when CheckDirectoryForName returns."""

    def __init__(self, dir_bp):
        super().__init__(internal=True)
        self.dir_bp = dir_bp

    def stop(self):
        print(f"<<< Exiting CheckDirectoryForName")
        self.dir_bp.enabled = False
        return False

    def out_of_scope(self):
        self.dir_bp.enabled = False


# Create breakpoints
dir_bp = DirectoryLoadBreakpoint()
CheckDirectoryEntryBreakpoint(dir_bp)

# Continue only if attached to a target
try:
    gdb.execute("continue")
except gdb.error as e:
    print(f"Note: {e} (will continue when target is attached)")
