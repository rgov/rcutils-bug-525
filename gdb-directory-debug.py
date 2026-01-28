import gdb

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
    def __init__(self):
        super().__init__("Directory.cxx:236", gdb.BP_BREAKPOINT)
        self.iteration = 0

    def stop(self):
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
            for i in range(min(size, 50)):  # Limit to 50 entries
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

DirectoryLoadBreakpoint()
# Continue only if attached to a target
try:
    gdb.execute("continue")
except gdb.error as e:
    print(f"Note: {e} (will continue when target is attached)")
