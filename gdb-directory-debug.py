import gdb

def get_errno():
    try:
        return int(gdb.parse_and_eval("*(int*)__errno_location()"))
    except:
        return None

count = 0

class ReaddirBreakpoint(gdb.Breakpoint):
    """Break on readdir call, then check errno after it returns."""
    def __init__(self):
        super().__init__("readdir", gdb.BP_BREAKPOINT)

    def stop(self):
        global count
        count += 1
        gdb.execute("finish", to_string=True)
        errno = get_errno()
        ret = int(gdb.parse_and_eval("$x0"))  # return value on aarch64
        print(f"readdir #{count}: ret={hex(ret)}, errno={errno}")
        return False

ReaddirBreakpoint()
print("[INIT] Breakpoint set on readdir()")

# Continue only if attached to a target
try:
    gdb.execute("continue")
except gdb.error as e:
    print(f"Note: {e} (will continue when target is attached)")


# =============================================================================
# CALL GRAPH ANALYSIS (skipped for now)
# =============================================================================
if False:
    import re

    def get_symbol_at_addr(addr):
        """Get symbol name at address, or empty string if none."""
        try:
            result = gdb.execute(f"info symbol {addr}", to_string=True)
            if "No symbol" in result:
                return ""
            return result.split()[0]
        except:
            return ""

    def get_full_function_disasm(addr):
        """Disassemble function at addr. Returns list of (addr, instruction) tuples."""
        try:
            result = gdb.execute(f"disassemble {addr}", to_string=True)
            lines = []
            for line in result.split('\n'):
                m = re.match(r'[=>\s]*0x([0-9a-fA-F]+)\s+<[^>]+>:\s+(.+)', line)
                if m:
                    insn_addr = int(m.group(1), 16)
                    insn_text = m.group(2).strip()
                    lines.append((insn_addr, insn_text))
            return lines
        except Exception as e:
            return []

    def find_call_targets(disasm_lines):
        """Find all bl (branch-link) call targets from disassembly."""
        targets = []
        for addr, insn in disasm_lines:
            m = re.search(r'\bbl\s+0x([0-9a-fA-F]+)', insn)
            if m:
                target = int(m.group(1), 16)
                targets.append(target)
        return list(set(targets))

    def build_call_graph(start_addr, max_depth=10):
        """Recursively build complete call graph starting from address."""
        visited = {}
        to_visit = [(start_addr, 0)]

        while to_visit:
            addr, depth = to_visit.pop(0)

            if addr in visited:
                continue

            if depth > max_depth:
                visited[addr] = {
                    'symbol': get_symbol_at_addr(addr),
                    'insn_count': 0,
                    'callees': [],
                    'depth': depth,
                    'skipped': 'max_depth'
                }
                continue

            sym = get_symbol_at_addr(addr)

            if '@plt' in sym:
                visited[addr] = {
                    'symbol': sym,
                    'insn_count': 0,
                    'callees': [],
                    'depth': depth,
                    'skipped': 'plt'
                }
                continue

            disasm = get_full_function_disasm(addr)
            if not disasm:
                visited[addr] = {
                    'symbol': sym,
                    'insn_count': 0,
                    'callees': [],
                    'depth': depth,
                    'skipped': 'no_disasm'
                }
                continue

            callees = find_call_targets(disasm)

            visited[addr] = {
                'symbol': sym,
                'insn_count': len(disasm),
                'callees': callees,
                'depth': depth,
                'skipped': None
            }

            for callee in callees:
                if callee not in visited:
                    to_visit.append((callee, depth + 1))

        return visited

    def print_call_graph(graph, start_addr):
        """Print the call graph in a readable format."""
        total = len(graph)
        plt_count = sum(1 for v in graph.values() if v.get('skipped') == 'plt')
        analyzed = sum(1 for v in graph.values() if v.get('skipped') is None)

        print(f"\nSummary: {total} total addresses, {analyzed} analyzed, {plt_count} PLT stubs")

        max_depth = max(v['depth'] for v in graph.values())

        for depth in range(max_depth + 1):
            funcs_at_depth = [(a, v) for a, v in graph.items() if v['depth'] == depth]
            if not funcs_at_depth:
                continue

            print(f"\n--- Depth {depth} ({len(funcs_at_depth)} functions) ---")

            for addr, info in sorted(funcs_at_depth, key=lambda x: x[0]):
                sym = info['symbol'] or f"<{hex(addr)}>"

                if info.get('skipped'):
                    print(f"  {hex(addr)}: {sym} [{info['skipped']}]")
                else:
                    print(f"  {hex(addr)}: {sym}")
                    print(f"      {info['insn_count']} insns, {len(info['callees'])} calls")

    def find_errno_access(graph):
        """Find any functions in the graph that access errno."""
        errno_funcs = []
        for addr, info in graph.items():
            if info.get('skipped'):
                continue
            disasm = get_full_function_disasm(addr)
            for insn_addr, insn in disasm:
                if '__errno_location' in insn:
                    errno_funcs.append((addr, info['symbol'], insn_addr, insn))
                    break
        return errno_funcs
