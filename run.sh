#!/usr/bin/env bash
# run.sh — Build and sweep RLIMIT_AS headroom to trigger the readdir+vector errno bug
set -euo pipefail

echo "=== readdir() + std::vector errno pollution reproducer ==="
echo

# ── 1. Install & compile ────────────────────────────────────────────────
echo "[1/3] Installing g++ and strace..."
apt-get update -qq && apt-get install -y -qq g++ strace >/dev/null 2>&1
echo "[2/3] Compiling repro.cpp..."
g++ -O2 -o /work/repro /work/repro.cpp

# ── 2. Create test directory with 256 long-named files ──────────────────
echo "[3/3] Creating test directory..."
TESTDIR=$(mktemp -d)
seq -w 1 256 | xargs -I{} touch "$TESTDIR/file_{}_padding_to_defeat_sso_xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx"
echo "      ${TESTDIR}: $(ls -1 "$TESTDIR" | wc -l) files"

# ── 3. Sweep a narrow headroom range (known window is ~20 kB) ───────────
echo
echo "Sweeping headroom (kB above VmSize):"
BUG_FOUND=0
BUG_HEADROOM=0

for headroom in $(seq 40 -4 4); do
    RC=0
    OUTPUT=$(/work/repro "$TESTDIR" "$headroom" 2>&1) || RC=$?

    case $RC in
        0) STATUS="BUG"
           if [ "$BUG_FOUND" -eq 0 ]; then BUG_FOUND=1; BUG_HEADROOM=$headroom; fi ;;
        1) STATUS="ok " ;;
        *) STATUS="OOM" ;;
    esac
    printf "  %3d kB  →  %s\n" "$headroom" "$STATUS"
done

echo
if [ "$BUG_FOUND" -eq 1 ]; then
    echo "BUG HIT at headroom=${BUG_HEADROOM} kB — re-running under strace:"
    echo
    strace -e trace=mmap,brk /work/repro "$TESTDIR" "$BUG_HEADROOM" 2>&1 | \
        grep -E '(mmap|brk|BUG|RLIMIT|ENOMEM)' || true
    echo
    echo "mmap()=-1 ENOMEM then brk()=0x... (success) proves malloc fell back,"
    echo "leaving errno polluted from the failed mmap."
else
    echo "Bug NOT reproduced — glibc/kernel version may handle this differently."
fi

rm -rf "$TESTDIR"
echo "Done."
