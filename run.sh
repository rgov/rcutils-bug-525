#!/usr/bin/env bash
# run.sh — Build and sweep RLIMIT_AS headroom to trigger the readdir+vector errno bug
set -euo pipefail

echo "=== readdir() + std::vector errno pollution reproducer ==="
echo

# ── 1. Install dependencies ──────────────────────────────────────────────
echo "[1/5] Installing g++ and strace..."
apt-get update -qq && apt-get install -y -qq g++ strace >/dev/null 2>&1
echo "      Done."

# ── 2. Compile ────────────────────────────────────────────────────────────
echo "[2/5] Compiling repro.cpp..."
g++ -O2 -o /work/repro /work/repro.cpp
echo "      Done."

# ── 3. Create test directory with 256 long-named files ───────────────────
echo "[3/5] Creating test directory with 256 files..."
TESTDIR=$(mktemp -d)
for i in $(seq 1 256); do
    # ~70-char filenames defeat libstdc++ SSO (threshold is 15 chars)
    printf -v name "file_%04d_padding_to_defeat_small_string_optimization_abcdefghijklmnop" "$i"
    touch "${TESTDIR}/${name}"
done
echo "      Created 256 files in ${TESTDIR}"
NFILES=$(ls -1 "${TESTDIR}" | wc -l)
echo "      Verified: ${NFILES} files (+ . and .. = $((NFILES + 2)) entries)"

# ── 4. Baseline run (no limit) to measure VmPeak ────────────────────────
echo "[4/5] Baseline run (no memory limit)..."
BASELINE=$(/work/repro "${TESTDIR}") || true
echo "      ${BASELINE}"
VMPEAK_KB=$(echo "${BASELINE}" | sed -n 's/^VmPeak: *\([0-9]*\) kB.*/\1/p')
echo "      VmPeak = ${VMPEAK_KB} kB"

# ── 5. Sweep RLIMIT_AS headroom to find the window ───────────────────────
echo "[5/5] Sweeping RLIMIT_AS headroom to find the mmap-fail / sbrk-succeed window..."
echo "      (headroom = extra kB of virtual memory above current VmSize)"
echo

BUG_FOUND=0
BUG_HEADROOM=0

# Sweep from generous headroom down to zero in 4 kB steps
for headroom in $(seq 256 -4 0); do
    RC=0
    OUTPUT=$(/work/repro "${TESTDIR}" "${headroom}" 2>&1) || RC=$?

    if echo "${OUTPUT}" | grep -q "BUG REPRODUCED"; then
        STATUS="BUG"
        if [ "${BUG_FOUND}" -eq 0 ]; then
            BUG_FOUND=1
            BUG_HEADROOM=${headroom}
        fi
    elif echo "${OUTPUT}" | grep -q "NO BUG"; then
        STATUS="ok "
    elif echo "${OUTPUT}" | grep -q "ALLOC FAILURE"; then
        STATUS="OOM"
    else
        STATUS="???"
    fi

    printf "  headroom %4d kB  →  %s  (exit %d)\n" "${headroom}" "${STATUS}" "${RC}"
done

echo
echo "────────────────────────────────────────────────────────"

if [ "${BUG_FOUND}" -eq 1 ]; then
    echo "BUG HIT at headroom ${BUG_HEADROOM} kB"
    echo
    echo "Re-running with strace to show mmap → ENOMEM → brk → success:"
    echo
    strace -e trace=mmap,brk /work/repro "${TESTDIR}" "${BUG_HEADROOM}" 2>&1 | \
        grep -E '(mmap|brk|BUG REPRODUCED|NO BUG|VmPeak|ENOMEM|RLIMIT)' || true
    echo
    echo "Above: mmap(...) = -1 ENOMEM followed by brk(...) = 0x... (success)"
    echo "This proves malloc fell back to brk after mmap failed, leaving errno polluted."
else
    echo "Bug NOT reproduced in this sweep range."
    echo "This may happen if glibc or kernel version handles this differently."
fi

echo
echo "────────────────────────────────────────────────────────"

# ── Cleanup ───────────────────────────────────────────────────────────────
rm -rf "${TESTDIR}"
echo "Cleaned up. Done."
