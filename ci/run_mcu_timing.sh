#!/bin/bash
# Per-tick instruction-count profile of the drone reflex loop on Cortex-M
# under qemu -icount. Usage: run_mcu_timing.sh <elf> <qemu-machine> [shift]
#   mps2-an385 = Cortex-M3, mps2-an500 = Cortex-M7.
# With -icount shift=S one guest instruction = 2^S ns of virtual time and
# the emulated SysTick runs at 25 MHz (40 ns), so one SysTick = 40 / 2^S
# instructions. The ELF records every tick's SysTick delta in g_tick_hist;
# this script reads the array through the monitor and prints the hash,
# p50 / p90 / p99 / max per-tick instruction counts, how many ticks are
# heavy, and the derived bound at 600 MHz for 1 and 2 cycles per
# instruction. Deterministic: the same ELF gives the same numbers. This is
# an instruction count, not a silicon cycle count.
set -u
ELF=$1; MACHINE=$2; SHIFT=${3:-0}
NM=${NM:-arm-none-eabi-nm}
addr() { $NM "$ELF" | awk -v s="$1" '$3 == s {print $1}'; }
A_HASH=$(addr g_drone_loop_hash); A_HIST=$(addr g_tick_hist); A_CNT=$(addr g_tick_count)
OUT=$( ( sleep 25
         printf 'xp /gx 0x%s\n' "$A_HASH"
         printf 'xp /wx 0x%s\n' "$A_CNT"
         printf 'xp /2000wx 0x%s\n' "$A_HIST"
         sleep 2
         printf 'quit\n' ) | timeout 120 qemu-system-arm \
           -machine "$MACHINE" -nographic -serial none -monitor stdio \
           -icount shift=$SHIFT,align=off -kernel "$ELF" 2>&1 | tr -d '\033\r' | tr -d '[]K' )
HASH=$(echo "$OUT" | grep -i "^0*$A_HASH:" | head -1 | sed 's/.*: *//')
CNT=$(echo "$OUT" | grep -i "^0*$A_CNT:" | head -1 | sed 's/.*: *//')
VALS=$(echo "$OUT" | grep -E '^0*[0-9a-f]+: 0x' | grep -v -i "^0*$A_HASH:" | grep -v -i "^0*$A_CNT:"   | sed 's/^[^:]*: *//' | tr ' ' '
' | grep -E '^0x' | head -n $((CNT))   | while read -r h; do printf '%d
' $(( (h * 40) >> SHIFT )); done)
N=$(echo "$VALS" | wc -l)
SORTED=$(echo "$VALS" | sort -n)
pick() { echo "$SORTED" | sed -n "$1p"; }
P50=$(pick $(( N * 50 / 100 + 1 ))); P90=$(pick $(( N * 90 / 100 ))); P99=$(pick $(( N * 99 / 100 ))); MAX=$(pick "$N")
MEAN=$(echo "$VALS" | awk '{ s += $1 } END { printf "%d", s / NR }')
HEAVY=$(echo "$VALS" | awk -v m="$MEAN" '$1 > 2 * m { c++ } END { print c + 0 }')
echo "MCU_TIMING $MACHINE hash=$HASH ticks=$N instr/tick p50=$P50 p90=$P90 p99=$P99 max=$MAX mean=$MEAN heavy_ticks(>2x_mean)=$HEAVY"
echo "MCU_TIMING $MACHINE at 600 MHz: p50 $(( P50 / 600 ))-$(( P50 * 2 / 600 )) us, max $(( MAX / 600 ))-$(( MAX * 2 / 600 )) us (1-2 cycles/instr); tick budget 1000 us"
