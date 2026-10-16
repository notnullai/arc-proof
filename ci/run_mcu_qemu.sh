#!/bin/bash
# Verify the drone reflex-loop golden hash on emulated Cortex-M3 and
# Cortex-M7. The ELFs in targets/ are prebuilt (binary-only engine, same
# provenance as lib/) and are run unmodified under qemu-system-arm; each
# program computes its loop hash into a global and spins, and the qemu
# monitor reads the value from guest memory. No semihosting needed.
#
# Requires: qemu-system-arm, arm-none-eabi-nm (apt: gcc-arm-none-eabi qemu-system-arm)
set -u

ELF_DIR="$(cd "$(dirname "$0")/.." && pwd)/targets"
GOLDEN=0xac29f2cfa8126d1b

fail() { echo "FAIL: $1"; exit 1; }

run_one() {
    ELF=$1
    MACHINE=$2
    SYMBOL=g_drone_loop_hash

    [ -f "$ELF" ] || fail "$ELF not found"
    ADDR=$(arm-none-eabi-nm "$ELF" | awk -v s="$SYMBOL" '$3 == s {print $1}')
    [ -n "$ADDR" ] || fail "$SYMBOL not found in $ELF"

    OUT=$( ( sleep 15
             printf 'xp /gx 0x%s\n' "$ADDR"
             sleep 1
             printf 'quit\n' ) | timeout 90 qemu-system-arm \
               -machine "$MACHINE" -nographic -serial none -monitor stdio \
               -kernel "$ELF" 2>&1 )
    OUT_CLEAN=$(printf '%s' "$OUT" | tr -d '\033\133\135' | tr -d '[]K')
    echo "$OUT_CLEAN" | grep -q "$GOLDEN" || fail "$SYMBOL mismatch on $MACHINE: $OUT_CLEAN"
    echo "PASS: $SYMBOL = $GOLDEN on $MACHINE"
}

run_one "$ELF_DIR/mcu_drone_loop_smoke_m3" mps2-an385
run_one "$ELF_DIR/mcu_drone_loop_smoke_m7" mps2-an500
echo "PASS: both Cortex-M twins match the committed golden hash"
