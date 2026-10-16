/* Host twin of the Cortex-M drone full-loop smoke test (Plan Step 5a):
   the complete arc_run on the fixed 4-ESC brain, compared against the
   committed golden hash. CI runs this on x86_64 and aarch64; the qemu
   jobs run the real Cortex-M builds of the same sequence. */
#include "mcu/mcu_drone_seq.hpp"
#include <cstdio>

int main() {
    static uint8_t brain[arc_mcu_drone::kBrainBytes];
    static uint8_t eng_arena[arc_mcu_drone::kEngineArena];
    static uint8_t safe_arena[arc_mcu_drone::kSafetyArena];
    static uint8_t sd_arena[arc_mcu_drone::kSetpointsArena];

    const uint64_t h =
        arc_mcu_drone::run_loop_hash(brain, eng_arena, safe_arena, sd_arena);

    printf("DRONE_LOOP_HASH %016llx\n", (unsigned long long)h);
    if (h == ARC_MCU_DRONE_LOOP_EXPECTED_HASH) {
        printf("PASS: matches committed golden drone loop hash\n");
        return 0;
    }
    printf("FAIL: expected %016llx\n",
           (unsigned long long)ARC_MCU_DRONE_LOOP_EXPECTED_HASH);
    return 1;
}
