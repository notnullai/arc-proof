/* Host twin of the worst-case plasticity timing twins (gap register G8):
   the same armed sequence as mcu_drone_loop_timing_x500_480_worst*, checked
   on the host. Every armed plasticity call must flip every synapse it visits
   (rows x (hidden - motor)), leave the Hamming distance where it was armed
   and see every post neuron spike, so the Cortex-M timing of the same
   sequence times the expensive path of every branch. */
#include "mcu/mcu_drone_seq.hpp"
#include <cstdio>

int main() {
    static uint8_t brain[arc_mcu_drone::kBrainBytes];
    static uint8_t eng_arena[arc_mcu_drone::kEngineArena];
    static uint8_t safe_arena[arc_mcu_drone::kSafetyArena];
    static uint8_t sd_arena[arc_mcu_drone::kSetpointsArena];

    const uint64_t h =
        arc_mcu_drone::run_loop_hash(brain, eng_arena, safe_arena, sd_arena);
    const int plastic = arc_mcu_drone::kHidden - arc_mcu_drone::kMotor;
#ifdef ARC_MCU_PLASTICITY_SLICES
    const uint32_t per_call = (uint32_t)((plastic + ARC_MCU_PLASTICITY_SLICES - 1) / ARC_MCU_PLASTICITY_SLICES * plastic);
#else
    const uint32_t per_call = (uint32_t)(plastic * plastic);
#endif
    printf("WORST_PLASTICITY hash=%016llx armed=%u bad=%u flips_max=%u flips_total=%u (per call %u)\n",
           (unsigned long long)h, (unsigned)arc_mcu_drone::g_worst_armed, (unsigned)arc_mcu_drone::g_worst_bad,
           (unsigned)arc_mcu_drone::g_worst_flips_max, (unsigned)arc_mcu_drone::g_worst_flips_total, (unsigned)per_call);
    if (h == 0 || arc_mcu_drone::g_worst_armed < 100 || arc_mcu_drone::g_worst_bad != 0 || arc_mcu_drone::g_worst_flips_max != per_call) {
        printf("FAIL: an armed plasticity call missed the worst case\n");
        return 1;
    }
    printf("PASS: every armed call flipped every synapse it visits\n");
    return 0;
}
