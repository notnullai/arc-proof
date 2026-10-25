/* Host run of the plasticity differential test (tests/mcu/plasticity_equiv.hpp):
   arc_plasticity_step against the word-level rule on random states. */
#include "mcu/plasticity_equiv.hpp"
#include <cstdio>

int main() {
    static uint8_t brain[arc_mcu_drone::kBrainBytes];
    static uint8_t arena[arc_mcu_drone::kEngineArena];
    static arc_equiv::Snap ref, got;
    const uint32_t trials = 4000;
    const uint32_t bad = arc_equiv::run(brain, arena, trials, 0x9E3779B9u, &ref, &got);
    printf("PLASTICITY_EQUIV trials=%u differ=%u\n", (unsigned)trials, (unsigned)bad);
    if (bad != 0) { printf("FAIL\n"); return 1; }
    printf("PASS: every pass matches the word-level rule\n");
    return 0;
}
