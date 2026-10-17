/* Per-tick cost of the full drone reflex loop on Cortex-M, measured with
   SysTick under qemu -icount (deterministic virtual time: with
   `-icount shift=0` one guest instruction advances virtual time by 1 ns,
   so SysTick at the emulated 25 MHz counts instructions / 40). Reports
   the max and total per-tick SysTick deltas over the same 2000-tick
   sequence as mcu_drone_loop_smoke (hash cross-checked). Read out by the
   host via the qemu monitor like the smoke's hash. This is an
   instruction-count bound, not a silicon cycle count: the DWT row in
   docs/performance.md stays a hardware-day item. */
#include <stdint.h>

#define SYST_CSR  (*(volatile uint32_t*)0xE000E010u)
#define SYST_RVR  (*(volatile uint32_t*)0xE000E014u)
#define SYST_CVR  (*(volatile uint32_t*)0xE000E018u)

extern "C" {
volatile unsigned long long g_drone_loop_hash = 0;
volatile unsigned long long g_tick_systicks_total = 0;
volatile unsigned long g_tick_systicks_max = 0;
volatile unsigned long g_tick_count = 0;
volatile unsigned long g_tick_max_at = 0;   /* tick index of the worst tick */
volatile unsigned long g_tick_systicks_max_steady = 0;   /* worst tick after the first */
volatile unsigned long g_timing_done = 0;
volatile unsigned long g_tick_hist[2048];   /* per-tick SysTick deltas, read by the host */
}

static uint32_t s_prev = 0;
static int s_started = 0;

static inline void timing_hook() {
    const uint32_t cur = SYST_CVR & 0xFFFFFFu;
    if (s_started) {
        const uint32_t d = (s_prev - cur) & 0xFFFFFFu;   /* down-counter */
        g_tick_systicks_total += d;
        if (d > g_tick_systicks_max) { g_tick_systicks_max = d; g_tick_max_at = g_tick_count; }
        if (g_tick_count > 0 && d > g_tick_systicks_max_steady) g_tick_systicks_max_steady = d;
        if (g_tick_count < 2048) g_tick_hist[g_tick_count] = d;
        ++g_tick_count;
    }
    s_prev = cur;
    s_started = 1;
}
#define ARC_MCU_TICK_HOOK() timing_hook()

#include "mcu_drone_seq.hpp"

static uint8_t g_brain[arc_mcu_drone::kBrainBytes];
static uint8_t g_eng_arena[arc_mcu_drone::kEngineArena];
static uint8_t g_safe_arena[arc_mcu_drone::kSafetyArena];
static uint8_t g_sd_arena[arc_mcu_drone::kSetpointsArena];

extern "C" int main() {
    for (size_t i = 0; i < sizeof(g_eng_arena); ++i) g_eng_arena[i] = 0;
    for (size_t i = 0; i < sizeof(g_safe_arena); ++i) g_safe_arena[i] = 0;
    SYST_RVR = 0xFFFFFFu;
    SYST_CVR = 0;
    SYST_CSR = 5u;                 /* enable, processor clock, no interrupt */
    g_drone_loop_hash = (unsigned long long)arc_mcu_drone::run_loop_hash(
        g_brain, g_eng_arena, g_safe_arena, g_sd_arena);
    g_timing_done = 1;
    for (;;) { }
}
