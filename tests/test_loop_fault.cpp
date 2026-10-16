/* Loop-level fault test: a mid-run E-stop must produce the software ramp —
   interpolate from the last commanded setpoint to the safe setpoint over
   safe_ramp_ticks, then hold — not an instant zero. */
#include "arc/arc.h"
#include "arc/arc_hal.h"
#include "arc/arc_loop.h"
#include "arc/arc_plasticity.h"
#include "arc/arc_safety.h"
#include "arc/arc_types.h"
#include "test_plastic_util.hpp"
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <vector>

struct FaultCtx {
    arc_engine* e;
    uint32_t tick;
    uint32_t estop_at;
    uint32_t seed;
    std::vector<std::vector<int32_t>> history;   /* written setpoints per tick */
};

static void fctx_reward(void* ctx, int32_t* r, int n) {
    FaultCtx* c = (FaultCtx*)ctx;
    (void)n;
    for (int w = 0; w < c->e->geom.input_words; ++w) {
        c->seed = c->seed * 1664525u + 1013904223u;
        c->e->input[w] = c->seed;
    }
    r[0] = 0; r[1] = 0; r[2] = 0;   /* zero reward: no flips, but loop runs */
}
static uint8_t fctx_phase(void* ctx) { (void)ctx; return 0; }
static uint64_t fctx_now(void* ctx) { return (uint64_t)((FaultCtx*)ctx)->tick * 1000; }
static bool fctx_estop(void* ctx) { return ((FaultCtx*)ctx)->tick >= ((FaultCtx*)ctx)->estop_at; }
static void fctx_wait(void* ctx, uint64_t /*deadline*/) { ((FaultCtx*)ctx)->tick++; }
static void fctx_kick(void* ctx) { (void)ctx; }
static void fctx_write(void* ctx, const int32_t* sp, int n) {
    FaultCtx* c = (FaultCtx*)ctx;
    c->history.push_back(std::vector<int32_t>(sp, sp + n));
}

int main() {
    int failed = 0;

    const int HID = 32;
    arc_plasticity_config cfg = pltest::default_cfg();
    cfg.plasticity_interval = 10;
    std::vector<int32_t> wp = {65536, 0, 0};

    pltest::Harness h(64, HID, 4, cfg, wp, /*excite=*/1);
    if (!h) { printf("FAIL: harness\n"); return 1; }
    arc_engine* e = h.e;

    int32_t safe_sp[3] = {0, 0, 0};
    int32_t safe_ramp[3] = {10, 10, 0};
    int32_t safe_buf[3] = {0, 0, 0};
    arc_safety_config scfg;
    std::memset(&scfg, 0, sizeof(scfg));
    scfg.max_setpoint_q16 = 65536;
    scfg.max_rate_per_tick_q16 = 10000;
    scfg.safe_setpoint_q16 = safe_sp;
    scfg.safe_ramp_ticks = safe_ramp;
    scfg.tick_budget_us = 1000;
    scfg.watchdog_misses = 3;
    scfg.safe_setpoints_buf = safe_buf;
    std::vector<uint8_t> sarena(4096);
    arc_safety* s = nullptr;
    if (arc_safety_create(&scfg, sarena.data(), sarena.size(), &s) != ARC_OK) {
        printf("FAIL: safety\n"); return 1;
    }

    uint8_t groups[4] = {0, 0, 1, 1};
    std::vector<uint8_t> sd_arena(arc_setpoints_required_bytes(groups, 4) + 16);
    arc_setpoints* sd = nullptr;
    if (arc_setpoints_create(groups, 4, 32768, sd_arena.data(), sd_arena.size(), &sd) != ARC_OK) {
        printf("FAIL: setpoints\n"); return 1;
    }

    const uint32_t estop_at = 300;
    const uint32_t total = estop_at + 40;

    FaultCtx ctx;
    ctx.e = e;
    ctx.tick = 0;
    ctx.estop_at = estop_at;
    ctx.seed = 0xC0DE;

    arc_hal hal;
    std::memset(&hal, 0, sizeof(hal));
    hal.ctx = &ctx;
    hal.write_motors = fctx_write;
    hal.now_us = fctx_now;
    hal.estop_asserted = fctx_estop;
    hal.kick_hardware_watchdog = fctx_kick;
    hal.wait_until = fctx_wait;
    hal.read_reward = fctx_reward;
    hal.read_phase = fctx_phase;

    arc_loop_config lcfg;
    std::memset(&lcfg, 0, sizeof(lcfg));
    lcfg.period_us = 1000;
    lcfg.start_us = 0;
    lcfg.max_ticks = total;

    arc_run(e, s, sd, nullptr, 0, &hal, &lcfg);

    /* History index == tick (wait_until increments before each tick body). */
    if (ctx.history.size() < total - 1) { printf("FAIL: history short (%zu)\n", ctx.history.size()); return 1; }

    /* Find the first fault-path tick: the one after estop_at. */
    const size_t pre = estop_at - 1;          /* last normal tick */
    const size_t f0 = estop_at;               /* first fault tick */
    const int32_t last_cmd = ctx.history[pre][0];

    /* The ramp must start from the last commanded value (not zero). */
    const int32_t first_fault = ctx.history[f0][0];
    if (last_cmd > 0) {
        if (!(first_fault < last_cmd && first_fault >= 0)) {
            printf("FAIL: ramp did not start from current (%d -> %d)\n",
                   last_cmd, first_fault);
            failed++;
        } else { printf("PASS: ramp starts from last commanded (%d -> %d)\n", last_cmd, first_fault); }
    } else {
        printf("SKIP: no commanded motion before estop (last_cmd=%d)\n", last_cmd);
    }

    /* After safe_ramp_ticks from the fault tick, the setpoint must be 0. */
    const size_t done = f0 + 10;
    if (done < ctx.history.size()) {
        if (ctx.history[done][0] != 0 || ctx.history[done][1] != 0) {
            printf("FAIL: setpoint not at safe after ramp (%d, %d)\n",
                   ctx.history[done][0], ctx.history[done][1]);
            failed++;
        } else { printf("PASS: safe setpoint reached after ramp_ticks\n"); }
    }

    /* And it holds. */
    const size_t later = ctx.history.size() - 1;
    if (ctx.history[later][0] != 0 || ctx.history[later][1] != 0) {
        printf("FAIL: setpoint drifted after ramp\n"); failed++;
    } else { printf("PASS: safe setpoint holds\n"); }

    printf("\nLoop-fault tests: %s\n", failed ? "FAILED" : "PASSED");
    return failed ? 1 : 0;
}
