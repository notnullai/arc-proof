/* Fault-injection coverage for the loop path (complements test_loop_fault
   and test_safety): watchdog overrun inside arc_run, estop asserted then
   released (latched until clear), sensor dropout (stale frame), a single
   late tick (tolerated), and the loop-level plasticity gate set on fault
   (not hand-poked). */
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

struct InjCtx {
    arc_engine* e;
    uint32_t tick;
    uint32_t total;
    uint32_t estop_from;     /* inclusive; 0xffffffff = never */
    uint32_t estop_to;       /* exclusive */
    bool drop_inputs;        /* stop refreshing e->input after tick 100 */
    bool late_tick_once;     /* one 5000 us gap at tick 200 */
    bool always_late;        /* every tick advances 5000 us */
    uint32_t seed;
    int gate_seen_faulted;   /* fault_gated observed while fault_state */
    int gate_seen_normal;
    std::vector<std::vector<int32_t>> history;
};

static void inj_reward(void* ctx, int32_t* r, int n) {
    InjCtx* c = (InjCtx*)ctx;
    (void)n;
    if (!(c->drop_inputs && c->tick > 100)) {
        for (int w = 0; w < c->e->geom.input_words; ++w) {
            c->seed = c->seed * 1664525u + 1013904223u;
            c->e->input[w] = c->seed;
        }
    }
    r[0] = 0; r[1] = 0; r[2] = 0;
}
static uint8_t inj_phase(void* ctx) { (void)ctx; return 0; }
static uint64_t inj_now(void* ctx) {
    InjCtx* c = (InjCtx*)ctx;
    /* always_late: 2x budget every tick so the software watchdog counts
       a miss each tick. Otherwise a normal 1 kHz clock, plus one injected
       late tick when late_tick_once is set. */
    uint64_t us = (uint64_t)c->tick * (c->always_late ? 2000 : 1000);
    if (c->late_tick_once && c->tick == 200) us += 5000;
    return us;
}
static bool inj_estop(void* ctx) {
    InjCtx* c = (InjCtx*)ctx;
    return c->tick >= c->estop_from && c->tick < c->estop_to;
}
static void inj_wait(void* ctx, uint64_t) {
    InjCtx* c = (InjCtx*)ctx;
    c->tick++;
}
static void inj_kick(void* ctx) { (void)ctx; }
static void inj_write(void* ctx, const int32_t* sp, int n) {
    InjCtx* c = (InjCtx*)ctx;
    c->history.push_back(std::vector<int32_t>(sp, sp + n));
}

/* Gate observation happens against the safety pointer carried in ctx. */
struct GateProbe {
    InjCtx* ctx;
    arc_safety* s;
    arc_engine* e;
};
static GateProbe g_probe;
static void probe_wait(void* ctx, uint64_t d) {
    (void)ctx;
    (void)d;
    InjCtx* c = g_probe.ctx;
    if (c->e->plasticity && g_probe.s) {
        if (arc_safety_fault_state(g_probe.s)) {
            if (c->e->plasticity->fault_gated) c->gate_seen_faulted++;
        } else if (!c->e->plasticity->fault_gated) {
            c->gate_seen_normal++;
        }
    }
    c->tick++;
}

static bool setup(pltest::Harness& h, arc_engine** e_out, arc_safety** s_out,
                  std::vector<uint8_t>& sarena, std::vector<uint8_t>& sd_arena,
                  arc_setpoints** sd_out, arc_safety_config& scfg,
                  int32_t* safe_sp, int32_t* safe_ramp, int32_t* safe_buf) {
    if (!h) return false;
    g_probe.e = h.e;
    *e_out = h.e;

    std::memset(&scfg, 0, sizeof(scfg));
    scfg.max_setpoint_q16 = 65536;
    scfg.max_rate_per_tick_q16 = 10000;
    scfg.safe_setpoint_q16 = safe_sp;
    scfg.safe_ramp_ticks = safe_ramp;
    scfg.tick_budget_us = 1000;
    scfg.watchdog_misses = 3;
    scfg.safe_setpoints_buf = safe_buf;
    sarena.assign(4096, 0);
    if (arc_safety_create(&scfg, sarena.data(), sarena.size(), s_out) != ARC_OK)
        return false;

    uint8_t groups[4] = {0, 0, 1, 1};
    sd_arena.assign(arc_setpoints_required_bytes(groups, 4) + 16, 0);
    return arc_setpoints_create(groups, 4, 32768,
                                sd_arena.data(), sd_arena.size(), sd_out) == ARC_OK;
}

int main() {
    int failed = 0;
    /* sd has groups {0,0,1,1} -> n_dofs=2. safety n_dofs is discovered by
       scanning until BOTH setpoint and ramp are 0, so a 3rd terminator
       entry keeps it at 2, matching sd. */
    int32_t safe_sp[3] = {0, 0, 0};
    int32_t safe_ramp[3] = {10, 10, 0};
    int32_t safe_buf[3] = {0, 0, 0};
    arc_safety_config scfg;
    std::vector<uint8_t> sarena, sd_arena;
    arc_safety* s = nullptr;
    arc_setpoints* sd = nullptr;
    arc_engine* e = nullptr;

    /* --- Case 1: watchdog overrun inside arc_run + loop-level gate --- */
    {
        pltest::Harness hcase(64, 32, 4, [] {
            arc_plasticity_config c = pltest::default_cfg();
            c.plasticity_interval = 10;
            return c;
        }(), std::vector<int32_t>{65536, 0, 0}, 1);
        if (!setup(hcase, &e, &s, sarena, sd_arena, &sd, scfg,
                   safe_sp, safe_ramp, safe_buf)) {
            printf("FAIL: case1 setup\n"); return 1;
        }
        InjCtx ctx{};
        ctx.e = e;
        ctx.total = 60;
        ctx.estop_from = 0xffffffffu;
        ctx.estop_to = 0;
        ctx.seed = 0xA5A5;
        ctx.always_late = true;
        g_probe.ctx = &ctx;
        g_probe.s = s;
        g_probe.e = e;

        arc_hal hal;
        std::memset(&hal, 0, sizeof(hal));
        hal.ctx = &ctx;
        hal.write_motors = inj_write;
        hal.now_us = inj_now;
        hal.kick_hardware_watchdog = inj_kick;
        hal.wait_until = probe_wait;
        hal.read_reward = inj_reward;
        hal.read_phase = inj_phase;

        arc_loop_config lcfg;
        std::memset(&lcfg, 0, sizeof(lcfg));
        lcfg.period_us = 1000;
        lcfg.max_ticks = ctx.total;
        arc_run(e, s, sd, nullptr, 0, &hal, &lcfg);

        if (!arc_safety_fault_state(s)) {
            printf("FAIL: watchdog overrun did not fault\n"); failed++;
        } else { printf("PASS: watchdog overrun faults inside arc_run\n"); }

        if (ctx.gate_seen_faulted == 0) {
            printf("FAIL: loop never set plasticity gate while faulted\n");
            failed++;
        } else { printf("PASS: loop set plasticity gate on fault\n"); }

        const size_t n = ctx.history.size();
        if (n < 20) {
            printf("FAIL: case1 history short (%zu)\n", n); failed++;
        } else {
            const std::vector<int32_t>& held = ctx.history[n - 1];
            if (held[0] != 0 || held[1] != 0) {
                printf("FAIL: ramp did not reach safe (%d,%d)\n",
                       held[0], held[1]); failed++;
            } else { printf("PASS: overrun ramp reaches safe and holds\n"); }
        }
        arc_safety_clear(s);
    }

    /* --- Case 2: estop assert -> release -> latched -> clear -> normal --- */
    {
        pltest::Harness hcase(64, 32, 4, [] {
            arc_plasticity_config c = pltest::default_cfg();
            c.plasticity_interval = 10;
            return c;
        }(), std::vector<int32_t>{65536, 0, 0}, 1);
        if (!setup(hcase, &e, &s, sarena, sd_arena, &sd, scfg,
                   safe_sp, safe_ramp, safe_buf)) {
            printf("FAIL: case2 setup\n"); return 1;
        }
        InjCtx ctx{};
        ctx.e = e;
        ctx.total = 420;
        ctx.estop_from = 300;
        ctx.estop_to = 360;       /* released for the last 60 ticks */
        ctx.seed = 0xBEEF;
        g_probe.ctx = &ctx;
        g_probe.s = s;
        g_probe.e = e;

        arc_hal hal;
        std::memset(&hal, 0, sizeof(hal));
        hal.ctx = &ctx;
        hal.write_motors = inj_write;
        hal.now_us = inj_now;
        hal.estop_asserted = inj_estop;
        hal.kick_hardware_watchdog = inj_kick;
        hal.wait_until = probe_wait;
        hal.read_reward = inj_reward;
        hal.read_phase = inj_phase;

        arc_loop_config lcfg;
        std::memset(&lcfg, 0, sizeof(lcfg));
        lcfg.period_us = 1000;
        lcfg.max_ticks = ctx.total;
        arc_run(e, s, sd, nullptr, 0, &hal, &lcfg);

        if (!arc_safety_estop_asserted(s) && !arc_safety_fault_state(s)) {
            printf("FAIL: estop left no trace\n"); failed++;
        } else { printf("PASS: estop observed by safety layer\n"); }
        if (!arc_safety_fault_state(s)) {
            printf("FAIL: fault not latched after estop release\n"); failed++;
        } else { printf("PASS: fault latched after estop release\n"); }

        const size_t n = ctx.history.size();
        const std::vector<int32_t>& tail = ctx.history[n - 1];
        if (tail[0] != 0 || tail[1] != 0) {
            printf("FAIL: not holding safe after release (%d,%d)\n",
                   tail[0], tail[1]); failed++;
        } else { printf("PASS: holds safe while latched\n"); }

        /* Clear + a short recovery run: normal writes must resume. */
        arc_safety_clear(s);
        if (arc_safety_fault_state(s)) {
            printf("FAIL: clear did not reset fault\n"); failed++;
        } else { printf("PASS: arc_safety_clear resets fault\n"); }

        ctx.history.clear();
        ctx.tick = 0;
        ctx.estop_from = 0xffffffffu;
        ctx.estop_to = 0;
        lcfg.max_ticks = 40;
        arc_run(e, s, sd, nullptr, 0, &hal, &lcfg);
        int motion = 0;
        for (size_t i = 0; i < ctx.history.size(); ++i)
            for (size_t g = 0; g < ctx.history[i].size(); ++g)
                if (ctx.history[i][g] != 0) motion = 1;
        if (!motion) {
            printf("FAIL: no motion after clear\n"); failed++;
        } else { printf("PASS: normal writes resume after clear\n"); }
    }

    /* --- Case 3: sensor dropout (stale frame) keeps running, no fault --- */
    {
        pltest::Harness hcase(64, 32, 4, [] {
            arc_plasticity_config c = pltest::default_cfg();
            c.plasticity_interval = 10;
            return c;
        }(), std::vector<int32_t>{65536, 0, 0}, 1);
        if (!setup(hcase, &e, &s, sarena, sd_arena, &sd, scfg,
                   safe_sp, safe_ramp, safe_buf)) {
            printf("FAIL: case3 setup\n"); return 1;
        }
        InjCtx ctx{};
        ctx.e = e;
        ctx.total = 300;
        ctx.estop_from = 0xffffffffu;
        ctx.estop_to = 0;
        ctx.drop_inputs = true;
        ctx.seed = 0x1234;

        arc_hal hal;
        std::memset(&hal, 0, sizeof(hal));
        hal.ctx = &ctx;
        hal.write_motors = inj_write;
        hal.now_us = inj_now;
        hal.kick_hardware_watchdog = inj_kick;
        hal.wait_until = inj_wait;
        hal.read_reward = inj_reward;
        hal.read_phase = inj_phase;

        arc_loop_config lcfg;
        std::memset(&lcfg, 0, sizeof(lcfg));
        lcfg.period_us = 1000;
        lcfg.max_ticks = ctx.total;
        arc_run(e, s, sd, nullptr, 0, &hal, &lcfg);

        if (arc_safety_fault_state(s)) {
            printf("FAIL: dropout caused a fault\n"); failed++;
        } else { printf("PASS: stale frame runs without fault\n"); }
        if (ctx.history.size() < ctx.total - 1) {
            printf("FAIL: dropout stopped the loop (%zu)\n",
                   ctx.history.size()); failed++;
        } else { printf("PASS: loop completes through dropout\n"); }
        int bounded = 1;
        int32_t worst = 0;
        size_t worst_tick = 0;
        int worst_g = 0;
        for (size_t i = 0; i < ctx.history.size(); ++i)
            for (size_t g = 0; g < ctx.history[i].size(); ++g) {
                if (ctx.history[i][g] > worst) {
                    worst = ctx.history[i][g];
                    worst_tick = i;
                    worst_g = (int)g;
                }
                if (-ctx.history[i][g] > worst) {
                    worst = -ctx.history[i][g];
                    worst_tick = i;
                    worst_g = (int)g;
                }
                if (ctx.history[i][g] > 65536 || ctx.history[i][g] < -65536)
                    bounded = 0;
            }
        if (!bounded) {
            printf("FAIL: dropout unbounded setpoint (worst %d at tick %zu dof %d)\n",
                   worst, worst_tick, worst_g);
            failed++;
        } else { printf("PASS: setpoints stay clamped through dropout\n"); }
    }

    /* --- Case 4: single late tick tolerated --- */
    {
        pltest::Harness hcase(64, 32, 4, [] {
            arc_plasticity_config c = pltest::default_cfg();
            c.plasticity_interval = 10;
            return c;
        }(), std::vector<int32_t>{65536, 0, 0}, 1);
        if (!setup(hcase, &e, &s, sarena, sd_arena, &sd, scfg,
                   safe_sp, safe_ramp, safe_buf)) {
            printf("FAIL: case4 setup\n"); return 1;
        }
        InjCtx ctx{};
        ctx.e = e;
        ctx.total = 300;
        ctx.estop_from = 0xffffffffu;
        ctx.estop_to = 0;
        ctx.late_tick_once = true;
        ctx.seed = 0x5678;

        arc_hal hal;
        std::memset(&hal, 0, sizeof(hal));
        hal.ctx = &ctx;
        hal.write_motors = inj_write;
        hal.now_us = inj_now;
        hal.kick_hardware_watchdog = inj_kick;
        hal.wait_until = inj_wait;
        hal.read_reward = inj_reward;
        hal.read_phase = inj_phase;

        arc_loop_config lcfg;
        std::memset(&lcfg, 0, sizeof(lcfg));
        lcfg.period_us = 1000;
        lcfg.max_ticks = ctx.total;
        arc_run(e, s, sd, nullptr, 0, &hal, &lcfg);

        if (arc_safety_fault_state(s)) {
            printf("FAIL: single late tick tripped watchdog\n"); failed++;
        } else { printf("PASS: single late tick tolerated (misses < 3)\n"); }
        if (arc_safety_watchdog_counter(s) >= 3) {
            printf("FAIL: watchdog counter not decayed (%d)\n",
                   arc_safety_watchdog_counter(s)); failed++;
        } else { printf("PASS: watchdog counter decayed after good tick\n"); }
    }

    printf("\nFault-injection tests: %s\n", failed ? "FAILED" : "PASSED");
    return failed ? 1 : 0;
}
