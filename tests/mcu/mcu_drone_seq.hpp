#pragma once
/* Shared MCU/host DRONE full-loop sequence (Plan Step 5a): the complete
   arc_run — forward tick, plasticity, 4-DoF setpoint decode, safety hover
   ramp, and the estop fault path — on a fixed ARC002 brain, hashed with
   FNV-1a. Compiled by the Cortex-M smoke test, the host reference test, and
   the on-target Teensy bench binary; the committed expected hash proves the
   whole reflex loop is byte-exact across x86_64 / aarch64 / Cortex-M3-M7.
   No heap: all buffers static. */
#include "arc/arc.h"
#include "arc/arc_brain.h"
#include "arc/arc_plasticity.h"
#include "arc/arc_types.h"
#include "arc/arc_hal.h"
#include "arc/arc_loop.h"
#include "arc/arc_safety.h"
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#ifdef ARC_MCU_DRONE_LOOP_DEBUG
#include <stdio.h>
#define DLOOP_DBG(...) printf(__VA_ARGS__)
#else
#define DLOOP_DBG(...) ((void)0)
#endif

namespace arc_mcu_drone {

/* Geometry override for the timing twins of the shipped x500 priors (input
   480 / 512 / 704 bits, 96 hidden): the tick cost is word-level popcounts over
   hidden x input words, so a fixed brain of the same geometry times the same
   work. Default 64/64 is the bit-exact golden twin. */
#ifndef ARC_MCU_INPUT_BITS
#define ARC_MCU_INPUT_BITS 64
#endif
#ifndef ARC_MCU_HIDDEN
#define ARC_MCU_HIDDEN 64
#endif
#ifndef ARC_MCU_PLASTICITY_INTERVAL
#define ARC_MCU_PLASTICITY_INTERVAL 10   /* ticks per plasticity pass (the shipped priors use 10) */
#endif

enum {
    kInputBits = ARC_MCU_INPUT_BITS,
    kHidden = ARC_MCU_HIDDEN,      /* neurons 0..3 are the four ESCs */
    kMotor = 4,
    kInputWords = kInputBits / 32,
    kHiddenWords = kHidden / 32,
    kNPhases = 2,
    kWpCount = kNPhases * ARC_N_REWARD_COMPONENTS,
    kAff = kHidden * kInputWords,
    kRec = kHidden * kHiddenWords,
    kBrainBytes = ARC_HEADER_SIZE_V2
                + kWpCount * 4
                + kAff * 2 * 4
                + kRec * 2 * 4
                + kHidden * 4
                + kMotor,
    kEngineArena = (ARC_MCU_INPUT_BITS == 64 && ARC_MCU_HIDDEN == 64) ? 65536 : 262144,
    kSafetyArena = 1024,
    kSetpointsArena = 256,
    kLoopTicks = 2000,
    kEstopTick = 1500,
};

/* Filled in from the host twin; also checked by the qemu script and the
   on-target bench binary. */
#define ARC_MCU_DRONE_LOOP_EXPECTED_HASH 0xac29f2cfa8126d1bULL

static void put_u32(uint8_t* b, size_t off, uint32_t v) {
    b[off] = (uint8_t)v; b[off + 1] = (uint8_t)(v >> 8);
    b[off + 2] = (uint8_t)(v >> 16); b[off + 3] = (uint8_t)(v >> 24);
}
static void put_i32(uint8_t* b, size_t off, int32_t v) { put_u32(b, off, (uint32_t)v); }
static void put_u64(uint8_t* b, size_t off, uint64_t v) {
    for (int i = 0; i < 8; ++i) b[off + i] = (uint8_t)(v >> (8 * i));
}

/* ARC002 64/64/4 brain, 4 independent WTA groups, DIRECT decoder, plastic.
   Same construction style as arc_mcu::build_brain (fixed excitations,
   deterministic — no training involved). */
static void build_brain(uint8_t* brain) {
    memset(brain, 0, kBrainBytes);
    memcpy(brain, "ARC002\0\0", 8);
    put_u32(brain, 8, 2);
    put_u32(brain, 16, kInputBits);
    put_u32(brain, 20, kHidden);
    put_u32(brain, 24, kMotor);
    put_u32(brain, 28, kMotor);      /* 4 WTA groups */
    put_u32(brain, 32, kInputWords);
    put_u32(brain, 36, kHiddenWords);
    put_i32(brain, 40, 65);
    put_u32(brain, 44, 64881);       /* leak */
    brain[48] = 0;                   /* decoder DIRECT */
    brain[49] = kNPhases;
    brain[50] = (uint8_t)(kWpCount & 0xFF);
    brain[51] = (uint8_t)((kWpCount >> 8) & 0xFF);
    put_u64(brain, 52, 0xD20FC4A1B35E7917ULL);
    put_u32(brain, 60, 60000);
    put_u32(brain, 64, 3);
    put_u32(brain, 68, 100);
    put_u32(brain, 72, 32768);
    put_u32(brain, 76, ARC_MCU_PLASTICITY_INTERVAL);
    put_u32(brain, 80, 6553);
    put_u32(brain, 84, 50);
    put_u32(brain, 88, 5);
    put_u32(brain, 92, 100);
    put_u32(brain, 96, 0);

    const size_t wp_off = ARC_HEADER_SIZE_V2;
    put_i32(brain, wp_off + 0, 65536);  put_i32(brain, wp_off + 4, 16384);  put_i32(brain, wp_off + 8, 0);
    put_i32(brain, wp_off + 12, 0);     put_i32(brain, wp_off + 16, 16384); put_i32(brain, wp_off + 20, 65536);

    static uint32_t aff_exc[kAff];
    static uint32_t rec_exc[kRec];
    memset(aff_exc, 0, sizeof(aff_exc));
    memset(rec_exc, 0, sizeof(rec_exc));
    for (int n = 0; n < kHidden; ++n)
        for (int k = 0; k < 4; ++k) {
            const int j = (n + k * 16) % kInputBits;
            aff_exc[n * kInputWords + (j >> 5)] |= 1u << (j & 31);
        }
    for (int n = kMotor; n < kHidden; ++n) {
        const int motor = n % kMotor;
        rec_exc[motor * kHiddenWords + (n >> 5)] |= 1u << (n & 31);
    }

    const size_t aff_off = wp_off + kWpCount * 4;
    for (size_t i = 0; i < kAff; ++i) {
        put_u32(brain, aff_off + i * 4, aff_exc[i]);
        put_u32(brain, aff_off + (kAff + i) * 4, 0);
    }
    const size_t rec_off = aff_off + kAff * 2 * 4;
    for (size_t i = 0; i < kRec; ++i) {
        put_u32(brain, rec_off + i * 4, rec_exc[i]);
        put_u32(brain, rec_off + (kRec + i) * 4, 0);
    }
    const size_t thr_off = rec_off + kRec * 2 * 4;
    for (int i = 0; i < kHidden; ++i)
        put_i32(brain, thr_off + (size_t)i * 4, i < kMotor ? 8 : 2);
}

struct DLoopCtx {
    arc_engine* e;
    uint64_t h;
    uint32_t tick;
    uint32_t now_us;
};

static inline void fold_u32(uint64_t* h, uint32_t v) {
    for (int b = 0; b < 4; ++b) {
        *h ^= (uint8_t)(v >> (8 * b));
        *h *= 1099511628211ULL;
    }
}

static inline void dloop_write_motors(void* ctx, const int32_t* sp, int n) {
    DLoopCtx* c = (DLoopCtx*)ctx;
    for (int i = 0; i < n; ++i) fold_u32(&c->h, (uint32_t)sp[i]);
}

static inline void dloop_reward(void* ctx, int32_t* r, int n) {
    DLoopCtx* c = (DLoopCtx*)ctx;
    (void)n;
#ifdef ARC_MCU_DENSE_INPUT
    /* timing-only stress (changes the hash): pseudo-random input words, so
       hidden neurons fire, traces are nonzero and synapses flip. The default
       one-bit-per-tick input never fires a hidden neuron, so its plasticity
       pass only decays zero traces. */
    static uint32_t lcg = 0x2545F491u;
    for (int w = 0; w < kInputWords; ++w) {
        lcg = lcg * 1664525u + 1013904223u;
        c->e->input[w] = lcg;
    }
#else
    const int bit = (int)(c->tick % kInputBits);
    for (int w = 0; w < kInputWords; ++w) c->e->input[w] = 0;
    c->e->input[bit >> 5] = 1u << (bit & 31);
#endif
    r[0] = 65536; r[1] = 0; r[2] = 0;
}

static inline uint8_t dloop_phase(void* ctx) { (void)ctx; return 0; }
static inline bool dloop_estop(void* ctx) { return ((DLoopCtx*)ctx)->tick >= kEstopTick; }
#ifndef ARC_MCU_TICK_HOOK
#define ARC_MCU_TICK_HOOK() ((void)0)   /* timing builds sample SysTick here; no effect on the hash */
#endif
#ifndef ARC_MCU_TICK_RESAMPLE
#define ARC_MCU_TICK_RESAMPLE() ((void)0)   /* restart the tick clock (excludes the arming below) */
#endif

#ifdef ARC_MCU_WORST_PLASTICITY
/* Worst-case plasticity twin (gap register G8; timing only, own hash). Before
   every tick, outside the timed window, the rows the coming plasticity call
   visits are armed so that every branch of arc_plasticity_step goes the
   expensive way: every post neuron spikes (the trace pass loads pre_f), every
   trace crosses the threshold on the same side (|E| near the clamp; SIGN -1:
   E < -th, both compares run), every synapse can move (state 0) and is
   drifted before and after the move (prior = -SIGN), so it is never blocked
   and the Hamming distance stays put; with AT_BOUND the distance sits at the
   ceiling, so every word takes the ceiling-aware loop and every iteration
   runs the ceiling mask; every flip cooldown has expired. So every one of
   the call's (rows x (hidden - motor)) synapses flips. The next tick's wait
   checks that it did: flips, final state of every armed synapse, the
   Hamming distance, the post spikes. g_worst_bad counts armed ticks where
   any check failed; the host twin requires 0. The arming state is outside
   what learning reaches (more drifted synapses than the distance counts),
   so the timed figure bounds the reachable worst case from above. */
#ifndef ARC_MCU_WORST_SIGN
#define ARC_MCU_WORST_SIGN -1
#endif
#ifndef ARC_MCU_WORST_AT_BOUND
#define ARC_MCU_WORST_AT_BOUND 1          /* 0: distance 0, the fast (unblocked) word loop */
#endif
extern "C" {
volatile uint32_t g_worst_armed = 0;      /* armed ticks checked */
volatile uint32_t g_worst_bad = 0;        /* armed ticks that missed the worst case */
volatile uint32_t g_worst_flips_max = 0;  /* most flips in one plasticity call */
volatile uint32_t g_worst_flips_total = 0;
}
struct WorstArm { int lo, hi; uint32_t fc0, ham, expect; };
static WorstArm g_warm = {0, 0, 0, 0, 0};

static inline int8_t worst_sign(int pre) {
#if ARC_MCU_WORST_SIGN == 0
    return (pre & 1) ? (int8_t)1 : (int8_t)-1;   /* mixed: both mask words change */
#else
    (void)pre;
    return (int8_t)ARC_MCU_WORST_SIGN;
#endif
}

static inline void worst_check(arc_engine* e) {
    arc_plasticity* p = e->plasticity;
    if (g_warm.expect == 0) return;
    const int H = e->geom.hidden_neurons, M = e->geom.motor_neurons, nw = e->geom.hidden_words;
    const uint32_t flips = p->flip_count - g_warm.fc0;
    bool ok = flips == g_warm.expect && *p->hamming == g_warm.ham;
    for (int post = g_warm.lo; post < g_warm.hi; ++post) {
        if (e->spike[post] != 1) ok = false;
        for (int pre = M; pre < H; ++pre) {
            const uint32_t b = 1u << (pre & 31);
            const size_t w = (size_t)post * (size_t)nw + (size_t)(pre >> 5);
            const int st = (e->rec_exc[w] & b) ? 1 : (e->rec_inh[w] & b) ? -1 : 0;
            if (st != worst_sign(pre)) ok = false;   /* moved 0 -> SIGN */
            if (p->E[(size_t)post * (size_t)H + (size_t)pre] != 0) ok = false;
        }
    }
    ++g_worst_armed;
    if (!ok) ++g_worst_bad;
    if (flips > g_worst_flips_max) g_worst_flips_max = flips;
    g_worst_flips_total += flips;
    g_warm.expect = 0;
}

static inline void worst_arm(arc_engine* e, uint32_t tick) {
    arc_plasticity* p = e->plasticity;
    const int H = e->geom.hidden_neurons, M = e->geom.motor_neurons, nw = e->geom.hidden_words;
    g_warm.expect = 0;
    if (tick + 1 >= (uint32_t)kEstopTick) return;   /* the estop fault gates learning */
    /* Rows of the coming call, as arc_plasticity_step picks them. */
    const uint32_t interval = p->cfg.plasticity_interval;
    uint32_t slices = p->slices;
    if (slices > interval) slices = interval;
    int lo, hi;
    if (slices <= 1) {
        if (p->tick_accum + 1 < interval) return;
        lo = M; hi = H;
    } else {
        const uint32_t k = p->tick_accum;
        if (k >= slices) return;
        const int per = (H - M + (int)slices - 1) / (int)slices;
        lo = M + (int)k * per; hi = lo + per;
        if (hi > H) hi = H;
        if (lo >= hi) return;
    }
    uint32_t* rex = (uint32_t*)e->rec_exc;
    uint32_t* rin = (uint32_t*)e->rec_inh;
    for (int post = lo; post < hi; ++post) {
        for (int pre = M; pre < H; ++pre) {
            const size_t idx = (size_t)post * (size_t)H + (size_t)pre;
            const int8_t sg = worst_sign(pre);
            p->E[idx] = sg < 0 ? -(int32_t)0x3FFFFFFF : (int32_t)0x3FFFFFFF;
            p->last_flip[idx] = tick - 100u;
            const uint32_t b = 1u << (pre & 31);
            const size_t w = (size_t)post * (size_t)nw + (size_t)(pre >> 5);
            rex[w] &= ~b; rin[w] &= ~b;                       /* state 0 */
#ifdef ARC_MCU_WORST_WALK
            /* Moves back onto the prior (even pre: prior = the target, Hamming
               -1) alternate with moves off it (odd pre: prior 0, +1), so at the
               ceiling every move off the prior is checked and allowed. */
            p->prior_exc[w] &= ~b; p->prior_inh[w] &= ~b;
            if ((pre & 1) == 0) { if (sg > 0) p->prior_exc[w] |= b; else p->prior_inh[w] |= b; }
#else
            if (sg < 0) { p->prior_exc[w] |= b; p->prior_inh[w] &= ~b; }   /* prior +1 */
            else        { p->prior_exc[w] &= ~b; p->prior_inh[w] |= b; }   /* prior -1 */
#endif
        }
    }
    const uint32_t pn = (uint32_t)(H - M);
    const uint32_t bound = (uint32_t)(((uint64_t)p->cfg.max_hamming_fraction_q16 * (pn * pn)) >> 16);
    *p->hamming = ARC_MCU_WORST_AT_BOUND ? bound : 0u;
    *p->mod_state = 65536;                                   /* m_sign = +1 */
    for (int n = 0; n < H; ++n) e->V[n] = 1 << 20;           /* every neuron spikes +1 */
    g_warm.lo = lo; g_warm.hi = hi;
    g_warm.fc0 = p->flip_count;
    g_warm.ham = *p->hamming;
    g_warm.expect = (uint32_t)(hi - lo) * pn;
}
#endif

static inline void dloop_wait(void* ctx, uint64_t /*deadline*/) {
    DLoopCtx* c = (DLoopCtx*)ctx;
    ARC_MCU_TICK_HOOK();
#ifdef ARC_MCU_WORST_PLASTICITY
    worst_check(c->e);
    worst_arm(c->e, c->tick);
    ARC_MCU_TICK_RESAMPLE();
#endif
#if defined(ARC_MCU_DENSE_INPUT) && defined(ARC_MCU_NO_PLASTICITY)
    /* inference stress: no reward callback, so the dense input is fed here */
    static uint32_t lcg_inf = 0x2545F491u;
    for (int w = 0; w < kInputWords; ++w) {
        lcg_inf = lcg_inf * 1664525u + 1013904223u;
        c->e->input[w] = lcg_inf;
    }
#endif
    ++c->tick;
    c->now_us += 1000;
}
static inline uint64_t dloop_now(void* ctx) { return (uint64_t)((DLoopCtx*)ctx)->now_us; }
static inline void dloop_kick(void* ctx) { (void)ctx; }

/* Full drone loop; returns the FNV-1a of the whole 4-DoF command stream
   plus the final hamming and revert counts. 0 on setup failure. */
static inline uint64_t run_loop_hash(uint8_t* brain, uint8_t* eng_arena,
                              uint8_t* safe_arena, uint8_t* sd_arena) {
    build_brain(brain);
    arc_engine* e = NULL;
    if (arc_load(brain, kBrainBytes, eng_arena, kEngineArena, &e) != ARC_OK) {
        DLOOP_DBG("dloop: arc_load failed\n");
        return 0;
    }
#ifdef ARC_MCU_PLASTICITY_SLICES
    /* timing twin variant: amortised plasticity pass (changes the hash; the
       bit-exact twin is the build without this define) */
    arc_plasticity_set_slices(e->plasticity, ARC_MCU_PLASTICITY_SLICES);
#endif

    /* Drone safety: fault ramps to HOVER (32768), never zero (safety
       contract drone example). Arrays terminated with a {0} entry. */
    int32_t safe_sp[5] = {32768, 32768, 32768, 32768, 0};
    int32_t safe_ramp[5] = {20, 20, 20, 20, 0};
    int32_t safe_buf[5] = {0, 0, 0, 0, 0};
    arc_safety_config scfg;
    memset(&scfg, 0, sizeof(scfg));
    scfg.max_setpoint_q16 = 65536;
    scfg.max_rate_per_tick_q16 = 10000;
    scfg.safe_setpoint_q16 = safe_sp;
    scfg.safe_ramp_ticks = safe_ramp;
    scfg.tick_budget_us = 1000;
    scfg.watchdog_misses = 3;
    scfg.safe_setpoints_buf = safe_buf;
    arc_safety* s = NULL;
    if (arc_safety_create(&scfg, safe_arena, kSafetyArena, &s) != ARC_OK) {
        DLOOP_DBG("dloop: safety create failed\n");
        return 0;
    }

    const uint8_t groups[4] = {0, 1, 2, 3};
    arc_setpoints* sd = NULL;
    if (arc_setpoints_create(groups, 4, 16384, sd_arena, kSetpointsArena, &sd) != ARC_OK) {
        DLOOP_DBG("dloop: setpoints create failed\n");
        return 0;
    }

    DLoopCtx ctx;
    ctx.e = e;
    ctx.h = 1469598103934665603ULL;
    ctx.tick = 0;
    ctx.now_us = 0;

    arc_hal hal;
    memset(&hal, 0, sizeof(hal));
    hal.ctx = &ctx;
    hal.write_motors = dloop_write_motors;
#ifdef ARC_MCU_NO_PLASTICITY
    hal.read_reward = NULL;               /* inference only: how the flight default flies (plasticity off) */
    (void)&dloop_reward;
#else
    hal.read_reward = dloop_reward;
#endif
    hal.read_phase = dloop_phase;
    hal.estop_asserted = dloop_estop;
    hal.wait_until = dloop_wait;
    hal.now_us = dloop_now;
    hal.kick_hardware_watchdog = dloop_kick;

    arc_loop_config lcfg;
    memset(&lcfg, 0, sizeof(lcfg));
    lcfg.period_us = 1000;
    lcfg.max_ticks = kLoopTicks;

    arc_run(e, s, sd, NULL, 0, &hal, &lcfg);

    fold_u32(&ctx.h, *e->plasticity->hamming);
    fold_u32(&ctx.h, e->plasticity->revert_count);
    return ctx.h;
}

}  // namespace arc_mcu_drone
