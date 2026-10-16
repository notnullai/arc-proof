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

enum {
    kInputBits = 64,
    kHidden = 64,      /* neurons 0..3 are the four ESCs */
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
    kEngineArena = 65536,
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
    put_u32(brain, 76, 10);
    put_u32(brain, 80, 6553);
    put_u32(brain, 84, 50);
    put_u32(brain, 88, 5);
    put_u32(brain, 92, 100);
    put_u32(brain, 96, 0);

    const size_t wp_off = ARC_HEADER_SIZE_V2;
    put_i32(brain, wp_off + 0, 65536);  put_i32(brain, wp_off + 4, 16384);  put_i32(brain, wp_off + 8, 0);
    put_i32(brain, wp_off + 12, 0);     put_i32(brain, wp_off + 16, 16384); put_i32(brain, wp_off + 20, 65536);

    uint32_t aff_exc[kAff];
    uint32_t rec_exc[kRec];
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
    const int bit = (int)(c->tick % kInputBits);
    for (int w = 0; w < kInputWords; ++w) c->e->input[w] = 0;
    c->e->input[bit >> 5] = 1u << (bit & 31);
    r[0] = 65536; r[1] = 0; r[2] = 0;
}

static inline uint8_t dloop_phase(void* ctx) { (void)ctx; return 0; }
static inline bool dloop_estop(void* ctx) { return ((DLoopCtx*)ctx)->tick >= kEstopTick; }
static inline void dloop_wait(void* ctx, uint64_t /*deadline*/) {
    DLoopCtx* c = (DLoopCtx*)ctx;
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
    hal.read_reward = dloop_reward;
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
