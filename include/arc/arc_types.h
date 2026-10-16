#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef int32_t arc_status;

enum {
    ARC_OK             = 0,
    ARC_ERR_ARG        = -1,  // null pointer, index out of range
    ARC_ERR_FORMAT     = -2,  // brain file format violation
    ARC_ERR_GEOMETRY   = -3,  // shape violation
    ARC_ERR_ARENA      = -4,  // arena too small or unaligned
    ARC_ERR_SAFETY     = -5,  // safety layer in fault state
};

typedef struct {
    int32_t input_bits;
    int32_t hidden_neurons;
    int32_t motor_neurons;
    int32_t input_words;
    int32_t hidden_words;
    int32_t n_groups;
    int32_t fire_threshold;
    uint32_t leak_q16;
} arc_geometry;

typedef struct {
    arc_geometry geom;

    const uint32_t* aff_exc;
    const uint32_t* aff_inh;
    const uint32_t* rec_exc;
    const uint32_t* rec_inh;
    const int32_t*  threshold;
    const uint8_t*  groups;

    int32_t*  V;
    int8_t*   spike;
    int8_t*   next;
    uint32_t* s_exc;
    uint32_t* s_inh;
    uint32_t* emitted;
    uint32_t* input;

    struct arc_plasticity* plasticity;   /* NULL = frozen (ARC001) */
} arc_engine;

typedef struct {
    int32_t* safe_setpoint_q16;
    int32_t* safe_ramp_ticks;
    int32_t* safe_setpoints_buf;
    int32_t  max_setpoint_q16;
    int32_t  max_rate_per_tick_q16;
    uint64_t tick_budget_us;
    int      watchdog_misses;
    int32_t  residual_clamp_q16;   /* RESIDUAL mode (2b) */
    int32_t  residual_rate_q16;    /* RESIDUAL mode (2b) */
    int32_t  _pad;                 /* explicit pad */
} arc_safety_config;

typedef struct arc_safety arc_safety;

typedef struct {
    void     (*write_motors)(void* ctx, const int32_t* setpoints_q16, int n_dofs);
    uint64_t (*now_us)(void* ctx);
    bool     (*estop_asserted)(void* ctx);
    void     (*kick_hardware_watchdog)(void* ctx);
    void     (*wait_until)(void* ctx, uint64_t deadline_us);
    void     (*log)(void* ctx, int level, const char* msg, size_t len);
    /* Optional plasticity hooks. Absent read_reward -> plasticity disabled
       (true frozen, bit-identical to ARC001 behavior). */
    void     (*read_reward)(void* ctx, int32_t* reward, int n_components);
    uint8_t  (*read_phase)(void* ctx);
    /* Optional: host-streamed snapshot sink (Tier A storage). */
    void     (*store_snapshot)(void* ctx, const uint8_t* buf, size_t len);
    /* Optional (2b): per-DoF limit mask, bit set = limit hit (masked). */
    uint32_t (*read_limit_mask)(void* ctx);
    /* Optional (2b, RESIDUAL): plant state for the PID loop. */
    void     (*read_state)(void* ctx, int32_t* measured_q16, int32_t* target_q16,
                           int n_dofs);
    void* ctx;
} arc_hal;

typedef struct {
    uint64_t period_us;
    uint64_t start_us;
    uint64_t max_ticks;           /* 0 = run forever */
    uint8_t* snapshot_buf;        /* optional; used for fault-time snapshot */
    size_t   snapshot_buf_size;
    /* ALPHA mode: two bench-verified gain profiles (per-DoF arc_pid arrays).
       NULL = ALPHA falls back to the DIRECT decode path. */
    const void* gain_profile_lo;  /* const arc_pid* */
    const void* gain_profile_hi;  /* const arc_pid* */
} arc_loop_config;

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
[[maybe_unused]] static inline int arc_cpu_popcount(uint32_t v) {
#if defined(_MSC_VER)
    return (int)__popcnt(v);
#else
    return __builtin_popcount(v);
#endif
}

[[maybe_unused]] static inline int32_t arc_leak_apply(int32_t v, uint32_t leak_q16) {
    const int64_t prod = (int64_t)v * (int64_t)leak_q16;
    const int64_t q = (prod + ((prod >> 63) & 0xFFFF)) >> 16;
    return (int32_t)q;
}
#endif