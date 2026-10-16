#pragma once

#include "arc_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------------------------------------------------------------------
 * ARC-SERVO1 — reference servo-bus HAL profile (Dynamixel-class serial/CAN).
 *
 * The core (arc_servo_hal_create) is vendor-neutral: it packs setpoints into
 * 8-byte frames and pushes them through a transport vtable. The transport is
 * the vendor-specific part; two are shipped: a POSIX serial transport
 * (platforms/servo/transport_posix.cpp, Linux) and a deterministic mock for
 * hardware-in-the-loop testing (tests/mock/servo_mock.hpp).
 *
 * Frame (8 bytes, little-endian payload):
 *   [0] 0xA5          magic
 *   [1] servo id
 *   [2] command       0x01 SET_POSITION (payload = goal Q16.16)
 *                     0x02 GET_POSITION (response 0x82, payload = position)
 *   [3..6] i32 payload
 *   [7] checksum      low byte of the sum of bytes 0..6
 *
 * The setpoint payload is Q16.16 in the plant's units; converting to encoder
 * counts / duty / torque is the integrator's mapping (their plant, their
 * units). This profile deliberately does NOT close the position loop here —
 * the servo closes its own loop or the mapping is 1:1, both valid.
 * ------------------------------------------------------------------------- */

typedef struct arc_servo_transport {
    void* ctx;
    /* Returns 0 on success. open(device) is optional for mock transports. */
    int (*open)(void* ctx, const char* device);
    int (*write_frame)(void* ctx, const uint8_t* frame, int len);
    /* Returns the frame length, 0 on timeout, < 0 on error. */
    int (*read_frame)(void* ctx, uint8_t* frame, int max_len, uint32_t timeout_ms);
    uint64_t (*now_us)(void* ctx);
    void (*close)(void* ctx);
} arc_servo_transport;

typedef struct arc_servo_hal_config {
    const arc_servo_transport* transport;
    /* DoF index -> servo id on the bus (n_dofs entries, n_dofs <= 32). */
    const uint8_t* servo_ids;
    int32_t n_dofs;
    uint32_t read_timeout_ms;
    /* Optional: integrator planner provides per-DoF targets; without it the
       read_state hook fills target_q16 with 0 (planner-less operation). */
    void (*read_target)(void* ctx, int32_t* target_q16, int n_dofs);
    void* read_target_ctx;
} arc_servo_hal_config;

/* Fills *out (an arc_hal) wired to the configured transport. Arena-backed,
   allocation-free; 256 bytes is ample for n_dofs <= 32. Returns ARC_OK or
   ARC_ERR_ARG / ARC_ERR_ARENA. The resulting hal implements write_motors,
   read_state (measured positions; targets from read_target), and now_us.
   estop_asserted / read_reward / read_limit_mask / kick_hardware_watchdog
   are left for the integrator to wire (or wrap) — they are platform
   policy, not bus mechanics. */
arc_status arc_servo_hal_create(const arc_servo_hal_config* cfg,
                                void* arena, size_t arena_size,
                                arc_hal* out);

/* Frame helpers (also used by transports/tests). */
void arc_servo_frame_set_position(uint8_t* frame, uint8_t id, int32_t position_q16);
void arc_servo_frame_get_position(uint8_t* frame, uint8_t id);
/* Parses a response frame; returns ARC_OK and fills *position_q16 for 0x82. */
arc_status arc_servo_frame_parse_response(const uint8_t* frame, int len,
                                          int32_t* position_q16);

/* POSIX serial transport (defined only in the Linux build; declared here so
   integrators see the surface). Device path/baud are the integrator's call. */
arc_status arc_servo_transport_posix_create(const char* device, uint32_t baud,
                                            arc_servo_transport* out);
void arc_servo_transport_posix_destroy(arc_servo_transport* t);

#ifdef __cplusplus
}
#endif
