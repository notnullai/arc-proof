#pragma once

#include "arc_types.h"

#ifdef __cplusplus
extern "C" {
#endif

arc_status arc_safety_create(const arc_safety_config* cfg,
                              void* arena, size_t arena_size,
                              arc_safety** out);

arc_status arc_safety_pre_tick(arc_safety* s, const arc_engine* e);
arc_status arc_safety_post_tick(arc_safety* s, int32_t* setpoints_q16, int n_dofs);
arc_status arc_safety_heartbeat(arc_safety* s, uint64_t now_us);
arc_status arc_safety_kick(arc_safety* s);
void arc_safety_estop(arc_safety* s, bool asserted);
void arc_safety_clear(arc_safety* s);
bool arc_safety_ok(const arc_safety* s);
bool arc_safety_fault_state(const arc_safety* s);
bool arc_safety_estop_asserted(const arc_safety* s);
int arc_safety_watchdog_counter(const arc_safety* s);

/* Advance the software ramp one tick and write the resulting per-DoF
   setpoints into setpoints_q16. Used by the loop on the fault path (the
   ramp interpolates from the last commanded setpoint toward the safe
   setpoint; a fault_tick after completion writes the safe setpoint). */
arc_status arc_safety_fault_tick(arc_safety* s, int32_t* setpoints_q16, int n_dofs);

/* Live-DoF fault policy (cfg.live_dof_mask != 0): setpoints_q16 holds the
   live decoded output on entry; on return every DoF whose mask bit is
   clear carries the ramp (as arc_safety_fault_tick) and every DoF whose bit
   is set carries its live value clamped to max_setpoint and rate-limited
   against the last written value. */
arc_status arc_safety_fault_tick_live(arc_safety* s, int32_t* setpoints_q16, int n_dofs);
uint32_t   arc_safety_live_mask(const arc_safety* s);
/* Change the live-DoF mask at runtime: the HAL clears it while its sensor
   data is stale (a live torque loop on a frozen frame is worse than a
   zero-torque hold) and restores it when data is fresh. */
void       arc_safety_set_live_mask(arc_safety* s, uint32_t mask);

/* RESIDUAL mode (2b): fault -> zero; else clamp to residual_clamp_q16 and
   rate-limit to residual_rate_q16 per tick. */
arc_status arc_safety_post_tick_residual(arc_safety* s, int32_t* residuals_q16,
                                         int n_dofs);

/* Mask overrides (2b): limits is a per-DoF bitmask, 1 = limit hit. */
void arc_safety_set_overrides(arc_safety* s, uint32_t limits);
bool arc_safety_masked(const arc_safety* s, int g);
int32_t arc_safety_safe_setpoint(const arc_safety* s, int g);
int32_t arc_safety_max_setpoint(const arc_safety* s);

#ifdef __cplusplus
}
#endif