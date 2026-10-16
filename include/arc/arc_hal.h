#pragma once

#include "arc_types.h"
#include "arc_plasticity.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct arc_encoder arc_encoder;
typedef struct arc_setpoints arc_setpoints;

/* Arena-backed encoder (no allocation). */
size_t arc_encoder_required_bytes(int32_t input_bits);
arc_status arc_encoder_create(int32_t input_bits,
                              void* arena, size_t arena_size,
                              arc_encoder** out);
void arc_encoder_clear(arc_encoder* e);
void arc_encoder_thermometer(arc_encoder* e, int32_t v_q16, int32_t lo_q16, int32_t hi_q16, int32_t k);
void arc_encoder_tiles(arc_encoder* e, int32_t v_q16, int32_t period_q16, int32_t k, int32_t width);
void arc_encoder_bit(arc_encoder* e, int32_t on);
void arc_encoder_bits(arc_encoder* e, uint32_t mask, int32_t count);
void arc_encoder_skip(arc_encoder* e, int32_t count);
const uint32_t* arc_encoder_words(const arc_encoder* e);
int32_t arc_encoder_bits_used(const arc_encoder* e);

/* Arena-backed setpoint decoder (no allocation; n_motors <= 32 invariant).
   Callers size the arena with arc_setpoints_required_bytes. */
size_t arc_setpoints_required_bytes(const uint8_t* groups, int32_t n_motors);
arc_status arc_setpoints_create(const uint8_t* groups, int32_t n_motors,
                                uint32_t alpha_q16,
                                void* arena, size_t arena_size,
                                arc_setpoints** out);
const int32_t* arc_setpoints_tick(arc_setpoints* s, const uint32_t* emitted_words);
void arc_setpoints_reset(arc_setpoints* s);
int32_t arc_setpoints_n_dofs(const arc_setpoints* s);

/* Decoder mode (docs/phase 2.md 4.3). The setpoint smoothing is the same
   integrator; the mode decides how the loop interprets it. */
void arc_setpoints_set_mode(arc_setpoints* s, int mode);
int  arc_setpoints_mode(const arc_setpoints* s);

/* RESIDUAL: per-DoF corrective offset in [-1, 1] (Q16.16), same leaky
   smoothing; the safety layer clamps and rate-limits it. */
arc_status arc_setpoints_tick_residual(arc_setpoints* s,
                                       const uint32_t* emitted_words,
                                       int32_t* residuals_q16);

/* ALPHA: per-DoF gain-interpolation weight in [0, 1] (Q16.16), same leaky
   smoothing; the motor's position in its WTA group maps to [0, 1]. */
arc_status arc_setpoints_tick_alpha(arc_setpoints* s,
                                    const uint32_t* emitted_words,
                                    int32_t* alphas_q16);

#ifdef __cplusplus
}
#endif