#pragma once

#include "arc_types.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Q16.16 PID (docs/phase 2.md section 3.1). All scalars Q16.16. */
typedef struct {
    int32_t kp_q16;
    int32_t ki_q16;
    int32_t kd_q16;
    int32_t integral_q16;
    int32_t integral_min_q16;
    int32_t integral_max_q16;
    int32_t prev_error_q16;
    int32_t out_min_q16;
    int32_t out_max_q16;
    int32_t integral_decay_q16;   /* per-tick leak toward zero when masked */
    uint8_t  masked;              /* mask override active */
    uint8_t  _pad[7];
} arc_pid;

void    arc_pid_reset(arc_pid* p);
int32_t arc_pid_step(arc_pid* p, int32_t target_q16, int32_t measured_q16);
void    arc_pid_set_gains(arc_pid* p, int32_t kp, int32_t ki, int32_t kd);
void    arc_pid_set_masked(arc_pid* p, bool masked);

/* ALPHA mode: interpolate kp/ki/kd between two bench-verified profiles.
   alpha_q16 = 0 -> lo, 65536 -> hi. The integral state stays untouched
   (gain change is not an integral reset). */
void    arc_pid_set_gains_interp(arc_pid* p, const arc_pid* lo, const arc_pid* hi,
                                 int32_t alpha_q16);

/* Target for a masked DoF: drive toward what the plant can actually reach. */
int32_t arc_pid_target_for_masked(int32_t current_measured_q16);

#ifdef __cplusplus
}
#endif
