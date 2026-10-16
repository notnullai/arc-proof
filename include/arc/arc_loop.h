#pragma once

#include "arc/arc_types.h"
#include "arc/arc_hal.h"
#include "arc/arc_pid.h"

#ifdef __cplusplus
extern "C" {
#endif

/* pids/n_pids are used in RESIDUAL decoder mode; pass NULL/0 otherwise. */
void arc_run(arc_engine* e, arc_safety* s, arc_setpoints* sd,
             arc_pid* pids, int n_pids,
             const arc_hal* hal, const arc_loop_config* cfg);

#ifdef __cplusplus
}
#endif
