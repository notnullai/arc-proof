#include "arc/arc_safety.h"
#include "arc/arc.h"
#include "arc/arc_types.h"
#include <cstdio>
#include <cstdint>
#include <vector>
#include <cstring>

int main() {
    int passed = 0, failed = 0;

    arc_safety_config cfg = {};
    cfg.max_setpoint_q16 = 65536;
    cfg.max_rate_per_tick_q16 = 10000;
    cfg.tick_budget_us = 1000;
    cfg.watchdog_misses = 3;

    int32_t safe_setpoints[2] = {0, 0};
    int32_t safe_ramp_ticks[2] = {10, 10};
    int32_t safe_setpoints_buf[2] = {0, 0};
    cfg.safe_setpoint_q16 = safe_setpoints;
    cfg.safe_ramp_ticks = safe_ramp_ticks;
    cfg.safe_setpoints_buf = safe_setpoints_buf;

    size_t arena_size = 4096;  // Sufficient for 2 DoFs
    std::vector<uint8_t> arena(arena_size);
    arc_safety* s = nullptr;
    arc_status st = arc_safety_create(&cfg, arena.data(), arena_size, &s);
    if (st != ARC_OK) {
        printf("FAIL: arc_safety_create returned %d\n", st);
        failed++;
    } else {
        printf("PASS: arc_safety_create\n");
        passed++;
    }

    int32_t setpoints[2] = {50000, -50000};
    st = arc_safety_post_tick(s, setpoints, 2);
    if (setpoints[0] > cfg.max_setpoint_q16 || setpoints[0] < -cfg.max_setpoint_q16 ||
        setpoints[1] > cfg.max_setpoint_q16 || setpoints[1] < -cfg.max_setpoint_q16) {
        printf("FAIL: clamp not applied\n");
        failed++;
    } else {
        printf("PASS: clamp\n");
        passed++;
    }

    int32_t prev_setpoints[2] = {0, 0};
    int32_t new_setpoints[2] = {50000, -50000};
    st = arc_safety_post_tick(s, prev_setpoints, 2);
    st = arc_safety_post_tick(s, new_setpoints, 2);
    if (new_setpoints[0] - prev_setpoints[0] > cfg.max_rate_per_tick_q16) {
        printf("FAIL: rate limit not applied\n");
        failed++;
    } else {
        printf("PASS: rate limit\n");
        passed++;
    }

    arc_safety_estop(s, true);
    if (!arc_safety_fault_state(s)) {
        printf("FAIL: estop did not trigger fault state\n");
        failed++;
    } else {
        printf("PASS: estop triggers fault state\n");
        passed++;
    }

    /* The ramp must start from the last commanded setpoint, not zero. */
    {
        int32_t ramp_sp[2];
        arc_safety_estop(s, false);
        arc_safety_clear(s);

        int32_t cmd[2] = {50000, -50000};
        arc_safety_post_tick(s, cmd, 2);      /* track the commanded setpoint */
        arc_safety_estop(s, true);            /* fault: ramp from 50000 -> 0 */

        arc_safety_fault_tick(s, ramp_sp, 2);
        const bool from_current =
            ramp_sp[0] < 50000 && ramp_sp[0] > 0 &&
            ramp_sp[1] > -50000 && ramp_sp[1] < 0;
        if (!from_current) {
            printf("FAIL: ramp did not start from current (%d, %d)\n",
                   ramp_sp[0], ramp_sp[1]);
            failed++;
        } else {
            printf("PASS: ramp starts from current setpoint (%d)\n", ramp_sp[0]);
            passed++;
        }

        for (int i = 0; i < 9; ++i) arc_safety_fault_tick(s, ramp_sp, 2);
        if (ramp_sp[0] != 0 || ramp_sp[1] != 0) {
            printf("FAIL: ramp did not reach safe setpoint (%d, %d)\n",
                   ramp_sp[0], ramp_sp[1]);
            failed++;
        } else {
            printf("PASS: ramp reaches safe setpoint after ramp_ticks\n");
            passed++;
        }
    }

    arc_safety_clear(s);
    if (arc_safety_fault_state(s) || arc_safety_estop_asserted(s)) {
        printf("FAIL: clear did not reset fault state\n");
        failed++;
    } else {
        printf("PASS: clear resets fault state\n");
        passed++;
    }

    arc_safety_heartbeat(s, 1000);
    arc_safety_heartbeat(s, 2500);
    if (arc_safety_watchdog_counter(s) == 0) {
        printf("FAIL: watchdog not triggered on overrun\n");
        failed++;
    } else {
        printf("PASS: watchdog triggered on overrun\n");
        passed++;
    }

    arc_engine* e = nullptr;
    st = arc_safety_pre_tick(s, e);
    if (st == ARC_ERR_SAFETY) {
        printf("FAIL: pre_tick failed when not in fault state\n");
        failed++;
    } else {
        printf("PASS: pre_tick OK when safe\n");
        passed++;
    }

    arc_safety_estop(s, true);
    st = arc_safety_pre_tick(s, e);
    if (st != ARC_ERR_SAFETY) {
        printf("FAIL: pre_tick should fail in fault state\n");
        failed++;
    } else {
        printf("PASS: pre_tick fails in fault state\n");
        passed++;
    }

    printf("\nSafety tests: %d passed, %d failed\n", passed, failed);
    return failed == 0 ? 0 : 1;
}