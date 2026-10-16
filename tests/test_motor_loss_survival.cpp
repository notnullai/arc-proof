/* Motor-loss survival test (split of the former test_drone_sim phase 1):
   10000-tick episode via arc_run + arc_safety on the drone frame; motor 2
   fails at tick 9000; the plant must survive every tick (no fault) and
   hold the station bands under periodic wind. Safety runs bench-real
   settings (max_rate 10000, budget 1000 us, watchdog_misses 3). The tilt
   fault response lives in test_tilt_fault_response. */
#include "train/sim/drone_env.hpp"
#include "platforms/sim/hal_drone_sim.h"
#include "arc/arc.h"
#include "arc/arc_brain.h"
#include "arc/arc_hal.h"
#include "arc/arc_loop.h"
#include "arc/arc_plasticity.h"
#include "arc/arc_safety.h"
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <vector>

using namespace arc::sim;

struct SurvEp {
    DronePlant* plant = nullptr;
    DroneConfig* dcfg = nullptr;
    arc_engine* e = nullptr;
    uint32_t tick = 0;
    uint64_t hash = 1469598103934665603ULL;
    uint32_t survival = 0;
    int32_t max_alt_err = 0;
    int32_t max_xy_err = 0;
};
static SurvEp g_ep;
static void (*g_real_write)(void*, const int32_t*, int);

static void s_wait(void*, uint64_t) {
    SurvEp& ep = g_ep;
    if (ep.plant && ep.dcfg) {
        if (ep.tick == ep.dcfg->motor_failure_tick) {
            ep.plant->fail_motor(1);
            printf("  Motor 2 FAILED at tick %u\n", ep.tick);
        }
        ep.plant->phase = (uint8_t)((ep.tick / 500) & 1u);
        if (ep.tick > 0 && ep.tick % 300 == 0 &&
            ep.tick != ep.dcfg->motor_failure_tick) {
            if (ep.tick % 600 == 0)
                ep.plant->roll_rate_q16 += 8192;
            else
                ep.plant->pitch_rate_q16 += 8192;
        }
    }
    if (ep.e && ep.plant && ep.dcfg) {
        drone_encode(*ep.plant, ep.e->input, ep.dcfg->input_bits);
    }
}

static void s_write(void* ctx, const int32_t* sp, int n) {
    SurvEp& ep = g_ep;
    (void)sp;
    (void)n;
    if (ep.e) {
        uint32_t emitted = 0;
        arc_get_motor(ep.e, &emitted);
        ep.hash = drone_fnv1a(ep.hash, emitted);
    }
    if (ep.plant && ep.dcfg) {
        if (ep.tick >= 1000 && ep.tick < ep.dcfg->motor_failure_tick) {
            const int32_t ae = q16_abs(ep.plant->alt_q16 - ep.dcfg->target_alt_q16);
            const int32_t xe = q16_abs(ep.plant->x_q16 - ep.dcfg->target_x_q16);
            const int32_t ye = q16_abs(ep.plant->y_q16 - ep.dcfg->target_y_q16);
            const int32_t xy = (xe > ye) ? xe : ye;
            if (ae > ep.max_alt_err) ep.max_alt_err = ae;
            if (xy > ep.max_xy_err) ep.max_xy_err = xy;
        }
    }
    if (g_real_write) g_real_write(ctx, sp, n);
    if (ep.plant && !ep.plant->check_fault()) ep.survival++;
    ep.tick++;
}

int main() {
    DroneConfig dcfg;
    dcfg.ticks = 10000;
    dcfg.motor_failure_tick = 9000;
    dcfg.fire_threshold = 1;
    dcfg.target_alt_q16 = 65536;
    dcfg.target_x_q16 = 0;
    dcfg.target_y_q16 = 0;
    dcfg.alt_band_q16 = 19661;
    dcfg.xy_band_q16 = 32768;
    dcfg.wind_period = 700;
    dcfg.wind_impulse_q16 = 6553;

    DronePlant plant;
    plant.init(dcfg, 0x12345678u);
    plant.roll_q16 = 17116;
    plant.roll_rate_q16 = 19661;
    plant.pitch_q16 = 17116;
    plant.alt_q16 = dcfg.target_alt_q16;

    DroneBrain bb(dcfg);
    if (bb.brain.empty()) { printf("FAIL: DroneBrain build\n"); return 1; }
    DroneHarness h(bb);
    if (!h) { printf("FAIL: DroneHarness load\n"); return 1; }
    arc_engine* e = h.e;
    arc_plasticity* p = h.p;

    const uint8_t groups[4] = {0, 1, 2, 3};
    const size_t sd_need = arc_setpoints_required_bytes(groups, 4);
    std::vector<uint8_t> sd_arena(sd_need + 16, 0);
    arc_setpoints* sd = NULL;
    if (arc_setpoints_create(groups, 4, 16384,
                             sd_arena.data(), sd_arena.size(), &sd) != ARC_OK) {
        printf("FAIL: arc_setpoints_create\n");
        return 1;
    }

    /* Bench-real safety profile (tests/mcu/mcu_drone_seq.hpp). */
    int32_t safe_sp[5] = {32768, 32768, 32768, 32768, 0};
    int32_t safe_ramp[5] = {20, 20, 20, 20, 0};
    arc_safety_config scfg;
    std::memset(&scfg, 0, sizeof(scfg));
    scfg.max_setpoint_q16 = 65536;
    scfg.max_rate_per_tick_q16 = 10000;
    scfg.safe_setpoint_q16 = safe_sp;
    scfg.safe_ramp_ticks = safe_ramp;
    scfg.tick_budget_us = 1000;
    scfg.watchdog_misses = 3;
    std::vector<uint8_t> sarena(4096, 0);
    arc_safety* s = nullptr;
    if (arc_safety_create(&scfg, sarena.data(), sarena.size(), &s) != ARC_OK) {
        printf("FAIL: arc_safety_create\n");
        return 1;
    }

    printf("Motor-loss survival: %u ticks, motor failure at tick %u\n",
           dcfg.ticks, dcfg.motor_failure_tick);

    SurvEp& ep = g_ep;
    ep.plant = &plant;
    ep.dcfg = &dcfg;
    ep.e = e;
    ep.tick = 0;
    ep.hash = 1469598103934665603ULL;
    ep.survival = 0;
    ep.max_alt_err = 0;
    ep.max_xy_err = 0;

    arc_hal hal = arc_hal_drone_sim_create(&plant);
    g_real_write = hal.write_motors;
    hal.wait_until = s_wait;
    hal.write_motors = s_write;

    static uint8_t snap_buf[4096];
    arc_loop_config lcfg;
    std::memset(&lcfg, 0, sizeof(lcfg));
    lcfg.period_us = 1000;
    lcfg.start_us = 0;
    lcfg.max_ticks = dcfg.ticks;
    lcfg.snapshot_buf = snap_buf;
    lcfg.snapshot_buf_size = sizeof(snap_buf);

    arc_run(e, s, sd, nullptr, 0, &hal, &lcfg);

    printf("  Survival: %u/%u ticks\n", ep.survival, dcfg.ticks);
    printf("  Station bands [1000,%u): max|alt_err|=%d (band %d) "
           "max|xy_err|=%d (band %d)\n",
           dcfg.motor_failure_tick, ep.max_alt_err, dcfg.alt_band_q16,
           ep.max_xy_err, dcfg.xy_band_q16);
    printf("MOTOR_LOSS_HASH %016llx\n", (unsigned long long)ep.hash);
    printf("  Flips: %u, hamming: %u\n", p->flip_count, *p->hamming);

    int pass = 1;
    if (ep.survival != dcfg.ticks) {
        printf("  FAIL: survival %u/%u\n", ep.survival, dcfg.ticks);
        pass = 0;
    }
    if (arc_safety_fault_state(s)) {
        printf("  FAIL: safety faulted during motor-loss episode\n");
        pass = 0;
    }
    if (ep.max_alt_err > dcfg.alt_band_q16) {
        printf("  FAIL: altitude outside band (%d > %d)\n",
               ep.max_alt_err, dcfg.alt_band_q16);
        pass = 0;
    }
    if (ep.max_xy_err > dcfg.xy_band_q16) {
        printf("  FAIL: x/y outside band (%d > %d)\n",
               ep.max_xy_err, dcfg.xy_band_q16);
        pass = 0;
    }
    if (p->flip_count == 0) {
        printf("  FAIL: no flips occurred\n");
        pass = 0;
    }

    /* Golden hash — motor output sequence for regression (bench-real
       safety profile). Regenerate when the plant or profile changes. */
    const volatile uint64_t golden_hash = 0x8087c5b25d1381ceULL;
    if (golden_hash != 0 && ep.hash != golden_hash) {
        printf("  FAIL: hash mismatch (got %016llx, want %016llx)\n",
               (unsigned long long)ep.hash, (unsigned long long)golden_hash);
        pass = 0;
    }

    if (pass) printf("PASS\n");
    else printf("FAIL\n");
    return pass ? 0 : 1;
}
