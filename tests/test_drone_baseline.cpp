/* Baseline comparison (preliminary — toy plant): classical Q16.16 PID
   cascade (arc_pid, test-side, no engine) vs the reflex engine (DIRECT
   prior + plasticity + bench-real safety) on the same drone plant, same
   seed, same disturbance schedule (rate bumps, wind, motor failure at
   tick 9000). Prints a metrics table for both arms. Assertions cover
   runnability and determinism only — the comparative numbers are
   measured output, not a gate. Labels stay "preliminary — toy plant"
   until re-run on PX4 SITL / the airframe. */
#include "train/sim/drone_env.hpp"
#include "platforms/sim/hal_drone_sim.h"
#include "arc/arc.h"
#include "arc/arc_brain.h"
#include "arc/arc_hal.h"
#include "arc/arc_loop.h"
#include "arc/arc_pid.h"
#include "arc/arc_plasticity.h"
#include "arc/arc_safety.h"
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

using namespace arc::sim;

namespace {

struct Metrics {
    int32_t max_alt_err = 0;    /* [1000, fail) */
    int32_t max_xy_err = 0;     /* [1000, fail) */
    int32_t max_tilt = 0;       /* whole run */
    uint32_t survival = 0;      /* ticks without tilt fault */
    uint32_t fault_rises = 0;   /* tilt-fault rise edges */
    uint32_t recovery = 0xFFFFFFFFu; /* ticks from fail until 100 consecutive in-band; never */
    int64_t reward_sum = 0;     /* tracking reward rv[0], whole run */
    uint64_t hash = 1469598103934665603ULL;

    bool operator==(const Metrics& o) const {
        return max_alt_err == o.max_alt_err && max_xy_err == o.max_xy_err
            && max_tilt == o.max_tilt && survival == o.survival
            && fault_rises == o.fault_rises && recovery == o.recovery
            && reward_sum == o.reward_sum && hash == o.hash;
    }
};

void apply_schedule(DronePlant& p, const DroneConfig& c, uint32_t tick) {
    if (tick == c.motor_failure_tick) p.fail_motor(1);
    p.phase = (uint8_t)((tick / 500) & 1u);
    if (tick > 0 && tick % 300 == 0 && tick != c.motor_failure_tick) {
        if (tick % 600 == 0) p.roll_rate_q16 += 8192;
        else                 p.pitch_rate_q16 += 8192;
    }
}

void observe(Metrics& m, const DronePlant& p, const DroneConfig& c,
             uint32_t tick, bool was_fault) {
    const int32_t ae = q16_abs(p.alt_q16 - c.target_alt_q16);
    const int32_t xe = q16_abs(p.x_q16 - c.target_x_q16);
    const int32_t ye = q16_abs(p.y_q16 - c.target_y_q16);
    const int32_t xy = (xe > ye) ? xe : ye;
    const int32_t tilt = q16_abs(p.roll_q16) + q16_abs(p.pitch_q16);
    if (tick >= 1000 && tick < c.motor_failure_tick) {
        if (ae > m.max_alt_err) m.max_alt_err = ae;
        if (xy > m.max_xy_err) m.max_xy_err = xy;
    }
    if (tilt > m.max_tilt) m.max_tilt = tilt;
    const bool fault = p.check_fault();
    if (fault && !was_fault) m.fault_rises++;
    if (!fault) m.survival++;
    int32_t rv[3];
    drone_reward_vec(p, p.motor_thrust, 4, rv);
    m.reward_sum += rv[0];
}

void pid_init(arc_pid& p, int32_t kp, int32_t ki, int32_t kd,
              int32_t omin, int32_t omax, int32_t imin, int32_t imax) {
    std::memset(&p, 0, sizeof(p));
    p.integral_min_q16 = imin;
    p.integral_max_q16 = imax;
    p.out_min_q16 = omin;
    p.out_max_q16 = omax;
    p.integral_decay_q16 = 0;
    p.masked = 0;
    arc_pid_set_gains(&p, kp, ki, kd);
}

int32_t clamp32(int32_t v, int32_t lo, int32_t hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

Metrics run_pid_arm(const DroneConfig& dcfg, uint32_t seed) {
    DronePlant plant;
    plant.init(dcfg, seed);
    plant.roll_q16 = 17116;
    plant.roll_rate_q16 = 19661;
    plant.pitch_q16 = 17116;
    plant.alt_q16 = dcfg.target_alt_q16;

    arc_pid alt, xpid, ypid, rollp, pitchp, yawp;
    pid_init(alt,   196610, 6553, 131072, -32768, 32768, -131072, 131072);
    pid_init(xpid,   32768,    0,  16384, -13107, 13107, -65536, 65536);
    pid_init(ypid,   32768,    0,  16384, -13107, 13107, -65536, 65536);
    pid_init(rollp, 262144,    0,  65536, -65536, 65536, -65536, 65536);
    pid_init(pitchp,262144,    0,  65536, -65536, 65536, -65536, 65536);
    pid_init(yawp,   65536,    0,      0,  -8192,  8192, -65536, 65536);

    Metrics m;
    bool was_fault = false;
    uint32_t in_band_run = 0;

    for (uint32_t t = 0; t < dcfg.ticks; ++t) {
        apply_schedule(plant, dcfg, t);

        const int32_t alt_err = clamp32(dcfg.target_alt_q16 - plant.alt_q16,
                                        -131072, 131072);
        const int32_t throttle = 32768 + arc_pid_step(&alt, 0, -alt_err);
        const int32_t pitch_tgt = arc_pid_step(&xpid, dcfg.target_x_q16, plant.x_q16);
        const int32_t roll_tgt  = arc_pid_step(&ypid, dcfg.target_y_q16, plant.y_q16);
        const int32_t roll_out  = arc_pid_step(&rollp, roll_tgt, plant.roll_q16);
        const int32_t pitch_out = arc_pid_step(&pitchp, pitch_tgt, plant.pitch_q16);
        const int32_t yaw_out   = arc_pid_step(&yawp, 0, plant.yaw_rate_q16);

        int32_t cmd[4];
        cmd[0] = clamp32(throttle - pitch_out / 2 - yaw_out / 4, 0, 65536);
        cmd[1] = clamp32(throttle - roll_out / 2 + yaw_out / 4, 0, 65536);
        cmd[2] = clamp32(throttle + pitch_out / 2 - yaw_out / 4, 0, 65536);
        cmd[3] = clamp32(throttle + roll_out / 2 + yaw_out / 4, 0, 65536);

        observe(m, plant, dcfg, t, was_fault);
        was_fault = plant.check_fault();
        for (int i = 0; i < 4; ++i)
            m.hash = drone_fnv1a(m.hash, (uint32_t)cmd[i]);

        if (t >= dcfg.motor_failure_tick) {
            const int32_t ae = q16_abs(plant.alt_q16 - dcfg.target_alt_q16);
            const int32_t xe = q16_abs(plant.x_q16 - dcfg.target_x_q16);
            const int32_t ye = q16_abs(plant.y_q16 - dcfg.target_y_q16);
            const int32_t tilt = q16_abs(plant.roll_q16) + q16_abs(plant.pitch_q16);
            const bool in_band = ae < dcfg.alt_band_q16
                              && ((xe > ye) ? xe : ye) < dcfg.xy_band_q16
                              && tilt <= 51472;
            if (in_band) {
                in_band_run++;
                if (in_band_run == 100 && m.recovery == 0xFFFFFFFFu)
                    m.recovery = t - dcfg.motor_failure_tick + 1;
            } else {
                in_band_run = 0;
            }
        }

        plant.tick(cmd, 4);
    }
    return m;
}

/* Reflex arm: engine + plasticity + bench-real safety, hooked like
   test_motor_loss_survival. */
struct RefEp {
    DronePlant* plant = nullptr;
    DroneConfig* dcfg = nullptr;
    arc_engine* e = nullptr;
    uint32_t tick = 0;
    Metrics* m = nullptr;
    bool was_fault = false;
    uint32_t in_band_run = 0;
};
static RefEp g_ep;
static void (*g_real_write)(void*, const int32_t*, int);

static void r_wait(void*, uint64_t) {
    RefEp& ep = g_ep;
    if (ep.plant && ep.dcfg) {
        apply_schedule(*ep.plant, *ep.dcfg, ep.tick);
        drone_encode(*ep.plant, ep.e->input, ep.dcfg->input_bits);
    }
}

static void r_write(void* ctx, const int32_t* sp, int n) {
    RefEp& ep = g_ep;
    (void)sp;
    (void)n;
    if (ep.e) {
        uint32_t emitted = 0;
        arc_get_motor(ep.e, &emitted);
        ep.m->hash = drone_fnv1a(ep.m->hash, emitted);
    }
    if (ep.plant && ep.dcfg) {
        observe(*ep.m, *ep.plant, *ep.dcfg, ep.tick, ep.was_fault);
        ep.was_fault = ep.plant->check_fault();
        if (ep.tick >= ep.dcfg->motor_failure_tick) {
            const DronePlant& p = *ep.plant;
            const DroneConfig& c = *ep.dcfg;
            const int32_t ae = q16_abs(p.alt_q16 - c.target_alt_q16);
            const int32_t xe = q16_abs(p.x_q16 - c.target_x_q16);
            const int32_t ye = q16_abs(p.y_q16 - c.target_y_q16);
            const int32_t tilt = q16_abs(p.roll_q16) + q16_abs(p.pitch_q16);
            const bool in_band = ae < c.alt_band_q16
                              && ((xe > ye) ? xe : ye) < c.xy_band_q16
                              && tilt <= 51472;
            if (in_band) {
                ep.in_band_run++;
                if (ep.in_band_run == 100 && ep.m->recovery == 0xFFFFFFFFu)
                    ep.m->recovery = ep.tick - c.motor_failure_tick + 1;
            } else {
                ep.in_band_run = 0;
            }
        }
    }
    if (g_real_write) g_real_write(ctx, sp, n);
    ep.tick++;
}

Metrics run_reflex_arm(const DroneConfig& dcfg, uint32_t seed) {
    DronePlant plant;
    plant.init(dcfg, seed);
    plant.roll_q16 = 17116;
    plant.roll_rate_q16 = 19661;
    plant.pitch_q16 = 17116;
    plant.alt_q16 = dcfg.target_alt_q16;

    DroneBrain bb(dcfg);
    if (bb.brain.empty()) { printf("FAIL: DroneBrain build\n"); }
    DroneHarness h(bb);
    if (!h) { printf("FAIL: DroneHarness load\n"); }
    arc_engine* e = h.e;
    arc_plasticity* p = h.p;

    const uint8_t groups[4] = {0, 1, 2, 3};
    const size_t sd_need = arc_setpoints_required_bytes(groups, 4);
    std::vector<uint8_t> sd_arena(sd_need + 16, 0);
    arc_setpoints* sd = NULL;
    if (arc_setpoints_create(groups, 4, 16384,
                             sd_arena.data(), sd_arena.size(), &sd) != ARC_OK) {
        printf("FAIL: arc_setpoints_create\n");
    }

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
    }

    Metrics m;
    RefEp& ep = g_ep;
    ep.plant = &plant;
    ep.dcfg = const_cast<DroneConfig*>(&dcfg);
    ep.e = e;
    ep.tick = 0;
    ep.m = &m;
    ep.was_fault = false;
    ep.in_band_run = 0;

    arc_hal hal = arc_hal_drone_sim_create(&plant);
    g_real_write = hal.write_motors;
    hal.wait_until = r_wait;
    hal.write_motors = r_write;

    static uint8_t snap_buf[4096];
    arc_loop_config lcfg;
    std::memset(&lcfg, 0, sizeof(lcfg));
    lcfg.period_us = 1000;
    lcfg.start_us = 0;
    lcfg.max_ticks = dcfg.ticks;
    lcfg.snapshot_buf = snap_buf;
    lcfg.snapshot_buf_size = sizeof(snap_buf);

    arc_run(e, s, sd, nullptr, 0, &hal, &lcfg);
    (void)p;
    return m;
}

void print_row(const char* name, const Metrics& m) {
    printf("  %-8s max_alt=%6d max_xy=%6d max_tilt=%6d surv=%5u rises=%u "
           "recovery=%s reward=%lld\n",
           name, m.max_alt_err, m.max_xy_err, m.max_tilt, m.survival,
           m.fault_rises,
           m.recovery == 0xFFFFFFFFu ? "never" : std::to_string(m.recovery).c_str(),
           (long long)m.reward_sum);
}

}  // namespace

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
    const uint32_t seed = 0x12345678u;

    printf("Drone baseline (PRELIMINARY — toy plant): PID-only vs reflex, "
           "%u ticks, motor 1 fails at tick %u\n",
           dcfg.ticks, dcfg.motor_failure_tick);

    const Metrics pid1 = run_pid_arm(dcfg, seed);
    const Metrics pid2 = run_pid_arm(dcfg, seed);
    const Metrics ref1 = run_reflex_arm(dcfg, seed);
    const Metrics ref2 = run_reflex_arm(dcfg, seed);

    printf("Metrics table (station window [1000,%u), recovery = ticks from "
           "failure until 100 consecutive in-band):\n",
           dcfg.motor_failure_tick);
    print_row("pid", pid1);
    print_row("reflex", ref1);

    int pass = 1;
    if (!(pid1 == pid2)) {
        printf("FAIL: PID arm not deterministic\n");
        pass = 0;
    }
    if (!(ref1 == ref2)) {
        printf("FAIL: reflex arm not deterministic\n");
        pass = 0;
    }
    if (pid1.survival == 0 || ref1.survival == 0) {
        printf("FAIL: an arm never survived a tick\n");
        pass = 0;
    }
    if (pid1.hash == ref1.hash) {
        printf("INFO: arms produced identical command streams\n");
    }

    if (pass) printf("PASS\n");
    else printf("FAIL\n");
    return pass ? 0 : 1;
}
