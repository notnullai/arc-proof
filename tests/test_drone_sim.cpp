/* Drone simulation test: 10000-tick episode via arc_run + arc_safety +
   arc_setpoints on the 256-bit 8-channel frame (attitude + altitude +
   velocity + x/y station), motor failure at tick 9000, station-hold band
   check under periodic wind, then a force-tilt fault phase proving software
   ramp to hover, fault-gated plasticity, and snapshot-before-fault. */
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

/* Test-side HAL wrappers around the sim HAL (DroneHALCtx is file-private).
   wait_until: plant events + drone_encode before arc_step.
   write_motors: hash/history/survival/bands, then real plant.tick; force
   tilt after tick so end-of-tick estop sees it. */
struct DroneEp {
    DronePlant* plant = nullptr;
    DroneConfig* dcfg = nullptr;
    arc_engine* e = nullptr;
    arc_safety* s = nullptr;
    bool inject_events = false;
    bool force_tilt = false;
    bool hash_enabled = false;
    bool accumulate_reward = false;
    bool count_survival = false;
    bool record_history = false;
    bool track_bands = false;
    bool flips_recorded = false;
    uint32_t tick = 0;
    uint64_t hash = 1469598103934665603ULL;
    int64_t reward_sum = 0;
    int64_t reward_first = 0;
    int64_t reward_last = 0;
    uint32_t survival = 0;
    int snapshots = 0;
    uint32_t flips_at_fault = 0;
    int32_t max_alt_err = 0;
    int32_t max_xy_err = 0;
    int32_t last_rv[3] = {0, 0, 0};
    std::vector<std::vector<int32_t>> history;
};
static DroneEp g_ep;
static void (*g_real_write)(void*, const int32_t*, int);
static void (*g_real_reward)(void*, int32_t*, int);

static void ep_wait(void*, uint64_t) {
    DroneEp& ep = g_ep;
    if (ep.inject_events && ep.plant && ep.dcfg) {
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
    if (ep.force_tilt && ep.plant) {
        ep.plant->roll_q16 = 60000;   /* > TILT_LIMIT 51472 (~45°) */
    }
    if (ep.e && ep.plant && ep.dcfg) {
        drone_encode(*ep.plant, ep.e->input, ep.dcfg->input_bits);
    }
    if (ep.s && !ep.flips_recorded && arc_safety_fault_state(ep.s) && ep.e) {
        ep.flips_at_fault = ep.e->plasticity->flip_count;
        ep.flips_recorded = true;
        printf("  Fault path entered at tick %u (flips frozen at %u)\n",
               ep.tick, ep.flips_at_fault);
    }
}

static void ep_write(void* ctx, const int32_t* sp, int n) {
    DroneEp& ep = g_ep;
    if (ep.hash_enabled && ep.e) {
        uint32_t emitted = 0;
        arc_get_motor(ep.e, &emitted);
        ep.hash = drone_fnv1a(ep.hash, emitted);
        if (ep.tick % 1000 == 0 || ep.tick == 1 || ep.tick == 9999) {
            printf("  t=%5u roll=%6d pitch=%6d alt=%6d x=%6d y=%6d "
                   "motors=0x%x cmd=[%d,%d,%d,%d] flips=%u hamming=%u\n",
                   ep.tick,
                   ep.plant ? ep.plant->roll_q16 : 0,
                   ep.plant ? ep.plant->pitch_q16 : 0,
                   ep.plant ? ep.plant->alt_q16 : 0,
                   ep.plant ? ep.plant->x_q16 : 0,
                   ep.plant ? ep.plant->y_q16 : 0,
                   emitted, sp[0], sp[1], sp[2], sp[3],
                   ep.e->plasticity->flip_count,
                   ep.e->plasticity->hamming ? *ep.e->plasticity->hamming : 0);
        }
    }
    if (ep.track_bands && ep.plant && ep.dcfg) {
        /* Settle window: skip first 1000 ticks; stop before motor failure. */
        if (ep.tick >= 1000 && ep.tick < ep.dcfg->motor_failure_tick) {
            const int32_t ae = q16_abs(ep.plant->alt_q16 - ep.dcfg->target_alt_q16);
            const int32_t xe = q16_abs(ep.plant->x_q16 - ep.dcfg->target_x_q16);
            const int32_t ye = q16_abs(ep.plant->y_q16 - ep.dcfg->target_y_q16);
            const int32_t xy = (xe > ye) ? xe : ye;
            if (ae > ep.max_alt_err) ep.max_alt_err = ae;
            if (xy > ep.max_xy_err) ep.max_xy_err = xy;
        }
    }
    if (ep.record_history) {
        ep.history.push_back(std::vector<int32_t>(sp, sp + n));
    }
    if (g_real_write) g_real_write(ctx, sp, n);
    if (ep.force_tilt && ep.plant) {
        ep.plant->roll_q16 = 60000;
    }
    if (ep.count_survival && ep.plant && !ep.plant->check_fault()) {
        ep.survival++;
    }
    ep.tick++;
}

static void ep_reward(void* ctx, int32_t* r, int n) {
    if (g_real_reward) g_real_reward(ctx, r, n);
    DroneEp& ep = g_ep;
    if (n >= 3) {
        ep.last_rv[0] = r[0];
        ep.last_rv[1] = r[1];
        ep.last_rv[2] = r[2];
    }
    if (!ep.accumulate_reward) return;
    int64_t tr = 0;
    for (int i = 0; i < 3 && i < n; ++i) tr += r[i];
    ep.reward_sum += tr;
    if (ep.tick < 1000) ep.reward_first += tr;
    if (ep.tick >= 9000) ep.reward_last += tr;
}

static void ep_store(void*, const uint8_t*, size_t) {
    g_ep.snapshots++;
}

int main() {
    /* --- Setup --- */
    DroneConfig dcfg;
    dcfg.ticks = 10000;
    dcfg.motor_failure_tick = 9000;
    dcfg.fire_threshold = 1;
    dcfg.target_alt_q16 = 65536;   /* 1.0 m */
    dcfg.target_x_q16 = 0;
    dcfg.target_y_q16 = 0;
    dcfg.alt_band_q16 = 19661;     /* ±0.3 m */
    dcfg.xy_band_q16 = 32768;      /* ±0.5 m */
    dcfg.wind_period = 700;
    dcfg.wind_impulse_q16 = 6553;    /* 0.1 m/s */

    DronePlant plant;
    plant.init(dcfg, 0x12345678u);

    /* Start perturbed attitude, on station altitude. */
    plant.roll_q16 = 17116;        /* 15° */
    plant.roll_rate_q16 = 19661;   /* 0.3 rad/s */
    plant.pitch_q16 = 17116;       /* 15° */
    plant.alt_q16 = dcfg.target_alt_q16;

    /* Build ARC002 brain via DroneBrain (256-input, 64-hidden, 4-motor). */
    DroneBrain bb(dcfg);
    if (bb.brain.empty()) {
        printf("FAIL: DroneBrain build\n");
        return 1;
    }
    DroneHarness h(bb);
    if (!h) {
        printf("FAIL: DroneHarness load\n");
        return 1;
    }
    arc_engine* e = h.e;
    arc_plasticity* p = h.p;
    printf("encoder_geometry_hash %016llx\n",
           (unsigned long long)bb.pcfg.encoder_geometry_hash);

    /* Graded decode: leaky integrator, alpha=16384. */
    const uint8_t groups[4] = {0, 1, 2, 3};
    const size_t sd_need = arc_setpoints_required_bytes(groups, 4);
    std::vector<uint8_t> sd_arena(sd_need + 16, 0);
    arc_setpoints* sd = NULL;
    if (arc_setpoints_create(groups, 4, /*alpha=*/16384,
                             sd_arena.data(), sd_arena.size(), &sd) != ARC_OK) {
        printf("FAIL: arc_setpoints_create\n");
        return 1;
    }

    /* Hover safe setpoint (docs/safety_contract.md). */
    int32_t safe_sp[5] = {32768, 32768, 32768, 32768, 0};
    int32_t safe_ramp[5] = {20, 20, 20, 20, 0};
    arc_safety_config scfg;
    std::memset(&scfg, 0, sizeof(scfg));
    scfg.max_setpoint_q16 = 65536;
    scfg.max_rate_per_tick_q16 = 65536;
    scfg.safe_setpoint_q16 = safe_sp;
    scfg.safe_ramp_ticks = safe_ramp;
    scfg.tick_budget_us = 1000000;
    scfg.watchdog_misses = 1000000;
    std::vector<uint8_t> sarena(4096, 0);
    arc_safety* s = nullptr;
    if (arc_safety_create(&scfg, sarena.data(), sarena.size(), &s) != ARC_OK) {
        printf("FAIL: arc_safety_create\n");
        return 1;
    }

    /* --- Episode via arc_run --- */
    printf("Drone sim: %u ticks, motor failure at tick %u, wind every %u\n",
           dcfg.ticks, dcfg.motor_failure_tick, dcfg.wind_period);

    DroneEp& ep = g_ep;
    ep.plant = &plant;
    ep.dcfg = &dcfg;
    ep.e = e;
    ep.s = s;
    ep.inject_events = true;
    ep.force_tilt = false;
    ep.hash_enabled = true;
    ep.accumulate_reward = true;
    ep.count_survival = true;
    ep.record_history = false;
    ep.track_bands = true;
    ep.flips_recorded = false;
    ep.tick = 0;
    ep.hash = 1469598103934665603ULL;
    ep.reward_sum = ep.reward_first = ep.reward_last = 0;
    ep.survival = 0;
    ep.snapshots = 0;
    ep.flips_at_fault = 0;
    ep.max_alt_err = 0;
    ep.max_xy_err = 0;
    ep.last_rv[0] = ep.last_rv[1] = ep.last_rv[2] = 0;
    ep.history.clear();

    arc_hal hal = arc_hal_drone_sim_create(&plant);
    g_real_write = hal.write_motors;
    g_real_reward = hal.read_reward;
    hal.wait_until = ep_wait;
    hal.write_motors = ep_write;
    hal.read_reward = ep_reward;
    hal.store_snapshot = ep_store;

    static uint8_t snap_buf[4096];
    arc_loop_config lcfg;
    std::memset(&lcfg, 0, sizeof(lcfg));
    lcfg.period_us = 1000;
    lcfg.start_us = 0;
    lcfg.max_ticks = dcfg.ticks;
    lcfg.snapshot_buf = snap_buf;
    lcfg.snapshot_buf_size = sizeof(snap_buf);

    arc_run(e, s, sd, nullptr, 0, &hal, &lcfg);

    /* --- Phase 1 report --- */
    printf("Episode complete.\n");
    printf("  Survival: %u/%u ticks (%.1f%%)\n",
           ep.survival, dcfg.ticks,
           100.0 * ep.survival / dcfg.ticks);
    printf("  Reward first 1000: %lld\n", (long long)ep.reward_first);
    printf("  Reward last 1000:  %lld\n", (long long)ep.reward_last);
    printf("  Reward total:      %lld\n", (long long)ep.reward_sum);
    printf("  Station bands [1000,%u): max|alt_err|=%d (band %d) "
           "max|xy_err|=%d (band %d)\n",
           dcfg.motor_failure_tick, ep.max_alt_err, dcfg.alt_band_q16,
           ep.max_xy_err, dcfg.xy_band_q16);
    printf("DRONE_SIM_HASH %016llx\n", (unsigned long long)ep.hash);
    printf("  Flips: %u, reverts: %u, hamming: %u\n",
           p->flip_count, p->revert_count, *p->hamming);

    int pass = 1;

    if (ep.survival != dcfg.ticks) {
        printf("  FAIL: survival %u/%u\n", ep.survival, dcfg.ticks);
        pass = 0;
    }
    if (p->flip_count == 0) {
        printf("  FAIL: no flips occurred\n");
        pass = 0;
    }
    if (*p->hamming == 0) {
        printf("  FAIL: hamming is zero\n");
        pass = 0;
    }
    if (ep.snapshots != 0) {
        printf("  FAIL: snapshot fired without fault (%d)\n", ep.snapshots);
        pass = 0;
    }
    if (arc_safety_fault_state(s)) {
        printf("  FAIL: safety faulted during main episode\n");
        pass = 0;
    }
    /* Station-hold acceptance (Step 3): bands under periodic wind. */
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

    /* Golden hash — captures motor output sequence for regression.
       Regenerated for the 320-bit frame + translation physics (Step 3). */
    const volatile uint64_t golden_hash = 0xcf9ea1aec3de21f1ULL;
    if (golden_hash != 0 && ep.hash != golden_hash) {
        printf("  FAIL: hash mismatch (got %016llx, want %016llx)\n",
               (unsigned long long)ep.hash, (unsigned long long)golden_hash);
        pass = 0;
    }

    /* --- Phase 2: force tilt > 45° → ramp to hover, gated plasticity --- */
    printf("\nFault phase: force roll > 45°\n");
    const uint32_t flips_before_fault = p->flip_count;
    ep.inject_events = false;
    ep.force_tilt = true;
    ep.hash_enabled = false;
    ep.accumulate_reward = false;
    ep.count_survival = false;
    ep.track_bands = false;
    ep.record_history = true;
    ep.flips_recorded = false;
    ep.flips_at_fault = 0;
    ep.snapshots = 0;
    ep.history.clear();
    plant.roll_q16 = 60000;
    plant.roll_rate_q16 = 0;
    plant.pitch_q16 = 0;
    plant.pitch_rate_q16 = 0;

    lcfg.max_ticks = 50;
    arc_run(e, s, sd, nullptr, 0, &hal, &lcfg);

    const size_t n_hist = ep.history.size();
    printf("  history ticks: %zu, snapshots: %d, flips: %u -> %u\n",
           n_hist, ep.snapshots, flips_before_fault, p->flip_count);

    if (n_hist < 25) {
        printf("  FAIL: history short (%zu)\n", n_hist);
        pass = 0;
    }
    if (ep.snapshots < 1) {
        printf("  FAIL: snapshot-before-fault did not fire\n");
        pass = 0;
    }
    if (!ep.flips_recorded) {
        printf("  FAIL: never entered fault path (safety fault=%d)\n",
               (int)arc_safety_fault_state(s));
        pass = 0;
    } else if (p->flip_count != ep.flips_at_fault) {
        printf("  FAIL: plasticity not gated during fault (%u != %u)\n",
               p->flip_count, ep.flips_at_fault);
        pass = 0;
    } else {
        printf("  PASS: plasticity fault-gated (flips held at %u)\n",
               ep.flips_at_fault);
    }

    if (n_hist >= 25) {
        const int32_t* first_fault = ep.history[1].data();
        const int32_t* held = ep.history[21].data();
        bool ramp_ok = true;
        for (int i = 0; i < 4; ++i) {
            if (held[i] != 32768) ramp_ok = false;
        }
        const int32_t* last_normal = ep.history[0].data();
        if (ramp_ok) {
            printf("  PASS: reached hover (32768) after ramp, holds\n");
        } else {
            printf("  FAIL: not at hover after ramp: [%d,%d,%d,%d] "
                   "(first_fault=[%d,%d,%d,%d] last_normal=[%d,%d,%d,%d])\n",
                   held[0], held[1], held[2], held[3],
                   first_fault[0], first_fault[1], first_fault[2], first_fault[3],
                   last_normal[0], last_normal[1], last_normal[2], last_normal[3]);
            pass = 0;
        }
        bool dir_ok = true;
        for (int i = 0; i < 4; ++i) {
            const int32_t a = last_normal[i], b = first_fault[i];
            if (a < 32768 && b < a) dir_ok = false;
            if (a > 32768 && b > a) dir_ok = false;
            if (a < 32768 && b > 32768) dir_ok = false;
            if (a > 32768 && b < 32768) dir_ok = false;
        }
        if (dir_ok) {
            printf("  PASS: ramp moves toward hover from last command\n");
        } else {
            printf("  FAIL: ramp wrong direction\n");
            pass = 0;
        }
        const int32_t* last = ep.history.back().data();
        if (last[0] == 32768 && last[1] == 32768 &&
            last[2] == 32768 && last[3] == 32768) {
            printf("  PASS: final write holds hover\n");
        } else {
            printf("  FAIL: final write drifted [%d,%d,%d,%d]\n",
                   last[0], last[1], last[2], last[3]);
            pass = 0;
        }
    }

    if (pass) printf("PASS\n");
    else printf("FAIL\n");
    return pass ? 0 : 1;
}
