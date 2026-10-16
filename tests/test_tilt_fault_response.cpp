/* Tilt-fault response test (split of the former test_drone_sim phase 2):
   force roll past the 45 deg tilt limit from the first tick; the safety
   layer must ramp the setpoints to hover, hold them, freeze plasticity
   (loop-set fault gate), and fire snapshot-before-fault. Bench-real
   safety profile. Motor-loss survival lives in test_motor_loss_survival. */
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

struct TiltEp {
    DronePlant* plant = nullptr;
    DroneConfig* dcfg = nullptr;
    arc_engine* e = nullptr;
    arc_safety* s = nullptr;
    uint32_t tick = 0;
    int snapshots = 0;
    bool flips_recorded = false;
    uint32_t flips_at_fault = 0;
    bool gate_seen_at_fault = false;
    std::vector<std::vector<int32_t>> history;
};
static TiltEp g_ep;
static void (*g_real_write)(void*, const int32_t*, int);

static void t_wait(void*, uint64_t) {
    TiltEp& ep = g_ep;
    if (ep.plant) {
        ep.plant->roll_q16 = 60000;   /* > TILT_LIMIT 51472 (~45 deg) */
        ep.plant->roll_rate_q16 = 0;
        ep.plant->pitch_q16 = 0;
        ep.plant->pitch_rate_q16 = 0;
        drone_encode(*ep.plant, ep.e->input, ep.dcfg->input_bits);
    }
    if (ep.s && !ep.flips_recorded && arc_safety_fault_state(ep.s) && ep.e) {
        ep.flips_at_fault = ep.e->plasticity->flip_count;
        ep.gate_seen_at_fault = ep.e->plasticity->fault_gated != 0;
        ep.flips_recorded = true;
        printf("  Fault path entered at tick %u (flips=%u, gate=%d)\n",
               ep.tick, ep.flips_at_fault,
               (int)ep.e->plasticity->fault_gated);
    }
}

static void t_write(void* ctx, const int32_t* sp, int n) {
    TiltEp& ep = g_ep;
    ep.history.push_back(std::vector<int32_t>(sp, sp + n));
    if (g_real_write) g_real_write(ctx, sp, n);
    ep.tick++;
}

static void t_store(void*, const uint8_t*, size_t) {
    g_ep.snapshots++;
}

int main() {
    DroneConfig dcfg;
    dcfg.ticks = 50;
    dcfg.motor_failure_tick = 1000000;   /* never */
    dcfg.fire_threshold = 1;
    dcfg.target_alt_q16 = 65536;
    dcfg.target_x_q16 = 0;
    dcfg.target_y_q16 = 0;

    DronePlant plant;
    plant.init(dcfg, 0x12345678u);
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

    TiltEp& ep = g_ep;
    ep.plant = &plant;
    ep.dcfg = &dcfg;
    ep.e = e;
    ep.s = s;
    ep.tick = 0;
    ep.snapshots = 0;
    ep.flips_recorded = false;
    ep.flips_at_fault = 0;
    ep.gate_seen_at_fault = false;
    ep.history.clear();
    const uint32_t flips_before = p->flip_count;

    arc_hal hal = arc_hal_drone_sim_create(&plant);
    g_real_write = hal.write_motors;
    hal.wait_until = t_wait;
    hal.write_motors = t_write;
    hal.store_snapshot = t_store;

    static uint8_t snap_buf[4096];
    arc_loop_config lcfg;
    std::memset(&lcfg, 0, sizeof(lcfg));
    lcfg.period_us = 1000;
    lcfg.start_us = 0;
    lcfg.max_ticks = dcfg.ticks;
    lcfg.snapshot_buf = snap_buf;
    lcfg.snapshot_buf_size = sizeof(snap_buf);

    printf("Tilt fault: force roll > 45 deg from tick 0\n");
    arc_run(e, s, sd, nullptr, 0, &hal, &lcfg);

    int pass = 1;
    const size_t n_hist = ep.history.size();
    printf("  history ticks: %zu, snapshots: %d, flips: %u -> %u\n",
           n_hist, ep.snapshots, flips_before, p->flip_count);

    if (n_hist < 25) {
        printf("  FAIL: history short (%zu)\n", n_hist);
        pass = 0;
    }
    if (!arc_safety_fault_state(s)) {
        printf("  FAIL: no fault latched for forced tilt\n");
        pass = 0;
    }
    if (ep.snapshots < 1) {
        printf("  FAIL: snapshot-before-fault did not fire\n");
        pass = 0;
    }
    if (!ep.flips_recorded || !ep.gate_seen_at_fault) {
        printf("  FAIL: loop did not set plasticity fault gate\n");
        pass = 0;
    } else if (p->flip_count != ep.flips_at_fault) {
        printf("  FAIL: plasticity not gated during fault (%u != %u)\n",
               p->flip_count, ep.flips_at_fault);
        pass = 0;
    } else {
        printf("  PASS: loop-set fault gate held plasticity at %u\n",
               ep.flips_at_fault);
    }

    if (n_hist >= 25) {
        const int32_t* held = ep.history[21].data();
        bool ramp_ok = true;
        for (int i = 0; i < 4; ++i) {
            if (held[i] != 32768) ramp_ok = false;
        }
        if (ramp_ok) {
            printf("  PASS: reached hover (32768) after ramp, holds\n");
        } else {
            printf("  FAIL: not at hover after ramp: [%d,%d,%d,%d]\n",
                   held[0], held[1], held[2], held[3]);
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
