/* Host-side Arc HAL for the drone simulation.
   Wires the DronePlant to the arc_hal vtable. */
#include "hal_drone_sim.h"
#include "drone_plant.hpp"
#include "arc/arc_types.h"
#include <cstring>
#include <cstdio>

struct DroneHALCtx {
    arc::sim::DronePlant* plant;
    int32_t* reward_out;
    uint64_t sim_time_us;
    int32_t last_cmd[4];
    int n_cmds;
};

static void sim_write_motors(void* ctx, const int32_t* setpoints_q16, int n_dofs) {
    DroneHALCtx* c = (DroneHALCtx*)ctx;
    if (!c || !c->plant) return;
    c->plant->tick(setpoints_q16, n_dofs);
    for (int i = 0; i < n_dofs && i < 4; ++i) c->last_cmd[i] = setpoints_q16[i];
    c->n_cmds = n_dofs;
    c->sim_time_us += 1000;
}

static uint64_t sim_now_us(void* ctx) {
    DroneHALCtx* c = (DroneHALCtx*)ctx;
    return c ? c->sim_time_us : 0;
}

static bool sim_estop_asserted(void* ctx) {
    DroneHALCtx* c = (DroneHALCtx*)ctx;
    return c && c->plant ? c->plant->check_fault() : false;
}

static void sim_kick_watchdog(void*) {}
static void sim_wait_until(void*, uint64_t) {}
static void sim_log(void*, int, const char*, size_t) {}

static void sim_read_state(void* ctx, int32_t* measured_q16, int32_t* target_q16,
                            int n_dofs) {
    DroneHALCtx* c = (DroneHALCtx*)ctx;
    if (!c || !c->plant) return;
    int32_t gyro[3];
    c->plant->get_imu(gyro);
    int i = 0;
    for (; i < n_dofs && i < 3; ++i) measured_q16[i] = gyro[i];
    for (; i < n_dofs; ++i) measured_q16[i] = 0;
    for (i = 0; i < n_dofs; ++i) target_q16[i] = 0;
}

static void sim_read_reward(void* ctx, int32_t* reward, int n_components) {
    DroneHALCtx* c = (DroneHALCtx*)ctx;
    if (!c || !c->plant) return;
    int32_t rv[3] = {0, 0, 0};
    arc::sim::drone_reward_vec(*c->plant, c->last_cmd, c->n_cmds, rv);
    for (int i = 0; i < n_components && i < 3; ++i) reward[i] = rv[i];
    if (c->reward_out) {
        for (int i = 0; i < n_components && i < 3; ++i) c->reward_out[i] = rv[i];
    }
}

static uint8_t sim_read_phase(void* ctx) {
    DroneHALCtx* c = (DroneHALCtx*)ctx;
    if (c && c->plant) return c->plant->phase;
    return 0;
}

static DroneHALCtx g_drone_ctx;

arc_hal arc_hal_drone_sim_create(arc::sim::DronePlant* plant, int32_t* reward_out) {
    std::memset(&g_drone_ctx, 0, sizeof(g_drone_ctx));
    g_drone_ctx.plant = plant;
    g_drone_ctx.reward_out = reward_out;
    g_drone_ctx.sim_time_us = 0;
    g_drone_ctx.n_cmds = 0;
    std::memset(g_drone_ctx.last_cmd, 0, sizeof(g_drone_ctx.last_cmd));

    arc_hal hal;
    std::memset(&hal, 0, sizeof(hal));
    hal.ctx = &g_drone_ctx;
    hal.write_motors = sim_write_motors;
    hal.now_us = sim_now_us;
    hal.estop_asserted = sim_estop_asserted;
    hal.kick_hardware_watchdog = sim_kick_watchdog;
    hal.wait_until = sim_wait_until;
    hal.log = sim_log;
    hal.read_state = sim_read_state;
    hal.read_reward = sim_read_reward;
    hal.read_phase = sim_read_phase;
    return hal;
}
