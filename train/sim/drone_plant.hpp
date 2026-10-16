#pragma once
/* 6-DoF-lite quadrotor plant (roll, pitch, yaw rate + x/y/z translation)
   with IMU noise, motor dynamics, wind disturbance, and fault injection.
   All Q16.16 fixed-point. Deterministic per seed (xorshift32 PRNG). */
#include "fixed_q16.hpp"
#include <cstdint>
#include <cstring>

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable: 4514 4820)  /* unreferenced inline removed, padding */
#endif

namespace arc {
namespace sim {

/* Deterministic PRNG (xorshift32). */
struct DronePRNG {
    uint32_t state;
    DronePRNG() : state(0x12345678u) {}
    explicit DronePRNG(uint32_t seed) : state(seed ? seed : 1u) {}
    uint32_t next() {
        state ^= state << 13;
        state ^= state >> 17;
        state ^= state << 5;
        return state;
    }
    /* Return signed noise in [-amp, +amp]. */
    int32_t noise_q16(int32_t amp_q16) {
        uint32_t v = next();
        uint32_t range = (uint32_t)(amp_q16 * 2);
        int32_t offset = (int32_t)(v % range) - amp_q16;
        return offset;
    }
};

/* FNV-1a over the frame-layout descriptor. Recompute when channels change
   (frame layout is brain identity — docs/integration.md). */
inline uint64_t drone_encoder_geometry_hash(int32_t input_bits) {
    const char* s =
        "drone:roll,pitch,roll_rate,pitch_rate,alt_err,vel_z,x_err,y_err,"
        "vel_x,vel_y";
    uint64_t h = 1469598103934665603ULL;
    for (const char* p = s; *p; ++p) {
        h ^= (uint8_t)*p;
        h *= 1099511628211ULL;
    }
    h ^= (uint64_t)(uint32_t)input_bits;
    h *= 1099511628211ULL;
    return h;
}

struct DroneConfig {
    int32_t n_motors = 4;         /* X-frame quadrotor */
    /* 10 × 32: roll, pitch, roll_rate, pitch_rate,
       alt_err, vel_z, x_err, y_err, vel_x, vel_y. First four bit ranges
       unchanged from the 128-bit frame; 8-9 appended for PD station. */
    int32_t input_bits = 320;
    int32_t input_words = 320 / 32;
    int32_t hidden_neurons = 64;
    int32_t hidden_words = 64 / 32;
    int32_t motor_neurons = 4;
    int32_t n_groups = 4;
    int32_t fire_threshold = 65;
    uint32_t leak_q16 = 64881;
    uint32_t ticks = 10000;
    uint32_t motor_failure_tick = 5000;
    int32_t noise_amp_q16 = 655;         /* 0.01 rad/s gyro noise */

    /* Station targets (Q16.16 metres). */
    int32_t target_alt_q16 = 65536;      /* 1.0 m */
    int32_t target_x_q16 = 0;
    int32_t target_y_q16 = 0;

    /* Acceptance bands for station-hold (Q16.16 metres). */
    int32_t alt_band_q16 = 19661;        /* ±0.3 m */
    int32_t xy_band_q16 = 32768;         /* ±0.5 m */

    /* Periodic horizontal wind impulse every wind_period ticks (Q16.16 m/s). */
    uint32_t wind_period = 700;
    int32_t wind_impulse_q16 = 6553;     /* 0.1 m/s */
};

/* Plant. Attitude is 3-DoF Euler; translation is x/y/z in metres. */
struct DronePlant {
    /* Attitude state. */
    int32_t roll_q16;       /* rad, [-pi, pi] */
    int32_t pitch_q16;      /* rad, [-pi, pi] */
    int32_t yaw_rate_q16;   /* rad/s */
    int32_t roll_rate_q16;  /* rad/s */
    int32_t pitch_rate_q16; /* rad/s */

    /* Translation (Q16.16 metres / m/s). Hover-near-level: no quaternions. */
    int32_t alt_q16;
    int32_t vel_z;
    int32_t x_q16;
    int32_t y_q16;
    int32_t vel_x;
    int32_t vel_y;

    /* Motor model (first-order lag). */
    int32_t motor_thrust[4];
    int32_t motor_target[4];

    /* IMU noise state. */
    int32_t gyro_bias[3];
    int32_t accel_bias[3];

    uint8_t phase;
    mutable DronePRNG prng;
    DroneConfig cfg;
    uint32_t tick_count;
    int motor_failed[4];
    int wind_sign;           /* alternates each wind impulse */

    void init(const DroneConfig& c, uint32_t seed = 0x12345678u) {
        *this = DronePlant{};  /* zero-init all state (GCC -Werror=class-memaccess forbids memset(this)) */
        cfg = c;
        prng = DronePRNG(seed);
        wind_sign = 1;
    }

    /* X-frame:
         Motor 0: front-left  (-pitch torque → +m0 decreases pitch_torque)
         Motor 1: front-right (-roll)
         Motor 2: rear-right  (+pitch)
         Motor 3: rear-left   (+roll) */
    void tick(const int32_t* cmd_q16, int n_cmds) {
        int32_t effective_cmd[4];
        for (int i = 0; i < 4 && i < n_cmds; ++i) {
            effective_cmd[i] = motor_failed[i] ? 0 : cmd_q16[i];
        }

        const int32_t motor_tau = 32768;  /* 0.5 → 2-tick-ish lag */
        for (int i = 0; i < 4; ++i) {
            motor_target[i] = effective_cmd[i];
            int32_t diff = motor_target[i] - motor_thrust[i];
            motor_thrust[i] += q16_mul(diff, motor_tau);
        }

        int32_t roll_torque  = motor_thrust[3] - motor_thrust[1];
        int32_t pitch_torque = motor_thrust[2] - motor_thrust[0];
        int32_t yaw_torque   = motor_thrust[0] - motor_thrust[1]
                             + motor_thrust[2] - motor_thrust[3];

        const int32_t I_roll  = 131072;
        const int32_t I_pitch = 131072;
        const int32_t I_yaw   = 65536;
        const int32_t k_yaw   = 16384;

        int32_t roll_accel  = q16_mul(roll_torque, (int32_t)(((int64_t)65536 << 16) / I_roll));
        int32_t pitch_accel = q16_mul(pitch_torque, (int32_t)(((int64_t)65536 << 16) / I_pitch));
        int32_t yaw_accel   = q16_mul(yaw_torque * k_yaw / 65536,
                                      (int32_t)(((int64_t)65536 << 16) / I_yaw));

        const int32_t damping = 3276;
        roll_accel  -= q16_mul(roll_rate_q16, damping);
        pitch_accel -= q16_mul(pitch_rate_q16, damping);
        yaw_accel   -= q16_mul(yaw_rate_q16, damping);

        roll_rate_q16  += roll_accel / 1000;
        pitch_rate_q16 += pitch_accel / 1000;
        yaw_rate_q16   += yaw_accel / 1000;

        roll_q16  += roll_rate_q16 / 1000;
        pitch_q16 += pitch_rate_q16 / 1000;

        const int32_t PI_Q16 = 205887;
        while (roll_q16 > PI_Q16) roll_q16 -= PI_Q16 * 2;
        while (roll_q16 < -PI_Q16) roll_q16 += PI_Q16 * 2;
        while (pitch_q16 > PI_Q16) pitch_q16 -= PI_Q16 * 2;
        while (pitch_q16 < -PI_Q16) pitch_q16 += PI_Q16 * 2;

        /* --- Translation (hover-near-level Euler) ---
           Sign conventions (wired in drone_env P-controller):
             positive pitch → +x accel; positive roll → +y accel.
           Vertical: thrust surplus over hover (32768/motor average at level)
           drives climb; tilt projection reduces vertical thrust (cos ≈ 1
           for small angles — squared penalty only, near-level regime). */
        int32_t thrust_avg = (motor_thrust[0] + motor_thrust[1]
                            + motor_thrust[2] + motor_thrust[3]) / 4;
        const int32_t HOVER = 32768;
        const int32_t K_V = 65536;       /* 1.0 m/s² per unit thrust surplus */
        const int32_t K_H = 32768;       /* 0.5 m/s² per (thrust × rad tilt) */
        const int32_t DRAG_Z = 19661;    /* 0.3 /s */
        const int32_t DRAG_H = 13107;    /* 0.2 /s */

        int32_t az = q16_mul(thrust_avg - HOVER, K_V)
                   - q16_mul(vel_z, DRAG_Z);
        /* Small-angle: sin(θ) ≈ θ (rad, Q16.16). */
        int32_t ax = q16_mul(q16_mul(thrust_avg, pitch_q16), K_H)
                   - q16_mul(vel_x, DRAG_H);
        int32_t ay = q16_mul(q16_mul(thrust_avg, roll_q16), K_H)
                   - q16_mul(vel_y, DRAG_H);

        /* Periodic wind impulse (acceptance disturbance). */
        if (cfg.wind_period > 0 && tick_count > 0 &&
            (tick_count % cfg.wind_period) == 0) {
            vel_x += cfg.wind_impulse_q16 * wind_sign;
            wind_sign = -wind_sign;
        }

        vel_z += az / 1000;
        vel_x += ax / 1000;
        vel_y += ay / 1000;
        alt_q16 += vel_z / 1000;
        x_q16   += vel_x / 1000;
        y_q16   += vel_y / 1000;

        /* Soft bounds: keep translation in a sane envelope (no free-fall
           runaway if a motor dies — fault layer still gates on tilt). */
        const int32_t ALT_MAX = 655360;   /* 10 m */
        if (alt_q16 > ALT_MAX) { alt_q16 = ALT_MAX; if (vel_z > 0) vel_z = 0; }
        if (alt_q16 < -ALT_MAX) { alt_q16 = -ALT_MAX; if (vel_z < 0) vel_z = 0; }

        gyro_bias[0] += (int32_t)(prng.next() % 7) - 3;
        gyro_bias[1] += (int32_t)(prng.next() % 7) - 3;
        gyro_bias[2] += (int32_t)(prng.next() % 7) - 3;

        tick_count++;
    }

    void get_imu(int32_t* gyro_out) const {
        gyro_out[0] = roll_rate_q16 + gyro_bias[0]
                    + prng.noise_q16(cfg.noise_amp_q16);
        gyro_out[1] = pitch_rate_q16 + gyro_bias[1]
                    + prng.noise_q16(cfg.noise_amp_q16);
        gyro_out[2] = yaw_rate_q16 + gyro_bias[2]
                    + prng.noise_q16(cfg.noise_amp_q16);
    }

    bool check_fault() const {
        const int32_t TILT_LIMIT = 51472;
        return q16_abs(roll_q16) > TILT_LIMIT
            || q16_abs(pitch_q16) > TILT_LIMIT;
    }

    void fail_motor(int motor) {
        if (motor >= 0 && motor < 4) {
            motor_failed[motor] = 1;
            motor_thrust[motor] = 0;
            motor_target[motor] = 0;
        }
    }
};

/* Encode plant state into split-thermometer input words.
   Per-channel layout (32 bits):
     bits  0-14: thermometer of |value| when value >= 0
     bit   15:   sign  (0 = positive, 1 = negative)
     bits 16-30: thermometer of |value| when value < 0
     bit   31:   reserved (0)
   Channel indices 0-3 are identical to the original 128-bit frame. */
inline void drone_encode(const DronePlant& plant, uint32_t* frame, int bits) {
    const int ch = 32;
    std::memset(frame, 0, (size_t)(bits / 8));

    auto thermo_split = [&](int channel, int32_t value, int32_t max_val) {
        const int32_t a = q16_abs(value);
        const int offset = channel * ch;
        if (offset + 32 > bits) return;
        if (value >= 0) {
            for (int i = 0; i < 15; ++i) {
                int32_t thr = (int32_t)(((int64_t)(i + 1) * max_val) / 15);
                if (a > thr) {
                    frame[(offset + i) >> 5] |= 1u << ((offset + i) & 31);
                }
            }
        } else {
            frame[(offset + 15) >> 5] |= 1u << ((offset + 15) & 31);
            for (int i = 0; i < 15; ++i) {
                int32_t thr = (int32_t)(((int64_t)(i + 1) * max_val) / 15);
                if (a > thr) {
                    frame[(offset + 16 + i) >> 5] |= 1u << ((offset + 16 + i) & 31);
                }
            }
        }
    };

    const int32_t PI_Q16 = 205887;
    thermo_split(0, plant.roll_q16, PI_Q16);
    thermo_split(1, plant.pitch_q16, PI_Q16);
    const int32_t RATE_MAX = 65536;    /* ±1 rad/s — damping resolution */
    thermo_split(2, plant.roll_rate_q16, RATE_MAX);
    thermo_split(3, plant.pitch_rate_q16, RATE_MAX);
    /* Errors relative to station targets. Full-scale matches acceptance
       bands so in-band errors still light multiple thermometer bits. */
    const int32_t ALT_MAX = 32768;    /* ±0.5 m full-scale */
    const int32_t VZ_MAX  = 32768;    /* ±0.5 m/s */
    const int32_t XY_MAX  = 49152;    /* ±0.75 m — station vs attitude balance */
    const int32_t VH_MAX  = 32768;    /* ±0.5 m/s — strong braking gain */
    thermo_split(4, plant.alt_q16 - plant.cfg.target_alt_q16, ALT_MAX);
    thermo_split(5, plant.vel_z, VZ_MAX);
    thermo_split(6, plant.x_q16 - plant.cfg.target_x_q16, XY_MAX);
    thermo_split(7, plant.y_q16 - plant.cfg.target_y_q16, XY_MAX);
    thermo_split(8, plant.vel_x, VH_MAX);
    thermo_split(9, plant.vel_y, VH_MAX);
}

/* Reward vector, 3 components (ARC_N_REWARD_COMPONENTS).
   r[0] tracking: attitude + altitude + station errors (phase 0)
   r[1] effort:   -sum|cmd| (both phases)
   r[2] stability: near-level (and near-altitude) continuous measure (phase 1) */
inline void drone_reward_vec(const DronePlant& plant, const int32_t* cmd, int n_cmd,
                             int32_t* rv) {
    int32_t attitude_err = q16_abs(plant.roll_q16) + q16_abs(plant.pitch_q16)
                         + q16_abs(plant.yaw_rate_q16) / 4;
    const int32_t alt_err = plant.alt_q16 - plant.cfg.target_alt_q16;
    const int32_t x_err = plant.x_q16 - plant.cfg.target_x_q16;
    const int32_t y_err = plant.y_q16 - plant.cfg.target_y_q16;
    /* Station errors are metres (smaller scale than rad) — weight up. */
    rv[0] = -(attitude_err + q16_abs(alt_err)
              + 2 * (q16_abs(x_err) + q16_abs(y_err))) / 64;

    int32_t effort = 0;
    for (int i = 0; i < n_cmd; ++i) effort += q16_abs(cmd[i]);
    rv[1] = -effort / 256;

    const int32_t STABLE_LIMIT = 1134;
    const int32_t MAX_TILT = 65536;
    int32_t tilt = q16_abs(plant.roll_q16) + q16_abs(plant.pitch_q16);
    int32_t rv2;
    if (tilt < STABLE_LIMIT) {
        rv2 = 65536;
    } else if (tilt < MAX_TILT) {
        rv2 = 65536 - (tilt - STABLE_LIMIT) * 131072 / (MAX_TILT - STABLE_LIMIT);
    } else {
        rv2 = -65536;
    }
    /* Fold altitude into stability: outside ±1 band pulls the score down. */
    const int32_t at = q16_abs(alt_err);
    if (at > 65536) {
        rv2 -= (at - 65536) / 4;
        if (rv2 < -65536) rv2 = -65536;
    }
    rv[2] = rv2;
}

}  // namespace sim
}  // namespace arc

#ifdef _MSC_VER
#pragma warning(pop)
#endif
