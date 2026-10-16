#pragma once
/* Drone env for Arc: ARC002 brain build (P-controller afferents over the
   256-bit 8-channel frame), harness load, and episode instrumentation
   shared by the full-loop test. Mirrors train/sim/joint_env.hpp —
   plant/encode/reward stay free functions in drone_plant.hpp. */
#include "drone_plant.hpp"
#include "arc/arc.h"
#include "arc/arc_brain.h"
#include "arc/arc_plasticity.h"
#include "arc/arc_types.h"
#include <cstdint>
#include <cstring>
#include <vector>

namespace arc {
namespace sim {

inline void dput_u32(std::vector<uint8_t>& b, size_t off, uint32_t v) {
    b[off] = (uint8_t)v;
    b[off + 1] = (uint8_t)(v >> 8);
    b[off + 2] = (uint8_t)(v >> 16);
    b[off + 3] = (uint8_t)(v >> 24);
}
inline void dput_i32(std::vector<uint8_t>& b, size_t off, int32_t v) {
    dput_u32(b, off, (uint32_t)v);
}
inline void dput_u64(std::vector<uint8_t>& b, size_t off, uint64_t v) {
    dput_u32(b, off, (uint32_t)v);
    dput_u32(b, off + 4, (uint32_t)(v >> 32));
}

inline uint64_t drone_fnv1a(uint64_t h, uint32_t v) {
    for (int b = 0; b < 4; ++b) {
        h ^= (uint8_t)(v >> (8 * b));
        h *= 1099511628211ULL;
    }
    return h;
}

/* P-controller brain over the 320-bit frame (see drone_plant encode).
   Input layout:
     ch0 roll 0-31, ch1 pitch 32-63, ch2 roll_rate 64-95, ch3 pitch_rate 96-127
     ch4 alt_err 128-159, ch5 vel_z 160-191, ch6 x_err 192-223, ch7 y_err 224-255
     ch8 vel_x 256-287, ch9 vel_y 288-319

   X-frame mixing:
     roll_torque  = m3 - m1  → m1 for -roll,  m3 for +roll
     pitch_torque = m2 - m0  → m0 for -pitch, m2 for +pitch

   Sign convention (plant translation): +pitch → +x accel, +roll → +y accel.
   Station is excite-only (no cross-inhibit): with fire_threshold=1 a station
   inhibit on the opposite motor cancels attitude drive 1:1 and lets position
   error flip the aircraft past the tilt fault. Attitude keeps full
   excite+inhibit P; station only biases which motor fires more often.
     x_err>0 or vel_x>0 → need pitch<0 → m0 excite x_pos, vel_x_pos
     y_err>0 or vel_y>0 → need roll<0 → m1 excite y_pos, vel_y_pos
   Altitude: alt_err<0 (below) or vel_z<0 (falling) → all motors up;
   the inverse uses alt_pos/vz_pos inhibit (symmetric, same on all four). */
struct DroneBrain {
    std::vector<uint8_t> brain;
    DroneConfig dcfg;
    arc_plasticity_config pcfg;

    DroneBrain() = default;

    explicit DroneBrain(const DroneConfig& c) : dcfg(c), pcfg{} { build(); }

    /* Tuned plasticity matching the Step 1a/2 drone episode, calmed for the
       stronger station PD (trace_threshold/interval stop rapid wiring
       corruption from reaching the tilt fault mid-episode). */
    void build_default_pcfg() {
        std::memset(&pcfg, 0, sizeof(pcfg));
        pcfg.decoder_mode = ARC_DECODE_DIRECT;
        pcfg.n_phases = 2;
        pcfg.w_phase_count = 6;
        pcfg.encoder_geometry_hash = drone_encoder_geometry_hash(dcfg.input_bits);
        pcfg.trace_decay_q16 = 58982;
        pcfg.trace_threshold = 0;
        pcfg.modulator_deadzone_q16 = 1024;
        pcfg.modulator_tau_q16 = 32768;
        pcfg.plasticity_interval = 10;
        pcfg.max_hamming_fraction_q16 = 13107;  /* 20% — leave headroom */
        pcfg.revert_cooldown_ticks = 50;
        pcfg.flip_cooldown_ticks = 50;
        pcfg.stdp_window_ticks = 100;
        pcfg.beta_q16 = 0;
    }

    void build() {
        if (pcfg.w_phase_count == 0) build_default_pcfg();

        const int iw = dcfg.input_words;
        const int hw = dcfg.hidden_words;
        const int aff = dcfg.hidden_neurons * iw;
        const int rec = dcfg.hidden_neurons * hw;
        const int n_wp = 6;
        const int32_t w_phase[6] = {65536, 16384, 0, 0, 16384, 65536};

        const size_t body = (size_t)n_wp * 4
                          + (size_t)aff * 2 * 4
                          + (size_t)rec * 2 * 4
                          + (size_t)dcfg.hidden_neurons * 4
                          + (size_t)dcfg.motor_neurons;

        brain.assign(ARC_HEADER_SIZE_V2 + body, 0);
        std::memcpy(brain.data(), "ARC002\0\0", 8);
        dput_u32(brain, 8, 2);
        dput_u32(brain, 16, (uint32_t)dcfg.input_bits);
        dput_u32(brain, 20, (uint32_t)dcfg.hidden_neurons);
        dput_u32(brain, 24, (uint32_t)dcfg.motor_neurons);
        dput_u32(brain, 28, (uint32_t)dcfg.n_groups);
        dput_u32(brain, 32, (uint32_t)iw);
        dput_u32(brain, 36, (uint32_t)hw);
        dput_i32(brain, 40, dcfg.fire_threshold);
        dput_u32(brain, 44, dcfg.leak_q16);
        brain[48] = (uint8_t)pcfg.decoder_mode;
        brain[49] = pcfg.n_phases;
        brain[50] = (uint8_t)(pcfg.w_phase_count & 0xFF);
        brain[51] = (uint8_t)((pcfg.w_phase_count >> 8) & 0xFF);
        dput_u64(brain, 52, pcfg.encoder_geometry_hash);
        dput_u32(brain, 60, pcfg.trace_decay_q16);
        dput_u32(brain, 64, pcfg.trace_threshold);
        dput_u32(brain, 68, pcfg.modulator_deadzone_q16);
        dput_u32(brain, 72, pcfg.modulator_tau_q16);
        dput_u32(brain, 76, pcfg.plasticity_interval);
        dput_u32(brain, 80, pcfg.max_hamming_fraction_q16);
        dput_u32(brain, 84, pcfg.revert_cooldown_ticks);
        dput_u32(brain, 88, pcfg.flip_cooldown_ticks);
        dput_u32(brain, 92, pcfg.stdp_window_ticks);
        dput_u32(brain, 96, pcfg.beta_q16);

        const size_t wp_off = ARC_HEADER_SIZE_V2;
        for (int i = 0; i < n_wp; ++i)
            dput_i32(brain, wp_off + (size_t)i * 4, w_phase[i]);

        std::vector<uint32_t> aff_exc((size_t)aff, 0), aff_inh((size_t)aff, 0);
        std::vector<uint32_t> rec_exc((size_t)rec, 0), rec_inh((size_t)rec, 0);

        auto set_bit = [](std::vector<uint32_t>& v, int row, int words, int bit) {
            v[(size_t)row * words + (bit >> 5)] |= 1u << (bit & 31);
        };
        const int ROLL = 0, PITCH = 32, RRATE = 64, PRATE = 96;
        const int ALT = 128, VZ = 160;
        const int XE = 192, YE = 224, VX = 256, VY = 288;

        /* Motor 0: -pitch torque (fires -> m0 up -> pitch down).
           Attitude: excite pitch_pos, inhibit pitch_sign.
           Rate damp: +pitch_rate -> want -pitch torque -> excite.
           Station (excite-only): x_err>0 / vel_x>0 -> want pitch<0.
           Altitude: below/falling -> up; above/rising -> down (all motors). */
        for (int b = PITCH; b < PITCH + 15; ++b) set_bit(aff_exc, 0, iw, b);
        set_bit(aff_inh, 0, iw, PITCH + 15);
        for (int b = PRATE; b < PRATE + 15; ++b) set_bit(aff_exc, 0, iw, b);
        for (int b = PRATE + 16; b < PRATE + 31; ++b) set_bit(aff_inh, 0, iw, b);
        for (int b = XE; b < XE + 15; ++b) set_bit(aff_exc, 0, iw, b);
        for (int b = VX; b < VX + 15; ++b) set_bit(aff_exc, 0, iw, b);
        set_bit(aff_exc, 0, iw, ALT + 15);
        for (int b = ALT + 16; b < ALT + 31; ++b) set_bit(aff_exc, 0, iw, b);
        set_bit(aff_inh, 0, iw, ALT + 14);
        for (int b = ALT; b < ALT + 15; ++b) set_bit(aff_inh, 0, iw, b);
        set_bit(aff_exc, 0, iw, VZ + 15);
        for (int b = VZ + 16; b < VZ + 31; ++b) set_bit(aff_exc, 0, iw, b);
        for (int b = VZ; b < VZ + 15; ++b) set_bit(aff_inh, 0, iw, b);

        /* Motor 1: -roll torque (fires -> roll down).
           Attitude: excite roll_pos, inhibit roll_sign.
           Rate damp: +roll_rate -> want -roll torque -> excite.
           Station: y_err>0 / vel_y>0 -> want roll<0. */
        for (int b = ROLL; b < ROLL + 15; ++b) set_bit(aff_exc, 1, iw, b);
        set_bit(aff_inh, 1, iw, ROLL + 15);
        for (int b = RRATE; b < RRATE + 15; ++b) set_bit(aff_exc, 1, iw, b);
        for (int b = RRATE + 16; b < RRATE + 31; ++b) set_bit(aff_inh, 1, iw, b);
        for (int b = YE; b < YE + 15; ++b) set_bit(aff_exc, 1, iw, b);
        for (int b = VY; b < VY + 15; ++b) set_bit(aff_exc, 1, iw, b);
        set_bit(aff_exc, 1, iw, ALT + 15);
        for (int b = ALT + 16; b < ALT + 31; ++b) set_bit(aff_exc, 1, iw, b);
        set_bit(aff_inh, 1, iw, ALT + 14);
        for (int b = ALT; b < ALT + 15; ++b) set_bit(aff_inh, 1, iw, b);
        set_bit(aff_exc, 1, iw, VZ + 15);
        for (int b = VZ + 16; b < VZ + 31; ++b) set_bit(aff_exc, 1, iw, b);
        for (int b = VZ; b < VZ + 15; ++b) set_bit(aff_inh, 1, iw, b);

        /* Motor 2: +pitch torque (fires -> pitch up).
           Attitude: excite pitch_sign+neg, inhibit pitch_pos.
           Rate damp: -pitch_rate -> want +pitch torque -> excite.
           Station: x_err<0 / vel_x<0 -> want pitch>0. */
        set_bit(aff_exc, 2, iw, PITCH + 15);
        for (int b = PITCH + 16; b < PITCH + 31; ++b) set_bit(aff_exc, 2, iw, b);
        for (int b = PITCH; b < PITCH + 15; ++b) set_bit(aff_inh, 2, iw, b);
        set_bit(aff_exc, 2, iw, PRATE + 15);
        for (int b = PRATE + 16; b < PRATE + 31; ++b) set_bit(aff_exc, 2, iw, b);
        for (int b = PRATE; b < PRATE + 15; ++b) set_bit(aff_inh, 2, iw, b);
        for (int b = XE + 16; b < XE + 31; ++b) set_bit(aff_exc, 2, iw, b);
        for (int b = VX + 16; b < VX + 31; ++b) set_bit(aff_exc, 2, iw, b);
        set_bit(aff_exc, 2, iw, ALT + 15);
        for (int b = ALT + 16; b < ALT + 31; ++b) set_bit(aff_exc, 2, iw, b);
        set_bit(aff_inh, 2, iw, ALT + 14);
        for (int b = ALT; b < ALT + 15; ++b) set_bit(aff_inh, 2, iw, b);
        set_bit(aff_exc, 2, iw, VZ + 15);
        for (int b = VZ + 16; b < VZ + 31; ++b) set_bit(aff_exc, 2, iw, b);
        for (int b = VZ; b < VZ + 15; ++b) set_bit(aff_inh, 2, iw, b);

        /* Motor 3: +roll torque (fires -> roll up).
           Attitude: excite roll_sign+neg, inhibit roll_pos.
           Rate damp: -roll_rate -> want +roll torque -> excite.
           Station: y_err<0 / vel_y<0 -> want roll>0. */
        set_bit(aff_exc, 3, iw, ROLL + 15);
        for (int b = ROLL + 16; b < ROLL + 31; ++b) set_bit(aff_exc, 3, iw, b);
        for (int b = ROLL; b < ROLL + 15; ++b) set_bit(aff_inh, 3, iw, b);
        set_bit(aff_exc, 3, iw, RRATE + 15);
        for (int b = RRATE + 16; b < RRATE + 31; ++b) set_bit(aff_exc, 3, iw, b);
        for (int b = RRATE; b < RRATE + 15; ++b) set_bit(aff_inh, 3, iw, b);
        for (int b = YE + 16; b < YE + 31; ++b) set_bit(aff_exc, 3, iw, b);
        for (int b = VY + 16; b < VY + 31; ++b) set_bit(aff_exc, 3, iw, b);
        set_bit(aff_exc, 3, iw, ALT + 15);
        for (int b = ALT + 16; b < ALT + 31; ++b) set_bit(aff_exc, 3, iw, b);
        set_bit(aff_inh, 3, iw, ALT + 14);
        for (int b = ALT; b < ALT + 15; ++b) set_bit(aff_inh, 3, iw, b);
        set_bit(aff_exc, 3, iw, VZ + 15);
        for (int b = VZ + 16; b < VZ + 31; ++b) set_bit(aff_exc, 3, iw, b);
        for (int b = VZ; b < VZ + 15; ++b) set_bit(aff_inh, 3, iw, b);
        /* Hidden: heterogeneous dense afferents over the full frame. */
        for (int n = 4; n < dcfg.hidden_neurons; ++n) {
            for (int k = 0; k < 64; ++k) {
                const int j = (n * 3 + k * 5) % dcfg.input_bits;
                set_bit(aff_exc, n, iw, j);
            }
        }

        /* Recurrent: hidden-to-hidden only. */
        for (int n = 4; n < dcfg.hidden_neurons; ++n) {
            for (int k = 0; k < 3; ++k) {
                int target = 4 + ((n - 4) * 3 + k * 11) % (dcfg.hidden_neurons - 4);
                set_bit(rec_exc, target, hw, n);
            }
        }

        const size_t aff_off = wp_off + (size_t)n_wp * 4;
        for (size_t i = 0; i < (size_t)aff; ++i) {
            dput_u32(brain, aff_off + i * 4, aff_exc[i]);
            dput_u32(brain, aff_off + ((size_t)aff + i) * 4, aff_inh[i]);
        }
        const size_t rec_off = aff_off + (size_t)aff * 2 * 4;
        for (size_t i = 0; i < (size_t)rec; ++i) {
            dput_u32(brain, rec_off + i * 4, rec_exc[i]);
            dput_u32(brain, rec_off + ((size_t)rec + i) * 4, rec_inh[i]);
        }
        const size_t thr_off = rec_off + (size_t)rec * 2 * 4;
        for (int i = 0; i < dcfg.hidden_neurons; ++i) {
            dput_i32(brain, thr_off + (size_t)i * 4,
                     (i < dcfg.motor_neurons) ? dcfg.fire_threshold : 6);
        }
        const size_t grp_off = thr_off + (size_t)dcfg.hidden_neurons * 4;
        for (int i = 0; i < dcfg.motor_neurons; ++i)
            brain[grp_off + (size_t)i] = (uint8_t)i;
    }
};

struct DroneHarness {
    std::vector<uint8_t> arena;
    arc_engine* e = nullptr;
    arc_plasticity* p = nullptr;

    explicit DroneHarness(const DroneBrain& bb) {
        arc_geometry geom;
        size_t need = 0;
        if (arc_get_required_memory(bb.brain.data(), bb.brain.size(),
                                    &geom, &need) != ARC_OK) return;
        arena.assign(need, 0);
        if (arc_load(bb.brain.data(), bb.brain.size(),
                     arena.data(), arena.size(), &e) != ARC_OK) {
            e = nullptr;
            return;
        }
        p = e->plasticity;
    }
    explicit operator bool() const { return e && p; }
};

}  // namespace sim
}  // namespace arc
