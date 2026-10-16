#pragma once
/* Shared harness for plasticity unit tests: builds an ARC002 brain in memory,
   loads it through arc_load, and exposes the engine + plasticity state. */
#include "arc/arc.h"
#include "arc/arc_brain.h"
#include "arc/arc_plasticity.h"
#include "arc/arc_types.h"
#include <cstdint>
#include <cstring>
#include <vector>

namespace pltest {

inline void put_u32(std::vector<uint8_t>& b, size_t off, uint32_t v) {
    b[off] = (uint8_t)v; b[off + 1] = (uint8_t)(v >> 8);
    b[off + 2] = (uint8_t)(v >> 16); b[off + 3] = (uint8_t)(v >> 24);
}
inline void put_i32(std::vector<uint8_t>& b, size_t off, int32_t v) { put_u32(b, off, (uint32_t)v); }
inline void put_u64(std::vector<uint8_t>& b, size_t off, uint64_t v) {
    for (int i = 0; i < 8; ++i) b[off + i] = (uint8_t)(v >> (8 * i));
}

inline arc_plasticity_config default_cfg() {
    arc_plasticity_config c;
    std::memset(&c, 0, sizeof(c));
    c.decoder_mode = ARC_DECODE_DIRECT;
    c.n_phases = 1;
    c.w_phase_count = ARC_N_REWARD_COMPONENTS;
    c.encoder_geometry_hash = 0x1122334455667788ull;
    c.trace_decay_q16 = 60000;
    c.trace_threshold = 5;
    c.modulator_deadzone_q16 = 100;
    c.modulator_tau_q16 = 32768;
    c.plasticity_interval = 10;
    c.max_hamming_fraction_q16 = 6553;
    c.revert_cooldown_ticks = 50;
    c.flip_cooldown_ticks = 5;
    c.stdp_window_ticks = 100;
    c.beta_q16 = 0;
    return c;
}

inline void build_brain(std::vector<uint8_t>& brain, int input_bits, int hidden,
                        int motor, const arc_plasticity_config& cfg,
                        const std::vector<int32_t>& w_phase,
                        int excite = 0) {
    const int iw = input_bits / 32, hw = hidden / 32;
    const size_t aff = (size_t)hidden * iw;
    const size_t rec = (size_t)hidden * hw;
    const size_t body = (size_t)cfg.w_phase_count * 4 +
                        aff * 2 * 4 + rec * 2 * 4 + (size_t)hidden * 4 + (size_t)motor;

    brain.assign(ARC_HEADER_SIZE_V2 + body, 0);
    std::memcpy(brain.data(), "ARC002\0\0", 8);
    put_u32(brain, 8, 2);
    put_u32(brain, 16, (uint32_t)input_bits);
    put_u32(brain, 20, (uint32_t)hidden);
    put_u32(brain, 24, (uint32_t)motor);
    put_u32(brain, 28, 1);
    put_u32(brain, 32, (uint32_t)iw);
    put_u32(brain, 36, (uint32_t)hw);
    put_i32(brain, 40, 65);
    put_u32(brain, 44, 64881);
    brain[48] = cfg.decoder_mode;
    brain[49] = cfg.n_phases;
    brain[50] = (uint8_t)(cfg.w_phase_count & 0xFF);
    brain[51] = (uint8_t)((cfg.w_phase_count >> 8) & 0xFF);
    put_u64(brain, 52, cfg.encoder_geometry_hash);
    put_u32(brain, 60, cfg.trace_decay_q16);
    put_u32(brain, 64, cfg.trace_threshold);
    put_u32(brain, 68, cfg.modulator_deadzone_q16);
    put_u32(brain, 72, cfg.modulator_tau_q16);
    put_u32(brain, 76, cfg.plasticity_interval);
    put_u32(brain, 80, cfg.max_hamming_fraction_q16);
    put_u32(brain, 84, cfg.revert_cooldown_ticks);
    put_u32(brain, 88, cfg.flip_cooldown_ticks);
    put_u32(brain, 92, cfg.stdp_window_ticks);
    put_u32(brain, 96, cfg.beta_q16);
    for (size_t i = 0; i < w_phase.size(); ++i)
        put_i32(brain, ARC_HEADER_SIZE_V2 + i * 4, w_phase[i]);

    if (excite) {
        /* All-ones aff_exc: random input words drive afferent matches above
           the fire threshold, so the engine actually spikes in loop tests. */
        const size_t aff_off = ARC_HEADER_SIZE_V2 + (size_t)cfg.w_phase_count * 4;
        for (size_t i = 0; i < aff; ++i)
            put_u32(brain, aff_off + i * 4, 0xFFFFFFFFu);
    }
}

struct Harness {
    std::vector<uint8_t> brain;
    std::vector<uint8_t> arena;
    arc_engine* e = nullptr;
    arc_plasticity* p = nullptr;

    Harness(int input_bits, int hidden, int motor,
            const arc_plasticity_config& cfg,
            const std::vector<int32_t>& w_phase,
            int excite = 0) {
        build_brain(brain, input_bits, hidden, motor, cfg, w_phase, excite);
        arc_geometry geom;
        size_t need = 0;
        if (arc_get_required_memory(brain.data(), brain.size(), &geom, &need) != ARC_OK) return;
        arena.assign(need, 0);
        arc_load(brain.data(), brain.size(), arena.data(), arena.size(), &e);
        p = e ? e->plasticity : nullptr;
    }

    explicit operator bool() const { return e != nullptr && p != nullptr; }
};

/* Current ternary state of recurrent synapse (pre -> post). */
inline int rec_state(const arc_engine* e, int pre, int post) {
    const int n_w = e->geom.hidden_words;
    const uint32_t mask = 1u << (pre & 31);
    const size_t w = (size_t)post * n_w + (pre >> 5);
    if (e->rec_exc[w] & mask) return 1;
    if (e->rec_inh[w] & mask) return -1;
    return 0;
}

}  // namespace pltest
