#pragma once
/* Differential test of arc_plasticity_step against its definition (gap register
   G8, faster bit-exact pass). The reference below is the word-level rule the
   runtime has shipped with: per post row and 32-synapse word, the traces are
   updated, the candidates are the synapses the update would move, and the
   word takes the unblocked rule when it cannot reach the Hamming ceiling
   (bound == 0 or popcount(candidates) < bound - ham) and the ceiling-aware
   rule otherwise; flips run in ascending order with the cooldown check, the
   trace reset, the last-flip tick and one flip-log entry each. Random states
   (traces, filters, spikes, weights, prior, last flips, distance near, at and
   past the ceiling, tiny ceilings, wrap, every decay form and threshold
   sign) are run through both, and every output is compared: traces, mask
   words, last flips, the flip log, the distance and the flip count. Shared by
   the host test and a Cortex-M image, so the ARMv7-M kernels are checked
   under QEMU too. No heap. */
#include "mcu_drone_seq.hpp"

namespace arc_equiv {

using namespace arc_mcu_drone;

enum { kH = kHidden, kM = kMotor, kNW = kHiddenWords, kSyn = kH * kH };

struct Rng {
    uint32_t s;
    uint32_t next() { s ^= s << 13; s ^= s >> 17; s ^= s << 5; return s; }
    uint32_t below(uint32_t n) { return next() % n; }
};

static inline int32_t ref_clamp(int64_t v) {
    const int64_t LIM = 0x3FFFFFFF;
    return (int32_t)(v > LIM ? LIM : v < -LIM ? -LIM : v);
}
static inline int32_t ref_q16(int32_t a, int32_t b) {
    const int64_t prod = (int64_t)a * (int64_t)b;
    return (int32_t)((prod + ((prod >> 63) & 0xFFFF)) >> 16);
}
static inline int ref_pc(uint32_t v) { int n = 0; while (v) { v &= v - 1; ++n; } return n; }

struct Snap {
    int32_t E[kSyn];
    uint32_t lf[kSyn];
    uint8_t log[ARC_FLIP_LOG_CAPACITY * ARC_FLIP_ENTRY_SIZE];
    uint32_t ex[kH * kNW], in[kH * kNW];
    uint32_t ham, fc;
};

static void snap_take(const arc_engine* e, Snap* s) {
    const arc_plasticity* p = e->plasticity;
    memcpy(s->E, p->E, sizeof(s->E));
    memcpy(s->lf, p->last_flip, sizeof(s->lf));
    memcpy(s->log, p->flip_log, sizeof(s->log));
    memcpy(s->ex, e->rec_exc, sizeof(s->ex));
    memcpy(s->in, e->rec_inh, sizeof(s->in));
    s->ham = *p->hamming;
    s->fc = p->flip_count;
}

/* The reference pass over rows kM..kH on the state in s (modified in place). */
static void ref_pass(const arc_engine* e, Snap* s, uint32_t tick) {
    const arc_plasticity* p = e->plasticity;
    const int32_t mod = *p->mod_state;
    const int32_t dz = (int32_t)p->cfg.modulator_deadzone_q16;
    const int m_sign = (mod > dz) ? 1 : (mod < -dz) ? -1 : 0;
    const int32_t th = (int32_t)((uint64_t)p->cfg.trace_threshold << 16);
    const uint32_t pn = (uint32_t)(kH - kM);
    const uint32_t bound = (uint32_t)(((uint64_t)p->cfg.max_hamming_fraction_q16 * (pn * pn)) >> 16);
    const uint32_t cd = p->cfg.flip_cooldown_ticks;
    const int learn = m_sign != 0 && !p->fault_gated;
    static const int8_t from_to[4][2] = {{1, 0}, {0, -1}, {-1, 0}, {0, 1}};
    for (int post = kM; post < kH; ++post) {
        const int sp = e->spike[post];
        const int32_t pf = p->post_f[post];
        for (int w = kM >> 5; w < kNW; ++w) {
            const int lo = (w << 5) > kM ? (w << 5) : kM;
            const int hi = (w << 5) + 32 < kH ? (w << 5) + 32 : kH;
            uint32_t pos = 0, neg = 0;
            for (int pre = lo; pre < hi; ++pre) {
                int32_t* E = &s->E[post * kH + pre];
                int32_t v = ref_clamp(ref_q16(*E, p->decayN_q16));
                v = ref_clamp((int64_t)v + (int64_t)p->pre_f[pre] * sp - (int64_t)pf * e->spike[pre]);
                *E = v;
                if (v > th) pos |= 1u << (pre & 31);
                else if (v < -th) neg |= 1u << (pre & 31);
            }
            if (!learn) continue;
            const size_t wi = (size_t)post * kNW + (size_t)w;
            uint32_t ex = s->ex[wi], in = s->in[wi];
            const uint32_t up = m_sign > 0 ? pos : neg, dn = m_sign > 0 ? neg : pos;
            const uint32_t in_s = in & ~ex;
            const uint32_t rem = (up & ~ex) | (dn & ~in_s);
            const uint32_t pe = p->prior_exc[wi], pi_s = p->prior_inh[wi] & ~pe;
            const uint32_t drift = (ex ^ pe) | (in_s ^ pi_s), zero = ~(ex | in);
            const uint32_t to_ex = up & zero, to_in = dn & zero;
            const uint32_t drift_to = (to_ex ^ pe) | (to_in ^ pi_s);
            const uint32_t h_inc = drift_to & ~drift, h_dec = drift & ~drift_to;
            const bool fast = bound == 0 || (s->ham < bound && (uint32_t)ref_pc(rem) < bound - s->ham);
            uint32_t done = 0;
            for (int bit = 0; bit < 32; ++bit) {
                const uint32_t m = 1u << bit;
                if (!(rem & m)) continue;
                if (!fast && s->ham >= bound && !(drift & m)) continue;     /* blocked */
                const int idx = post * kH + (w << 5) + bit;
                if (cd != 0 && tick - s->lf[idx] < cd) continue;
                done |= m;
                s->ham += ((h_inc >> bit) & 1u) - ((h_dec >> bit) & 1u);
                s->E[idx] = 0;
                s->lf[idx] = tick;
                uint8_t* fe = s->log + (size_t)(s->fc % ARC_FLIP_LOG_CAPACITY) * ARC_FLIP_ENTRY_SIZE;
                const uint64_t t64 = tick;
                memcpy(fe, &t64, 8);
                const uint32_t ui = (uint32_t)idx;
                memcpy(fe + 8, &ui, 4);
                const int k = (int)((((up >> bit) & 1u) << 1) | ((zero >> bit) & 1u));
                fe[12] = (uint8_t)from_to[k][0]; fe[13] = (uint8_t)from_to[k][1];
                fe[14] = (uint8_t)(int8_t)m_sign; fe[15] = p->phase;
                ++s->fc;
            }
            s->ex[wi] = (ex & ~done) | (to_ex & done);
            s->in[wi] = (in & ~done) | (to_in & done);
        }
    }
}

static int32_t rnd_trace(Rng& r, int32_t th) {
    switch (r.below(4)) {
    case 0:  return (int32_t)(r.next() % 0x7FFFFFFEu) - 0x3FFFFFFF;   /* anywhere */
    case 1:  return th + (int32_t)r.below(4000) - 2000;              /* near +th */
    case 2:  return -th + (int32_t)r.below(4000) - 2000;             /* near -th */
    default: return (int32_t)r.below(8) - 4;
    }
}

/* Runs `trials` random passes; returns the number that differ. */
static uint32_t run(uint8_t* brain, uint8_t* arena, uint32_t trials, uint32_t seed,
                    Snap* ref, Snap* got) {
    build_brain(brain);
    arc_engine* e = NULL;
    if (arc_load(brain, kBrainBytes, arena, kEngineArena, &e) != ARC_OK) return 0xFFFFFFFFu;
    arc_plasticity* p = e->plasticity;
    Rng r{seed};
    uint32_t bad = 0;
    const int32_t decays[5] = {p->decayN_q16, 32767, 40000, 65536, -20000};
    for (uint32_t t = 0; t < trials; ++t) {
        p->cfg.trace_threshold = r.below(8) == 0 ? 40000u + r.below(20000) : 1u + r.below(6);
        const int32_t th = (int32_t)((uint64_t)p->cfg.trace_threshold << 16);
        p->decayN_q16 = decays[r.below(5)];
        static const uint32_t fracs[5] = {0, 6553, 300, 16, 65535};
        p->cfg.max_hamming_fraction_q16 = fracs[r.below(5)];
        p->cfg.flip_cooldown_ticks = r.below(3) == 0 ? 0 : r.below(12);
        p->phase = (uint8_t)r.below(3);
        const uint32_t pn = (uint32_t)(kH - kM);
        const uint32_t bound = (uint32_t)(((uint64_t)p->cfg.max_hamming_fraction_q16 * (pn * pn)) >> 16);
        const uint32_t tick = 1000u + r.below(100000);
        for (int i = 0; i < kSyn; ++i) {
            p->E[i] = rnd_trace(r, th);
            p->last_flip[i] = tick - r.below(14);
        }
        for (int n = 0; n < kH; ++n) {
            e->spike[n] = (int8_t)((int)r.below(3) - 1);
            p->pre_f[n] = (int32_t)r.below(1600000) - 800000;
            p->post_f[n] = p->pre_f[n];
        }
        const uint32_t density = r.below(4);
        for (int i = 0; i < kH * kNW; ++i) {
            uint32_t a = r.next(), b = r.next();
            if (density == 0) { a &= r.next(); b &= r.next(); }
            ((uint32_t*)e->rec_exc)[i] = a;
            ((uint32_t*)e->rec_inh)[i] = b;
            p->prior_exc[i] = r.below(2) ? a : r.next();
            p->prior_inh[i] = r.below(2) ? b : r.next();
        }
        switch (r.below(6)) {
        case 0:  *p->hamming = r.below(40); break;                         /* can wrap */
        case 1:  *p->hamming = bound + r.below(4); break;                  /* at / past */
        case 2:  *p->hamming = bound > 40 ? bound - r.below(40) : r.below(40); break;
        case 3:  *p->hamming = r.next(); break;
        default: *p->hamming = r.below(bound + 1 + 64); break;
        }
        static const int32_t mods[4] = {65536, -65536, 0, 99};
        *p->mod_state = mods[r.below(4)];
        p->fault_gated = r.below(10) == 0;
        p->flip_count = r.next();
        for (int i = 0; i < ARC_FLIP_LOG_CAPACITY * ARC_FLIP_ENTRY_SIZE; ++i) p->flip_log[i] = (uint8_t)r.next();

        snap_take(e, ref);
        ref_pass(e, ref, tick);
        p->slices = 0;
        p->tick_accum = p->cfg.plasticity_interval - 1;
        p->cooldowns[0] = 0;
        arc_plasticity_step(e, tick);
        snap_take(e, got);
        if (memcmp(ref, got, sizeof(Snap)) != 0) ++bad;
    }
    return bad;
}

}  // namespace arc_equiv
