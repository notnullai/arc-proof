#pragma once

#include "arc_types.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Decoder modes (load-time property; v1 runtime ships DIRECT only). */
#define ARC_DECODE_DIRECT   0
#define ARC_DECODE_RESIDUAL 1
#define ARC_DECODE_ALPHA    2

/* Reward vector components: [jte, effort, compliance]. */
#define ARC_N_REWARD_COMPONENTS 3

/* Bounded flip-log ring (Tier A: host-streamed). 512 entries x 16 B = 8 KB. */
#define ARC_FLIP_LOG_CAPACITY 512
#define ARC_FLIP_ENTRY_SIZE   16

/* --------------------------------------------------------------------------
 *  Plasticity configuration: the meta-learned rule, shipped in ARC002.
 *  Field order is for clean alignment; parsed field-by-field from the header.
 * ------------------------------------------------------------------------ */
typedef struct {
    uint64_t encoder_geometry_hash;  /* pins encoder binding; new in ARC002 */
    uint8_t  decoder_mode;           /* ARC_DECODE_DIRECT in v1 */
    uint8_t  n_phases;               /* >= 1 */
    uint16_t w_phase_count;          /* == n_phases * 3 */
    uint32_t trace_decay_q16;        /* eligibility decay (Q16.16) */
    uint32_t trace_threshold;        /* raw count; flips need |E| > this */
    uint32_t modulator_deadzone_q16; /* reward dead zone (Q16.16) */
    uint32_t modulator_tau_q16;      /* reward low-pass time constant */
    uint32_t plasticity_interval;    /* ticks between plasticity passes */
    uint32_t max_hamming_fraction_q16; /* drift ceiling (Q16.16) */
    uint32_t revert_cooldown_ticks;  /* plasticity frozen after revert */
    uint32_t flip_cooldown_ticks;    /* min ticks between flips of one synapse */
    uint32_t stdp_window_ticks;      /* causal timing window */
    uint32_t beta_q16;               /* dual-component weight; must be 0 in v1 */
    uint32_t reserved;               /* explicit pad */
} arc_plasticity_config;

/* --------------------------------------------------------------------------
 *  Reference-distribution monitor (spec section 7.3). The reference stats are
 *  commissioning artifacts supplied by the integrator; the rolling window
 *  lives in a small arena the integrator provides via set_monitor.
 * ------------------------------------------------------------------------ */
typedef struct {
    uint32_t reward_window;      /* rolling window length, ticks */
    uint32_t revert_consecutive; /* consecutive failed windows to revert */
    int32_t  z_critical_q16;     /* one-sided z, e.g. 1.645 */
    int32_t  sqrt_window_q16;    /* sqrt(reward_window) in Q16.16, precomputed */
} arc_monitor_config;

typedef struct arc_monitor_state {
    int32_t* window;             /* ring [reward_window], Q16.16 */
    uint32_t w_head;
    uint32_t w_count;
    int64_t  w_sum;
    uint32_t consecutive;        /* consecutive failed windows */
    uint32_t _pad;               /* explicit pad */
} arc_monitor_state;

/* --------------------------------------------------------------------------
 *  Runtime plasticity state, arena-resident. NULL engine->plasticity = frozen.
 * ------------------------------------------------------------------------ */
typedef struct arc_plasticity {
    arc_plasticity_config cfg;
    int32_t*  w_phase;          /* [w_phase_count], phase-major, arena copy */
    int32_t*  E;                /* eligibility traces [hidden*hidden], Q16.16 */
    int32_t*  pre_f;            /* pre-spike filters [hidden], Q16.16 */
    int32_t*  post_f;           /* post-spike filters [hidden], Q16.16 */
    int32_t*  mod_state;        /* modulator low-pass state (1), Q16.16 */
    uint32_t* hamming;          /* current Hamming distance from prior (1) */
    uint32_t* cooldowns;        /* [2]: revert remaining, reserved */
    uint8_t*  flip_log;         /* ring: ARC_FLIP_LOG_CAPACITY * 16 */
    uint32_t* last_flip;        /* per-synapse last-flip tick [hidden*hidden] */
    uint32_t* prior_exc;        /* prior rec mask copy [hidden*hidden_words] */
    uint32_t* prior_inh;        /* prior rec mask copy [hidden*hidden_words] */
    arc_monitor_state* mon;     /* NULL = monitor disabled */
    const int32_t* ref_mean_q16;/* [n_phases] commissioning means */
    const int32_t* ref_std_q16; /* [n_phases] commissioning stddevs */
    int32_t   hidden;           /* geometry cache */
    int32_t   decayN_q16;       /* trace_decay_q16 ^ plasticity_interval */
    arc_monitor_config mon_cfg;
    uint32_t  flip_count;       /* total flips since load */
    uint32_t  tick_accum;       /* ticks since last plasticity pass */
    uint32_t  revert_count;     /* reverts since load */
    uint32_t  last_revert_tick;
    uint8_t   phase;            /* latched phase tag */
    uint8_t   fault_gated;      /* set by the loop on safety fault: no flips */
    uint8_t   slices;           /* amortisation: 0/1 = whole pass on one tick (bit-exact default),
                                   n > 1 = hidden rows split over the first n ticks of each interval */
    uint8_t   slice_skip;       /* this interval's pass is gated (revert / cooldown) */
    uint8_t   _pad[4];          /* explicit pad */
} arc_plasticity;

/* 16-byte flip-log entry. */
typedef struct {
    uint64_t tick;
    uint32_t synapse_index;
    int8_t   from_state;
    int8_t   to_state;
    int8_t   modulator_sign;
    uint8_t  phase;
} arc_flip_entry;

/* --------------------------------------------------------------------------
 *  Plasticity API. All integer, deterministic, allocation-free.
 * ------------------------------------------------------------------------ */

/* Pure ternary transition: current state {-1,0,1}, eligibility sign, modulator
   sign -> next state. Strengthen: Neg->Zero->Pos; weaken: Pos->Zero->Neg. */
int arc_ternary_update(int cur, int e_sign, int m_sign);

/* Per tick: decay the per-neuron pre/post filters, accumulate spikes. */
void arc_plasticity_tick(arc_plasticity* p, const int8_t* spike);

/* Per tick (optional): ingest the reward vector, update the modulator filter
   and the latched phase. NULL reward = no-op. */
void arc_plasticity_reward(arc_plasticity* p, const int32_t* reward,
                           int n_components, uint8_t phase);

/* Call every tick; runs the batched plasticity pass every
   cfg.plasticity_interval ticks. Returns the number of flips applied. */
uint32_t arc_plasticity_step(arc_engine* e, uint32_t tick);

/* Amortised pass (runtime knob, not in the brain file): with slices n > 1
   the hidden rows of each plasticity pass are processed in n equal chunks on
   the first n ticks of every plasticity_interval, so the peak per-tick cost
   is ~1/n of the whole pass. 0 or 1 keeps the whole pass on the last tick
   of the interval, bit-exact with every golden. n is clamped to the
   interval. The modulator sign, Hamming bound and flip log are the same
   per chunk; each row still sees exactly one trace update per interval. */
arc_status arc_plasticity_set_slices(arc_plasticity* p, uint32_t slices);

/* Wire the reference-distribution monitor. ref_mean/ref_std are per-phase
   Q16.16 commissioning stats ([n_phases] each). arena holds the rolling
   window (reward_window entries). NULL refs or arena disables the monitor. */
arc_status arc_plasticity_set_monitor(arc_plasticity* p,
                                      const arc_monitor_config* cfg,
                                      const int32_t* ref_mean_q16,
                                      const int32_t* ref_std_q16,
                                      void* arena, size_t arena_size);

/* Revert: zero traces, flush modulator, restore prior weights, reset the
   monitor window, and start the revert cooldown. Returns the new
   revert_count. */
uint32_t arc_plasticity_revert(arc_engine* e, uint32_t tick);

/* Integer square root (Newton), for commissioning tools and tests. */
uint32_t arc_isqrt(uint32_t x);

#ifdef __cplusplus
}
#endif
