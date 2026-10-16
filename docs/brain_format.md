# ARC001 Brain Format

## Overview

ARC001 is the frozen brain format for the Arc reflex runtime. It stores final ternary masks directly — no `ternary_thresh` reconstruction at load. Smaller, faster, unambiguous.

## Design Goals

1. Self-contained
2. Fixed layout, no padding
3. Little-endian, contractually
4. Validated at load
5. Versioned
6. Alignment-agnostic on read

## Layout

### Header (64 bytes)

| Offset | Size | Name | Description |
|--------|------|------|-------------|
| 0 | 8 | magic | "ARC001\0\0" |
| 8 | 4 | version | 1 |
| 12 | 4 | flags | 0 (reserved) |
| 16 | 4 | input_bits | Multiple of 32 |
| 20 | 4 | hidden_neurons | Multiple of 32 |
| 24 | 4 | motor_neurons | ≤ 32 |
| 28 | 4 | n_groups | ≤ 32 |
| 32 | 4 | input_words | input_bits / 32 |
| 36 | 4 | hidden_words | hidden_neurons / 32 |
| 40 | 4 | fire_threshold | Motor threshold |
| 44 | 4 | leak_q16 | Q16.16 leak rate |
| 48 | 20 | reserved | Zero |

### Body

Arrays stored sequentially, no padding:

| Array | Type | Dimensions | Size |
|-------|------|------------|------|
| aff_exc | u32 | [hidden_neurons][input_words] | 4 × hidden × input_words |
| aff_inh | u32 | [hidden_neurons][input_words] | 4 × hidden × input_words |
| rec_exc | u32 | [hidden_neurons][hidden_words] | 4 × hidden × hidden_words |
| rec_inh | u32 | [hidden_neurons][hidden_words] | 4 × hidden × hidden_words |
| threshold | i32 | [hidden_neurons] | 4 × hidden |
| groups | u8 | [motor_neurons] | motor_neurons |

### Mask Layout

Matches `CpuReservoirNet` from KHALM:

- `aff_exc[n * input_words + w]`: bit `j & 31` of word `j >> 5` is input line `j`'s excitatory weight for post neuron `n`
- `aff_inh[n * input_words + w]`: bit `j & 31` of word `j >> 5` is input line `j`'s inhibitory weight for post neuron `n`
- `rec_exc[n * hidden_words + w]`: bit `p & 31` of word `p >> 5` is pre neuron `p`'s excitatory weight for post neuron `n`
- `rec_inh[n * hidden_words + w]`: bit `p & 31` of word `p >> 5` is pre neuron `p`'s inhibitory weight for post neuron `n`

Motor neurons are indices `[0, motor_neurons)` in the hidden population.

## Endianness Contract

The format is **little-endian**. This is a contract with the writer, not a claim about the target.

- `flags` bit 0 (`ARC_FLAG_BODY_BYTESWAPPED`) is reserved for future use
- Current implementation: LE targets only (x86_64, aarch64, Cortex-M)
- Big-endian hosts unsupported — documented here

## Size Example

Typical reflex brain:
- `input_bits = 128`
- `hidden_neurons = 64`
- `motor_neurons = 8`
- `hidden_words = 2`

```
Header:                    64 bytes
aff_exc/inh: 64 × 4 × 2 =  512 bytes each
rec_exc/inh: 64 × 2 × 2 =  256 bytes each
threshold:   64 × 4    =  256 bytes
groups:       8 × 1    =    8 bytes
Total:                    ~1.8 KB
```

Fits on MCU with no problem.

## Validation (arc_load)

`arc_load` rejects:

1. Wrong magic (`"ARC001"`)
2. Wrong version (≠ 1)
3. Geometry violations:
   - `input_bits % 32 != 0`
   - `hidden_neurons % 32 != 0`
   - `motor_neurons > 32`
   - `n_groups > 32`
   - `input_words != input_bits / 32`
   - `hidden_words != hidden_neurons / 32`
4. `brain_size != 64 + body_size`
5. Any group ID ≥ `n_groups`
6. Unaligned arena (arena must be 4-byte aligned)

On rejection, `arc_last_error()` returns a specific message.

## Loading

```c
// 1. Read header, validate, get geometry and arena size
arc_geometry geom;
size_t arena_bytes;
arc_status s = arc_get_required_memory(header, 64, &geom, &arena_bytes);

// 2. Allocate arena (caller-supplied, 4-byte aligned)
void* arena = aligned_alloc(4, arena_bytes);

// 3. Load brain into arena
arc_engine* engine;
s = arc_load(brain_data, brain_size, arena, arena_bytes, &engine);

// 4. Use engine
arc_set_input(engine, frame);
arc_step(engine);
arc_get_motor(engine, motor_words);
```

The arena holds all mutable state (`V`, `spike`, `next`, `s_exc`, `s_inh`, `emitted`, `input`). No heap allocation after init.

## Conversion from KHALM003

Use `tools/brain_convert.cpp`:

```
brain_convert input.khalm003 output.arc001 \
    128 64 8 65 64881 [groups...]
```

- Reads KHALM003 accumulator block (afferent + recurrent)
- Applies ternary threshold (default 10) to generate masks
- Writes ARC001 with ternary masks directly
- No float math at load time

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 1 | 2026-09-17 | Initial release |
| 2 | 2026-09-20 | Plasticity: 128-byte header, plastic parameter block, `decoder_mode`, `w_phase`, `encoder_geometry_hash` (see below) |

---

# ARC002 Brain Format

The plastic-prior format. An ARC002 brain is a **prior** plus a **plasticity rule**: the runtime adapts the recurrent ternary masks online under the rule's bounds. An ARC001 brain loads as frozen (no plastic block → plasticity off, bit-identical to pre-pivot behavior).

## Header (128 bytes, little-endian)

| Offset | Size | Name | Notes |
|---|---|---|---|
| 0 | 8 | magic | `"ARC002\0\0"` |
| 8 | 4 | version | 2 |
| 12 | 4 | flags | 0 |
| 16–44 | — | geometry | Same fields/offsets as ARC001 |
| 48 | 1 | decoder_mode | 0 = DIRECT; 1/2 rejected in v1 runtime |
| 49 | 1 | n_phases | ≥ 1 |
| 50 | 2 | w_phase_count | must equal `n_phases * 3` |
| 52 | 8 | encoder_geometry_hash | pins encoder binding (new; ARC001 had none) |
| 60 | 4 | trace_decay_q16 | eligibility decay (Q16.16) |
| 64 | 4 | trace_threshold | raw count; flips need `\|E\| > threshold` |
| 68 | 4 | modulator_deadzone_q16 | reward dead zone (Q16.16) |
| 72 | 4 | modulator_tau_q16 | reward low-pass constant (Q16.16) |
| 76 | 4 | plasticity_interval | ticks between plasticity passes |
| 80 | 4 | max_hamming_fraction_q16 | drift ceiling (Q16.16) |
| 84 | 4 | revert_cooldown_ticks | plasticity frozen after revert |
| 88 | 4 | flip_cooldown_ticks | min ticks between flips of one synapse |
| 92 | 4 | stdp_window_ticks | causal timing window |
| 96 | 4 | beta_q16 | dual-component weight; must be 0 in v1 |
| 100 | 28 | reserved | zero |

## Body

```
w_phase   i32 [w_phase_count]    phase-major: [jte, effort, compliance] per phase
aff_exc   u32 [hidden * input_words]
aff_inh   u32 [hidden * input_words]
rec_exc   u32 [hidden * hidden_words]
rec_inh   u32 [hidden * hidden_words]
threshold i32 [hidden]
groups    u8  [motor_neurons]
```

Mask layout identical to ARC001. `rec_exc`/`rec_inh` are the plastic subset; `aff_*`, `threshold`, `groups` are frozen.

## Validation (ARC002)

All ARC001 geometry checks, plus:

- `decoder_mode == ARC_DECODE_DIRECT` (v1 runtime)
- `n_phases >= 1`
- `w_phase_count == n_phases * 3`
- `beta_q16 == 0`

## Arena (v2)

The engine struct now lives in the arena (`arc_load` returns `*out = arena`). Layout:

```
[arc_engine]
[aff_exc, aff_inh, rec_exc, rec_inh, threshold, groups]
[V, spike, next, s_exc, s_inh, emitted, input]
[arc_plasticity]                          (8-byte aligned)
[E traces: int16 [hidden*hidden]]
[pre_f, post_f: int16 [hidden] each]
[mod_state: i32, hamming: u32, cooldowns: u32[2]]
[flip_log ring: 512 entries x 16 bytes]
[w_phase copy: i32 [w_phase_count]]
```

`engine->plasticity` is `NULL` for frozen (ARC001) loads. `arc_get_required_memory` returns the exact v2 arena size.

## Determinism contract (amended per spec §16)

Trace decay is **batched**: pre/post filters decay per tick (O(hidden)); traces update only on the plasticity pass (`decay^N` precomputed in Q16.16, one multiply per synapse per pass). Same prior, same inputs, same reward stream → byte-identical learned weights on every platform.