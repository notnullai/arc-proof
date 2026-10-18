# Arc — Performance and Footprint

Measured numbers for the binary-only engine shipped in this repository
(built from the private Arc repo at commit `fba7469`). Every row is measured, none estimated; rows marked private repo are
reported here and reproducible only from that repo.

## Tick time (host, x86_64)

| Metric | Value | Notes |
|---|---|---|
| Average tick | 0.40 – 1.2 µs | varies with host load; `test_tick_budget` (private repo), 100k ticks |
| Worst tick observed | 26.8 µs | scheduler noise on a shared host, not engine work |
| Budget | 1000 µs | `tick_budget_us` in the safety config; the test fails above it |

`test_tick_budget` lives in the private repo and is not part of this
proof suite; it is excluded from its CI on windows-msvc and linux-arm64
(shared-runner scheduler noise). The number that matters for the robot
is the on-target DWT cycle-count, below.

## On-target tick time (Cortex-M7, Teensy 4.x)

**Restated 2026-10-18:** the first two rows below were measured on the
timing twin's fixed 64-input × 64-hidden brain, not a shipped prior. The
shipped x500 priors are 96 hidden with 480–832 input bits; the tick cost is
word-level popcounts over hidden × input words, so they do 4–6× the
inference work. At the real geometries (same sequence and harness,
`ci/run_mcu_timing.sh <target> mps2-an500|mps2-an385`, QEMU icount,
1 instruction = 1 ns):

| geometry (input / hidden) | prior | mode | Cortex-M7 worst tick | Cortex-M3 worst tick | lowest clock for 1 kHz at 1 / 2 cycles per instruction | target |
|---|---|---|---|---|---|---|
| 832 / 96 | **flight default** (compiled cascade) | inference, plasticity off (how it flies) | **125,200** | 125,280 | **125 / 250 MHz** | `targets/mcu_drone_loop_timing_x500_832_inference_m7` / `_m3` |
| 832 / 96 | flight default | inference, dense-input stress (hidden layer firing) | 125,240 | 125,240 | 126 / 251 MHz | `targets/mcu_drone_loop_timing_x500_832_inference_dense_m7` / `_m3` |
| 480 / 96 | learning demonstration (trained) | inference | 86,120 | 86,200 | 86 / 172 MHz | `targets/mcu_drone_loop_timing_x500_480_inference_m7` / `_m3` |
| 480 / 96 | learning demonstration | inference, dense-input stress | 86,080 | 86,080 | 87 / 173 MHz | `targets/mcu_drone_loop_timing_x500_480_inference_dense_m7` / `_m3` |
| 480 / 96 | learning demonstration | plasticity on, **dense-input stress** | **408,760** | 411,520 | 409 / 818 MHz | `targets/mcu_drone_loop_timing_x500_480_dense_m7` / `_m3` |
| 480 / 96 | learning demonstration | plasticity on, **dense-input stress, amortised over 10 ticks** | **138,640** | 139,120 | **139 / 278 MHz** | `targets/mcu_drone_loop_timing_x500_480_sliced_dense_m7` / `_m3` |
| 480 / 96 | learning demonstration | plasticity on, plain twin (traces stay zero) | 331,480 | 331,040 | – | `targets/mcu_drone_loop_timing_x500_480_m7` / `_m3` |
| 480 / 96 | learning demonstration | plasticity on, plain twin, amortised | 114,400 | 114,480 | – | `targets/mcu_drone_loop_timing_x500_480_sliced_m7` / `_m3` |
| 64 / 64 | synthetic golden twin (not a shipped prior) | plasticity on / amortised | 127,920 / 33,680 | 127,560 / 33,720 | – | `targets/mcu_drone_loop_timing_m7`, `_sliced_m7` |

The timing images run a fixed brain at the shipped geometry, not the shipped weights. For inference that times the same work: `arc_core`'s inference loops over every hidden × input word and every hidden × hidden word unconditionally (popcounts), and the only data-dependent work is each neuron's threshold compare and the winner-take-all bookkeeping for the motor neurons (4 here) that spike. The dense-input rows confirm it: with 71 % of the hidden layer firing each tick, the flight default's worst tick is 125,240, against 125,200 on the plain twin.

**The plasticity pass is data-dependent, and the plain twin understated it (found 2026-10-18).** The plain twin feeds one input bit per tick against a hidden threshold of 2, so its hidden layer never fires. Every trace stays zero and nothing flips, so its plasticity-on rows only time the pass decaying zeros. The dense-input stress twins feed pseudo-random input words. Under that stress, 58 % of synapse visits cross the trace threshold and about 285 synapses flip per pass. They carry their own hashes (480 plain 0x06d80bb3fbefac89, amortised 0x83e4d89dc5e02f94, inference 0xd09e9fba89e283cf; 832 inference 0x628fde62eca6d3f1). On the old pass they cost 927,600 instructions, or 191,600 amortised; the earlier published 496,760 / 132,360 were the plain twin. The pass was then rewritten without changing a result (every golden hash unchanged). The rewrite keeps loop invariants in registers and makes the skips word-level and bit-parallel against the 32-synapse mask words; flips stay in ascending order. It also specialises the trace loop on the post spike and uses the ARMv7-M SMLAL and SSAT instructions, which the M3 has too. **1 kHz with learning on (amortised, dense stress) needs ≥ 139 MHz at 1 cycle/instr, ≥ 278 MHz at 2** (600 MHz M7: 4.3× / 2.2× margin). Not claimed: an analytic worst case. The stress twin is a measurement, and a pass that flips every one of the 8,464 hidden↔hidden synapses has not been timed (gap register G8).

For what flies, 125,200 instructions fits 1 kHz on a 600 MHz Cortex-M7
with 4.8× margin at 1 cycle/instr and 2.4× at 2; a ≤ 120 MHz Cortex-M3
does not fit 1 kHz. The earlier "~85 MHz amortised" figure held for the
synthetic twin only.

Synthetic twin (kept for the record):

| Metric | Value | Notes |
|---|---|---|
| Instruction count per tick under QEMU `-icount` (Cortex-M7 build, `targets/mcu_drone_loop_timing_m7`, same 2000-tick sequence as the golden twin, hash matches) | p50 **23,040** instr; every 10th tick (plasticity pass) **197,440** instr; mean 30,440 | `ci/run_mcu_timing.sh targets/mcu_drone_loop_timing_m7 mps2-an500` — deterministic; Cortex-M3 build: 23,120 / 197,520 |
| Same twin with the plasticity pass amortised over 10 ticks (`targets/mcu_drone_loop_timing_sliced_m7`, runtime knob in the engine; the default build above is unchanged and bit-exact) | p50 40,560 instr; **worst tick 40,640**, no heavy ticks; mean 30,548 | `ci/run_mcu_timing.sh targets/mcu_drone_loop_timing_sliced_m7 mps2-an500` — fits 1 kHz at 2 cycles/instr down to ~85 MHz; Cortex-M3 build (`_sliced_m3`): 40,640 / 40,720 |
| Derived tick time at 600 MHz | normal tick 38–77 µs, plasticity tick 329–658 µs (1–2 cycles/instr) | inside the 1000 µs budget with the plasticity pass as the dominant term; the cycles-per-instruction factor is what silicon adds |
| DWT cycle-counted tick min/avg/max | **hardware day** | no silicon run yet; the on-target binary (`targets/teensy4_teensy_drone_loop.hex`) prints it over UART on first boot |
| QEMU tick time | not representative | instruction-set emulation, no cycle accuracy |

## Flash and RAM (arm-none-eabi, Cortex-M7 soft-float, `-Os`)

`arm-none-eabi-size` on the committed on-target ELFs:

| Target | text | data | bss | Flash (text+data) | RAM (data+bss) |
|---|---|---|---|---|---|
| `teensy_drone_loop` (board image, ships as `.hex`) | 24668 | 368 | 69924 | 25.0 KB | 68.6 KB |
| `teensy_robot` (wheeled board image, private repo, not shipped here) | 22860 | 332 | 70036 | 22.7 KB | 68.7 KB |
| `targets/mcu_drone_loop_smoke_m7` (QEMU smoke, in repo) | 13353 | 320 | 69284 | 13.4 KB | 67.8 KB |
| `targets/mcu_drone_loop_smoke_m3` (QEMU smoke, in repo) | 13169 | 320 | 69284 | 13.2 KB | 67.8 KB |

The bulk of `bss` is the caller-supplied static engine arena (the board
images allocate 70 KB). The engine library itself:

| Library | Size |
|---|---|
| `lib/armv7e-m-soft/libarc_runtime.a` | 39668 bytes |
| `lib/armv7e-m-hard/libarc_runtime.a` | 39692 bytes |
| `lib/x64-linux/libarc_runtime.a` | 54 KB |
| `lib/x64-windows/arc_runtime.lib` | 113 KB |

## Arena and model sizes

`arc_get_required_memory(header, ...)` returns the exact engine arena
size (linear in `input_words`/`hidden_neurons`; no manual math). The
committed priors:

| Model | File | Geometry |
|---|---|---|
| `models/prior_car.arc002` | 5902 bytes | 128 × 96 × 2 |
| `models/prior_servo.arc002` | 2446 bytes | 64 × 64 × 2 |

## Reproduce

```
# flash/RAM of the committed QEMU smoke ELFs (any arm-none-eabi toolchain)
arm-none-eabi-size targets/mcu_drone_loop_smoke_m3 targets/mcu_drone_loop_smoke_m7

# library sizes
ls -l lib/armv7e-m-soft/libarc_runtime.a lib/armv7e-m-hard/libarc_runtime.a
```

The two board-image rows come from the private repo's M7 cross build
(the same commit that produced `targets/teensy4_teensy_drone_loop.hex`);
the hex is the runnable form of the `teensy_drone_loop` row.
