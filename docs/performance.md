# Arc — Performance and Footprint

Measured numbers for the binary-only engine shipped in this repository
(libraries and `targets/` built from the private Arc repo at commit `0010edb`, bridge binaries at `3ce6f7d`). Every row is measured, none estimated; rows marked private repo are
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

Every number in this section was measured on the ELFs in `targets/`
(built from the private repo at `0010edb`), with
`ci/run_mcu_timing.sh <target> mps2-an500|mps2-an385` under QEMU icount
(1 instruction = 1 ns, SysTick resolution 40 instructions). The x500 rows
run a fixed brain at the shipped geometry (96 hidden, 480 or 832 input
bits), not the shipped weights; the 64 × 64 row is the synthetic golden
twin, not a shipped prior. Since 2026-10-18 the popcount at the heart of
every tick is an inline branch-free SWAR sequence in the CPU abstraction
(`include/arc/arc_types.h`; it was a libgcc `__popcountsi2` call on
Cortex-M). Since 2026-10-19 the popcounts of a neuron are fused into one
running sum (exact identities: pc(x & ai) = pc(x) − pc(x & ~ai), and the
excitatory and inhibitory spike words are disjoint, so six popcounts per
word pair become one SWAR pair folded every 15 words), the plasticity
flips settle state, Hamming change and log bytes as word masks, and the
trace decay of every shipped prior drops a clamp that provably cannot
bind. Every result is an exact integer, so every golden hash is
unchanged; the 2026-10-18 figures are in the last column.

| geometry (input / hidden) | prior | mode | Cortex-M7 worst tick | Cortex-M3 worst tick | lowest clock for 1 kHz at 1 / 2 cycles per instruction | 2026-10-18 (M7 / M3) | target |
|---|---|---|---|---|---|---|---|
| 832 / 96 | **flight default** (compiled cascade) | inference, plasticity off (how it flies) | **79,880** | 80,160 | **81 / 161 MHz** | 103,800 / 104,160 | `targets/mcu_drone_loop_timing_x500_832_inference_m7` / `_m3` |
| 832 / 96 | flight default | inference, dense-input stress (hidden layer firing) | 79,880 | 80,240 | 81 / 161 MHz | 103,840 / 104,040 | `targets/mcu_drone_loop_timing_x500_832_inference_dense_m7` / `_m3` |
| 480 / 96 | learning demonstration (trained) | inference | 54,080 | 54,200 | 55 / 109 MHz | 71,040 / 71,440 | `targets/mcu_drone_loop_timing_x500_480_inference_m7` / `_m3` |
| 480 / 96 | learning demonstration | inference, dense-input stress | 54,000 | 54,160 | 55 / 109 MHz | 71,000 / 71,200 | `targets/mcu_drone_loop_timing_x500_480_inference_dense_m7` / `_m3` |
| 480 / 96 | learning demonstration | plasticity on, **dense-input stress** | **316,080** | 320,360 | 321 / 641 MHz | 393,760 / 396,600 | `targets/mcu_drone_loop_timing_x500_480_dense_m7` / `_m3` |
| 480 / 96 | learning demonstration | plasticity on, **dense-input stress, amortised over 10 ticks** | **93,200** | 93,240 | **94 / 187 MHz** | 123,600 / 124,200 | `targets/mcu_drone_loop_timing_x500_480_sliced_dense_m7` / `_m3` |
| 480 / 96 | learning demonstration | plasticity on, plain twin (traces stay zero) | 260,840 | 261,040 | – | 316,400 / 316,280 | `targets/mcu_drone_loop_timing_x500_480_m7` / `_m3` |
| 480 / 96 | learning demonstration | plasticity on, plain twin, amortised | 78,240 | 78,320 | – | 99,360 / 99,680 | `targets/mcu_drone_loop_timing_x500_480_sliced_m7` / `_m3` |
| 64 / 64 | synthetic golden twin (not a shipped prior) | plasticity on / amortised | 105,840 / 26,360 | 105,960 / 26,440 | – | 124,000 / 29,800, 123,880 / 30,000 | `targets/mcu_drone_loop_timing_m7`, `_sliced_m7` (and `_m3`) |

For inference the fixed brain times the same work as the shipped weights:
`arc_core`'s inference loops over every hidden × input word and every
hidden × hidden word unconditionally (popcounts), and the only
data-dependent work is each neuron's threshold compare and the
winner-take-all bookkeeping for the motor neurons (4 here) that spike. The
dense-input rows confirm it: with 71 % of the hidden layer firing each tick
the flight default's worst tick is 79,880, the same as on the plain twin.

**The plasticity pass is data-dependent, and the plain twin understates it
(found 2026-10-18).** The plain twin feeds one input bit per tick against
a hidden threshold of 2, so its hidden layer never fires: every trace stays
zero and nothing flips, and its plasticity-on rows only time the pass
decaying zeros. The dense-input stress twins feed pseudo-random input
words; under that stress 58 % of synapse visits cross the trace threshold
and about 285 synapses flip per pass. They carry their own hashes (480
plain 0x06d80bb3fbefac89, amortised 0x83e4d89dc5e02f94, inference
0xd09e9fba89e283cf; 832 inference 0x628fde62eca6d3f1; all four reproduce
from the files here). On the original pass they cost 927,600 instructions,
or 191,600 amortised; the earlier published 496,760 / 132,360 were the
plain twin. The pass was then rewritten without changing a result (every
golden hash unchanged): loop invariants in registers, word-level
bit-parallel skips against the 32-synapse mask words, flips in ascending
order, the trace loop specialised on the post spike, and the ARMv7-M SMLAL
and SSAT instructions (the M3 has them too). Not claimed: an analytic worst
case for the pass. The stress twin is a measurement, and a pass that flips
every one of the 8,464 hidden↔hidden synapses has not been timed (gap
register G8).

So, counted in instructions: the flight default fits 1 kHz on a 600 MHz
Cortex-M7 with 7.5× margin at 1 cycle/instr and 3.7× at 2; the learning
demonstration with learning on (amortised, dense stress) needs ≥ 94 MHz
at 1 cycle/instr, ≥ 187 MHz at 2.

### Static worst-case cycle bound (gap register G9)

QEMU counts instructions, not cycles. `tools/mcu_cycle_bound.py` turns the
instruction counts into a cycle bound without hardware: it runs a timing
ELF under `qemu -icount` with `-d in_asm,exec,nochain` and the
`memory_region_ops_read` trace event, cuts the trace into ticks at the
firmware's own SysTick CVR reads, and gets the exact executed instruction
mix of every tick (it checks each tick's count against the firmware's
SysTick delta and stops if any differs by one SysTick count or more). Each
executed instruction is then charged a worst-case cost and the worst of the
1,999 ticks is reported. Cost tables (zero wait states):

- **Cortex-M4:** Arm DDI 0439B (Cortex-M4 TRM) Tables 3-1 / 3-2 and
  section 3.3.2: data processing, MUL, long multiplies, SSAT, bitfield and
  DSP multiplies 1; MLA/MLS 2; LDR/STR 2; LDRD/STRD 3; LDM/STM/PUSH/POP
  1+N (+P if PC is loaded); branches 1+P; TBB/TBH ≥ 6; SDIV/UDIV ≤ 12;
  IT 1; P = 3. On top: every branch charged as taken, +1 per load/store
  for the address-generation interlock, no load/store pipelining credit,
  +1 for LDR [PC], IT never folded.
- **Cortex-M3:** Arm DDI 0337 (Cortex-M3 TRM r2p0, chapter 18): the same
  figures except UMULL/SMULL 3–5 and UMLAL/SMLAL 4–7, charged at 5 and 7.
- **Cortex-M7:** Arm publishes no per-instruction cycle table for the M7
  (DDI 0489). The M7 column is a **stated envelope, not vendor data**: no
  dual issue; every load 4; LDRD 5; LDM 2+N; store 2; multiply and long
  multiply 2; divide 12; every branch and PC write a full mispredict of
  1+8. It is the number a board run must confirm, not a guarantee.

Worst tick in cycles and the lowest clock for 1 kHz, on the shipped images
(`python tools/mcu_cycle_bound.py --elf targets/<image>_m7 --machine mps2-an500 --models m4,m7`,
and `--elf targets/<image>_m3 --machine mps2-an385 --models m3,m4`; needs
`qemu-system-arm` and Python 3). Exact instructions from the trace in the
first column; in brackets the 2026-10-18 figures (inline popcount), then
those with the libgcc popcount call before it.

| workload (exact instructions, M3 / M7 image) | image | Cortex-M3 | Cortex-M4 | Cortex-M7 envelope |
|---|---|---|---|---|
| flight default, 832 / 96 inference, dense stress (80,219 / 79,848) | `x500_832_inference_dense_m3` | 124,082 → **125 MHz** (140,858; 203,691) | 123,422 → 124 MHz (140,390; 203,223) | – |
| same | `x500_832_inference_dense_m7` | – | 123,061 → **124 MHz** (140,241; 203,425) | 156,042 → **157 MHz** (169,671; 294,527) |
| learning demonstration, 480 / 96 amortised × 10, dense stress (92,726 / 93,194 at the M3 / M4 worst tick) | `x500_480_sliced_dense_m3` | 167,964 → **168 MHz** (205,829; 249,692) | 161,378 → **162 MHz** (199,435; 243,298) | – |
| same | `x500_480_sliced_dense_m7` | – | 155,836 → 156 MHz (195,292; 239,458) | 199,456 → **200 MHz** (248,233; 332,904) |

So on this bound a 600 MHz Cortex-M7 flies the flight default with 3.8×
margin on the envelope and runs the learning demonstration with 3.0×; a
168 or 180 MHz Cortex-M4 fits the flight default at 1 kHz (124 MHz needed:
1.35× / 1.45× margin) and now also the learning demonstration with learning
on (162 MHz needed on the M3-build image, 156 on the M7-build one: 1.04× /
1.08× at 168 MHz, 1.11× / 1.16× at 180 MHz; it was 200 / 196 MHz). The
168 MHz margin is thin and rests on the requirements below; a Cortex-M3 at
≤ 120 MHz fits neither (125 / 168 MHz needed).

Requirements on the firmware for these numbers to hold: code and per-tick
data in zero-wait memory (ITCM / DTCM on an M7, zero-wait SRAM or CCM on an
M3/M4; each wait state adds at most `wf × (instructions + branches) + wd ×
data words` per tick, and flash behind a cache is not covered); no
interrupt load inside the tick; naturally aligned accesses (set
`CCR.UNALIGN_TRP` on the board run so an unaligned access faults instead of
slowing silently). The bound is analytic: published M3/M4 costs and a
stated M7 envelope times the exact instruction mix. What a board run still
adds (G9): the true cycles per instruction, the M7 envelope's confirmation
and the real memory map; one DWT_CYCCNT reading of the same image's worst
tick on the delivered board closes it.

Synthetic twin, original measurements (kept for the record). They were
taken on the images built from `fba7469`, before the plasticity-pass
rewrite and the inline popcount; the images in `targets/` now give the
64 × 64 row of the table above (p50 17,360 / 17,400 instructions on M7 /
M3), and the commands below reproduce that row, not these figures:

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
| `teensy_drone_loop` (board image, ships as `.hex`) | 27464 | 368 | 70952 | 27.2 KB | 69.6 KB |
| `teensy_robot` (wheeled board image, private repo, not shipped here) | 25568 | 332 | 70036 | 25.3 KB | 68.7 KB |
| `targets/mcu_drone_loop_smoke_m7` (QEMU smoke, in repo) | 16128 | 320 | 70312 | 16.1 KB | 69.0 KB |
| `targets/mcu_drone_loop_smoke_m3` (QEMU smoke, in repo) | 16172 | 320 | 70312 | 16.1 KB | 69.0 KB |

The bulk of `bss` is the caller-supplied static engine arena (the board
images allocate 70 KB). The engine library itself:

| Library | Size |
|---|---|
| `lib/armv7e-m-soft/libarc_runtime.a` | 47234 bytes |
| `lib/armv7e-m-hard/libarc_runtime.a` | 50258 bytes |
| `lib/x64-linux/libarc_runtime.a` | 60 KB |
| `lib/x64-windows/arc_runtime.lib` | 124 KB |

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
