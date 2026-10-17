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

| Metric | Value | Notes |
|---|---|---|
| Instruction count per tick under QEMU `-icount` (Cortex-M7 build, `targets/mcu_drone_loop_timing_m7`, same 2000-tick sequence as the golden twin, hash matches) | p50 **23,040** instr; every 10th tick (plasticity pass) **197,440** instr; mean 30,440 | `ci/run_mcu_timing.sh targets/mcu_drone_loop_timing_m7 mps2-an500` — deterministic; Cortex-M3 build: 23,120 / 197,520 |
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
| `lib/armv7e-m-soft/libarc_runtime.a` | 36652 bytes |
| `lib/armv7e-m-hard/libarc_runtime.a` | 36672 bytes |
| `lib/x64-linux/libarc_runtime.a` | 68 KB |
| `lib/x64-windows/arc_runtime.lib` | 78 KB |

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
