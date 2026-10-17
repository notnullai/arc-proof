# Arc Proof

Runnable evidence for the Arc reflex runtime: **deterministic, bit-exact
across architectures, with a tested safety contract** — verified on
Windows x64 and Linux x86_64 from one command.

The engine itself ships here only as a **binary-only library** (this
repo proves behavior; it does not publish the engine source).

## What the tests prove

| Claim | Test | Expected output |
|---|---|---|
| Bit-exact reflex loop (same bytes as target hardware) | `test_mcu_drone_loop_reference` | `DRONE_LOOP_HASH ac29f2cfa8126d1b` — `PASS` |
| Motor-loss survival under the **bench-real safety profile** (rate limit 10000, watchdog budget 1000 µs, 3 misses): motor killed at tick 9000, station bands hold, no safety fault | `test_motor_loss_survival` | golden `8087c5b25d1381ce` — `PASS` |
| Tilt-fault response: roll forced past 45° from tick 0 -> loop sets the plasticity gate, safety ramps to hover, holds | `test_tilt_fault_response` | `PASS` |
| Drone plant invariants (hover + failure boundedness) | `test_drone_sim_env` | `PASS` |
| Setpoint rate limit (a step input cannot snap outputs) | `test_safety` | `PASS: rate limit` |
| Fault ramp behavior (watchdog + ramp to hover) | `test_loop_fault` | `PASS` |
| Plasticity module honors the fault gate flag (set directly; the loop->flag path is exercised by `test_tilt_fault_response`) | `test_fault_gates_plasticity` | `PASS` |
| Fault injection through the real loop: watchdog overrun, estop latch until clear, sensor dropout, single late tick tolerated | `test_fault_injection` | `PASS` |
| Loader robustness — `arc_load` is the only untrusted-input parser: truncations, magic, header bombs, 2000 seeded bitflips never crash | `test_arc_load_fuzz` | `PASS` |
| Committed trained priors load and replay deterministically across reset | `test_brain_prior_load` | `PASS` |
| ESC pulse mapping: 1000/1500/2000 us -> exactly 18750/28125/37500 counts | `test_esc_pulse` | `PASS` |

**Cross-architecture:** the hash `ac29f2cfa8126d1b` is verified on
**Windows x64**, **Linux x86_64**, and on **QEMU-emulated Cortex-M3 and
Cortex-M7** (`ci/run_mcu_qemu.sh` reads the hash out of guest memory for
each machine). One sequence, one hash, every platform shown here.
**Linux ARM64** ships as a library (`lib/x64-linux-aarch64/`) and all 11
tests cross-link against it, but the hash has not yet been run on ARM64
hardware or emulation; the manual CI job does that when dispatched.
Local verification log: `docs/verification/local-2026-10-17.md`.

## Run it — one command (Docker, Linux x86_64)

```sh
docker build --platform linux/amd64 -t arc-proof .
```

The build compiles everything, runs all 11 suites, and runs the QEMU
M3/M7 golden-twin checks. A green build **is** the proof. Re-run the
suite in the image with `docker run --rm arc-proof`.

## Run it — by hand

Requires: CMake >= 3.15 and any C++17 compiler (MSVC, GCC, or Clang).

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

Expected: `100% tests passed, 0 tests failed out of 11`.

The QEMU twin check needs `qemu-system-arm` and `arm-none-eabi-nm`
(`apt install qemu-system-arm gcc-arm-none-eabi`):

```sh
bash ci/run_mcu_qemu.sh
```

Expected: `PASS: g_drone_loop_hash = 0xac29f2cfa8126d1b` for both
`mps2-an385` (Cortex-M3) and `mps2-an500` (Cortex-M7).

## Continuous integration

The GitHub Actions workflow (`.github/workflows/ci.yml`, four jobs:
Windows, Linux x64, Linux ARM64, QEMU twins) is **manual-only**
(`workflow_dispatch`) — hosted runners bill the account. The Docker
path above replaces it for local verification.

## Hardware and interfaces

The on-target artifacts shipped here target a **Teensy 4.x (i.MX RT1062,
Cortex-M7)** robot line:

| Artifact | Role |
|---|---|
| `targets/teensy4_teensy_drone_loop.hex` | Flashable drone reflex-loop image (props-off bench bring-up; prints tick timing + loop hash over UART1 on first boot) |
| `targets/mcu_drone_loop_smoke_m3` / `_m7` | QEMU-verifiable twins of the same loop |
| `lib/armv7e-m-soft/libarc_runtime.a` | Engine for Cortex-M7 soft-float (the hex's ABI) |
| `lib/armv7e-m-hard/libarc_runtime.a` | Engine for Cortex-M7 hard-float |

Sensor/actuator interfaces on the bench line: 4× 400 Hz PWM ESCs
(1000–2000 µs pulse), ICM-42688-P IMU over LPSPI, active-low e-stop on
pin 10. Flash and RAM figures: `docs/performance.md`. All shipped
binaries are checksummed in `SHA256SUMS`.

## Scope and limitations

- The engine ships **binary-only**. Determinism is regression-checked by
  the committed golden hashes for this build; a reader cannot
  source-audit for uninitialized reads or platform-dependent behavior in
  the engine itself. The published simulator, headers and tests are the
  inspectable half. Engine source is available under NDA — see LICENSE.
- No real-silicon run yet: every Cortex-M number here is QEMU or
  cross-compiled, not measured on a board. The `.hex` is ready to flash.
- No third-party reproduction has been published yet: this repository is
  new. Run it yourself — the checks above are the entire claim.

## What is in this repository

- `include/arc/` — the public API headers of the engine
- `train/sim/` — the fixed-point simulator (plant + episode env) the
  drone tests run against
- `platforms/sim/` — host glue for running the loop in-process
- `tests/` — the proof tests above, exactly as run in the engine's
  private CI
- `models/` — two trained priors (`prior_car.arc002`,
  `prior_servo.arc002`)
- `lib/` — the engine as a prebuilt static library:
  - `lib/x64-windows/arc_runtime.lib` (MSVC 19.44, x64)
  - `lib/x64-linux/libarc_runtime.a` (GCC 13, x86_64)
  - `lib/x64-linux-aarch64/libarc_runtime.a` (cross GCC, ARM64)
  - `lib/armv7e-m-soft/libarc_runtime.a`, `lib/armv7e-m-hard/libarc_runtime.a`
- `targets/` — Cortex-M3/M7 QEMU twins + the Teensy 4 `.hex`
- `docs/` — `safety_contract.md`, `brain_format.md`, `performance.md`,
  verification log
- `SHA256SUMS` — checksums of every binary artifact
- `ci/run_mcu_qemu.sh` — the QEMU twin verifier
- `Dockerfile` — the one-command verifier

## What is not in this repository

Engine implementation source, the trainer, and the target-hardware
support code. The libraries and target artifacts here were built from
the private repository at commit `fba7469`.

## License

Proprietary — see [LICENSE](LICENSE). Published as a proof artifact;
view and run to verify the claims, nothing more. Engine source is not
publicly redistributable; evaluation access under NDA on request.
