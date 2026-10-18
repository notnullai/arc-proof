# Arc Proof

A deterministic, fail-safe reflex runtime, verified on five targets
(Windows x64, Linux x86_64, Linux ARM64, QEMU Cortex-M3, QEMU Cortex-M7).
**Give us a board: DWT cycle numbers + hover video within the week.**

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
| **PRELIMINARY — toy plant:** PID-cascade vs reflex baseline, same plant/seed/schedule — both survive, deterministic | `test_drone_baseline` | `PASS` (metrics table printed) |
| **PX4 SITL (Gazebo Harmonic x500 + PX4 EKF), measured 2026-10-17:** Arc 1 kHz tick locked to sim time; sensor dropout → fault path → hover hold → auto-clear (raw hold within 0.20 m / 3°, stabilised 5 cm / 0.6°); motor loss → fault path < 1.2 s in every run; Arc-in-the-loop cascade hovers like PX4 (1.0 m ± 2–7 cm, station 4–7 cm) and recovers from 3.35 N gusts in 2.2 s; reflex residual safe at 10 % authority, not at 25 % | method + numbers: `docs/px4_sitl_method.md`, table: `docs/px4_sitl_summary_table.md` | scenario statistics, not bit-exact (bridge code stays private) |
| **Trained prior flies the x500 as sole controller** (2026-10-17): 1.1 m ± 1–2 cm, 0.5 m station, 60 s × 4, plasticity on or off, 0 faults; wind survived | `docs/px4_sitl_method.md` | prior bytes and training in the private repo |
| **Flight default: compiled station cascade** (2026-10-18): a gain-matched frame (832 bits) compiles the station cascade exactly into ternary weights; plasticity off, no learning claim. Gazebo x500: hover station 0.24–0.30 m (trained prior 0.52–0.55 m); wind 3/3 at 2.96–3.05 m excursion and 17.6–18.4° peak tilt, no rotor saturation, against the trained prior in the same matrix at 3.19–4.44 m, 34.7–39.2°, 0.46–0.64 %; all 43 plant-mismatch cells incl. 60 ms delay. The trained prior stays as the learning demonstration | `docs/px4_sitl_method.md`, `docs/gap_register.md` G4/G7/G8/G15–G18 |
| **Arc process loss mid-flight** (2026-10-18): Arc killed or hung while flying the Gazebo x500, an unmodified PX4 takes over (route outputs, arm in the air, Takeoff then Hold): 13 / 13 survived, 0 / 1 without the fallback; takeover costs 0.16–0.70 m from a 1.1 m hover, stated as a 1 m minimum altitude | `docs/bridge_loss_method.md` | scenario statistics, fresh container per run |
| **Fly the flight default in Gazebo yourself** (2026-10-18): the PX4 bridge ships as stripped release binaries with the engine statically linked (`bin/linux-x86_64/px4_bridge`, `bin/windows-x64/px4_bridge.exe`), with the flight default `models/prior_x500_candidate_v5.arc002` and the Python harness (`tools/px4/`): hover, wind and the Arc-loss handover from a clean Linux machine with Docker and the public PX4 image; offline check, recorded PX4 stream in, same rotor-command hash out on both binaries | `docs/run_it_yourself.md` | offline: `hash=00cffd5d35dc57d9`; flights: the published ranges, up to run-to-run noise (scenario statistics). Bridge source stays private; a stripped binary can still be disassembled |
| **Gap register** (2026-10-18): every remaining weakness a hardware signatory would raise, each measured and closed, in progress, or stated as physics/hardware; plant-mismatch sweep (43 cells, mass/thrust ±30 %, delay 20–60 ms) and 30/60-minute plasticity-on soak with no drift | `docs/gap_register.md` | hermetic, private test `test_x500_robustness` |
| **Frozen estimator with a live IMU** (2026-10-18): the one fatal Track E class, attitude stale > 0.3 s, is carried by the raw gyro (attitude) and baro (altitude) for up to 2 s in the PX4 integration layer: hermetic crashes 46 → 7 of 137 (gyro alone 43: the frozen altitude channel did most of the damage), Gazebo 0.3–3 s freezes 8/8 and random schedules 12/12 survived; total dropouts (no records) stay fatal; opt-in, default bit-exact | `docs/px4_sitl_method.md` | hermetic + Gazebo scenario statistics |
| **Random faults** (2026-10-17): ≈2,900 randomly timed, overlapping faults through the loop, contract never missed; disturbances and GPS-loss dropouts survivable on the real x500, total dropouts > ~0.3 s fatal for any controller; opt-in live-axis fault policy in the runtime (default bit-exact) | `docs/px4_sitl_method.md` | hermetic campaign + Gazebo schedules |
| **Cortex-M per-tick instruction count at the shipped geometries** (restated 2026-10-18): flight default (832 bits, inference) worst tick 125,200 instr on M7, 125,280 on M3 → 1 kHz from 125 MHz at 1 cycle/instr, 250 MHz at 2; flight default under a dense-input stress twin (hidden layer firing) 125,240 on M7 and M3. Learning demonstration with learning on, after the bit-exact plasticity-pass rewrite of 2026-10-18 and measured under dense-input stress: 408,760, amortised 138,640 (139,120 on M3). The earlier 496,760 / 132,360 were measured on a twin whose hidden layer never fires, so its traces stayed zero; that pass cost 927,600 / 191,600 under the same stress. The earlier 23,040 / 197,440 figures were a 64 × 64 synthetic twin | `docs/performance.md` |
| **Amortised plasticity pass, rewritten bit-exact (2026-10-18)**: at the learning demonstration's geometry (480/96), under dense-input stress (traces nonzero, synapses flipping), worst tick 408,760 → 138,640 instr amortised over 10 ticks, no heavy ticks. 1 kHz with learning on needs ≥ 139 MHz at 1 cycle/instr, ≥ 278 MHz at 2. Every golden hash unchanged. **Not claimed:** an analytic worst case; the stress twin is a measurement (G8 open) | `docs/performance.md`, `docs/gap_register.md` G8 |

**Cross-architecture:** the hash `ac29f2cfa8126d1b` is verified on
**Windows x64**, **Linux x86_64**, **Linux ARM64** (all 12 suites
cross-built against `lib/x64-linux-aarch64/` and executed under
`qemu-aarch64`, see `ci/aarch64-linux-gnu.cmake`), and on **QEMU-emulated
Cortex-M3 and Cortex-M7** (`ci/run_mcu_qemu.sh` reads the hash out of
guest memory for each machine). One sequence, one hash, every platform
shown here. ARM64 is emulated, not silicon.
Local verification log: `docs/verification/local-2026-10-17.md`.

## Run it — one command (Docker, Linux x86_64)

```sh
docker build --platform linux/amd64 -t arc-proof .
```

The build compiles everything, runs all 12 suites on x86_64, runs the
same 12 suites cross-built for ARM64 under `qemu-aarch64`, and runs the QEMU
M3/M7 golden-twin checks. A green build **is** the proof. Re-run the
suite in the image with `docker run --rm arc-proof`.

## Run it — by hand

Requires: CMake >= 3.15 and any C++17 compiler (MSVC, GCC, or Clang).

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

Expected: `100% tests passed, 0 tests failed out of 12`.

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
  `prior_servo.arc002`) and the x500 flight default
  (`prior_x500_candidate_v5.arc002`, compiled, 832 bits)
- `bin/` — the PX4 bridge, prebuilt and stripped (Linux x86_64, Windows x64)
- `tools/px4/` — the PX4 SITL harness (Python), see `docs/run_it_yourself.md`
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
the private repository at commit `fba7469`. Evaluators can review the
source under NDA: a pinned-commit escrow snapshot plus a verification
walkthrough session is available on request (see License).

## License

Proprietary — see [LICENSE](LICENSE). Published as a proof artifact;
view and run to verify the claims, nothing more. Engine source is not
publicly redistributable; evaluation access under NDA on request.
