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
| Full drone episode, **phase 1**: motor killed at tick 9000, station-hold survives (watchdog and rate limit deliberately relaxed here to isolate plant dynamics) | `test_drone_sim` | `PASS` |
| **Phase 2**: roll forced past the 45° tilt limit -> safety ramps to hover, plasticity freezes, snapshot fires before the fault | `test_drone_sim` | golden `cf9ea1aec3de21f1` — `PASS` |
| Setpoint rate limit (a step input cannot snap outputs) | `test_safety` | `PASS: rate limit` |
| Fault ramp behavior (watchdog + ramp to hover) | `test_loop_fault` | `PASS` |
| Plasticity module honors the fault gate flag (set directly; the loop->flag path is exercised by `test_drone_sim` phase 2) | `test_fault_gates_plasticity` | `PASS` |
| ESC pulse mapping: 1000/1500/2000 us -> exactly 18750/28125/37500 counts | `test_esc_pulse` | `PASS` |

**Cross-architecture:** the hash `ac29f2cfa8126d1b` is verified in this
repository's CI on **Windows x64**, **Linux x86_64**, **Linux ARM64**
(native `arm64` runner), and on **QEMU-emulated Cortex-M3 and
Cortex-M7** (`ci/run_mcu_qemu.sh` reads the hash out of guest memory for
each machine). One sequence, one hash, every platform shown here.

## Run it

Requires: CMake >= 3.15 and any C++17 compiler (MSVC, GCC, or Clang).

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

Expected: `100% tests passed, 0 tests failed out of 6`.

The QEMU twin check needs `qemu-system-arm` and `arm-none-eabi-nm`
(`apt install qemu-system-arm gcc-arm-none-eabi`):

```sh
bash ci/run_mcu_qemu.sh
```

Expected: `PASS: g_drone_loop_hash = 0xac29f2cfa8126d1b` for both
`mps2-an385` (Cortex-M3) and `mps2-an500` (Cortex-M7).

## Scope and limitations

- The engine ships **binary-only**. Determinism is regression-checked by
  the committed golden hashes for this build; a reader cannot
  source-audit for uninitialized reads or platform-dependent behavior in
  the engine itself. The published simulator, headers and tests are the
  inspectable half.
- `test_safety` exercises the rate limit on one axis, one step — it
  proves the cap exists and applies, not an exhaustive response surface.
- The phase-1 drone episode deliberately relaxes watchdog and rate
  limits (see table above); safety-critical ramp behavior is proven by
  phase 2 and by `test_safety` / `test_loop_fault`.
- No third-party reproduction has been published yet: this repository is
  new. Run it yourself — the checks above are the entire claim.

## What is in this repository

- `include/arc/` — the public API headers of the engine
- `train/sim/` — the fixed-point simulator (plant + episode env) the
  drone test runs against
- `platforms/sim/` — host glue for running the loop in-process
- `tests/` — the proof tests above, exactly as run in the engine's
  private CI
- `models/` — two trained priors (`prior_car.arc002`,
  `prior_servo.arc002`) used by the engine's replay tests
- `lib/` — the engine as a prebuilt static library:
  - `lib/x64-windows/arc_runtime.lib` (MSVC 19.44, x64)
  - `lib/x64-linux/libarc_runtime.a` (GCC 13, x86_64)
  - `lib/x64-linux-aarch64/libarc_runtime.a` (cross GCC, ARM64)
- `targets/` — the drone reflex-loop twin prebuilt for emulated
  Cortex-M3 and Cortex-M7 (`ci/run_mcu_qemu.sh` runs them unmodified)
- `ci/run_mcu_qemu.sh` — the QEMU twin verifier

## What is not in this repository

Engine implementation source, the trainer, and the target-hardware
support code. The libraries and target ELFs here were built from the
private repository at commit `fba7469`.

## License

Proprietary — see [LICENSE](LICENSE). Published as a proof artifact;
view and run to verify the claims, nothing more.
