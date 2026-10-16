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
| Full 10,000-tick drone episode with a motor killed at tick 9000, safety ramps to hover, plasticity freezes, snapshot fires | `test_drone_sim` | golden `cf9ea1aec3de21f1` — `PASS` |
| Setpoint rate limit (a step input cannot snap outputs) | `test_safety` | `PASS: rate limit` |
| Fault ramp behavior (watchdog + ramp to hover) | `test_loop_fault` | `PASS` |
| Plasticity frozen while faulted | `test_fault_gates_plasticity` | `PASS` |
| ESC pulse mapping: 1000/1500/2000 us -> exactly 18750/28125/37500 counts | `test_esc_pulse` | `PASS` |

The hash `ac29f2cfa8126d1b` is the same value produced in the engine's
own CI on x86_64, aarch64, and on QEMU-simulated Cortex-M3 and
Cortex-M7 — the bit-exactness claim is that **one sequence, one hash,
every platform**.

## Run it

Requires: CMake >= 3.15 and any C++17 compiler (MSVC, GCC, or Clang).

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

Expected: `100% tests passed, 0 tests failed out of 6`.

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

## What is not in this repository

Engine implementation source, the trainer, and the target-hardware
support code. The libraries here were built from the private repository
at commit `fba7469`.

## License

Proprietary — see [LICENSE](LICENSE). Published as a proof artifact;
view and run to verify the claims, nothing more.
