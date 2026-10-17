# Gap register — what a hardware signatory would still ask (2026-10-18)

Goal: a Ukraine Defense proof-of-concept strong enough that an evaluator
releases flight hardware. The standard for this file: the evaluator finds
nothing we did not already find, measure, and either close or state as
physics. Every row is one gap, how it was found, its status and the
closing action. Numbers come from the named tests and runs; nothing here
is estimated.

Status key: **closed** (measured, in the default ctest or the Gazebo
evidence), **open/in progress** (work queued or running), **physics**
(not closable by any controller; stated as an airframe requirement),
**hardware** (needs a physical board or airframe).

| # | gap | found by | status | closing action / statement |
|---|---|---|---|---|
| G1 | Attitude estimate stale > ~0.3 s is fatal under every policy (the bridge's only attitude source is PX4's EKF stream) | Track E (`test_x500_random_faults`, Gazebo `--stall-kind ekf`) | **in progress** | gyro-integrated attitude fallback in the PX4 frame adapter (behind the profile, default off, bit-exact), measured as Track F: hermetic sweep + Gazebo 0.3/1/3 s + random schedules |
| G2 | Total sensor loss (attitude and rates gone) > ~0.3 s is fatal | Track E | **physics** | a quad is open-loop unstable with a few hundred ms divergence time; PX4's own answer is a failsafe, not recovery. Requirement on the airframe: a redundant IMU path that survives the failure (the partial-dropout runs show that is sufficient) |
| G3 | Plant mismatch: the prior was validated only at the nominal x500 | this file | **closed** | `test_x500_robustness` A: 43 cells, mass ×0.7–1.3, thrust ×0.7–1.3, delay 20–60 ms, 10 s with wind kicks: 39/43 survive; **every cell in the ±20 % / ≤ 40 ms envelope survives within 0.4 m altitude / 0.75 m station** (contract, asserted). The 4 losses are all at 60 ms delay on a light or over-thrusted airframe (0.7/1.0, 0.8/1.0, 1.0/1.2, 1.0/1.3); nominal at 60 ms survives at 59° peak tilt |
| G4 | Observation delay ≥ 60 ms (3 frames at 50 Hz) off-nominal is fatal | G3 sweep | **open** | the prior is trained with 1–2 frames of delay. Closing action: retrain with delay randomised to 1–3 frames and re-run G3 (queued behind G1: it changes `models/prior_x500.arc002` and the pinned hashes). Until then the stated requirement is a ≤ 40 ms estimator-to-actuator path, which PX4 on an FMU meets |
| G5 | Online plasticity validated for only 60 s on Gazebo; residual arm at 25 % authority with plasticity on crashed 1 of 2 hover runs (C4) | Track C/D | **closed (hermetic)**, Gazebo long run queued | `test_x500_robustness` B: 30 min (ctest) and 60 min (run, 3.6 M ticks) of plasticity on with wind kicks every 2 s: 0 faults, last-minute bands / first-minute bands = 0.96 (off: 1.13), so no drift. The C4 crash is the untrained prior's plasticity on top of a cascade; the trained prior at sole authority is the deployment case. Gazebo 10-min plasticity-on runs queued behind G1 (shares the Docker host) |
| G6 | Loss of the Arc process itself mid-flight (host crash, bridge killed): PX4 is silenced by the arbitration params, so the motors coast | design review of `SIM_GZ_EC_FUNC` arbitration | **open** | add a supervisor handover: the motor publisher zeros / hands back to PX4 when the bridge heartbeat stops (`gz_motor_pub.py` watchdog, PX4 `FUNC` restore); test by `kill -9` of the bridge in hover. On the target this is the FMU/IO watchdog; queued behind G1 (Docker) |
| G7 | Position authority: trained reflex station 0.52–0.55 m vs cascade/PX4 0.04–0.07 m; wind excursion 3.6–4.2 m vs PX4 1.9 m | Track D | **open**, stated | the position loop is the weak loop by design of the frame (±0.5 rad attitude channel, 1/16 decoder step). Closing action: a position-channel retrain (finer x/y error channels, or the residual deployment with the cascade owning position) measured on Gazebo; queued with G4. Stated plainly in `docs/px4_sitl.md` D |
| G8 | Tick budget on slower silicon: the plasticity pass is 197,440 instructions | Track D timing | **open**, bounded | at 1 kHz the plasticity tick fits at ≤ 2 cycles/instr on a ≥ 480 MHz Cortex-M7 (823 µs) and at 1 cycle/instr on ≥ 216 MHz (914 µs); it does **not** fit on a 168 MHz M4 (1,175 µs at CPI 1). Normal ticks (23,040 instr) fit everywhere down to 100 MHz. Closing action: amortise the plasticity pass across ticks (core change, goldens re-pinned) or run plasticity at 500 Hz on slower parts. Until then: target ≥ 400 MHz M7 |
| G9 | Cycles-per-instruction on real silicon unconfirmed (QEMU icount counts instructions, not cycles; caches, flash wait states, FPU-free integer path) | Track D | **hardware** | one SysTick-instrumented run of `mcu_drone_loop_timing` on the delivered board, compared to the predicted 38–77 / 329–658 µs; the sequence and hash are already fixed so the comparison is one number |
| G10 | Motor loss on a quad | Track C/E | **physics** | no quad controller recovers a lost rotor without yaw spin; PX4 crashes identically. The contract (fault entered, hold, no runaway) is what is measured and claimed |
| G11 | Auto-resume after a fault without operator confirmation (`auto_clear_records`, default 25 fresh level records = 0.5 s) | design review | **open** (small) | the bridge resumes on its own; `auto_clear_records = 0` currently clears at once rather than never, and the knob is not on the CLI. Closing action: 0 = never (operator `arc_safety_clear` only), `--auto-clear N|off` on the bridge CLI, one test. Queued behind G1 (same file) |
| G12 | The evaluator cannot run the reflex on the Gazebo x500 themselves (bridge and engine source are private; the public proof ships a binary-only engine for the six headline tests only) | public repo review | **open — owner's decision** | ship a binary-only bridge + the shipped prior in the public Docker image so the SITL numbers reproduce on their machine; exposes the prior's weights (14 KB) but no engine or training source. Recommended before the hardware ask |
| G13 | `test_tick_budget` fails on the desktop under Gazebo load | Track C | **closed**, stated | host-side watchdog test; passes idle (63/63). Not a target property: on the MCU the tick is the only load. Stated in `docs/px4_sitl.md` |
| G14 | Untrusted-input robustness of the model loader | security review | **closed** | `test_arc_load_fuzz` (truncations, magic corruption, header bombs, seeded bitflips: every input returns a status, none crashes), `test_fuzz`; the EKF record parser reads fixed-size records from the owner's own FC and bounds every field |

Closing order (one Docker user at a time): G1 (running) → G6 → G5 Gazebo →
G4 + G7 retrain and re-measure (G3, Track D/E tables re-pinned) → G8 if the
target silicon is below 400 MHz. G9 is the hardware day. G12 is the owner's
call.
