# PX4 SITL evidence — method and numbers (2026-10-17)

What was run, how, and what came out. The bridge source stays private;
this page carries the method and the measured numbers so the result can
be judged and, with the private repo under NDA, rerun.

## Setup

- Plant: Gazebo Harmonic `x500` (2.06 kg, `motorConstant` 8.55e-6,
  `maxRotVelocity` 1000 rad/s) from `px4io/px4-sitl-gazebo`, headless,
  fresh container per run. PX4 SITL runs its EKF; its ESC outputs are
  unassigned (`SIM_GZ_EC_FUNC1..4 = 0`) when Arc flies, so exactly one
  publisher writes the motor topic.
- Arc: PX4 EKF state (MAVLink `ATTITUDE` + `LOCAL_POSITION_NED`, ~50 Hz)
  → 320-bit frame (the same encoder the toy-plant tests use) →
  `arc_run` at 1 kHz with its clock locked to the Gazebo clock (records
  applied at their timestamp, one frame serves ~20 ticks) → Q16 motor
  setpoints → rotor speed ω = ω_hover·√(sp/32768), ω_hover = 769 rad/s
  (from the model constants; PX4's own hover measured 768.7).
- Record path: separate processes over OS pipes (max stall ≤ 3 ms); a
  Python-threaded path with up to 88 ms stalls made the same controller
  limit-cycle at 1 Hz, which is itself a result worth knowing.
- Claim boundary: Arc's tick is integer-deterministic from the frame
  onward (same record stream → same command bytes, hash per run);
  everything across the bridge is float; all numbers are scenario
  statistics, 1–3 runs per cell, min..max reported.

## Arms

`px4`: PX4's own controller. `pid`: a bridge-side NED cascade with Arc
ticking alongside and owning the fault path. `residual`: the cascade
plus a bounded share of the untrained reflex's deviation from hover,
after Arc's safety clamp and rate limit. `reflex`: the untrained prior
alone. Fault policy: `raw` = Arc's ramp-to-hover goes to the rotors
unchanged; `stabilised` = the ramp sets thrust, the attitude loop stays.

## Scenarios

hover 60 s; wind = 3 N + 1.5 N lateral force for 1 s every 6 s (7
pulses) via a resident Gazebo injector (one-shot publishes were found
to be lost); motor loss = Gazebo MotorFailure on motor 1 at ~27 s sim;
dropout = the record stream to Arc stalled for 1 s at 30 s with the
airframe intact.

## Results

The full table is `px4_sitl_summary_table.md`. In words:

- Hover: `pid` and PX4 are equivalent (1.0 m ± 2–7 cm, station 4–7 cm,
  tilt < 0.6°); `pid` lifts off in 0.5 s.
- Gusts: PX4 1.7–1.9 m excursion, 28–31° tilt, never back inside the
  0.25 m band between pulses; Arc cascade with a 29° tilt cap 2.0 m,
  28°, back in band 2.2 s after each pulse.
- Motor loss: unflyable for every arm, PX4 included (tilt fault within
  1.2 s of injection). Arc's contract held in every run: estop at 45°
  → fault path for the rest of the run.
- Dropout: Arc keeps ticking on the held frame, asserts estop, ramps to
  hover, holds, clears 0.5 s after records return (≈1.4 s on the fault
  path). Raw hold: within 0.20 m / 3°. Stabilised: 5 cm / 0.6°.
- Reflex as shipped: alone it never commands above hover thrust. As a
  residual, 10 % authority is indistinguishable from `pid`; 25 % with
  online plasticity crashed one of two hover runs; plasticity off held.

## Trained prior (added 2026-10-17)

A prior trained for this plant (supervised imitation of the cascade
through the runtime's own spike decoder; 480-bit frame with attitude at
±0.5 rad, the decoder's integrator state and yaw rate as channels; the
four WTA groups are thrust / roll / pitch / yaw, mixed to rotors on the
HAL side) flies the x500 as the **sole controller**: 1.11–1.13 m with
1.1–1.7 cm altitude jitter, station within 0.52–0.55 m, tilt ≤ 5.2°, 60 s
per run, four runs (one with online plasticity on), zero faults. Under
the same seven 3.35 N gusts it survives with altitude within 0.32 m but
is pushed 3.6–4.2 m (PX4 1.9 m, the Arc cascade 2.0–2.9 m). Motor loss
remains unflyable for every arm. The prior's bytes, the training command
and the selection procedure are in the private repo; the EKF record
streams of every run are committed there and replayable.

## Safe-state envelope (added 2026-10-17)

A quad is open-loop unstable. The runtime's "ramp to hover thrust, zero
torque" hold survives a 0.3 s sensor dropout in 1 of 2 runs and tumbles
within 0.7 s in all four 1.0 s runs across controllers; keeping an
attitude loop alive on the ramp turns the same 1.0 s dropout into 5 cm
and 0.6°. The contract text now carries these numbers.

## On-target timing bound (added 2026-10-17)

`ci/run_mcu_timing.sh targets/mcu_drone_loop_timing_m7 mps2-an500`
runs the golden 2000-tick sequence (hash matches) under QEMU with
deterministic instruction counting: p50 23,040 instructions per tick,
197,440 on the plasticity tick (every 10th), mean 30,479; Cortex-M3
build within 0.4 %. At 600 MHz and 1–2 cycles per instruction that is
38–77 µs for a normal tick and 329–658 µs for the plasticity tick,
inside the 1000 µs budget. The cycles-per-instruction factor is what a
hardware day confirms.

## Random faults, not fixed ones (added 2026-10-17)

Every earlier fault was injected at a fixed time. A seeded campaign
through the runtime loop with the trained prior (300 ten-second episodes
per sweep, 1–4 random, overlapping events each: total sensor dropout,
estimator-frozen dropout with rates fresh, GPS-loss dropout with
attitude/rates/baro fresh, gusts, ±2 rad/s attitude kicks, estop pulses,
motor loss; ≈2,900 events over five sweeps) never missed the contract:
estop → fault within one tick, exact safe-state hold, resume after
clear, every time. Survival by class: gusts and kicks ≥ 97 %, GPS-loss
dropouts ≥ 93 %, estop pulses 96 % with the opt-in live-torque fault
policy; total or attitude-frozen dropouts longer than a few hundred
milliseconds are fatal under any policy because the airframe is
open-loop unstable. On Gazebo with random schedules (6 seeds): the Arc
cascade survived 6 of 6, the trained reflex 4 of 6 with both losses
being total dropouts; partial dropouts (GPS loss 1–3 s, estimator frozen
1 s) were survived by both arms. The runtime now ships an opt-in
per-axis fault policy (thrust ramps, torque axes keep the live loop;
default behaviour bit-exact) that removes most estop-pulse and kick
crashes and is not claimed to help stale-frame faults.

## Frozen estimator with a live IMU (added 2026-10-18)

Track E's one fatal class was an attitude estimate stale for more than
~0.3 s, because the bridge's only attitude source was PX4's EKF stream.
The PX4 integration layer now takes a second record field set from the
raw IMU (`HIGHRES_IMU`, requested at 50 Hz: gyro and baro altitude, each
with its own timestamp, plus the ATTITUDE message's timestamp). When the
attitude timestamp is older than 100 ms but the gyro is fresh, the last
good attitude is propagated by fixed-point Euler integration of the body
rates for at most 2 s, and the altitude and vertical-velocity channels are
carried on the baro altitude (offset to continue the last good value);
past the window, or with the gyro stale too, the frame is declared stale
and the bridge estops exactly as before. Opt-in; the default record,
behaviour and every hash are unchanged. Nothing in the runtime changed.

Measured the same way as Track E. Hermetic (300 seeded episodes, same
≈2,900 events): frozen-estimator crashes 46 of 137 → 43 with the gyro
alone → **7** with gyro + baro; isolating that class (750 freezes of
50–1000 ms over 300 episodes) 232 → 204 → **15** episodes lost, with the
178 ground impacts of the no-fallback case going to 0: three quarters of
the losses came from the frozen altitude channel, not the attitude, so an
attitude-only fallback is not enough. With the freezes stretched to
150–3000 ms (past the window) 14 of 137 are lost, all in the post-window
hold. Gazebo (fresh container per run, PX4 x500, freeze at 30 s): 0.3 s,
1 s and 3 s freezes survived by both the trained reflex and the cascade
(8 of 8; on the 3 s freeze the bridge carried 2 s and then held hover for
the last second), and the six Track E random schedules with their
dropouts delivered as estimator freezes survived 12 of 12 (seeds 1, 3, 5,
6 froze for 0.21–0.33 s; the fallback engaged and no fault tick was
taken). What this does not change: a total dropout, where no record
arrives at all, has nothing to fall back on and keeps the limits measured
before; beyond the 2 s window the hover hold applies.

## What this does and does not show

It shows the runtime's timing, fault contract and determinism on
PX4-grade physics and sensors, an Arc-in-the-loop controller that
matches PX4's own on the same plant, a trained prior flying the plant
on its own authority (looser than PX4 on station, weaker under gusts),
the measured limits of the hover-hold safe state, and a deterministic
per-tick instruction bound on Cortex-M, and a frozen estimator survived
for up to 2 s on the raw IMU. Silicon cycle counts remain the
hardware-day item, now with a predicted range to confirm.
