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

## What this does and does not show

It shows the runtime's timing, fault contract and determinism on
PX4-grade physics and sensors, and an Arc-in-the-loop controller that
matches PX4's own on the same plant. It does not show the untrained
reflex flying on its own; that needs a prior trained on this plant.
Silicon numbers (DWT tick time) remain hardware-day items.
