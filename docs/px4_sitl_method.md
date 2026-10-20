# PX4 SITL evidence — method and numbers (2026-10-17)

What was run, how, and what came out. The bridge source stays private;
this page carries the method and the measured numbers so the result can
be judged. The stripped bridge binaries, the flight default and the
harness are public, so the flight-default runs can be repeated without
us: `docs/run_it_yourself.md`.

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

*Restated 2026-10-18:* those figures are the 64 × 64 synthetic twin. At
the shipped priors' geometry (96 hidden) the flight default (832 input
bits, inference only) is 125,200 instructions per worst tick on M7 and
125,280 on M3, and 125,240 under dense-input stress with the hidden layer
firing. With learning on (learning demonstration, plasticity pass
amortised over 10 ticks, dense-input stress) it is 138,640 on M7, so 1 kHz
needs ≥ 139 / 278 MHz at 1 / 2 cycles per instruction. The earlier
132,360 was measured on a twin whose hidden layer never fires. Table in
`docs/performance.md`.

*Popcount inlined and cycle bound, 2026-10-18:* the popcount is now an
inline branch-free sequence (it was a libgcc call on Cortex-M), every
hash unchanged. Since 2026-10-19 the popcounts are fused into one
running sum per neuron and the plasticity flips and trace decay are
reworked, again with every hash unchanged. On the shipped images the
flight default is 79,880 instructions per worst tick on M7 (80,160 on M3)
and the learning demonstration amortised under dense stress 93,200
(93,240). A static worst-case cycle bound from the exact executed
instruction mix (`tools/mcu_cycle_bound.py`) gives the flight default
124,082 cycles on Cortex-M3, 123,061 on Cortex-M4 (Arm TRM tables) and
156,042 on a stated Cortex-M7 envelope (≥ 125 / 124 / 157 MHz for 1 kHz),
and the learning demonstration with learning on 167,964 / 161,378 /
199,456 (≥ 168 / 162 / 200 MHz), so a 168/180 MHz Cortex-M4 fits both.
The bound is analytic; a board's DWT cycle count confirms it.

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
raw IMU (`HIGHRES_IMU`, requested at 50 Hz: gyro, accelerometer and baro
altitude, each with its own timestamp, plus the ATTITUDE message's
timestamp). When the attitude timestamp is older than 100 ms but the
gyro is fresh, the last good attitude is propagated by fixed-point Euler
integration of the body rates for at most 2 s, and the altitude and
vertical-velocity channels are carried by a baro + accelerometer
complementary filter (specific force rotated by the carried attitude,
gravity removed, integrated, pulled towards the baro with a 1.5 s time
constant, offset to continue the last good value); past the window, or
with the gyro stale too, the frame is declared stale and the bridge
estops exactly as before. Opt-in; the default record, behaviour and
every hash are unchanged. Nothing in the runtime changed.

A first version carried the vertical velocity as a baro difference. It
survived with the trained prior but regressed the flight default (the
compiled, gain-matched controller) on Gazebo: PX4 SITL's pressure
altitude carries 0.23 m of correlated noise per 50 Hz record, so a baro
difference is ±9 m/s of velocity noise, which saturated the controller's
vertical-velocity channel and the thrust axis (1 s freeze: 1.4 m
altitude excursion; 3 s freeze lost). Replaying the recorded stream
through the bridge shows the thrust axis hitting 0 and full scale with
that carry and staying within ±20 % of hover with the complementary
filter. The hermetic sensor model was made SITL-like (baro sd 0.23 m,
lag-1 autocorrelation 0.7, records 16/24 ms apart, accelerometer noise
±0.05 m/s²) and the altitude integral is held while the altitude is
carried.

Measured the same way as Track E. Hermetic (300 seeded episodes, same
≈2,900 events, SITL-like sensor model): frozen-estimator crashes 46 of
137 → 43 with the gyro alone → **4** with the full carry for the trained
prior, and 46 → 46 → **1** for the flight default; isolating that class
(750 freezes of 50–1000 ms over 300 episodes) 232 → 214 → **10**
episodes lost for the trained prior and 253 → 250 → **0** for the flight
default, with the ground impacts of the no-fallback case (178 and 249)
going to 0: most of the losses came from the frozen altitude channel,
not the attitude, so an attitude-only fallback is not enough. With the
freezes stretched to 150–3000 ms (past the window) 16 and 9 of 137 are
lost, in the post-window hold. Gazebo (fresh container per run, PX4 x500, freeze at 30 s): 0.3 s,
1 s and 3 s freezes survived by both the trained reflex and the cascade
(8 of 8; on the 3 s freeze the bridge carried 2 s and then held hover for
the last second), and the six Track E random schedules with their
dropouts delivered as estimator freezes survived 12 of 12 (seeds 1, 3, 5,
6 froze for 0.21–0.33 s; the fallback engaged and no fault tick was
taken). Those Gazebo runs used the first carry and the trained prior.
The flight default's Gazebo runs (Track H, below) found the first carry's
regression on v5 and, after the complementary filter, survived 10 of 10.
What this does not change: a total dropout, where no record
arrives at all, has nothing to fall back on and keeps the limits measured
before; beyond the 2 s window the hover hold applies.

## Position loop: gain-matched frame, three priors (added 2026-10-18)

The trained x500 prior held attitude and altitude but its station hold
was weak (0.52 m on Gazebo vs 0.05 m for the cascade it was trained
from), seed-dependent (1 of 5 training seeds passed the gates) and lost
4 of 9 cells at 60 ms observation delay. Cause: the policy's direct
path is one ternary weight (−1/0/+1) per thermometer bit, and with
equal-range channels one attitude bit is worth 2.4 actuator bits of the
cascade's output and one position bit 0.7, ratios a per-bit ternary map
cannot express.

Method. A new 512-bit frame (v3) sizes every channel's first bits so
that one bit moves the cascade's axis target by exactly one actuator
bit, with cubic spacing beyond for range, plus an altitude-error
integral channel kept by the frame source. In that frame the cascade's
linearised law is a sum of ±1 weights per bit and compiles exactly. The
candidate keeps that compiled law as a fixed direct path and
trains (supervised DAgger) only a residual hidden layer that sees the
outer thermometer bits, so it is silent near hover and active on large
errors. The 480-bit frame and the prior trained on it are unchanged.

Three priors ship, each claim stated for what it is. The flight default
stays the trained v2 prior (plasticity off) until the candidate passes
the Gazebo wind matrix (owner decision):

| prior | what it is | seeds | hermetic: 43 mismatch cells / 60 ms cells | Gazebo hover station | Gazebo wind (3 N, 1 s every 6 s) |
|---|---|---|---|---|---|
| candidate | compiled cascade + learned residual, plasticity off | 4 of 5 pass every gate | 43/43, 9/9 | **0.18–0.23 m** (3/3) | **1/3 survived**, 11.4 m on the survivor (open) |
| compiled reference | the cascade in ternary weights, not learned | seed-independent | 43/43, 9/9 | not flown | not flown |
| trained v2 prior (**flight default**) | pure DAgger, 480-bit frame, unchanged | 1 of 5 pass | 39/43, 5/9 | 0.52–0.55 m (3/3) | 2/2, 3.6–4.2 m |

Random-fault campaign (same 300 seeded episodes per sweep): the candidate survives more episodes than the trained v2 prior in all
8 sweeps (for example 128 → 149, and 173 → 199 with the raw-IMU
fallback), with more crashes on ±2 rad/s attitude kicks; the compiled
reference alone survives far fewer (53–93), which is what the residual
is for.

Not claimed: strong gusts on the candidate (1 of 3 Gazebo wind runs
survived; a hermetic gust test calibrated to those runs, 6 N in place
of 3 N, reproduces it: 11.65 m vs 11.4 m on Gazebo), and online plasticity on the candidate (1 tip-over in 3
Gazebo hover runs, not reproduced hermetically). Both are open in the
gap register (G16, G17) with their closing work.

## Flight default: compiled station cascade (added 2026-10-18)

Two more structural faults surfaced in Gazebo wind and were fixed in the
frame, not in training. The rate channels compressed above 0.52 rad/s,
so kick recovery needed extra damping from a trained residual that then
rang at 60 ms delay on a light airframe; the attitude channels compressed
past 0.2 rad, so the compiled law lost stiffness at large tilt and
overshot in gusts. Frame v5 (832 bits) keeps rates and attitude linear in
extra bands. The flight default is the cascade compiled into that frame
with a softer, capped station/velocity law, no training. A hermetic gust
test calibrated to Gazebo (6 N in the hermetic plant reproduces the
Gazebo 3 N response) gates excursion and peak tilt.

| Gazebo wind, 3 N for 1 s every 6 s, same matrix | survived | excursion | peak tilt | altitude error | rotor saturated |
|---|---|---|---|---|---|
| flight default (compiled) | 3/3 | 2.96–3.05 m | 17.6–18.4° | 0.09–0.11 m | 0 % |
| learning demonstration (trained) | 3/3 | 3.19–4.44 m | 34.7–39.2° | 0.28–0.32 m | 0.46–0.64 % |

Hover: 0.24–0.30 m station (trained 0.52–0.55 m). Hermetic: all 43
plant-mismatch cells including every 60 ms cell; random faults, more
episodes survive than the trained prior in all 8 sweeps. Plasticity does
nothing on the compiled flight default; learning claims belong to the
trained prior. Cortex-M7 worst tick 79,880 instructions (103,800 before
the fused popcount, 125,200 before the inline popcount).

## Failure tests on the flight default v5 (Track H, added 2026-10-19)

The failure tests of Tracks E and F had been flown on the trained prior.
Track H repeats them on the flight default (candidate v5, plasticity off),
same Gazebo x500 setup, fresh container per run, freeze injected at 30 s
of a 60 s run. Per-run rows: `docs/px4_sitl_summary_table.md`, Track H.

**A regression found and fixed.** The first freeze runs used the bridge
of that day, whose altitude carry differenced the baro. On v5 a 1.0 s
estimator freeze cost 1.40 m of altitude and a 3.0 s freeze was lost at
33.6 s (the trained prior: 0.18 m / 0.77 m on the same test). The cause is
in "Frozen estimator with a live IMU" above: SITL baro noise saturated the
gain-matched vertical-velocity channel. The carry was replaced by the
baro + accelerometer complementary filter (v3 record, private `1dd7d78`;
the binaries in `bin/` contain it). After the fix, every v5 freeze run
survived (10 of 10):

| estimator freeze, imu fallback (gyro where stated) | runs | survived | altitude error max | tilt max | fault ticks |
|---|---|---|---|---|---|
| 0.3 s | 2 | 2 | 0.05 m | 3.4° | 0 |
| 1.0 s | 2 | 2 | 0.05–0.08 m | 3.3° | 0 |
| 1.0 s, gyro fallback | 1 | 1 | 0.05 m | 4.1° | 0 |
| 3.0 s (2 s carried, then the hover hold) | 2 | 2 | 0.14–0.16 m | 12.7–23.7° | 1,319–1,343 (the hold) |
| random schedules, estimator-freeze dropouts (seeds 1–3) | 3 | 3 | 0.05–0.08 m | 3.7–18.3° | 0 |

On the earlier bridge, random schedules seeds 1–6 survived 6 of 6. Without
any fallback, one 1.0 s freeze on v5 survived with 0.18 m altitude error
and 11.7° tilt (one run; the fallback cuts both). A **total** dropout (no
records at all) still has nothing to fall back on.

**Long flights.** Two 30-minute runs (1,800 s, about 1,726 s airborne):

| run | prior | faults | station / excursion max | tilt max | altitude |
|---|---|---|---|---|---|
| gusts 3 N for 1 s every 6 s, the whole flight | flight default v5, plasticity off | 0 | 2.65 m settled, 3.57 m in gusts | 20.0° | error ≤ 0.13 m |
| hover with plasticity **on** | learning demonstration (trained, 480 bits) | 0 | 0.595 m | 5.8° | 1.11 m ± 1.4 cm |

The second run is the 30-minute Gazebo evidence for online learning
without drift (gap register G5); the flight default has no learning to
test. Arc-process loss on v5 (also Track H) is in
`docs/bridge_loss_method.md`.

## Direct rig: Gazebo IMU in, motor topic out (added 2026-10-20)

Method (`tools/px4/run_scenario.py --rig direct --arc-side container`,
campaign `tools/px4/run_g11.sh`, sequential, fresh container per run, host
CPU under 25 % before each run, every run kept):

- **Sensor path.** `tools/px4/gz_imu_stream.py` replaces the MAVLink record
  streamer: one v3 record per Gazebo IMU sample (250 Hz on the stock world),
  stamped with its sim time; rates, gyro and accelerometer from Gazebo's IMU
  topic (body FLU to FRD); PX4's attitude propagated from its own stamp to
  the sample with the gyro; PX4's position, velocity and baro as received.
  PX4 is still the estimator, and the fallback for G6.
- **Command path.** Every bridge tick (`--every 1`) to the motor publisher,
  which publishes each line on arrival (`gz_motor_pub.py --immediate`)
  instead of re-sending the newest on its 250 Hz grid.
- **Witness.** `--witness` runs `tools/px4/gz_witness.py` beside either rig:
  Gazebo's IMU, motor command, motor-failure and model-pose topics on the sim
  clock; `tools/px4/rig_latency.py` turns it into failure-to-command latency
  and the true catch tilt.
- **Unchanged.** The bridge binaries under `bin/` (the G10 ones), the
  models, every golden hash; the MAVLink rig stays the
  default. Dropout and random-schedule scenarios are refused on the direct
  rig (it does not model record dropouts).

v5 regressions, mode off (G9 matrix: 1 m, 60 s; wind 3 N + 1.5 N for 1 s
every 6 s from 15 s; evidence table `docs/px4_sitl_results/g11/summary.md`; records in the private
repository):

| | survived | station / excursion | peak tilt | altitude error max |
|---|---|---|---|---|
| hover, direct rig | 3/3 | station 0.23–0.25 m | 3.0–3.1° | – |
| hover, G9 (MAVLink rig) | 3/3 | station 0.24–0.30 m | 3.3–3.4° | – |
| wind, direct rig | 3/3 | excursion 3.09–3.21 m | 17.6–17.7° | 0.09–0.11 m |
| wind, G9 (MAVLink rig) | 3/3 | excursion 2.96–3.05 m | 17.6–18.4° | 0.09–0.11 m |

Hover is the same or tighter; the wind excursion is 0.05–0.16 m larger on
the direct rig (tilt and altitude the same). Not attributed: the two rigs
differ in rate source (PX4's filtered rates against the raw IMU), record
rate and command timing at once, and G9 ran on another day.

One run is kept as `g11_hover_v5_2_streamer_exit_crash`: the direct
streamer crashed at exit (pybind11, while gz-transport was inside its
callback), 0.6 s before the end of the stream and after the 60 s window; the
exit was fixed (no unsubscribe, `os._exit` after closing) and the run
repeated. Machine-wide stalls of the Docker VM were 0.8–1.05 s in this
session (stall probe), about twice per run in every run on both rigs; sim
time advanced 4 ms through each (Gazebo stops too).

Rotor-loss results on this rig: `docs/rotor_loss.md` 4 (catch median 33°
against 81° on the MAVLink rig, 12/12 held). G6 handover on this rig:
`docs/bridge_loss_method.md` (13/13).

## What this does and does not show

It shows the runtime's timing, fault contract and determinism on
PX4-grade physics and sensors, an Arc-in-the-loop controller that
matches PX4's own on the same plant, a trained prior flying the plant
on its own authority (looser than PX4 on station, weaker under gusts),
the measured limits of the hover-hold safe state, and a deterministic
per-tick instruction count on Cortex-M with a static cycle bound from it,
a frozen estimator survived for up to 2 s on the raw IMU (trained prior
and flight default), and 30-minute flights in gusts (flight default) and
with plasticity on (learning demonstration) without a fault. Silicon cycle counts remain the
hardware-day item, now with a predicted range to confirm.
