# Rotor-loss mode (G10)

*Public copy.* The source files named below (`train/sim/…`, `integrations/px4/…`,
`tests/…`, `tools/*.cpp`) are in the private repository. This repository ships
what is needed to check the results: the bridge binaries under `bin/` (the mode
is in them, off by default, `--rotor-loss`), the model
`models/prior_x500_rotor_loss.arc002`, the Gazebo harness (`tools/px4/`:
`run_scenario.py --rotor-loss … --motor-scale …`, `run_g10.sh`, `run_g10_hold.sh`,
`summarize_rotor_loss.py`) and every run's evidence under `docs/rotor_loss_results/`.

Owner (2026-10-19): "start the rotor-loss work (G10), hermetic first, then
Gazebo, with honest limits stated."

**End goal.** The defense hardware proof: a quad flown by Arc that loses a
rotor in flight should not tumble. The mode must work for other airframes
through a profile, leave the flight default (candidate v5) unchanged in
normal flight, and say exactly where the physics stops it.

**What was believed before.** G10 said "physics: no quad controller
recovers a lost rotor without yaw spin". That is true as far as it goes:
yaw has to be given up. With yaw given up, a quad can keep its thrust axis
and altitude in a *relaxed hover*, spinning about an axis fixed in the body:

- M. W. Mueller, R. D'Andrea, "Stability and control of a quadrocopter
  despite the complete loss of one, two, or three propellers", IEEE ICRA
  2014, pp. 45–52, doi 10.1109/ICRA.2014.6906588; extended in "Relaxed hover
  solutions for multicopters", IJRR 35(8), 2016, doi 10.1177/0278364915596233
  (flight experiments for one and for two opposing propellers lost).
- S. Sun, L. Sijbers, X. Wang, C. de Visser, "High-speed flight of quadrotor
  despite loss of single rotor", IEEE RA-L 3(4), 2018, pp. 3201–3207, doi
  10.1109/LRA.2018.2851028 (incremental nonlinear dynamic inversion, wind
  tunnel flights above 9 m/s).

## 1. Physics on this airframe first

`rotor_loss_design equilibrium` solves the relaxed-hover equilibrium on the
plant's own wrench (rotor 2 dead; the pair on the other diagonal at equal
thrust f; the opposite rotor at ρ f). Gazebo x500 parameters (read from the
`px4io/px4-sitl-gazebo` model files): 2.064 kg, I = (0.0217, 0.0217, 0.040)
kg m², rotor max 8.55 N (1000 rad/s), hover 5.06 N per rotor, so T/W = 1.69.

| ρ | pair thrust / rotor max | spin rad/s | spin-axis tilt |
|---|---|---|---|
| 0.4 | 1.011 | 17.0 | 9.1° |
| 0.5 | 0.990 | 15.3 | 13.6° |
| **0.6** | **0.984** | **13.9** | **19.3°** |
| 0.7 | 1.004 | 12.7 | 26.4° |

The best split (ρ = 0.6, the profile's point) needs the pair at 98.4 % of
its maximum: **the stock Gazebo x500 has 1.6 % of thrust left in relaxed
hover**, before any control or any spin-up loss. With thrust ×1.1 the pair
sits at 88.6 %. The numbers include the gz motor model's thrust loss with
axial hub speed (`1 − v/25`), which at the relaxed-hover body rates (3–4
rad/s) costs one rotor ~5 %.

Transient: when a rotor stops in hover the body tips toward it at about 57
rad/s², and no remaining rotor can push back in that direction (the
opposite rotor only adds to it, the pair only acts across it). Only the
spin, once it builds, turns that rate into an axis the pair can damp. The
time to detect the failure therefore decides how much height and tilt the
recovery costs.

## 2. What was built

All in the integration and training layers; `src/` is untouched and every
existing golden hash and model is unchanged (full ctest 91/91 with
`test_rotor_loss`).

| Piece | Where | What |
|---|---|---|
| Full 6-DoF plant | `train/sim/rigid6.hpp` | quaternion rigid body, Euler's equations with ω × Iω, RK4 at 1 kHz; the gz `MulticopterMotorModel`: rotor speed lag 12.5 ms up / 25 ms down, thrust k_f ω², axial-flow loss, rotor drag `−k_d ω v⊥` (the gz model's only damping), a rotor that stops. The existing `DronePlant` ("6-DoF-lite", no gyroscopic coupling, translation body-yaw aligned) cannot represent a spinning quad, and its goldens stay as they were |
| Law | `train/sim/rotor_loss.hpp` | reduced-attitude control after Mueller & D'Andrea in the inversion form of Sun et al.: a linear law on the reference axis error, its rate and integral, and on altitude; an inversion adds the gyroscopic and rotor-drag torques and divides by the spin geometry; a spin-up schedule moves the target from level to the relaxed-hover axis as the spin builds; one canonical frame serves all four rotors of a symmetric X quad (mirror / 180° rotation) |
| Mixer | same | torque first: the pair carries the torque across its diagonal by its differential, the opposite rotor the one-sided torque along it, the collective takes what fits; per-rotor limits include the axial-flow loss |
| Calibration | `rotor_loss_detect.hpp` | rotor command per newton of weight, learned in level flight before the failure; mass and thrust errors act only through it when payload scales mass and inertia together |
| Position loop | `RotorLossCtl::inputs` | reference generator in the integration layer: world up tilted by a = −k_p p − k_d v (filtered, the CoM circles at the spin rate), \|a\| ≤ 1.5 m/s² |
| Compiled brain | `rotor_loss_frame.hpp`, `tools/rotor_loss_compile.cpp`, `models/prior_x500_rotor_loss.arc002` | the linear law (all reduced-attitude and altitude gains) as ternary weights over a gain-matched 736-bit frame, as the flight default v5 was built; 690 nonzero weights, no hidden neurons, nothing trained, plasticity off; sha256 `b134915b…80f`, regenerated bit-for-bit by `rotor_loss_compile` |
| Detector | `rotor_loss_detect.hpp` | per record: measured angular acceleration over ≥ 15 ms minus the one predicted from Arc's own rotor commands (gz motor-lag model, clamped at saturation, hover-calibrated); fires when the residual points along one rotor's direction (35° cone), exceeds both 6 rad/s² and 0.4 of the torque that rotor makes now (a dead rotor removes all of it, ~57 rad/s² at hover; a 3.35 N Gazebo gust makes 6.4), and appeared as a step (under 0.15 of that torque 100 ms earlier; a stalled command path ramps up instead), and is not explained by frozen actuation (the motors kept a command from up to 600 ms earlier: the residual under that hypothesis is under 0.25 of the suspect's torque and under half the healthy-airframe residual; a dead rotor's missing thrust is in no frozen command); arms after 2 s of level flight and only while its own residual (10 s RMS) has stayed under 5 rad/s² for 3 s, so a limit-cycling or badly modelled vehicle disarms it |
| Arbitration | `Px4Bridge`, `--rotor-loss <brain>` | default off (bit-identical flight default). On: provisional hand-over at the first detection; the suspect rotor keeps its last command until the next window confirms it is dead (its thrust still missing); a rotor found alive (a kick) hands back to v5 with its state intact; confirmation needs 0.6 of the suspect's torque still missing, along the suspect. From the hand-over on the mode flies on its own attitude, integrated from the record rates, and from 2 s after it (blended over 1 s) on its own height and climb rate, baro and accelerometer fused through that attitude (PX4's attitude and vertical velocity fail in the spin, section 4). Confirmed: rotor-loss brain for the rest of the flight. The loop runs in 1-tick chunks only when on |
| Tests | `tests/test_rotor_loss.cpp` (ctest) | engine emissions = compiled law on 20,000 random frames; v5 with the mode on and off: identical rotor commands over 59 s of gusts; ×1.2-thrust holds and nominal descents for all four rotors |

What the ternary brain computes and what it does not: it computes the
whole feedback law of the inner loop (every gain of the reduced attitude and
the altitude). The integration layer computes the geometry around it: the
reference axis from the position loop, the record propagation, the dynamic
inversion and the rotor allocation, as the bridge's axis mixer does for v5.

## 3. Hermetic results (real bridge, v5 + detector + rotor-loss brain, full 6-DoF x500)

`test_rotor_loss <v5> <rl> --matrix`: 4 rotors × {hover, mid-gust} × {3, 10,
30 m} × 6 airframes × 3 delays × 2 record streams = 864 cases; the failure
comes 8 s after the station is reached (the detector arms after 2 s of level
flight and 3 s of a healthy model), in the gust cases 0.5 s into the next
3 N + 1.5 N pulse, which acts 3.5 cm off the centre of mass (the tipping
Gazebo's wind wrench gives, section 4); failure phase against the record
clock seeded per case. SITL stream: records at 50 Hz as the PX4 MAVLink
stream the Gazebo runs use, delay 0 / 20 / 40 ms. On-board: a 1 kHz gyro as
on the flight controller, delay 0 / 2 / 4 ms. **Every case: the right rotor
identified, none missed.** Evidence: `docs/rotor_loss_results/hermetic_matrix.txt`.

| airframe | stream | held | landed < 3 m/s | landed ≥ 3 m/s | detection ms | max height lost m | max touchdown m/s | max tilt | max drift m | spin rad/s |
|---|---|---|---|---|---|---|---|---|---|---|
| nominal (T/W 1.69) | SITL 50 Hz | 0 | 0 | 72 | 17–76 | (descent) | 21.0 | 127° | 5.3 | ≤ 15 |
| nominal (T/W 1.69) | on-board 1 kHz | 0 | 4 | 68 | 12–22 | (descent) | 7.1 | 78° | 2.4 | ≤ 15 |
| mass ×1.2 (T/W 1.41) | SITL | 0 | 0 | 72 | 16–75 | (fall) | 14.9 | 124° | 5.3 | ≤ 14 |
| mass ×1.2 (T/W 1.41) | on-board | 0 | 0 | 72 | 13–22 | (fall) | 12.7 | 79° | 2.5 | ≤ 14 |
| thrust ×0.8 (T/W 1.35) | SITL | 0 | 0 | 72 | 19–77 | (fall) | 22.8 | 130° | 5.7 | ≤ 12 |
| thrust ×0.8 (T/W 1.35) | on-board | 0 | 0 | 72 | 12–22 | (fall) | 13.8 | 93° | 2.3 | ≤ 12 |
| thrust ×1.1 (T/W 1.86) | SITL | 22 | 9 | 41 | 17–78 | 30 | 12.1 | 121° | 4.9 | ≤ 16 |
| thrust ×1.1 (T/W 1.86) | on-board | 42 | 17 | 13 | 12–23 | 17.7 | 4.1 | 74° | 2.3 | ≤ 16 |
| thrust ×1.2 (T/W 2.03) | SITL | 46 | 3 | 23 | 18–77 | 30 | 6.5 | 121° | 4.9 | ≤ 17 |
| thrust ×1.2 (T/W 2.03) | on-board | **68** | 4 | 0 | 12–22 | **7.1** | 2.6 | 68° | **2.3** | ≤ 16 |
| mass ×0.8 (T/W 2.11) | SITL | 48 | 5 | 19 | 14–79 | 16.5 | 8.5 | 128° | 5.8 | ≤ 15 |
| mass ×0.8 (T/W 2.11) | on-board | **65** | 7 | 0 | 12–23 | **6.1** | 2.8 | 79° | **2.2** | ≤ 15 |

By height and delay (8 cases each = 4 rotors × hover/gust):

| airframe, stream | 3 m | 10 m | 30 m |
|---|---|---|---|
| thrust ×1.2, on-board, 0/2/4 ms | 6–7 held, the rest landed < 3 m/s | 8/8 held | 8/8 held |
| mass ×0.8, on-board | 5–7 held, the rest < 3 m/s | 8/8 held | 8/8 held |
| thrust ×1.1, on-board | 0 held; 5–6 < 3 m/s, 2–3 hard | 6/8 held, 1–2 hard | 8/8 held |
| thrust ×1.2, SITL 0 ms | 6 held, 2 < 3 m/s | 8/8 held | 8/8 held |
| thrust ×1.2, SITL 20 ms | 3 held, 5 hard | 6 held, 2 hard | 8/8 held |
| thrust ×1.2, SITL 40 ms | 1 < 3 m/s, 7 hard | 8 hard | 7 held, 1 hard |

Reading it:

- **T/W ≥ 2.0 with the gyro on board: relaxed hover from 10 m up in every
  case** (height lost ≤ 7 m in the worst mid-gust case, drift ≤ 2.3 m,
  spin ~15 rad/s); from 3 m, 6–7 of 8 hold and the rest touch down softly.
- **The stock x500 (T/W 1.69) cannot hold altitude after losing a rotor.**
  The mode keeps it the right way up and spinning while the pair runs at its
  limit, and it comes down: from 3 m softly in 4 of 24 on-board cases, from
  10 and 30 m hard (up to 7 m/s on board, 21 m/s through the SITL stream).
- **T/W ≤ 1.41: a fall with the attitude held.**
- **Detection latency decides the rest.** 12–23 ms with an on-board gyro,
  14–79 ms through the 50 Hz MAVLink stream; at 40 ms of stream delay most
  cases below 30 m land hard.

The detector's thresholds (section 2) were set after Gazebo runs of
section 4 (a gust's torque, a stalled command path): the hermetic gusts now
carry that torque, detection and confirmation are scaled to the suspect
rotor's own torque (~57 rad/s² at hover), and a step onset is required. The
price: detection 3–8 ms slower on board.

False triggers (`--false-trigger 60`, evidence
`docs/rotor_loss_results/hermetic_false_trigger.txt`): 56 min of flight
with 6 N gusts every 6 s (with their torque), mass ×0.8 / ×1.2, thrust ×0.8
/ ×1.2, both streams, delays 0 / 20 / 40 ms; a third of the flights add 180
attitude kicks (±2 rad/s per axis, the random-fault sweep's range), a third
add 120 actuation stalls (the plant keeps its last command for 100–400 ms,
as Gazebo showed, section 4); no rotor failing; the mode hands over for
real. **0 false confirmations**: 28 provisional hand-overs, all handed
back to v5. Crashes: 12 with the mode, 12 without (400 ms stalls and kicks
v5 itself does not survive). The stalls first gave 1 false confirmation in
120: 106 ms into a 328 ms stall the residual along one rotor had built up
to 0.42 of its torque, 0.14 of it 100 ms earlier (just under the step-onset
limit), and the mode flew a healthy vehicle as a three-rotor one into the
ground. The frozen-actuation veto (section 2) removed it; the 864-case
matrix is identical case for case with and without the veto (same
detection times, same outcomes). The veto acts on the detection only: at
confirmation the mode's own compensation makes a dead rotor look like a
freeze (tried: 25 real losses handed back). Largest gust-only residual along a rotor 16.5 rad/s² (detection
needs ~23 at hover). Normal flight is unchanged: with the mode on and no
detection v5's rotor commands are bit-identical to the mode off (ctest).

Existing random-fault sweep (`test_x500_random_faults <v5> 300 all
--rotor-loss-observer`, evidence `random_faults_observer_v5.txt`): the
detector rides along as an observer on the 6-DoF-lite `DronePlant`; the
sweep's own output is identical with and without it. There v5 hovers in a
1 Hz bang-bang thrust cycle (all four rotor commands swing 0 ↔ maximum while
the altitude stays at 1.00 ± 0.02 m): the residual RMS sits above 5 rad/s²
in 97 % of records, so the health check keeps the detector disarmed nearly
always, and none of the 53 motor losses is detected. **4–5 failure-free
episodes per sweep of 300 (about 1.5 %) would still be confirmed as a rotor
loss** (5–6 before the frozen-actuation veto) in the short stretches where the cycle calms down (after gusts,
kicks, dropouts). On the full rigid body, where v5 flies smoothly, the same
logic made 0 false confirmations in 60 min. The finding stands as a limit:
the detector is only safe on a vehicle whose controller is not limit-cycling.

## 4. Gazebo (PX4 SITL x500)

`tools/px4/run_g10.sh` (sequential, fresh container per run, each run only
after no other scenario was running and host CPU had been under 25 %; Arc
side in the container next to PX4 as in G6, static Linux bridge from this
branch; v3 records). `--fail-motor` is the gz MotorFailure index, 1-based
(0 = none): 1–4 fail gz rotors 0–3. The stronger airframe is the stock
model with every rotor's `motorConstant` ×1.2 (`--motor-scale 1.2`, the
`.sdf` kept with the evidence). Evidence: `docs/rotor_loss_results/gazebo/`
(run JSON, records, logs, command-gap witnesses; table `summary.md`, made by
`tools/px4/summarize_rotor_loss.py`).

**Normal flight with the mode on** (no failure; the mode never acts on
v5's commands until a detection):

| run | result | detections | largest residual along a rotor |
|---|---|---|---|
| hover, 1 m, 60 s | station 0.22 m, tilt 3.6°, altitude error 0.05 m (G9 v5: 0.24–0.30 m, 3.3–3.4°) | 0 | 3.4 rad/s² |
| wind, 3 N + 1.5 N every 6 s, 60 s | excursion 2.97 m, tilt 19.1°, altitude error 0.12 m (G9 v5: 2.96–3.05 m, 17.6–18.4°, 0.09–0.11 m) | 0 | 9.7 rad/s² |
| control flights at 10 m, stock and ×1.2 motors, 70 s | altitude error ≤ 0.05 m, station ≤ 0.24 m | 0 | 3.6 / 4.6 rad/s² |

**Rotor failure** (onset = the first record whose angular acceleration
exceeds 20 rad/s²; the detection lands on that record):

| airframe, height | runs | rotor identified | confirmed after onset | outcome |
|---|---|---|---|---|
| stock x500 (T/W 1.69), v5 alone | 1 | – | – | flips (171°), hits at 13.3 m/s |
| stock x500, with the mode, 10 m | 4 (each rotor) | 4/4 | 16–24 ms | falls: touchdown 7.6–8.8 m/s in 3 runs, 13.0 m/s in one (tipped past 90° before control) |
| ×1.2 motors (T/W 2.03), v5 alone | 1 | – | – | flips (169°), hits at 13.5 m/s |
| ×1.2 motors, with the mode, 10 m | 4 | 4/4 | 16–24 ms | touchdown 2.9–5.5 m/s after 2.3–3.3 s, upright at contact |
| ×1.2 motors, with the mode, 30 m, PX4's height | 4 | 4/4 | 16–24 ms | relaxed hover held to the end of the run (36 s): lowest point 10.6–13.1 m, then back at the station; last 20 s altitude 24.5–33.6 m, position within 3.3 m (`gazebo/`) |
| **×1.2 motors, with the mode, 30 m, own height (current)** | **6 (rotor 4 three times)** | **6/6** | **16–24 ms** | **held to the end (36–37 s): lowest point 6.5–19.3 m; last 20 s altitude 29.1–30.6 m, position within 1.5 m, spin 14.1–15.4 rad/s** (`gazebo_hold/`) |
| ×1.2 motors, with the mode, 10 m, own height (current) | 4 | 4/4 | 16–24 ms | touchdown 4.1–6.3 m/s after 2.1–2.7 s (`gazebo_hold/`) |

So in Gazebo, as in the hermetic plant: the stock x500 cannot hold after a
rotor loss; with the thrust margin it holds, but the transient (detection
through a 50 Hz MAVLink stream, the tip before control, the thrust-limited
spin-up) costs 11–24 m of height, and from 10 m it reaches the ground first,
at 2.9–6.3 m/s on the ×1.2 airframe. The first 2 s after the hand-over run
the same code in both campaigns; the 10 m runs end inside them, so their
spread (2.9–5.5 then 4.1–6.3 m/s) is Gazebo's run to run, and so is most of
the spread of the lowest point at 30 m.

The hold: with PX4's height the altitude swung ±4.5 m around the station
(a ~5.5 s cycle between full and minimum collective); with the mode's own
height it stays within 29.1–30.6 m (hermetic: ±0.2 m). Cause and fix in
item 5 below.

What Gazebo found that the hermetic plant did not, each fixed and each kept
as evidence:

1. **Gusts carry torque.** A 3.35 N wind pulse tipped the x500 at 4–6
   rad/s², and the first detector (a flat 6 rad/s² threshold) confirmed a
   rotor loss in normal flight and cut a healthy rotor
   (`g10_wind_rl_detector_v1_false_trigger.*`). Since then detection and
   confirmation are scaled to the suspect rotor's own torque, and the
   hermetic gusts act off the centre of mass.
2. **PX4's attitude estimate fails in the spin.** Comparing each recorded
   attitude with the previous one propagated by the recorded body rates, the
   disagreement per 20 ms record grows with the spin: ≤ 0.2° up to 10 rad/s,
   0.5° at 13, 1.3° at 14.3, 3–8° at 15 rad/s; the law then acted on a wrong
   tilt and diverged (`g10_host_ekf_spin_divergence.*`). The mode now
   carries its own attitude from the hand-over on (record rates integrated,
   seeded from the estimator while the spin is slow).
3. **The command path can stall.** In one run with the Arc side on the
   Windows host, commands stopped reaching Gazebo for 348 ms of sim time
   while records kept arriving; the vehicle drifted on a frozen command, the
   detector read "commanded torque not happening" along one rotor and
   confirmed it (`g10_host_stall_false_trigger.*`). The detector now
   requires a step onset; the campaign runs the Arc side in the container.
   Hermetic stalls of 100–400 ms still give 1 false confirmation in 120
   (section 3).
4. **Commands arrive in 20 ms bursts.** The bridge's clock follows record
   time, so a record's 20 ticks are computed at once and Gazebo holds the
   newest command for a record period; the hermetic harness reproduces it
   (`cmd_hold_us`), and it costs height in the transient.
5. **PX4's vertical velocity fails in the spin too.** In the held 30 m runs
   PX4 reported a descent of 2–4 m/s while its own altitude (and the baro)
   rose at 1–4 m/s: about 4.5 s after the failure its vertical velocity
   takes on a 2–4 m/s bias that stays (its attitude, which the velocity
   integration rotates through, is wrong; item 2). The altitude loop
   climbed on full collective while believing it was falling, overshot,
   cut to minimum, and repeated: the ±4.5 m cycle. The mode now flies on its
   own height and climb rate: the accelerometer through the carried
   attitude, fused with the barometer (second-order complementary filter,
   1.5 rad/s, seeded from PX4 at the hand-over; Gazebo baro noise 0.12 m,
   accelerometer 0.29 m/s² sd, both now in the hermetic records). Replayed
   on the earlier records, the filter and PX4 agree within 0.4 m and
   0.25 m/s for the first 4 s; then PX4's climb rate departs. Switching to it at the
   hand-over lost the attitude twice of two on rotor 4 at 30 m (the small
   difference in collective in the 80° transient, where the collective sits
   at the mixer's limit; `gazebo_hold_early_switch/`); the mode therefore
   keeps PX4's height for 2 s and blends over 1 s, before PX4's goes
   wrong.

## 5. Limits, stated

- **One rotor lost, nothing else.** Two or three rotors, a damaged
  propeller that still turns (partial thrust), a motor that stalls
  intermittently: not modelled, not tested.
- **Thrust margin.** Holding altitude needs T/W ≳ 1.86 with an on-board gyro
  and height; every tested case holds at T/W ≥ 2.0 from 10 m up hermetically,
  and from 30 m in Gazebo. The stock Gazebo x500 (1.69) only falls with the
  attitude held; heavier or weaker airframes fall faster.
- **Height to recover.** In Gazebo the transient costs 11–24 m on the ×1.2
  airframe (10 runs); from 10 m it lands (2.9–6.3 m/s). With an on-board
  1 kHz gyro the hermetic loss is ≤ 7 m. The tip reaches 80° before the
  law catches it, close to where the attitude is lost (> 90°): an early
  change of collective there was enough to lose it (section 4, item 5).
- **Yaw is given up.** The vehicle spins at 13–17 rad/s (2–2.7 rev/s) for
  the rest of the flight; payload pointing (camera, antenna) is lost.
- **Tilt.** The relaxed-hover axis is 19° off the body axis by design;
  transients reach 68–80° on board and 120–130° through the 50 Hz stream
  hermetically.
- **Altitude and position.** In Gazebo relaxed hover the altitude stays
  within 29.1–30.6 m of a 30 m station and position within 1.5 m (6 runs,
  last 20 s); hermetic ±0.2 m and 2.3 m. Position comes from PX4's
  horizontal estimate, which the spin may bias as it did the vertical one
  (not seen beyond 1.5 m). Position is held, not navigated.
- **Landing.** No commanded descent profile; an airframe that can hold
  holds the station altitude; a landing still spins.
- **Detection latency** depends on the sensor path: the 50 Hz MAVLink
  stream used in SITL costs height that an on-board 1 kHz gyro does not.
- **The estimator.** PX4's attitude and vertical velocity are not usable
  at the relaxed-hover spin (above); the mode flies on rates integrated from
  the hand-over, which drift with the gyro bias over minutes, and on baro
  and accelerometer through that attitude (an attitude error tilts gravity
  into the vertical channel; a constant accelerometer error b gives a
  height error b / 2.25 m and no climb-rate error). A baro disturbed by the
  spinning rotors' downwash is not modelled (Gazebo's baro has no
  aerodynamics). Tested up to 37 s in Gazebo and 25 s hermetically.
- **False confirmations.** 0 in 56 min of gusts, kicks and 120 hermetic
  actuation stalls of 100–400 ms on the full rigid body (1 of 120 before
  the frozen-actuation veto); about 1.5 % of failure-free episodes on the
  6-DoF-lite plant where v5 limit-cycles. A false confirmation turns a
  healthy vehicle into a spinning three-rotor one; it then holds only if
  the airframe has the margin. The detector assumes a controller that is
  not limit-cycling; a stall longer than 600 ms, or one that ends while a
  rotor is failing, is outside what the veto looks back over.
- **Arming.** Not armed for the first ~5 s of level flight; kicks and strong
  disturbances disarm it for a few seconds, and a rotor lost in that window
  is not detected until it re-arms.
- **Profiles.** The law assumes a symmetric X quad (one law for all rotors
  through the canonical frame); another airframe needs its own profile, an
  asymmetric one a profile per failed rotor (not built). Tested only on the
  x500 models.
- **Plant fidelity.** Rotor gyroscopics (1 % of the body's), the rolling
  moment, blade flapping, ground effect and body aerodynamics of a spinning
  vehicle are not modelled hermetically; Gazebo has the same rotor model and
  no body aerodynamics.
