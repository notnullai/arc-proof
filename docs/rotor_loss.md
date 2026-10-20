# Rotor-loss mode (G10)

*Public copy.* The source files named below (`train/sim/…`, `integrations/px4/…`,
`tests/…`, `tools/*.cpp`) are in the private repository. This repository ships
what is needed to check the results: the bridge binaries under `bin/` (the mode
is in them, off by default, `--rotor-loss`), the model
`models/prior_x500_rotor_loss.arc002`, the Gazebo harness (`tools/px4/`:
`run_scenario.py --rotor-loss … --motor-scale …`, `run_g10.sh`, `run_g10_hold.sh`, `run_g10_spread.sh`,
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
| Detector | `rotor_loss_detect.hpp` | per record: measured angular acceleration over ≥ 15 ms minus the one predicted from Arc's own rotor commands (gz motor-lag model, clamped at saturation, hover-calibrated); fires when the residual points along one rotor's direction (35° cone), exceeds both 6 rad/s² and 0.4 of the torque that rotor makes now (a dead rotor removes all of it, ~57 rad/s² at hover; a 3.35 N Gazebo gust makes 6.4), and appeared as a step (under 0.15 of that torque 100 ms earlier; a stalled command path ramps up instead), stays under 1.3× of that torque (a dead rotor removes its own torque, no more; an attitude kick reads 1.5× and up), and is not explained by frozen actuation (the motors kept a command from up to 600 ms earlier: the residual under that hypothesis is under 0.25 of the suspect's torque and under half the healthy-airframe residual; a dead rotor's missing thrust is in no frozen command); arms after 2 s of level flight and only while its own residual (10 s RMS) has stayed under 5 rad/s² for 3 s, so a limit-cycling or badly modelled vehicle disarms it |
| Arbitration | `Px4Bridge`, `--rotor-loss <brain>` | default off (bit-identical flight default). On: provisional hand-over at the first detection; the suspect rotor keeps its last command until the next window confirms it is dead (its thrust still missing); a rotor found alive (a kick) hands back to v5 with its state intact; confirmation needs 0.6 of the suspect's torque still missing, along the suspect. On the tick whose record fires the detector, the rotor opposite the suspect is cut at once (`rotor_loss_preempt`; section 3, the catch). From the hand-over on the mode flies on its own attitude, integrated from the record rates (clocked by the record when the estimator's attitude stamp is frozen), and from 2 s after it (blended over 1 s) on its own height and climb rate, baro and accelerometer fused through that attitude (PX4's attitude and vertical velocity fail in the spin, section 4). Confirmed: rotor-loss brain for the rest of the flight. The loop runs in 1-tick chunks only when on |
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
stream the Gazebo runs use, delay 0 / 20 / 40 ms, and the bridge's command
held for a record period as Gazebo holds it (section 4, item 4; added in this
revision: the earlier table had no hold and flattered the SITL rows). On-board:
a 1 kHz gyro as on the flight controller, delay 0 / 2 / 4 ms. Evidence:
`docs/rotor_loss_results/hermetic_matrix.txt`, the same without the
pre-emptive cut `hermetic_matrix_no_preempt.txt`, and per delay
`hermetic_catch_summary.md`.

| airframe | stream | held | landed < 3 m/s | landed ≥ 3 m/s | wrong / missed | detection ms | max touchdown m/s | spin rad/s |
|---|---|---|---|---|---|---|---|---|
| nominal (T/W 1.69) | SITL 50 Hz | 0 | 0 | 72 | 0 / 0 | 17–76 | 22.7 | ≤ 15 |
| nominal (T/W 1.69) | on-board 1 kHz | 0 | 5 | 67 | 0 / 0 | 12–22 | 7.1 | ≤ 15 |
| mass ×1.2 (T/W 1.41) | SITL | 0 | 0 | 72 | 0 / 0 | 16–75 | 14.1 | ≤ 13 |
| mass ×1.2 (T/W 1.41) | on-board | 0 | 0 | 72 | 0 / 0 | 13–22 | 12.6 | ≤ 14 |
| thrust ×0.8 (T/W 1.35) | SITL | 0 | 0 | 72 | 0 / 1 | 18–80 | 15.5 | ≤ 12 |
| thrust ×0.8 (T/W 1.35) | on-board | 0 | 0 | 72 | 0 / 0 | 12–22 | 13.8 | ≤ 12 |
| thrust ×1.1 (T/W 1.86) | SITL | 18 | 11 | 43 | 0 / 3 | 19–78 | 22.7 | ≤ 16 |
| thrust ×1.1 (T/W 1.86) | on-board | 42 | 17 | 13 | 0 / 0 | 12–23 | 4.0 | ≤ 16 |
| thrust ×1.2 (T/W 2.03) | SITL | 36 | 4 | 32 | 0 / 24 | 18–76 | 22.9 | ≤ 17 |
| thrust ×1.2 (T/W 2.03) | on-board | **68** | 4 | 0 | 0 / 0 | 12–22 | 2.5 | ≤ 16 |
| mass ×0.8 (T/W 2.11) | SITL | 23 | 2 | 47 | 0 / 45 | 14–53 | 23.1 | ≤ 15 |
| mass ×0.8 (T/W 2.11) | on-board | **65** | 7 | 0 | 0 / 0 | 12–23 | 2.8 | ≤ 15 |

The catch, ×1.2 thrust (catch tilt = the largest tilt in the first 1.5 s;
24 cases per row; pre-emptive cut off → on):

| stream, delay | held / soft / hard | catch tilt min–median–max | max height lost when held | max touchdown |
|---|---|---|---|---|
| on-board, 0 ms | 23/1/0 → 23/1/0 | 9–20–61° → 9–19–61° | 5.9 → 5.8 m | 2.5 → 2.4 m/s |
| SITL, 0 ms | 15/6/3 → **21/3/0** | 20–59–95° → **9–35–78°** | 17.3 → **11.6 m** | 4.2 → **2.9 m/s** |
| SITL, 20 ms | 9/1/14 → 14/1/9 | 37–85–120° → 15–60–97° | 21.4 → 14.7 m | 6.8 → 5.6 m/s |
| SITL, 40 ms | 1/0/23 → 1/0/23 (22 not armed) | – | – | – |

Reading it:

- **T/W ≥ 2.0 with the gyro on board: relaxed hover from 10 m up in every
  case** (height lost ≤ 7 m, drift ≤ 2.3 m, spin ~15 rad/s; median catch
  tilt 20°); from 3 m, 6–7 of 8 hold and the rest touch down softly.
- **The stock x500 (T/W 1.69) cannot hold altitude after losing a rotor.**
  The mode keeps it the right way up and spinning while the pair runs at its
  limit, and it comes down: up to 7 m/s on board, 23 m/s through the SITL
  stream.
- **T/W ≤ 1.41: a fall with the attitude held.**
- **What sets the catch.** The dead rotor leaves the opposite rotor's torque
  unbalanced; nothing but that rotor's thrust going can stop the tip (the
  pair's differential acts across it, and the dead rotor's direction has no
  rotor left). Until it is gone the body rolls up to ~3 rad/s; after that
  only the yaw spin turning the frame (the pair at full thrust, ~6.5 rad/s²
  of yaw) can catch it. So the catch tilt follows the time from the failure
  to the opposite rotor's spin-down: detection latency, command path and its
  25 ms motor lag. On board that is ~30 ms (median 20°); through the 50 Hz
  stream with Gazebo's command hold, 50–90 ms.
- **The pre-emptive cut.** The detecting record is accepted inside a tick
  that v5 has already computed; under the 20 ms SITL hold the mode's first
  command then acted a record later. The bridge now zeroes the rotor
  opposite the suspect on that very tick (`rotor_loss_preempt`, on). Through
  the SITL stream it takes 15–25° off the median catch, 4–7 m off the height
  lost and turns 3–5 hard landings in 24 into holds; on board, where the
  next tick is 1 ms away, it changes 1–3°. The cut is one tick; a
  provisional hand-over found wrong hands back as before.
- **Where the detector cannot arm.** With 20–40 ms of stream delay on the
  light (mass ×0.8) and the strong (thrust ×1.2) airframe, v5 itself
  oscillates under the SITL hold (11–22° of tilt before any failure, model
  residual RMS up to 26 rad/s²); the detector's health check keeps it
  disarmed, and 69 of those 96 failures are not detected (3 more on thrust ×1.1 at 40 ms). That is v5's
  delay limit (G4) showing through, not a detector miss on a healthy model;
  it is listed in section 5.

The detector's thresholds (section 2) were set after Gazebo runs of
section 4 (a gust's torque, a stalled command path): the hermetic gusts now
carry that torque, detection and confirmation are scaled to the suspect
rotor's own torque (~57 rad/s² at hover), and a step onset is required. The
price: detection 3–8 ms slower on board.

False triggers (`--false-trigger 60`, evidence
`docs/rotor_loss_results/hermetic_false_trigger.txt`): 55 min of flight
with 6 N gusts every 6 s (with their torque), mass ×0.8 / ×1.2, thrust ×0.8
/ ×1.2, both streams (SITL with the command hold), delays 0 / 20 / 40 ms; a
third of the flights add 180 attitude kicks (±2 rad/s per axis, the
random-fault sweep's range), a third add 120 actuation stalls (the plant
keeps its last command for 100–400 ms, as Gazebo showed, section 4); no
rotor failing; the mode hands over for real. **0 false confirmations**: 3
provisional hand-overs, all handed back to v5 (28 before the kick ceiling,
below). Crashes: 16 kick and stall flights crash with and without the mode
alike; 2 gust-only flights at 40 ms crash with no detection at all (v5
alone). The stalls first gave 1 false confirmation in 120: 106 ms into a
328 ms stall the residual along one rotor had built up to 0.42 of its
torque, 0.14 of it 100 ms earlier (just under the step-onset limit), and
the mode flew a healthy vehicle as a three-rotor one into the ground. The
frozen-actuation veto (section 2) removed it; the matrix was identical case
for case with and without it. The veto acts on the detection only: at
confirmation the mode's own compensation makes a dead rotor look like a
freeze (tried: 25 real losses handed back). Largest gust-only residual along
a rotor 16.5 rad/s² (detection needs ~23 at hover). Normal flight is
unchanged: with the mode on and no detection v5's rotor commands are
bit-identical to the mode off (ctest).

**Random faults** (`--random-faults 300`, evidence
`hermetic_random_faults_replay.txt`; ctest gate 6 replays the worst
episodes). `test_x500_random_faults`' own schedules, the same generator and
seeds, flown on the full rigid body through the real bridge with v5 and the
mode on, both streams, 1 m hover, from 6 s (armed): its gusts as velocity
steps, its attitude kicks, record dropouts, frozen estimator, frozen
horizontal position, and its motor losses (estop pulses are an input of its
own loop, not of the bridge, and are not replayed). The first replay found
two real faults that the 6-DoF-lite sweep hid:

1. **A frozen estimator made false detections.** The detector took its clock
   from the record's attitude stamp; when the estimator freezes, that stamp
   stops while the rates stay fresh, and fresh rates differenced over a
   stopped clock read as a huge residual: 1 false confirmation and 4
   provisional hand-overs that crashed a 1 m hover v5 survives. Gazebo's
   estimator-stall records freeze the stamp the same way. The bridge now
   uses the record time whenever the attitude stamp is more than 40 ms old,
   for the detector and for the mode's own attitude.
2. **Attitude kicks read as rotor losses.** A ±2 rad/s rate step makes a
   residual of 1.5× the rotor's whole torque and more; a dead rotor can only
   remove its own. Detections above 1.3× are now refused.

After both: **600 flights, 0 false confirmations; crashes 129 with the mode
and 129 without, the same flights**; 94 of 106 rotor losses confirmed on
the right rotor, the other 12 lost next to a dropout or a kick that keeps
the detector disarmed, and all 12 crash at 1 m with or without the mode.

The same sweep's observer on the 6-DoF-lite `DronePlant`
(`random_faults_observer_v5.txt`) still counts 1–4 would-be confirmations
per 300 episodes (5–6 before these fixes). There v5 hovers in a 1 Hz
bang-bang thrust cycle (all four rotor commands swing 0 ↔ maximum, the
residual RMS above 5 rad/s² in 97 % of records), the detector is disarmed
nearly always and detects none of the 53 motor losses, and the observer
cannot fly the mode's provisional window. The replay above is the
evaluation; the observer stays as a cross-check on a plant the detector
does not model.

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

**The spread, settled with more runs** (`gazebo_spread_nocut/`,
`gazebo_spread_cut/`: 12 runs each, 6 at 10 m and 6 at 30 m, rotors mixed;
with `gazebo_hold/`, 34 runs of the ×1.2 airframe with the current height
logic). The catch tilt is the largest tilt in the first 1.5 s after the
onset, while PX4's attitude still follows its rates.

| | 30 m held | lowest point when held | 10 m touchdown | catch tilt |
|---|---|---|---|---|
| `gazebo_hold/` (10 runs) | 6/6 | 6.5–19.3 m | 4.1–6.3 m/s (4) | 67–101°, median 93° |
| without the pre-emptive cut (12) | 6/6 | 4.1–9.1 m | 1.8–5.6 m/s, one 12.6 (6) | 71–110°, median 93° |
| with the cut (12) | 4/6 (2 tumbled, 21–22 m/s) | 6.0–14.1 m | 2.6–6.1 m/s, one 12.5 (6) | 73–107°, median 93° |
| all 34 | 16/18 | 4.1–19.3 m (11–26 m lost) | 1.8–6.3 m/s, 2 of 16 above 12 m/s | 67–110°, median 93° |

So the spread is real and wide: the catch in Gazebo peaks at 70–110° (not
80° as written before), the dip at 30 m is 11–26 m, and about one failure
in eight at 10–30 m tips past ~105° and is lost (the two 30 m tumbles and
the two 12.5 m/s landings). What decides it is the peak roll rate the
vehicle reaches before the opposite rotor's thrust is gone: the four lost
runs peaked at 4.45–4.6 rad/s, the held runs and soft landings at 3.2–4.2.

**The cut does not shorten the Gazebo catch.** The median peak roll rate is
4.0 rad/s without the cut and 4.1 with it (12 runs each), and the rate
keeps growing for 60–80 ms after the detecting record in both; the command
log shows the cut on the detecting tick and the mode's commands 2 ms later.
The latency sits in the SITL pipeline, not in the bridge's tick order: each
record's rates are 12–16 ms older than the record, the record reaches the
bridge after the MAVLink path, and the command goes back through a pipe to
the Gazebo publisher, so the cut acts 20–40 ms of sim time after the
detecting record. The hermetic 20 ms command hold therefore overstated
Gazebo's command path; the hermetic SITL gain of the cut (section 3) is not
seen in Gazebo. The cut stays in (one tick, 1–3° on board, no effect in
Gazebo either way: 2 of 6 tumbles with it against 0 of 6 and 0 of 6
without are within the peak-rate spread), and the gain claimed is the
hermetic one only. Shortening the Gazebo catch needs a shorter sensor and
command path (the Arc side reading Gazebo's IMU and motor topics directly,
or lockstep), not a controller change; on the real vehicle Arc runs on board
with the gyro at 1 kHz, where the hermetic catch is 20° median.

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
- **The catch and the height to recover.** In Gazebo the vehicle tips to
  70–110° before the law catches it (median 93°, 34 runs) and about one
  failure in eight is lost (tips past ~105°: 2 of 18 at 30 m, 2 of 16 land
  at 12.5 m/s from 10 m); the held ones lose 11–26 m at 30 m. The cause is
  the SITL pipeline's 50–80 ms from the failure to the opposite rotor's
  thrust going (section 4); the controller cannot remove it, and the
  pre-emptive cut does not change it in Gazebo. With an on-board 1 kHz gyro
  the hermetic catch is 20° median (61–68° when the failure comes inside a
  gust) and the height lost ≤ 7 m. Not shown in Gazebo: a shorter sensor
  path there (follow-up).
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
- **False confirmations.** 0 in 55 min of gusts, kicks and 120 hermetic
  actuation stalls of 100–400 ms on the full rigid body (1 of 120 before
  the frozen-actuation veto); 0 in the random-fault sweep's schedules
  replayed on the full rigid body (600 flights; 1 before the
  estimator-clock fix); 1–4 per 300 would-be confirmations remain in the
  observer on the 6-DoF-lite plant, where v5 limit-cycles and the
  provisional window cannot be flown. A false confirmation turns a
  healthy vehicle into a spinning three-rotor one; it then holds only if
  the airframe has the margin. The detector assumes a controller that is
  not limit-cycling; a stall longer than 600 ms, or one that ends while a
  rotor is failing, is outside what the veto looks back over.
- **Where the flight default itself wobbles.** With 20–40 ms of extra
  sensor delay under the SITL command hold, on airframes with T/W ≥ 2.0,
  v5 oscillates (11–22° before any failure); the detector's health check
  keeps it disarmed, so a rotor lost there is not caught (69 of 96 such
  hermetic cases). It is v5's delay limit (G4), and it removes the rotor-
  loss protection with it.
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
