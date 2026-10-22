# Arc process loss mid-flight: handover to PX4 (gap register G6)

Measured 2026-10-18 on PX4 SITL + Gazebo Harmonic x500 (`px4io/px4-sitl-gazebo`),
fresh container per run. The bridge source is private; the stripped bridge binary, the flight default and the harness are public, so these runs can be repeated: `docs/run_it_yourself.md`.

## The gap

In arc-owns mode PX4 is the estimator only: disarmed, motor outputs unassigned
(`SIM_GZ_EC_FUNC1..4 = 0`), and Arc's bridge writes the rotor commands. If the
Arc process dies or hangs, nothing commands the rotors. Measured without a
fallback: the vehicle crashes within 0.5 s of the kill (row `kill_reflex_nostandby`).

## The handover

The motor-command writer next to the simulator (`tools/px4/gz_motor_pub.py
--handover-ms 200`) is the watchdog, because it is the last hop before the
motors. When Arc's command stream closes (crash, kill) or goes silent for
200 ms of plant time (hang), it stops writing for good and hands the vehicle
to PX4 over one MAVLink connection it opened before flight (since 2026-10-19;
before that, a chain of PX4 shell-client calls):

1. route PX4's outputs to the four motors: four `PARAM_SET`
   (`SIM_GZ_EC_FUNCn = 100 + n`) in one burst, confirmed by their four
   `PARAM_VALUE` echoes;
2. wait until PX4's Gazebo bridge reports all four channels routed
   (`px4-gz_bridge status`, the one shell call left: no MAVLink message reports
   the mixer's routing);
3. arm PX4 in the air (force-arm `COMMAND_LONG`) and switch it to Takeoff
   (`DO_SET_MODE`) in one burst, confirmed by both `COMMAND_ACK`s. Takeoff
   climbs to `MIS_TAKEOFF_ALT` and switches itself to Hold.

That is the cold takeover (`--takeover cold`). Since 2026-10-22 the default is
the warm takeover (G13 below): the same routing, PX4 switched to Offboard while
still disarmed and armed with the arm command alone, a velocity-hold catch, and
Takeoff entered from the live state; the standby's hover thrust is kept at the
one being flown.

Authority is one-way: it never returns to Arc within a flight. Run with
`run_scenario.py --standby px4`; `--scenario bridge_loss --loss-kind kill|hang`
kills or suspends the bridge process mid-hover. `--arc-side container` runs the
Arc side next to PX4, as on a companion computer (see the v5 section).

Five PX4 behaviours shaped this design. Each was found by measurement:

| finding | consequence |
|---|---|
| PX4 applies output routing only while disarmed; a change made while armed is ignored (`gz_bridge status` keeps `func: 0`) | PX4 cannot sit armed and silent and then be routed at handover; it must be routed first, then armed |
| an armed PX4 with unrouted outputs winds up: its loops integrate errors its outputs cannot remove (thrust setpoint drove to 0 or to 1.0; rate integrators drove a constant torque) | the standby PX4 stays disarmed, so it has no state to wind up; takeover starts from fresh controllers |
| the mixer picks up routing on its next parameter update; arming before that left channels unrouted (random tumbles on takeover) | the handover waits until all four channels report routed before arming |
| in position modes PX4 forces zero thrust until its takeoff stage completes (1 s spool-up plus a 3 s ramp by default) | the fallback sets `COM_SPOOLUP_TIME = 0` and `MPC_TKO_RAMP_T = 0` at boot and uses Takeoff, so PX4 flies from its first cycle |
| with every output channel disabled, PX4's output mixer runs at its 300 ms low rate and applies new routing only on that tick (`_lowrate_schedule_interval`, `src/lib/mixer_module`): routing took 30–291 ms after the `PARAM_VALUE` echoes, which set most of the old 100–330 ms takeover | the standby sets `SIM_GZ_EC_FUNC5 = 1` (Constant_Min on an unused channel) at boot, so the mixer runs at 250 Hz; channel 1 stays unset, so PX4 still publishes nothing to the motors (0 messages measured). Routing then takes 13–22 ms |

## Results

### First design, trained reflex and cascade (2026-10-18)

Per-run records, logs and JSON are kept in the private repo; the per-run table is below the summary.

| case | runs | survived | detection | route + arm + takeoff | altitude lost (from ~1.1 m) | tilt max | drift max |
|---|---|---|---|---|---|---|---|
| Arc killed, trained reflex flying | 8 | **8 / 8** | 3–27 ms (stream closed) | 105–330 ms | 0.16–0.70 m | 2.4–9.8° | 0.06–0.30 m |
| Arc hung (process suspended), trained reflex | 3 | **3 / 3** | 201–203 ms (200 ms watchdog) | 122–185 ms | 0.49–0.60 m | 3.8–18.6° | 0.26–0.52 m |
| Arc killed, Arc cascade flying | 2 | **2 / 2** | 23–26 ms | 95–157 ms | 0.52–0.55 m | 1.5–2.0° | 0.05–0.07 m |
| no fault, fallback armed | 2 | 2 / 2, no takeover during flight | the only takeover fired at end of stream, after the 50 s window | — | — | — | — |
| Arc killed, **no fallback** | 1 | 0 / 1 | — | — | crashed within 0.5 s of the kill | 64° | — |

After a takeover PX4 climbs to its takeoff altitude (2.0 m here) and holds,
within 0.07 m of the loss point for the cascade runs.

#### Per run, first design

| run | arm | loss | fallback | survived | survival s | trigger | stale ms | route+arm+takeoff ms | alt before m | alt min m | alt lost m | tilt max deg | drift max m |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| g6_kill_reflex_w | reflex | kill | px4 | True | 51.2 | eof | 12 | 282 | 1.12 | 0.66 | 0.47 | 6.6 | 0.21 |
| g6_kill_reflex_x | reflex | kill | px4 | True | 51.08 | eof | 26 | 120 | 1.12 | 0.62 | 0.50 | 3.0 | 0.13 |
| hang_reflex_1 | reflex | hang | px4 | True | 51.3 | stale | 203 | 122 | 1.07 | 0.57 | 0.49 | 3.8 | 0.26 |
| hang_reflex_2 | reflex | hang | px4 | True | 50.94 | stale | 203 | 163 | 1.13 | 0.52 | 0.60 | 18.6 | 0.49 |
| hang_reflex_3 | reflex | hang | px4 | True | 51.0 | stale | 201 | 185 | 1.13 | 0.53 | 0.60 | 15.1 | 0.52 |
| kill_pid_1 | pid | kill | px4 | True | 50.96 | eof | 26 | 95 | 1.00 | 0.45 | 0.55 | 1.5 | 0.05 |
| kill_pid_2 | pid | kill | px4 | True | 51.06 | eof | 23 | 157 | 1.00 | 0.48 | 0.52 | 2.0 | 0.07 |
| kill_reflex_1 | reflex | kill | px4 | True | 50.76 | eof | 24 | 162 | 0.97 | 0.82 | 0.16 | 2.6 | 0.09 |
| kill_reflex_2 | reflex | kill | px4 | True | 50.53 | eof | 25 | 224 | 1.10 | 0.57 | 0.53 | 7.5 | 0.15 |
| kill_reflex_3 | reflex | kill | px4 | True | 50.61 | eof | 27 | 330 | 1.07 | 0.37 | 0.70 | 9.8 | 0.30 |
| kill_reflex_4 | reflex | kill | px4 | True | 51.12 | eof | 17 | 217 | 1.11 | 0.63 | 0.48 | 7.6 | 0.08 |
| kill_reflex_5 | reflex | kill | px4 | True | 51.34 | eof | 3 | 143 | 1.02 | 0.61 | 0.41 | 2.7 | 0.08 |
| kill_reflex_6 | reflex | kill | px4 | True | 50.9 | eof | 7 | 105 | 1.11 | 0.59 | 0.52 | 2.4 | 0.06 |
| kill_reflex_nostandby | reflex | kill | none | False | 29.3 | - | None | None | 1.11 | -0.34 | 1.45 | 64.3 | 0.39 |
| nofault_reflex_1 | reflex | - | px4 | True | 51.12 | eof | 4 | 42 | nan | nan | nan | nan | nan |
| nofault_reflex_2 | reflex | - | px4 | True | 51.12 | eof | 2 | 324 | nan | nan | nan | nan | nan |

### Flight default v5: transport and takeover latency (2026-10-19)

The flight default (candidate v5, 832 bits, plasticity off) with this
handover failed in Track H, on the Arc side running on the Windows host
(records over UDP out of the container, commands back through a
`docker exec -i` pipe). Records and logs are kept in the private repo:

| matrix | survived | false takeovers | command gaps | route + arm + takeoff |
|---|---|---|---|---|
| `h2_*` (before a host reboot) | 9 / 11 (`h2_hang_v5_2`, `h2_kill_v5_3` crashed) | 2 / 2 no-fault runs | no gap log yet; the watchdog had seen 538–639 ms of silence when it fired | 148–2407 ms |
| `h4_*` (idle host) | 10 / 11 (`h4_kill_v5_2` crashed) | 1 / 2 no-fault runs (`stale` after a 233 ms gap) | 131–233 ms worst per run (gaps over 100 ms logged) | 87–1073 ms (1073 ms in the crash) |

Three causes, each measured:

1. **Machine-wide stalls.** A probe that only sleeps 2 ms in a loop
   (`tools/px4/stall_probe.py`, its own process in the container) wakes
   131–182 ms late once or twice per 47 s flight window, on an idle host, at
   the same moment as every command gap over 120 ms in flight. During such a
   gap the commands advance 20–28 ms of sim time and the PX4 record stream
   16–24 ms: Gazebo and PX4 stop too, so the plant is not starved. They are stalls of the Docker Desktop VM, not of a pipe and not of
   Arc. No layout inside the VM removes them.
2. **The watchdog's clock.** The watchdog measured wall time, so a VM stall
   longer than 200 ms read as a hung Arc while the plant was frozen: the
   `h4_nofault_v5_2` false takeover had that signature (a 233 ms gap, sim time
   not logged then). The watchdog now runs on the plant's clock (Gazebo sim
   time, `--watchdog-clock sim`, the default). The threshold stays 200 ms and
   a hang is still caught at 200 ms, because sim time keeps running while Arc
   is suspended (200–204 ms below). A 0.86 s freeze of the whole container
   (`docker pause`) in a no-fault flight caused no takeover and the flight went
   on. On hardware the same watchdog runs on the flight
   controller's clock, which is the plant's clock.
3. **Takeover latency.** Most of the old 87–1073 ms was PX4's output mixer
   waiting for its 300 ms low-rate tick before applying the routing (table
   above), plus 4 + 2 shell-client spawns. Now: one MAVLink burst per step, the
   standby keep-alive channel, one status call.

**Arc side next to PX4** (`--arc-side container`): the static Linux bridge
(a static build of the private repo at `ccf4fed` for the matrix; the `bin/linux-x86_64/px4_bridge` shipped at the time, built from `84c16c0`, was checked in the same layout: a hang caught at 204 ms, takeover 23 ms; the binary now in `bin/` is built from `5854e0f`, whose bridge and engine source is the same as `ccf4fed`'s, and has not been re-flown in this matrix), the record streamer on its own PX4 MAVLink link
(same stream rates as the GCS link), and the motor publisher, joined by local
pipes inside the container, as on a companion computer. Every command gap over
20 ms is logged with the sim time that passed, next to the streamer's record
arrival gaps and the stall probe, so each gap is attributed
(`tools/px4/gap_attribution.py`).

Matrix (records and logs in the private repo; per-run table below): v5, Arc side in the container, 50 s, loss at 30 s, sequential, fresh
container per run, host CPU below 25 % before each run.

| case | runs | survived | false takeovers | detection | route + arm + takeoff | altitude lost (from ~1.0 m) | tilt max |
|---|---|---|---|---|---|---|---|
| Arc killed | 6 | **6 / 6** | 0 | 4–20 ms (stream closed) | 26–36 ms | 0.48–0.54 m | 2.4–3.2° |
| Arc hung | 3 | **3 / 3** | 0 | 200–204 ms (plant time) | 21–30 ms | 0.45–0.52 m | 3.4–3.8° |
| no fault, fallback armed | 4 | **4 / 4** | **0** | the only takeover is at end of stream, after the 50 s window | (23–37 ms there) | — | 3.4–3.7° |
| baseline: Arc side on the Windows host, no fault | 2 | 2 / 2 | 0 | end of stream only | (18–26 ms) | — | 3.5–4.1° |

Command gaps in flight (from 3 s, after spin-up and lift-off, to the end of the
window):

| case | worst gap, wall | machine stalls among them | worst gap outside machine stalls | of which the Arc side added (not also a gap in PX4's own record stream) |
|---|---|---|---|---|
| Arc side in the container, 13 runs | 20.7–192.6 ms | 0–4 per run (the large ones 131–193 ms) | 20.7–60.2 ms (under 50 ms in 11 of 13) | 20.0–60.2 ms (under 50 ms in 12 of 13) |
| Arc side on the Windows host, 2 runs | 194.8–201.6 ms | 1–4 per run (up to 201.6 ms) | 25.6–33.2 ms | 23.3–27.4 ms |

The 50 ms target is met outside machine stalls in 11 of 13 runs. The two
exceptions: `g6_kill_v5_4`, 52.0 ms, which the record stream shows as well (a
52 ms hole in PX4's own telemetry), and `g6_nofault_v5_4`, 60.2 ms of sim time
without a command after a 40 ms hole in the record stream, so about 20 ms of it
is the Python streamer, bridge and publisher chain. In 10 of 15 runs (both layouts)
the publisher also has one 101–118 ms gap 1.5–1.9 s after start, during
spin-up, before the 3 s mark; the publisher's own view of sim time mostly stops
in it (0–20 ms, once 44 ms) and the stall probe does not see it, so it is
inside the publisher process; its cause is not established. It is excluded
from the in-flight figures and listed in the per-run table (last column).

What the numbers show about the transport: on an idle host today, the Windows
pipe adds little (25.6–33.2 ms outside stalls, against 20.7–60.2 ms in the
container). The 100–233 ms gaps of `h4` are the size of the machine-wide
stalls measured here, but `h4` had no stall probe, so that attribution is not
proven for those runs. The container layout stays the measured default for G6
because it is the deployment layout (Arc on a companion computer next to the
autopilot) and has no host-to-VM hop; the fixes that changed the outcome are
the plant-clock watchdog (0 false takeovers in 6 no-fault runs) and the
takeover path (21–37 ms, from up to 1073 ms).

Altitude lost did not shrink with the faster takeover (0.45–0.54 m, against
0.16–0.70 m before), so it is set by PX4's takeover from fresh controllers, not
by the handover latency.

#### Per run, flight default v5

| run | arm | loss | fallback | survived | survival s | trigger | stale ms | route+arm+takeoff ms | alt before m | alt min m | alt lost m | tilt max deg | drift max m | false takeover | worst gap in flight ms | machine stalls in flight | worst gap in flight outside stalls ms | worst gap in flight added by the Arc side ms | worst gap incl. spin-up ms |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| g6_hang_v5_1 | reflex | hang | px4 | True | 50.08 | stale | 200 | 30 | 0.99 | 0.47 | 0.52 | 3.7 | 0.17 | False | 181.6 | 3 | 39.0 | 22.9 | 181.6 |
| g6_hang_v5_2 | reflex | hang | px4 | True | 49.66 | stale | 200 | 21 | 1.01 | 0.55 | 0.45 | 3.4 | 0.07 | False | 176.4 | 3 | 46.4 | 22.1 | 176.4 |
| g6_hang_v5_3 | reflex | hang | px4 | True | 49.96 | stale | 204 | 22 | 0.99 | 0.52 | 0.47 | 3.8 | 0.22 | False | 182.0 | 1 | 28.6 | 21.2 | 182.0 |
| g6_hostbase_v5_1 | reflex | - | px4 | True | 50.38 | eof | 8 | 18 | nan | nan | nan | nan | nan | False | 194.8 | 1 | 33.2 | 27.4 | 194.8 |
| g6_hostbase_v5_2 | reflex | - | px4 | True | 50.38 | eof | 4 | 26 | nan | nan | nan | nan | nan | False | 201.6 | 4 | 25.6 | 23.3 | 201.6 |
| g6_kill_v5_1 | reflex | kill | px4 | True | 49.88 | eof | 4 | 36 | 1.02 | 0.52 | 0.50 | 2.6 | 0.10 | False | 25.6 | 0 | 25.6 | 21.4 | 25.6 |
| g6_kill_v5_2 | reflex | kill | px4 | True | 50.06 | eof | 4 | 27 | 0.97 | 0.47 | 0.50 | 2.4 | 0.09 | False | 165.1 | 1 | 24.1 | 20.6 | 165.1 |
| g6_kill_v5_3 | reflex | kill | px4 | True | 49.58 | eof | 8 | 31 | 0.97 | 0.48 | 0.49 | 3.1 | 0.20 | False | 149.6 | 4 | 48.4 | 38.4 | 149.6 |
| g6_kill_v5_4 | reflex | kill | px4 | True | 50.09 | eof | 20 | 29 | 1.00 | 0.45 | 0.54 | 3.2 | 0.17 | False | 171.8 | 1 | 52.0 | 27.7 | 171.8 |
| g6_kill_v5_5 | reflex | kill | px4 | True | 49.84 | eof | 16 | 27 | 0.99 | 0.51 | 0.48 | 2.9 | 0.13 | False | 20.7 | 0 | 20.7 | 20.0 | 173.1 |
| g6_kill_v5_6 | reflex | kill | px4 | True | 49.99 | eof | 16 | 26 | 1.03 | 0.50 | 0.53 | 3.2 | 0.11 | False | 151.6 | 1 | 21.9 | 21.8 | 151.6 |
| g6_nofault_v5_1 | reflex | - | px4 | True | 50.26 | eof | 4 | 24 | nan | nan | nan | nan | nan | False | 192.6 | 1 | 29.7 | 20.3 | 192.6 |
| g6_nofault_v5_2 | reflex | - | px4 | True | 50.2 | eof | 4 | 23 | nan | nan | nan | nan | nan | False | 180.6 | 1 | 21.8 | 21.2 | 180.6 |
| g6_nofault_v5_3 | reflex | - | px4 | True | 50.1 | eof | 0 | 37 | nan | nan | nan | nan | nan | False | 185.0 | 2 | 33.9 | 33.9 | 185.0 |
| g6_nofault_v5_4 | reflex | - | px4 | True | 49.96 | eof | 0 | 29 | nan | nan | nan | nan | nan | False | 186.5 | 2 | 60.2 | 60.2 | 186.5 |

### The direct rig (G11, 2026-10-20)

The same matrix (v5, PX4 standby, plant-clock watchdog at 200 ms, 50 s,
loss at 30 s, Arc side in the container) on the direct rig
(`run_scenario.py --rig direct`, `docs/px4_sitl_method.md`, direct rig): records from
Gazebo's IMU topic at 250 Hz with PX4's estimate, every tick published as it
arrives. The watchdog and the takeover path are the same code. Per-run
table `docs/px4_sitl_results/g6/direct/summary.md` (records in the private
repository; script `tools/px4/run_g11.sh` phase `g6`).

| case | runs | survived | false takeovers | detection | route + arm + takeoff | altitude lost (from ~1.0 m) | tilt max |
|---|---|---|---|---|---|---|---|
| Arc killed | 6 | **6 / 6** | 0 | 0–4 ms (stream closed) | 15–31 ms | 0.44–0.56 m | 2.2–3.0° |
| Arc hung | 3 | **3 / 3** | 0 | 200 ms (plant time) | 16–19 ms | 0.47–0.55 m | 3.2–4.7° |
| no fault, fallback armed | 4 | **4 / 4** | **0** | only at end of stream, after the window | (19–31 ms there) | — | — |

The same as on the MAVLink rig (0.45–0.54 m lost, 2.4–3.8°, 21–36 ms takeover):
altitude lost is PX4's takeover from fresh controllers, not the transport.
Command gaps: the worst in flight in 12 of 13 runs is a machine-wide VM
stall of 1.06–1.08 s (seen by the stall probe; sim time advanced 4 ms
through it); outside machine stalls 0 in 11 runs, 36 ms and 104 ms in two,
both also holes in the IMU record stream; the Arc side added 0 ms in all 13.

`summarize_bridge_loss.py` now takes the loss time in sim time from the
bridge's last logged tick: these VM stalls put sim time about 2 s behind the
wall clock at the loss, and the scheduled (wall) time had moved the
before/after windows across it (the earlier runs' stalls were too short to
matter; without the bridge log it falls back to the wall time as before).

### The direct rig at 1 kHz (G12, 2026-10-21)

The same matrix on the direct rig with the G12 options `--record-clock own
--physics-hz 1000` (`docs/px4_sitl_method.md`, direct rig, G12): a 1 ms physics step and the IMU
at 1 kHz, so PX4's gyro, rate loop and motor outputs, which fly the takeover,
run at 1 kHz instead of 250. Watchdog and takeover path unchanged. Per-run table
`docs/px4_sitl_results/g6/direct_g12/summary.md` (raw run records kept in the private repository;
script `P=g12 EXTRA="--record-clock own --physics-hz 1000" tools/px4/run_g11.sh
<out> bin/linux-x86_64/px4_bridge g6`).

| case | runs | survived | false takeovers | detection | route + arm + takeoff | altitude lost (from ~1.0 m) | tilt max |
|---|---|---|---|---|---|---|---|
| Arc killed | 6 | **6 / 6** | 0 | 0–2 ms (stream closed) | 14–25 ms | 0.47–0.54 m | 1.2–3.0° |
| Arc hung | 3 | **3 / 3** | 0 | 200–201 ms (plant time) | 15–30 ms | 0.46–0.53 m | 3.6–4.9° |
| no fault, fallback armed | 4 | **4 / 4** | **0** | only at end of stream, after the window | (16–23 ms there) | — | — |

13 / 13 and 0 false takeovers, as at 250 Hz (G11). Altitude lost and tilt
did not change with PX4's rate loop at 1 kHz (G11: 0.44–0.56 m, 2.2–4.7°),
which again puts the loss in PX4's takeover from fresh controllers, not in
the loop rate or the transport. Command gaps: the worst in flight in 11 of
13 runs is a machine-wide VM stall of 0.67–1.14 s (stall probe); kill 2 and
5 had no gap over 20 ms in flight; outside machine stalls 0 in all 13, and
the Arc side added 0 ms in all 13.

### Warm takeover (G13, 2026-10-22)

The altitude lost above did not change with the transport or PX4's loop rate,
so it came from what PX4 restarts from. PX4's own flight log (ulog, kept for
every standby run since this change; `tools/px4/takeover_trace.py`) shows two
causes, both in stock PX4 (`PX4-Autopilot` `61907962`, the SITL image's build):

1. **The takeoff trajectory starts downward.** While PX4 is disarmed its
   position controller is in its ground state, and every cycle the flight mode
   manager resets the auto flight task (`FlightModeManager.cpp`: `reActivate()`
   while `takeoff_state < RAMPUP`), which sets the trajectory to the current
   position and **0.7 m/s down** (`FlightTaskAuto::reActivate`, hard-coded
   "on ground"). Hold and Takeoff share that task, so the Takeoff armed in the
   air started from 0.7 m/s down and its jerk-limited trajectory needed
   ~0.7 s to reverse: the setpoint itself went 0.44 m down
   (`docs/px4_sitl_results/g6/warm/dev/p0_kill.takeover.txt`).
2. **Hover thrust from the parameter.** Until PX4 counts itself flying, and
   until its hover-thrust estimator is valid (5–6 s after the takeover), its
   controller uses `MPC_THR_HOVER` (0.60 on the x500 model); the x500 hovers at
   0.72 (PX4's own estimate, and Arc's commands).

Integrators were not the cause (they are reset in the ground state either way).

The warm takeover (`gz_motor_pub.py --takeover warm`, the default; no PX4
change, stock parameters and MAVLink only):

- **Hover thrust:** during flight the watchdog keeps the standby PX4's
  `MPC_THR_HOVER` at the hover thrust being flown: Arc's mean motor command over
  the last second, mapped through PX4's own output model (`SIM_GZ_EC_MIN/MAX`,
  `THR_MDL_FAC`, read from PX4), checked once a second and sent past a 0.005
  change (3–31 updates per 50 s flight). Nothing extra is sent at the handover.
- **The catch:** during the routing wait PX4 is switched, still disarmed, to
  Offboard behind a velocity-hold setpoint stream (0.1 m/s up, which PX4 reads
  as a takeoff request; 50 Hz until the arm, 500 Hz after), and the arm burst
  carries the arm command alone. PX4's velocity loop, which has no jerk-limited
  trajectory, catches the vehicle.
- **Into Takeoff from the live state:** once PX4 reports itself in the air
  (`EXTENDED_SYS_STATE`, 3 samples) and the vehicle has stopped sinking
  (`LOCAL_POSITION_NED` |vz| < 0.15 m/s, 2 samples), the watchdog switches to
  Takeoff and stops the stream. Coming from Offboard (no flight task), PX4
  starts the Takeoff task with no prior setpoint, i.e. from the current
  position and velocity (`FlightModeManager::switchTask`). From then on PX4
  flies on its own, as before.

Two conditions in that sequence were found by measurement. Entering Takeoff
while the land detector still reported "landed" (it starts so at arming) put
the controller back in its ground state, and a falling vehicle then lost 1.1 m
(`dev/p1_kill_warm.takeover.txt`). Entering Takeoff while the vehicle was still
sinking after a hang made the trajectory keep descending for 0.3 s
(`dev/g13r_g6m_hang_4.takeover.txt`; 0.23 and 0.84 m lost).

Results (v5, Arc side in the container, PX4 standby, plant-clock watchdog at
200 ms, 50 s, loss at 30 s; sequential, fresh container, host CPU below 25 %
before each run). Evidence `docs/px4_sitl_results/g6/warm/` (per-run tables
`summary.md`, PX4's takeover traces `*.takeover.txt`, two full ulogs in
`ulog/`; raw bridge records and witness streams kept in the private repository; scripts `tools/px4/run_g11.sh` phases `g6` and `g6mav`, `final.sh`).

| rig | case | runs | survived | false takeovers | detection | route + arm (PX4 in control) | Takeoff entered | altitude lost, warm | altitude lost, cold (before) | tilt max |
|---|---|---|---|---|---|---|---|---|---|---|
| direct, 1 kHz | Arc killed | 6 | **6 / 6** | 0 | 1 ms | 13–29 ms | +131–193 ms | **−0.00–0.06 m** | 0.47–0.54 m (G12) | 0.9–2.8° |
| direct, 1 kHz | Arc hung | 3 | **3 / 3** | 0 | 200–201 ms | 14–32 ms | +138–231 ms | **0.02–0.09 m** | 0.46–0.53 m (G12) | 2.7–4.8° |
| direct, 1 kHz | no fault | 4 | **4 / 4** | **0** | end of stream only | (22–51 ms there) | | | | |
| MAVLink, 250 Hz | Arc killed | 6 | **6 / 6** | 0 | 0–20 ms | 19–35 ms | +152–223 ms | **−0.01–0.10 m** | 0.48–0.54 m (v5) | 1.7–2.8° |
| MAVLink, 250 Hz | Arc hung | 3 | **3 / 3** | 0 | 200–204 ms | 22–32 ms | +123–202 ms | **−0.00–0.08 m** | 0.45–0.52 m (v5) | 4.1–8.2° |
| MAVLink, 250 Hz | no fault | 4 | **4 / 4** | **0** | end of stream only | (18–25 ms there) | | | | |

Same-day A/B, cold and warm interleaved, 6 kills each, same build
(`ab_direct/`, `ab_mavlink/`):

| rig | altitude lost cold → warm | route + arm cold → warm (mean, range) | of which arm/mode acks cold → warm |
|---|---|---|---|
| direct, 1 kHz | 0.45–0.53 → **0.01–0.04 m** | 23.8 (19–28) → 24.0 (14–36) ms | 7.2 → 6.0 ms |
| MAVLink | 0.42–0.53 → **0.01–0.09 m** | 25.8 (21–32) → 23.8 (17–33) ms | 11.3 → 10.5 ms |

Detection and the time until PX4 is armed and in control are unchanged, and no
false takeover occurred: the watchdog and the routing are the same code; the
arm burst carries one command instead of two. The tilt peak after a hang on the
MAVLink rig is higher than cold (4.1–8.2° against 3.4–3.8°), from the catch's
hard vertical arrest at 250 Hz; on the direct rig it is not (2.7–4.8° against
3.6–4.9°). Command gaps outside machine stalls: direct 0 in 12 of 13 runs
(88 ms in one, not added by the Arc side); MAVLink 20.5–37.6 ms, of which the
Arc side added 20.1–21.8 ms.

Development runs, kept in the run folders, not in the evidence: the first
version read PX4's parameters at the publisher's start, before it drained Arc's
command pipe, which delayed the pipe and caused 100–250 ms command gaps during
spin-up and two false takeovers on the MAVLink rig (fixed: the reads run after
the reader starts); one later version had one false takeover in 13 no-fault
flights on the MAVLink rig after a 428 ms command gap that only the Arc side
saw (PX4's record stream and the stall probe show no hole; cause not
established; not seen in the 102 warm runs since); earlier versions took
2.5–3 ms longer to arm (Offboard switched in the arm burst), now removed, as
the A/B shows.

## What this does and does not claim

- Arc crashing or hanging mid-flight is survivable: 13 of 13 losses survived
  in the first design (trained reflex and cascade), and 9 of 9 on the flight
  default v5 with 0 false takeovers in 4 no-fault flights (2026-10-19);
  0 of 1 without the fallback. The PX4 that takes over is unmodified, with five
  stock parameters set at boot (`COM_SPOOLUP_TIME`, `MPC_TKO_RAMP_T`,
  `MIS_TAKEOFF_ALT`, the motor functions off, and the keep-alive function on
  ESC channel 5).
- The takeover costs altitude: 0.16–0.70 m from a 1.0–1.1 m hover with the
  earlier (cold) takeover; with the warm takeover (G13, the default since
  2026-10-22) −0.01–0.10 m in 30 losses on two rigs. **The fallback needs at
  least 1 m above ground**: this stays the stated operating limit until the
  warm takeover has been measured from lower hovers.
- Route + arm + takeoff takes 21–37 ms in SITL (one MAVLink connection, the
  keep-alive channel). On hardware the same sequence is a flight-controller-side
  watchdog (an IO or ESC timeout that switches to the FMU's own mixer). The SITL
  figures are not a measurement of that path.
- A hang is detected by the 200 ms watchdog, so it costs about 200 ms more
  than a crash. The watchdog counts plant time, so a stall of the simulation
  machine (up to 0.86 s tested) is not taken for a hang.
- Command gaps in the SITL harness: machine-wide stalls of the Docker Desktop
  VM up to 193 ms remain (the plant stops with them); outside them the worst
  in-flight gap is 20.7–60.2 ms, and a 101–118 ms publisher gap during
  spin-up is unexplained. What would remove the VM stalls from the measurement is
  the same matrix on a native Linux host; what would remove the remaining
  Python hops is a bridge that reads PX4's MAVLink stream itself. Neither is
  done.
