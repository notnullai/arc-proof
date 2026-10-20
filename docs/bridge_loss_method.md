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

## What this does and does not claim

- Arc crashing or hanging mid-flight is survivable: 13 of 13 losses survived
  in the first design (trained reflex and cascade), and 9 of 9 on the flight
  default v5 with 0 false takeovers in 4 no-fault flights (2026-10-19);
  0 of 1 without the fallback. The PX4 that takes over is unmodified, with five
  stock parameters set at boot (`COM_SPOOLUP_TIME`, `MPC_TKO_RAMP_T`,
  `MIS_TAKEOFF_ALT`, the motor functions off, and the keep-alive function on
  ESC channel 5).
- The takeover costs altitude: 0.16–0.70 m from a 1.0–1.1 m hover.
  **The fallback needs at least 1 m above ground**, and this is stated as an
  operating limit.
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
