# Arc process loss mid-flight: handover to PX4 (gap register G6)

Measured 2026-10-18 on PX4 SITL + Gazebo Harmonic x500 (`px4io/px4-sitl-gazebo`),
fresh container per run. The bridge and tooling are private; this page is method and numbers.

## The gap

In arc-owns mode PX4 is the estimator only: disarmed, motor outputs unassigned
(`SIM_GZ_EC_FUNC1..4 = 0`), and Arc's bridge writes the rotor commands. If the
Arc process dies or hangs, nothing commands the rotors. Measured without a
fallback: the vehicle crashes within 0.5 s of the kill (row `kill_reflex_nostandby`).

## The handover

The motor-command writer next to the simulator (200 ms watchdog) is the watchdog, because it is the last hop before the
motors. When Arc's command stream closes (crash, kill) or goes silent for
200 ms (hang), it stops writing for good and hands the vehicle to PX4:

1. route PX4's outputs to the four motors (`SIM_GZ_EC_FUNCn = 100 + n`);
2. wait until PX4's Gazebo bridge reports all four channels routed;
3. arm PX4 in the air and command Takeoff, which climbs to `MIS_TAKEOFF_ALT`
   and switches itself to Hold.

Authority is one-way: it never returns to Arc within a flight. Each test kills or suspends the bridge process mid-hover at 30 s.

Four PX4 behaviours shaped this design. Each was found by measurement:

| finding | consequence |
|---|---|
| PX4 applies output routing only while disarmed; a change made while armed is ignored (`gz_bridge status` keeps `func: 0`) | PX4 cannot sit armed and silent and then be routed at handover; it must be routed first, then armed |
| an armed PX4 with unrouted outputs winds up: its loops integrate errors its outputs cannot remove (thrust setpoint drove to 0 or to 1.0; rate integrators drove a constant torque) | the standby PX4 stays disarmed, so it has no state to wind up; takeover starts from fresh controllers |
| the mixer picks up routing on its next parameter update; arming before that left channels unrouted (random tumbles on takeover) | the handover waits until all four channels report routed before arming |
| in position modes PX4 forces zero thrust until its takeoff stage completes (1 s spool-up plus a 3 s ramp by default) | the fallback sets `COM_SPOOLUP_TIME = 0` and `MPC_TKO_RAMP_T = 0` at boot and uses Takeoff, so PX4 flies from its first cycle |

## Results

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

### Per run

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

## What this does and does not claim

- Arc crashing or hanging mid-flight is survivable: 13 of 13 losses survived,
  0 of 1 without the fallback. The PX4 that takes over is unmodified, with four
  stock parameters set at boot.
- The takeover costs altitude: 0.16–0.70 m from a 1.1 m hover. In the
  100–330 ms between Arc's last command and PX4's first, PX4 outputs idle while
  the routing settles. **The fallback needs at least 1 m above ground**, and
  this is stated as an operating limit.
- The gap is set by the PX4 shell-client round trips in SITL. On hardware the
  same sequence is a flight-controller-side watchdog (an IO or ESC timeout
  that switches to the FMU's own mixer). The SITL figures here are an upper
  bound on that path, not a measurement of it.
- A hang is detected by the 200 ms watchdog, so it costs about 200 ms more
  than a crash. A lower threshold trades that against false takeovers on host
  stalls, which reached 88 ms in earlier Track C runs.
