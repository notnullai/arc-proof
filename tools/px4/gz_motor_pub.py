"""Publish motor speed commands to the Gazebo x500 from inside the PX4
Gazebo container (gz.transport13 + gz.msgs). This is the C2 actuation
path: Arc's setpoints -> gz.msgs.Actuators on /x500_0/command/motor_speed
(the topic the four MulticopterMotorModel plugins subscribe to; the
similarly named /model/x500_0/command/motor_speed has no gz subscriber).

Usage (inside the container):
  python3 gz_motor_pub.py --const 700 --seconds 3 [--rate 250]
      publish a constant speed on all four motors, then zeros
  python3 gz_motor_pub.py --stdin [--rate 250]
      read "v0 v1 v2 v3" lines (rad/s) from stdin, publish the latest at
      --rate until EOF, then zeros

Arbitration note: PX4's gz_bridge publishes this topic only while armed
and only on outputs whose SIM_GZ_EC_FUNCn is non-zero. In arc-owns mode
the FUNCs are 0, so PX4 is silent even when armed.

Handover (gap register G6, --handover-ms N --mav URL): PX4 runs as the
estimator, disarmed and with its outputs unrouted (the normal arc-owns mode),
so its controllers are idle and nothing winds up. If no new command line
arrives for N ms of plant time (--watchdog-clock sim, the default: Gazebo sim
time, so a stall of the whole simulation machine, which freezes the plant too,
is not taken for a hung Arc; --watchdog-clock wall = this process's clock), or
stdin closes (the Arc process crashed, hung or was killed), this process
stops publishing for good and hands the motors to PX4
over one persistent MAVLink connection opened at start (--mav, a PX4 link
that answers this process only): PARAM_SET SIM_GZ_EC_FUNC1..4 = 101..104 in
one burst (PX4 accepts output routing only while disarmed) and wait for the
four PARAM_VALUE echoes; then wait until PX4's Gazebo bridge reports all four
channels routed (the one PX4 shell call left: no MAVLink message reports the
mixer's routing); then a force-arm COMMAND_LONG and a mode switch in one burst,
and wait for both COMMAND_ACKs. One-way: authority never returns to Arc in a
run. The motor plugins hold their last command in the gap.

Warm takeover (--takeover warm, the default since 2026-10-21; docs/bridge_loss_method.md): PX4 takes
over from the live state, not from what its controllers hold while disarmed and "landed":
- hover thrust: during flight this process keeps the standby PX4's MPC_THR_HOVER at the hover thrust
  being flown (the mean motor command over the last --hover-window-s, mapped through PX4's own output
  model: SIM_GZ_EC_MIN/MAX and THR_MDL_FAC, read at start), checked once a second. PX4 uses
  MPC_THR_HOVER until it counts itself flying and until its hover-thrust estimator is valid (5-6 s after
  a takeover); its default (0.60) was 0.12 under the x500's measured 0.72;
- trajectory: on the ground PX4's auto flight task resets its trajectory every cycle to 0.7 m/s
  downward (FlightTaskAuto::reActivate), so a Takeoff armed in the air began with a 0.5 m descent.
  The warm takeover arms in Offboard behind a velocity-hold stream from this process (0.1 m/s up, which
  PX4 reads as a takeoff request); PX4's velocity loop catches the vehicle. Once PX4 reports itself in
  the air and the vehicle has stopped sinking (|vz| < CATCH_VZ), it switches to Takeoff, which PX4
  then starts with no prior setpoint, i.e. from the current position and velocity
  (FlightModeManager::switchTask). The stream stops when Takeoff is acknowledged, so the fallback does
  not depend on this process afterwards.
--takeover cold is the earlier sequence (arm, Takeoff), kept for comparison.

Immediate mode (--immediate, the direct rig): each command line is published the moment it arrives,
from the reader thread, instead of the newest line being re-published on the --rate grid (which adds up
to one grid period, 4 ms at 250 Hz, plus the wait for the main loop). The watchdog is unchanged.

Gap log (--gap-log PATH): every interval between command lines longer than
--gap-log-ms (default 20 ms; records arrive every 20 ms and the bridge writes
a burst of lines per record) is written to PATH as "wall_time gap_ms
sim_gap_ms" (sim_gap_ms: Gazebo sim time that passed in the same interval,
from /world/default/clock; -1 before the first clock message), and a summary
line is printed at the end.
"""
import argparse
import collections
import re
import struct
import subprocess
import sys
import threading
import time

from gz.transport13 import Node

try:
    from gz.msgs10.actuators_pb2 import Actuators
    from gz.msgs10.clock_pb2 import Clock
except ImportError:  # pragma: no cover
    from gz.msgs11.actuators_pb2 import Actuators
    from gz.msgs11.clock_pb2 import Clock

TOPIC = '/x500_0/command/motor_speed'
ROUTED = re.compile(r'Channel  [0-3]: func: 10[1-4]')
RESEND_S = 0.03
CATCH_VZ = 0.15   # m/s: the warm takeover enters Takeoff below this vertical speed
CATCH_S = 1.5     # s: at most this long in the Offboard catch
HOVER_DEADBAND = 0.005   # PX4 thrust units: the standby MPC_THR_HOVER is updated past this change
EC_PARAMS = ['SIM_GZ_EC_MIN%d' % i for i in range(1, 5)] + ['SIM_GZ_EC_MAX%d' % i for i in range(1, 5)] + ['THR_MDL_FAC']


def _pid(m):
    pid = m.param_id if isinstance(m.param_id, str) else m.param_id.decode(errors='replace')
    return pid.rstrip('\x00')


def px4_thrust(n, s1, s2, ec):
    """PX4's normalized thrust for n motor-speed commands with per-motor sums s1 (of w) and s2 (of w^2):
    the mean of the inverse of PX4's output model (control allocator thrust curve THR_MDL_FAC f:
    thrust = f u^2 + (1 - f) u for the output u, and gz_bridge's linear map u -> SIM_GZ_EC_MIN..MAX
    rad/s), over the commands and the motors. Exact for commands inside EC_MIN..MAX."""
    f = ec['THR_MDL_FAC']
    t = 0.0
    for i in range(4):
        lo, hi = float(ec['SIM_GZ_EC_MIN%d' % (i + 1)]), float(ec['SIM_GZ_EC_MAX%d' % (i + 1)])
        m1, m2 = s1[i] / n, s2[i] / n
        eu = (m1 - lo) / (hi - lo)
        eu2 = (m2 - 2.0 * lo * m1 + lo * lo) / ((hi - lo) * (hi - lo))
        t += f * eu2 + (1.0 - f) * eu
    return t / 4.0


class Px4Link:
    """The watchdog's own MAVLink connection to PX4, opened before flight so the
    handover costs no connection setup. PX4 integer params travel byte-wise in
    PARAM_SET's float field (see mavparam.py)."""

    def __init__(self, url, wait_s=60.0):
        from pymavlink import mavutil
        self.mu = mavutil
        self.c = mavutil.mavlink_connection(url, source_system=255, source_component=191)
        self.tx = threading.Lock()   # the offboard stream thread sends on the same connection
        if self.c.wait_heartbeat(timeout=wait_s) is None:
            raise SystemExit('GZ_MOTOR_PUB: no PX4 heartbeat on %s' % url)

    def drain(self):
        while self.c.recv_match(blocking=False) is not None:
            pass

    def get_params(self, names, timeout_s=3.0):
        """PARAM_REQUEST_READ each name (before flight). Returns {name: value}, ints decoded."""
        got = {}
        dl = time.time() + timeout_s
        nxt = 0.0
        while len(got) < len(names) and time.time() < dl:
            if time.time() >= nxt:
                for n in names:
                    if n not in got:
                        with self.tx:
                            self.c.mav.param_request_read_send(self.c.target_system, self.c.target_component,
                                                               n.encode(), -1)
                nxt = time.time() + 0.2
            m = self.c.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.05)
            if m is None:
                continue
            pid = _pid(m)
            if pid in names:
                real = m.param_type in (self.mu.mavlink.MAV_PARAM_TYPE_REAL32, self.mu.mavlink.MAV_PARAM_TYPE_REAL64)
                got[pid] = m.param_value if real else struct.unpack('<i', struct.pack('<f', m.param_value))[0]
        return got

    def set_params(self, pairs, timeout_s=1.0):
        """PARAM_SET every pair in one burst (int values as INT32, float values as REAL32); resend the
        unconfirmed ones every 30 ms until each PARAM_VALUE echo carries the value. Returns the names
        still unconfirmed."""
        pending = dict(pairs)
        dl = time.time() + timeout_s
        nxt = 0.0
        while pending and time.time() < dl:
            if time.time() >= nxt:
                for name, v in pending.items():
                    if isinstance(v, int):
                        raw, typ = struct.unpack('<f', struct.pack('<i', v))[0], self.mu.mavlink.MAV_PARAM_TYPE_INT32
                    else:
                        raw, typ = float(v), self.mu.mavlink.MAV_PARAM_TYPE_REAL32
                    with self.tx:
                        self.c.mav.param_set_send(self.c.target_system, self.c.target_component, name.encode(), raw, typ)
                nxt = time.time() + RESEND_S
            m = self.c.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.005)
            if m is None:
                continue
            pid = _pid(m)
            if pid not in pending:
                continue
            want = pending[pid]
            if isinstance(want, int):
                ok = struct.unpack('<i', struct.pack('<f', m.param_value))[0] == want
            else:
                ok = abs(m.param_value - want) <= 1e-4
            if ok:
                del pending[pid]
        return sorted(pending)

    def offboard_stream(self, stop, fast, slow_s=0.02, fast_s=0.002):
        """Until stop is set: SET_POSITION_TARGET_LOCAL_NED, velocity only (0, 0, -0.1 m/s, i.e. hold and
        0.1 m/s up, which PX4 reads as a takeoff request), yaw rate 0; every slow_s until fast is set (the
        mode switch only needs the signal present; the arm burst is not slowed), then every fast_s."""
        while not stop.is_set():
            self.offboard_setpoint()
            time.sleep(fast_s if fast.is_set() else slow_s)

    def offboard_setpoint(self):
        mask = 1 | 2 | 4 | 64 | 128 | 256 | 1024   # ignore position, acceleration, yaw
        with self.tx:
            self.c.mav.set_position_target_local_ned_send(
                int(time.time() * 1000) & 0xFFFFFFFF, self.c.target_system, self.c.target_component,
                self.mu.mavlink.MAV_FRAME_LOCAL_NED, mask, 0, 0, 0, 0.0, 0.0, -0.1, 0, 0, 0, 0, 0)

    def commands(self, cmds, timeout_s=1.0, sent=(), until_ok=()):
        """Send COMMAND_LONGs in order in one burst (except those in sent, already sent, whose acks may be
        waiting); resend the unacknowledged ones (in order) every 30 ms, and those in until_ok also after a
        refusal. Returns {command: result}."""
        res = {}
        dl = time.time() + timeout_s
        nxt = 0.0 if len(sent) < len(cmds) else time.time() + RESEND_S
        first = True
        while len(res) < len(cmds) and time.time() < dl:
            if time.time() >= nxt:
                for c, p in cmds:
                    if c not in res and not (first and c in sent):
                        with self.tx:
                            self.c.mav.command_long_send(self.c.target_system, self.c.target_component, c, 0, *p)
                nxt, first = time.time() + RESEND_S, False
            m = self.c.recv_match(type='COMMAND_ACK', blocking=True, timeout=0.005)
            if m is not None and m.command in [c for c, _ in cmds] and m.command not in res:
                if m.result != 0 and m.command in until_ok and time.time() < dl - RESEND_S:
                    continue
                res[m.command] = m.result
        return res


def handover(link, gz_bridge, warm=False):
    """Route, check routed, arm, Takeoff. warm: the warm takeover (module docstring): arm in Offboard
    behind a velocity-hold stream, which catches the vehicle, and switch to Takeoff once PX4 reports
    itself in the air and the vehicle is no longer sinking or climbing (|vz| < CATCH_VZ).
    Returns (ok, status string, ms from start to PX4 armed and in control)."""
    mav = link.mu.mavlink
    t0 = time.time()
    link.drain()
    if not warm:
        return _handover_cold(link, gz_bridge, mav, t0)
    unconfirmed = link.set_params([('SIM_GZ_EC_FUNC%d' % i, 100 + i) for i in range(1, 5)])
    t_par = (time.time() - t0) * 1000.0
    # Offboard needs a setpoint before the mode switch. PX4 is switched to Offboard while still disarmed,
    # during the routing wait (which it outlasts), so the arm burst carries the arm command alone; the
    # stream runs at 50 Hz until the arm, at 500 Hz from then on.
    offboard = (mav.MAV_CMD_DO_SET_MODE, (mav.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 6, 0, 0, 0, 0, 0))
    link.offboard_setpoint()
    with link.tx:
        link.c.mav.command_long_send(link.c.target_system, link.c.target_component, offboard[0], 0, *offboard[1])
    stop, fast = threading.Event(), threading.Event()
    threading.Thread(target=link.offboard_stream, args=(stop, fast), daemon=True).start()
    polls = 0
    while polls < 100:
        r = subprocess.run([gz_bridge, 'status'], capture_output=True, text=True)
        if len(ROUTED.findall(r.stdout)) >= 4:
            break
        polls += 1
    t_rt = (time.time() - t0) * 1000.0
    acks = link.commands([offboard,   # its ack is usually waiting already
                          (mav.MAV_CMD_COMPONENT_ARM_DISARM, (1, 21196, 0, 0, 0, 0, 0))],   # 21196: force (in the air)
                         sent=(offboard[0],), until_ok=(offboard[0],))
    t_ctl = (time.time() - t0) * 1000.0
    fast.set()
    # the in-air report and the vertical speed for the catch, at 200 Hz from here (acks not awaited)
    with link.tx:
        for msg_id in (245, 32):   # EXTENDED_SYS_STATE, LOCAL_POSITION_NED
            link.c.mav.command_long_send(link.c.target_system, link.c.target_component,
                                         mav.MAV_CMD_SET_MESSAGE_INTERVAL, 0, msg_id, 5000, 0, 0, 0, 0, 0)
    off, arm = acks.get(mav.MAV_CMD_DO_SET_MODE), acks.get(mav.MAV_CMD_COMPONENT_ARM_DISARM)
    # The catch, in Offboard: PX4's velocity loop holds vz without a jerk-limited trajectory. Switch to
    # Takeoff only once PX4 reports itself in the air (its land detector starts "landed" at arming, and
    # while it does the position controller drops back to its ground state, where the auto task is reset
    # to 0.7 m/s down every cycle: a Takeoff entered then fell 1.1 m) and the vehicle has stopped sinking:
    # Takeoff's trajectory starts from the live velocity and acceleration, and from a sink it kept
    # descending for 0.3 s before reversing (0.23 and 0.84 m lost after a hang).
    n_air, n_still, vz, dl = 0, 0, float('nan'), time.time() + CATCH_S
    while (n_air < 3 or n_still < 2) and time.time() < dl:
        m = link.c.recv_match(type=['EXTENDED_SYS_STATE', 'LOCAL_POSITION_NED'], blocking=True, timeout=0.01)
        if m is None:
            continue
        if m.get_type() == 'EXTENDED_SYS_STATE':
            n_air = n_air + 1 if m.landed_state == mav.MAV_LANDED_STATE_IN_AIR else 0
        else:
            vz = m.vz
            n_still = n_still + 1 if (n_air >= 3 and abs(vz) < CATCH_VZ) else 0   # still once in the air
    t_up = (time.time() - t0) * 1000.0
    acks2 = link.commands([(mav.MAV_CMD_DO_SET_MODE, (mav.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4, 2, 0, 0, 0, 0))])
    tko = acks2.get(mav.MAV_CMD_DO_SET_MODE)
    stop.set()   # PX4 flies its own Takeoff now (a refused switch is reported INCOMPLETE)
    t_tko = (time.time() - t0) * 1000.0
    ok = not unconfirmed and polls < 100 and arm == 0 and off == 0 and tko == 0 and n_air >= 3
    return ok, ('warm, params +%.0f ms, routed_polls=%d +%.0f ms, params_unconfirmed=%s, offboard_ack=%s, '
                'arm_ack=%s, in_air=%s caught=%s (vz %.2f) +%.0f ms, takeoff_ack=%s +%.0f ms' % (
                    t_par, polls, t_rt, ','.join(unconfirmed) or '-', off, arm, n_air >= 3, n_still >= 2, vz,
                    t_up, tko, t_tko)), t_ctl


def _handover_cold(link, gz_bridge, mav, t0):
    unconfirmed = link.set_params([('SIM_GZ_EC_FUNC%d' % i, 100 + i) for i in range(1, 5)])
    t_par = (time.time() - t0) * 1000.0
    # PX4 applies output routing on the mixer's next parameter update, and only while disarmed;
    # arming before all four channels report routed left channels unrouted (random tumbles).
    polls = 0
    while polls < 100:
        r = subprocess.run([gz_bridge, 'status'], capture_output=True, text=True)
        if len(ROUTED.findall(r.stdout)) >= 4:
            break
        polls += 1
    t_rt = (time.time() - t0) * 1000.0
    # Takeoff with COM_SPOOLUP_TIME=0, MPC_TKO_RAMP_T=0 (set at boot) flies from its first cycle,
    # climbs to MIS_TAKEOFF_ALT and switches itself to Hold.
    acks = link.commands([
        (mav.MAV_CMD_COMPONENT_ARM_DISARM, (1, 21196, 0, 0, 0, 0, 0)),   # 21196: force (in the air)
        (mav.MAV_CMD_DO_SET_MODE, (mav.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4, 2, 0, 0, 0, 0)),   # AUTO, TAKEOFF
    ])
    arm = acks.get(mav.MAV_CMD_COMPONENT_ARM_DISARM)
    mode = acks.get(mav.MAV_CMD_DO_SET_MODE)
    ok = not unconfirmed and polls < 100 and arm == 0 and mode == 0
    return ok, 'cold, params +%.0f ms, routed_polls=%d +%.0f ms, params_unconfirmed=%s, arm_ack=%s, mode_ack=%s' % (
        t_par, polls, t_rt, ','.join(unconfirmed) or '-', arm, mode), (time.time() - t0) * 1000.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--topic', default=TOPIC)
    ap.add_argument('--rate', type=float, default=250.0)
    ap.add_argument('--const', type=float, default=None)
    ap.add_argument('--seconds', type=float, default=3.0)
    ap.add_argument('--stdin', action='store_true')
    ap.add_argument('--handover-ms', type=float, default=0.0,
                    help='hand the motors to PX4 after this many ms without a command line (0 = off)')
    ap.add_argument('--mav', default=None,
                    help='with --handover-ms: the PX4 MAVLink link for the handover, e.g. udp:127.0.0.1:14603')
    ap.add_argument('--gz-bridge', default='/opt/px4-gazebo/bin/px4-gz_bridge')
    ap.add_argument('--immediate', action='store_true',
                    help='publish each command line as it arrives (direct rig) instead of the newest on the --rate grid')
    ap.add_argument('--gap-log', default=None, help='write every command gap over --gap-log-ms here')
    ap.add_argument('--gap-log-ms', type=float, default=20.0)
    ap.add_argument('--watchdog-clock', default='sim', choices=['sim', 'wall'],
                    help='time base of --handover-ms: sim = Gazebo sim time (/world/default/clock, the plant\'s '
                         'clock; wall time until the first clock message), wall = this process\'s wall clock')
    ap.add_argument('--takeover', default='warm', choices=['warm', 'cold'],
                    help='with --handover-ms: warm = PX4 starts from the live state (hover thrust from the last '
                         'commands, Takeoff entered from Offboard), cold = the earlier arm + Takeoff')
    ap.add_argument('--hover-window-s', type=float, default=1.0,
                    help='--takeover warm: the hover thrust is the mean motor command over this window')
    args = ap.parse_args()
    if args.handover_ms > 0 and not args.mav:
        ap.error('--handover-ms needs --mav')

    link = Px4Link(args.mav) if args.handover_ms > 0 else None
    warm = args.handover_ms > 0 and args.takeover == 'warm'
    ec = {}
    ec_ready = threading.Event()

    def warm_setup():
        # PX4's output model, read after the reader has started: a late reader backs up the command
        # pipe (measured: 100-250 ms command gaps during spin-up and two false takeovers)
        names = EC_PARAMS + ['MPC_THR_MIN', 'MPC_THR_MAX']
        got = link.get_params(names)
        missing = [n for n in names if n not in got]
        if missing:
            print('GZ_MOTOR_PUB: PX4 params not read (%s): the takeover will be cold' % ','.join(missing), flush=True)
            return
        ec.update(got)
        ec_ready.set()
        print('GZ_MOTOR_PUB: warm takeover, PX4 output model EC_MIN %s EC_MAX %s THR_MDL_FAC %.3f' % (
            [ec['SIM_GZ_EC_MIN%d' % i] for i in range(1, 5)], [ec['SIM_GZ_EC_MAX%d' % i] for i in range(1, 5)],
            ec['THR_MDL_FAC']), flush=True)
    node = Node()
    pub = node.advertise(args.topic, Actuators)
    sim_t = [None]

    def on_clock(m):
        sim_t[0] = m.sim.sec + m.sim.nsec * 1e-9
    node.subscribe(Clock, '/world/default/clock', on_clock)
    stopped = [False]   # set once this process stops publishing (handover or end)
    latest = [0.0, 0.0, 0.0, 0.0]
    time.sleep(0.5)  # let discovery settle so the first messages land

    pub_lock = threading.Lock()
    hov = collections.deque()   # (time, the four commands) over the last --hover-window-s
    hov_s1, hov_s2 = [0.0] * 4, [0.0] * 4   # their per-motor sums of w and w^2
    count = [0]
    done = threading.Event()
    last_line = [time.time()]
    last_sim = [None]
    got_line = [False]
    gaps = {'n': 0, 'max': 0.0, 'lines': 0, 'sim_max': -1.0}
    gap_f = open(args.gap_log, 'w') if args.gap_log else None

    def reader():
        for line in sys.stdin:
            parts = line.split()
            if len(parts) >= 4:
                try:
                    vals = [float(p) for p in parts[:4]]
                except ValueError:
                    continue
                latest[:] = vals
                if warm:
                    tn = time.time()
                    hov.append((tn, vals))
                    for i in range(4):
                        hov_s1[i] += vals[i]
                        hov_s2[i] += vals[i] * vals[i]
                    while hov[0][0] < tn - args.hover_window_s:
                        old = hov.popleft()[1]
                        for i in range(4):
                            hov_s1[i] -= old[i]
                            hov_s2[i] -= old[i] * old[i]
                if args.immediate:
                    with pub_lock:
                        if not stopped[0]:
                            im = Actuators()
                            im.velocity.extend(vals)
                            pub.publish(im)
                            count[0] += 1
                now = time.time()
                st = sim_t[0]
                if got_line[0]:
                    gap = (now - last_line[0]) * 1000.0
                    sgap = (st - last_sim[0]) * 1000.0 if (st is not None and last_sim[0] is not None) else -1.0
                    gaps['max'] = max(gaps['max'], gap)
                    if gap > args.gap_log_ms:
                        gaps['n'] += 1
                        gaps['sim_max'] = max(gaps['sim_max'], sgap)
                        if gap_f:
                            gap_f.write('%.3f %.1f %.1f\n' % (now, gap, sgap))
                    if args.handover_ms > 0 and gap > args.handover_ms / 2:
                        # with the watchdog armed, report every input gap over half its threshold
                        print('GZ_MOTOR_PUB: input gap %.0f ms (sim %.0f ms) at %.3f' % (gap, sgap, now), flush=True)
                last_line[0] = now
                last_sim[0] = st
                got_line[0] = True
                gaps['lines'] += 1
        done.set()

    if args.stdin:
        threading.Thread(target=reader, daemon=True).start()
    elif args.const is not None:
        latest[:] = [args.const] * 4
    else:
        ap.error('need --const or --stdin')
    standby_stop = threading.Event()
    hover_sent = [None, 0]   # the last MPC_THR_HOVER sent to the standby PX4, number of updates

    def standby_hover():
        # Keep the standby PX4's MPC_THR_HOVER at the hover thrust being flown (the mean command over the
        # last --hover-window-s in PX4's units, within its thrust limits), checked once a second and sent
        # past HOVER_DEADBAND, so the handover burst carries nothing extra. PX4 uses MPC_THR_HOVER until
        # it counts itself flying and until its hover-thrust estimator is valid (5-6 s after a takeover).
        warm_setup()
        while ec_ready.is_set() and not standby_stop.wait(1.0):
            if not hov:
                continue
            hv = min(max(px4_thrust(len(hov), hov_s1, hov_s2, ec), ec['MPC_THR_MIN']), ec['MPC_THR_MAX'])
            if hover_sent[0] is not None and abs(hv - hover_sent[0]) < HOVER_DEADBAND:
                continue
            with link.tx:
                if standby_stop.is_set():
                    break
                link.c.mav.param_set_send(link.c.target_system, link.c.target_component, b'MPC_THR_HOVER',
                                          round(hv, 4), link.mu.mavlink.MAV_PARAM_TYPE_REAL32)
            hover_sent[0], hover_sent[1] = hv, hover_sent[1] + 1
    if warm:
        threading.Thread(target=standby_hover, daemon=True).start()

    period = 1.0 / args.rate
    n = 0
    t0 = time.time()
    deadline = None if args.stdin else t0 + args.seconds
    msg = Actuators()
    handed = None
    while True:
        if args.stdin and args.handover_ms > 0 and got_line[0]:
            wall_ms = (time.time() - last_line[0]) * 1000.0
            st, ls = sim_t[0], last_sim[0]
            # The watchdog runs on the plant's clock: a stall of the whole simulation machine stops
            # Gazebo too, so the vehicle is not starved and must not be taken over (docs/bridge_loss_method.md: such
            # stalls reach 150+ ms with 16-20 ms of sim time). Wall time before the first clock message.
            sim_clock = args.watchdog_clock == 'sim' and st is not None and ls is not None
            stale_ms = (st - ls) * 1000.0 if sim_clock else wall_ms
            if done.is_set() or stale_ms > args.handover_ms:
                with pub_lock:
                    stopped[0] = True
                t_h = time.time()
                why = 'eof' if done.is_set() else 'stale'
                standby_stop.set()
                with link.tx:   # an update in flight finishes first
                    pass
                ok, detail, ctl_ms = handover(link, args.gz_bridge, warm and ec_ready.is_set())
                detail += ', hover_thrust=%s (%d standby updates)' % (
                    '%.3f' % hover_sent[0] if hover_sent[0] is not None else 'boot', hover_sent[1])
                handed = (why, stale_ms, ctl_ms)
                print('GZ_MOTOR_PUB: HANDOVER to PX4 (%s, %.0f ms since last command, route+arm+takeoff %.0f ms, %s, %s, '
                      'watchdog clock %s, wall %.0f ms since last command) at %.3f'
                      % (why, stale_ms, handed[2], detail, 'ok' if ok else 'INCOMPLETE',
                         'sim' if sim_clock else 'wall', wall_ms, t_h), flush=True)
                break
        if args.stdin and done.is_set():
            break
        if deadline is not None and time.time() >= deadline:
            break
        if args.immediate:
            time.sleep(0.001)   # the reader publishes; this loop only runs the watchdog
            continue
        del msg.velocity[:]
        msg.velocity.extend(latest)
        pub.publish(msg)
        n += 1
        time.sleep(period)
    with pub_lock:
        stopped[0] = True
    n += count[0]
    # Leave the plant idle (not after a handover: PX4 owns the topic then).
    if handed is None:
        del msg.velocity[:]
        msg.velocity.extend([0.0, 0.0, 0.0, 0.0])
        for _ in range(5):
            pub.publish(msg)
            time.sleep(period)
    if args.stdin:
        print('GZ_MOTOR_PUB: command gaps: %d lines, %d over %.0f ms, worst %.1f ms (sim time in the logged gaps: worst %.1f ms)'
              % (gaps['lines'], gaps['n'], args.gap_log_ms, gaps['max'], gaps['sim_max']), flush=True)
    if gap_f:
        gap_f.close()
    print('GZ_MOTOR_PUB: %d messages in %.1fs on %s' % (n, time.time() - t0, args.topic),
          flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
