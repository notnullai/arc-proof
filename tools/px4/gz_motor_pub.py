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
mixer's routing); then a force-arm COMMAND_LONG and a Takeoff mode switch in
one burst, and wait for both COMMAND_ACKs. PX4's own controller then owns the
motors with fresh state. One-way: authority never returns to Arc in a run.
The motor plugins hold their last command in the gap.

Gap log (--gap-log PATH): every interval between command lines longer than
--gap-log-ms (default 20 ms; records arrive every 20 ms and the bridge writes
a burst of lines per record) is written to PATH as "wall_time gap_ms
sim_gap_ms" (sim_gap_ms: Gazebo sim time that passed in the same interval,
from /world/default/clock; -1 before the first clock message), and a summary
line is printed at the end.
"""
import argparse
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


class Px4Link:
    """The watchdog's own MAVLink connection to PX4, opened before flight so the
    handover costs no connection setup. PX4 integer params travel byte-wise in
    PARAM_SET's float field (see mavparam.py)."""

    def __init__(self, url, wait_s=60.0):
        from pymavlink import mavutil
        self.mu = mavutil
        self.c = mavutil.mavlink_connection(url, source_system=255, source_component=191)
        if self.c.wait_heartbeat(timeout=wait_s) is None:
            raise SystemExit('GZ_MOTOR_PUB: no PX4 heartbeat on %s' % url)

    def drain(self):
        while self.c.recv_match(blocking=False) is not None:
            pass

    def set_int_params(self, pairs, timeout_s=1.0):
        """PARAM_SET every pair in one burst; resend the unconfirmed ones every 30 ms
        until each PARAM_VALUE echo carries the value. Returns the names still unconfirmed."""
        pending = dict(pairs)
        dl = time.time() + timeout_s
        nxt = 0.0
        while pending and time.time() < dl:
            if time.time() >= nxt:
                for name, v in pending.items():
                    self.c.mav.param_set_send(self.c.target_system, self.c.target_component, name.encode(),
                                              struct.unpack('<f', struct.pack('<i', v))[0],
                                              self.mu.mavlink.MAV_PARAM_TYPE_INT32)
                nxt = time.time() + RESEND_S
            m = self.c.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.005)
            if m is None:
                continue
            pid = m.param_id if isinstance(m.param_id, str) else m.param_id.decode(errors='replace')
            pid = pid.rstrip('\x00')
            if pid in pending and struct.unpack('<i', struct.pack('<f', m.param_value))[0] == pending[pid]:
                del pending[pid]
        return sorted(pending)

    def commands(self, cmds, timeout_s=1.0):
        """Send COMMAND_LONGs in order in one burst; resend the unacknowledged ones (in
        order) every 30 ms. Returns {command: result}."""
        res = {}
        dl = time.time() + timeout_s
        nxt = 0.0
        while len(res) < len(cmds) and time.time() < dl:
            if time.time() >= nxt:
                for c, p in cmds:
                    if c not in res:
                        self.c.mav.command_long_send(self.c.target_system, self.c.target_component, c, 0, *p)
                nxt = time.time() + RESEND_S
            m = self.c.recv_match(type='COMMAND_ACK', blocking=True, timeout=0.005)
            if m is not None and m.command in [c for c, _ in cmds] and m.command not in res:
                res[m.command] = m.result
        return res


def handover(link, gz_bridge):
    """Route, check routed, arm, Takeoff. Returns a short status string."""
    mav = link.mu.mavlink
    link.drain()
    unconfirmed = link.set_int_params([('SIM_GZ_EC_FUNC%d' % i, 100 + i) for i in range(1, 5)])
    # PX4 applies output routing on the mixer's next parameter update, and only while disarmed;
    # arming before all four channels report routed left channels unrouted (random tumbles).
    polls = 0
    while polls < 100:
        r = subprocess.run([gz_bridge, 'status'], capture_output=True, text=True)
        if len(ROUTED.findall(r.stdout)) >= 4:
            break
        polls += 1
    # Takeoff with COM_SPOOLUP_TIME=0, MPC_TKO_RAMP_T=0 (set at boot) flies from its first cycle,
    # climbs to MIS_TAKEOFF_ALT and switches itself to Hold.
    acks = link.commands([
        (mav.MAV_CMD_COMPONENT_ARM_DISARM, (1, 21196, 0, 0, 0, 0, 0)),   # 21196: force (in the air)
        (mav.MAV_CMD_DO_SET_MODE, (mav.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4, 2, 0, 0, 0, 0)),   # AUTO, TAKEOFF
    ])
    arm = acks.get(mav.MAV_CMD_COMPONENT_ARM_DISARM)
    mode = acks.get(mav.MAV_CMD_DO_SET_MODE)
    ok = not unconfirmed and polls < 100 and arm == 0 and mode == 0
    return ok, 'routed_polls=%d, params_unconfirmed=%s, arm_ack=%s, mode_ack=%s' % (
        polls, ','.join(unconfirmed) or '-', arm, mode)


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
    ap.add_argument('--gap-log', default=None, help='write every command gap over --gap-log-ms here')
    ap.add_argument('--gap-log-ms', type=float, default=20.0)
    ap.add_argument('--watchdog-clock', default='sim', choices=['sim', 'wall'],
                    help='time base of --handover-ms: sim = Gazebo sim time (/world/default/clock, the plant\'s '
                         'clock; wall time until the first clock message), wall = this process\'s wall clock')
    args = ap.parse_args()
    if args.handover_ms > 0 and not args.mav:
        ap.error('--handover-ms needs --mav')

    link = Px4Link(args.mav) if args.handover_ms > 0 else None
    node = Node()
    pub = node.advertise(args.topic, Actuators)
    sim_t = [None]

    def on_clock(m):
        sim_t[0] = m.sim.sec + m.sim.nsec * 1e-9
    node.subscribe(Clock, '/world/default/clock', on_clock)
    time.sleep(0.5)  # let discovery settle so the first messages land

    latest = [0.0, 0.0, 0.0, 0.0]
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
                t_h = time.time()
                why = 'eof' if done.is_set() else 'stale'
                ok, detail = handover(link, args.gz_bridge)
                handed = (why, stale_ms, (time.time() - t_h) * 1000.0)
                print('GZ_MOTOR_PUB: HANDOVER to PX4 (%s, %.0f ms since last command, route+arm+takeoff %.0f ms, %s, %s, '
                      'watchdog clock %s, wall %.0f ms since last command) at %.3f'
                      % (why, stale_ms, handed[2], detail, 'ok' if ok else 'INCOMPLETE',
                         'sim' if sim_clock else 'wall', wall_ms, t_h), flush=True)
                break
        if args.stdin and done.is_set():
            break
        if deadline is not None and time.time() >= deadline:
            break
        del msg.velocity[:]
        msg.velocity.extend(latest)
        pub.publish(msg)
        n += 1
        time.sleep(period)
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
