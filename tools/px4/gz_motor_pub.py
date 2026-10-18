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

Handover (gap register G6, --handover-ms N): PX4 runs as the estimator,
disarmed and with its outputs unrouted (the normal arc-owns mode), so its
controllers are idle and nothing winds up. If no new command line arrives
for N ms, or stdin closes (the Arc process crashed, hung or was killed),
this process stops publishing for good, routes SIM_GZ_EC_FUNC1..4 =
101..104 (PX4 accepts output routing only while disarmed), force-arms PX4
in the air and commands Takeoff (then Hold) through the PX4 shell clients, so
PX4's own controller owns the motors with fresh state. One-way: authority
never returns to Arc in a run. The motor plugins hold their last command
in the gap.
"""
import argparse
import subprocess
import sys
import threading
import time

from gz.transport13 import Node

try:
    from gz.msgs10.actuators_pb2 import Actuators
except ImportError:  # pragma: no cover
    from gz.msgs11.actuators_pb2 import Actuators

TOPIC = '/x500_0/command/motor_speed'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--topic', default=TOPIC)
    ap.add_argument('--rate', type=float, default=250.0)
    ap.add_argument('--const', type=float, default=None)
    ap.add_argument('--seconds', type=float, default=3.0)
    ap.add_argument('--stdin', action='store_true')
    ap.add_argument('--handover-ms', type=float, default=0.0,
                    help='hand the motors to PX4 after this many ms without a command line (0 = off)')
    ap.add_argument('--px4-param', default='/opt/px4-gazebo/bin/px4-param')
    ap.add_argument('--px4-commander', default='/opt/px4-gazebo/bin/px4-commander')
    args = ap.parse_args()

    node = Node()
    pub = node.advertise(args.topic, Actuators)
    time.sleep(0.5)  # let discovery settle so the first messages land

    latest = [0.0, 0.0, 0.0, 0.0]
    done = threading.Event()
    last_line = [time.time()]
    got_line = [False]

    def reader():
        for line in sys.stdin:
            parts = line.split()
            if len(parts) >= 4:
                try:
                    vals = [float(p) for p in parts[:4]]
                except ValueError:
                    continue
                latest[:] = vals
                last_line[0] = time.time()
                got_line[0] = True
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
            stale_ms = (time.time() - last_line[0]) * 1000.0
            if done.is_set() or stale_ms > args.handover_ms:
                t_h = time.time()
                why = 'eof' if done.is_set() else 'stale'
                # PX4 applies output routing only while disarmed (verified on this image), so:
                # route the motors to PX4, wait until the Gazebo bridge reports all four motor
                # channels routed (the mixer applies the change on its next parameter update;
                # arming first can leave a channel unrouted: seen as random tumbles), then arm it
                # in the air and command Takeoff. In position modes PX4 holds zero thrust until its
                # takeoff stage completes; with COM_SPOOLUP_TIME=0 and MPC_TKO_RAMP_T=0 (set at
                # boot) that stage passes on the first cycle; Takeoff climbs to MIS_TAKEOFF_ALT
                # and switches itself to Hold.
                gzb = args.px4_commander.replace('px4-commander', 'px4-gz_bridge')
                cmd = (' && '.join('%s set SIM_GZ_EC_FUNC%d %d' % (args.px4_param, i, 100 + i) for i in range(1, 5)) +
                       '; i=0; while [ $i -lt 100 ]; do n=$(%s status | grep -cE "Channel  [0-3]: func: 10[1-4]"); '
                       '[ "$n" -ge 4 ] && break; i=$((i+1)); sleep 0.01; done; echo routed_polls=$i; ' % gzb +
                       '%s arm -f; %s mode auto:takeoff' % (args.px4_commander, args.px4_commander))
                r = subprocess.run(['sh', '-c', cmd], capture_output=True, text=True)
                handed = (why, stale_ms, (time.time() - t_h) * 1000.0, r.returncode)
                routed = [l for l in r.stdout.splitlines() if l.startswith('routed_polls=')]
                print('GZ_MOTOR_PUB: HANDOVER to PX4 (%s, %.0f ms since last command, route+arm+takeoff %.0f ms, %s, rc %d) at %.3f'
                      % (why, stale_ms, handed[2], routed[0] if routed else 'routed_polls=?', r.returncode, t_h), flush=True)
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
    print('GZ_MOTOR_PUB: %d messages in %.1fs on %s' % (n, time.time() - t0, args.topic),
          flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
