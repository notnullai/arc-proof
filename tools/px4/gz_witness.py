"""Latency witness for the Gazebo rig: logs, from Gazebo's own topics, when a rotor fails and when the
motor commands change, on Gazebo's sim clock. Same process in both rigs (mavlink and direct), so the
sensor-to-command latency is measured the same way before and after.

Runs inside the container beside the run:
  python3 gz_witness.py --seconds 75 --out witness.txt

Lines: "I sim_s gx gy gz"        every IMU sample (sim stamp, body FLU rad/s)
       "M sim_s w0 w1 w2 w3"     every message on the motor command topic; sim_s = stamp of the newest IMU
                                 sample received before it (the last completed physics step; the command
                                 acts from the next step on)
       "F sim_s n"               every motor-failure message (n = gz MotorFailure index, 1-based)
       "P sim_s x y z qw qx qy qz"  the model's true pose (world ENU), every dynamic-pose message (50 Hz)
tools/px4/rig_latency.py turns this into failure-to-cut latencies.
"""
import argparse
import sys
import threading
import time

from gz.transport13 import Node
try:
    from gz.msgs10.imu_pb2 import IMU
    from gz.msgs10.actuators_pb2 import Actuators
    from gz.msgs10.int32_pb2 import Int32
    from gz.msgs10.pose_v_pb2 import Pose_V
except ImportError:  # pragma: no cover
    from gz.msgs11.imu_pb2 import IMU
    from gz.msgs11.actuators_pb2 import Actuators
    from gz.msgs11.int32_pb2 import Int32
    from gz.msgs11.pose_v_pb2 import Pose_V

IMU_TOPIC = '/world/default/model/x500_0/link/base_link/sensor/imu_sensor/imu'
MOTOR_TOPIC = '/x500_0/command/motor_speed'
FAIL_TOPIC = '/model/x500_0/motor_failure/motor_number'
POSE_TOPIC = '/world/default/dynamic_pose/info'
MODEL = 'x500_0'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=60.0)
    ap.add_argument('--out', required=True)
    args = ap.parse_args()
    f = open(args.out, 'w')
    lock = threading.Lock()
    last = [0.0]

    def on_imu(m):
        t = m.header.stamp.sec + m.header.stamp.nsec * 1e-9
        with lock:
            last[0] = t
            f.write('I %.4f %.5f %.5f %.5f\n' % (t, m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z))

    def on_motor(m):
        with lock:
            v = list(m.velocity)[:4] + [0.0] * max(0, 4 - len(m.velocity))
            f.write('M %.4f %.1f %.1f %.1f %.1f\n' % (last[0], v[0], v[1], v[2], v[3]))

    def on_fail(m):
        with lock:
            f.write('F %.4f %d\n' % (last[0], m.data))

    def on_pose(m):
        t = m.header.stamp.sec + m.header.stamp.nsec * 1e-9
        for p in m.pose:
            if p.name == MODEL:
                o = p.orientation
                with lock:
                    f.write('P %.4f %.3f %.3f %.3f %.6f %.6f %.6f %.6f\n' % (t, p.position.x, p.position.y, p.position.z,
                                                                          o.w, o.x, o.y, o.z))
                break

    node = Node()
    node.subscribe(IMU, IMU_TOPIC, on_imu)
    node.subscribe(Actuators, MOTOR_TOPIC, on_motor)
    node.subscribe(Int32, FAIL_TOPIC, on_fail)
    node.subscribe(Pose_V, POSE_TOPIC, on_pose)
    t0 = time.time()
    while time.time() - t0 < args.seconds:
        time.sleep(0.2)
        with lock:
            f.flush()
    with lock:
        f.close()
        f = open('/dev/null', 'w')
    return 0


if __name__ == '__main__':
    sys.exit(main())
