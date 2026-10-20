"""Direct sensor path for the Gazebo rig (--rig direct): one v3 record per Gazebo IMU sample.

Runs inside the PX4 Gazebo container, in place of ekf_stream.py, and writes the same 100-byte v3
records to stdout for px4_bridge. The difference is where the fast fields come from:

  record time, rates, gyro, accelerometer   Gazebo's IMU topic (gz-transport), every sample (250 Hz on the
                                            stock world), stamped with the sample's sim time; body FLU
                                            converted to PX4's FRD (x, -y, -z), as PX4's own gz bridge does
  attitude                                  PX4's estimate (MAVLink ATTITUDE, 50 Hz) propagated from its own
                                            stamp to the IMU sample with the gyro samples in between, so the
                                            attitude in each record is at the record's time, as an on-board
                                            estimator running at the IMU rate gives it
  position, velocity, baro altitude         PX4's estimate (LOCAL_POSITION_NED, HIGHRES_IMU) as received

PX4 stays the estimator and the fallback. Only the inner-loop inputs skip the MAVLink path (whose rates are
12-16 ms older than its records and arrive at 50 Hz). PX4's time_boot_ms runs on the Gazebo clock, so
both sources share one time base (the offset between them is measured and reported at the end).

  python3 gz_imu_stream.py --port 14601 --seconds 72 --tee rec.bin [--keep-recording|--survive-sink-loss]
         [--gap-log stream_gaps.txt --gap-log-ms 20] [--lat-log rec_lat.txt]

--lat-log: per record "wall_time imu_stamp_s att_age_ms" (att_age_ms = record time - PX4 attitude stamp,
the span the attitude was propagated over).
"""
import argparse
import math
import os
import struct
import sys
import threading
import time
from collections import deque

from pymavlink import mavutil
from gz.transport13 import Node
try:
    from gz.msgs10.imu_pb2 import IMU
except ImportError:  # pragma: no cover
    from gz.msgs11.imu_pb2 import IMU

REC3 = struct.Struct('<Q12fQQ7f')
IMU_TOPIC = '/world/default/model/x500_0/link/base_link/sensor/imu_sensor/imu'


def euler_step(roll, pitch, yaw, p, q, r, dt):
    """ZYX Euler angles advanced by body rates (FRD) over dt."""
    sr, cr = math.sin(roll), math.cos(roll)
    cp = math.cos(pitch)
    if abs(cp) < 1e-3:
        cp = math.copysign(1e-3, cp)
    tp = math.sin(pitch) / cp
    roll += (p + (q * sr + r * cr) * tp) * dt
    pitch += (q * cr - r * sr) * dt
    yaw += ((q * sr + r * cr) / cp) * dt
    yaw = (yaw + math.pi) % (2.0 * math.pi) - math.pi
    return roll, pitch, yaw


class Px4State:
    """Latest PX4 estimator messages, read on the companion link by a thread of their own."""

    def __init__(self, port):
        self.c = mavutil.mavlink_connection('udp:0.0.0.0:%d' % port, source_system=255, source_component=198)
        self.lock = threading.Lock()
        self.att = None      # (t_us, roll, pitch, yaw)
        self.lpos = None     # (x, y, z, vx, vy, vz)
        self.baro = float('nan')
        self.n_att = 0
        self.stop = threading.Event()

    def wait(self, timeout=60.0):
        if self.c.wait_heartbeat(timeout=timeout) is None:
            return False
        # HIGHRES_IMU at 50 Hz (baro altitude), as ekf_stream.py asks for it
        self.c.mav.command_long_send(self.c.target_system, self.c.target_component,
                                     mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, 105, 20000, 0, 0, 0, 0, 0)
        return True

    def run(self):
        nb = time.time()
        while not self.stop.is_set():
            if time.time() >= nb:
                self.c.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                nb += 1.0
            m = self.c.recv_match(type=['ATTITUDE', 'LOCAL_POSITION_NED', 'HIGHRES_IMU'], blocking=True, timeout=0.2)
            if m is None:
                continue
            t = m.get_type()
            with self.lock:
                if t == 'ATTITUDE':
                    self.att = (int(m.time_boot_ms) * 1000, m.roll, m.pitch, m.yaw)
                    self.n_att += 1
                elif t == 'LOCAL_POSITION_NED':
                    self.lpos = (m.x, m.y, m.z, m.vx, m.vy, m.vz)
                else:
                    self.baro = m.pressure_alt


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', type=int, default=14601)
    ap.add_argument('--seconds', type=float, default=60.0)
    ap.add_argument('--tee', default=None)
    ap.add_argument('--imu-topic', default=IMU_TOPIC)
    ap.add_argument('--keep-recording', action='store_true',
                    help='G6 kill runs: when the consumer dies, stop forwarding but keep teeing the true stream')
    ap.add_argument('--survive-sink-loss', action='store_true',
                    help='G6 hang runs: forward through a bounded queue (drop when full), keep teeing after the consumer hangs')
    ap.add_argument('--gap-log', default=None, help='IMU arrival gaps over --gap-log-ms: "wall_time wall_gap_ms sim_gap_ms"')
    ap.add_argument('--gap-log-ms', type=float, default=20.0)
    ap.add_argument('--lat-log', default=None)
    ap.add_argument('--record', default='v3', choices=['v3'], help='accepted for symmetry with ekf_stream.py')
    args = ap.parse_args()

    px4 = Px4State(args.port)
    if not px4.wait():
        sys.stderr.write('GZ_IMU_STREAM: no heartbeat\n')
        return 1
    threading.Thread(target=px4.run, daemon=True).start()

    out = sys.stdout.buffer
    tee = open(args.tee, 'wb') if args.tee else None
    gap_f = open(args.gap_log, 'w') if args.gap_log else None
    lat_f = open(args.lat_log, 'w') if args.lat_log else None
    sink_q = None
    sink_dead = threading.Event()
    if args.survive_sink_loss:
        import queue
        sink_q = queue.Queue(maxsize=1000)

        def sink_writer():
            while True:
                b = sink_q.get()
                if b is None:
                    return
                try:
                    out.write(b)
                    out.flush()
                except (BrokenPipeError, OSError, ValueError):
                    sink_dead.set()
                    return
        threading.Thread(target=sink_writer, daemon=True).start()

    st = {'out': out, 'n': 0, 'last': None, 'gyro': deque(maxlen=64), 'prop': None, 'att_key': None,
          'closed': False, 'age_max': 0.0, 'age_sum': 0.0, 'off_min': None, 'off_max': None}
    lock = threading.Lock()
    nan = float('nan')

    def on_imu(m):
        t_us = int(m.header.stamp.sec) * 1000000 + int(m.header.stamp.nsec) // 1000
        # FLU -> FRD
        g = (m.angular_velocity.x, -m.angular_velocity.y, -m.angular_velocity.z)
        a = (m.linear_acceleration.x, -m.linear_acceleration.y, -m.linear_acceleration.z)
        with px4.lock:
            att, lpos, baro = px4.att, px4.lpos, px4.baro
        with lock:
            if st['closed']:
                return
            now = time.time()
            last = st['last']
            if last is not None and t_us <= last[1]:
                return   # repeated or old sample
            if gap_f is not None and last is not None and (now - last[0]) * 1000.0 > args.gap_log_ms:
                gap_f.write('%.3f %.1f %.1f\n' % (now, (now - last[0]) * 1000.0, (t_us - last[1]) / 1000.0))
                gap_f.flush()
            st['last'] = (now, t_us)
            st['gyro'].append((t_us, g))
            if att is None or lpos is None:
                return
            # attitude at t_us: PX4's estimate propagated from its stamp with the gyro samples since
            if st['att_key'] != att[0]:
                st['att_key'] = att[0]
                r, p, y = att[1], att[2], att[3]
                tp = att[0]
                for ts, gs in st['gyro']:
                    if ts <= att[0]:
                        continue
                    r, p, y = euler_step(r, p, y, gs[0], gs[1], gs[2], (ts - tp) * 1e-6)
                    tp = ts
                st['prop'] = (tp, r, p, y)
            else:
                tp, r, p, y = st['prop']
                if t_us > tp:
                    r, p, y = euler_step(r, p, y, g[0], g[1], g[2], (t_us - tp) * 1e-6)
                    st['prop'] = (t_us, r, p, y)
            off = (t_us - att[0]) / 1000.0
            st['off_min'] = off if st['off_min'] is None else min(st['off_min'], off)
            st['off_max'] = off if st['off_max'] is None else max(st['off_max'], off)
            st['age_sum'] += off
            if lat_f is not None:
                lat_f.write('%.4f %.4f %.1f\n' % (now, t_us * 1e-6, off))
            rec = REC3.pack(t_us, r, p, y, g[0], g[1], g[2], lpos[0], lpos[1], lpos[2], lpos[3], lpos[4], lpos[5],
                            t_us, t_us, g[0], g[1], g[2], baro if baro == baro else nan, a[0], a[1], a[2])
            if tee:
                tee.write(rec)
            st['n'] += 1
            if sink_q is not None:
                if not sink_dead.is_set():
                    try:
                        sink_q.put_nowait(rec)
                    except Exception:
                        pass
                return
            o = st['out']
            if o is None:
                return
            try:
                o.write(rec)
                o.flush()
            except (BrokenPipeError, OSError, ValueError):
                st['out'] = None
                if not args.keep_recording:
                    done.set()

    done = threading.Event()
    node = Node()
    node.subscribe(IMU, args.imu_topic, on_imu)
    t0 = time.time()
    while time.time() - t0 < args.seconds and not done.is_set():
        time.sleep(0.05)
    # No unsubscribe and no interpreter teardown while gz-transport may be inside the callback (that
    # crashed once in pybind11 at exit, cutting the stream 0.6 s early: g11_hover_v5_2_streamer_exit_crash):
    # close under the lock, report, and leave with os._exit.
    px4.stop.set()
    with lock:
        st['closed'] = True
        if tee:
            tee.close()
        if lat_f:
            lat_f.close()
        if gap_f:
            gap_f.close()
        try:
            if st['out'] is not None:
                st['out'].close()
        except OSError:
            pass
        st['out'] = None
    if sink_q is not None:
        sink_q.put(None)
    n = max(st['n'], 1)
    sys.stderr.write('GZ_IMU_STREAM: %d records in %.1fs gyro=gz-imu attitude=px4+propagated(%d) '
                     'att_age_ms mean %.1f range %s..%s\n'
                     % (st['n'], time.time() - t0, px4.n_att, st['age_sum'] / n,
                        '%.1f' % st['off_min'] if st['off_min'] is not None else '-',
                        '%.1f' % st['off_max'] if st['off_max'] is not None else '-'))
    sys.stderr.flush()
    os._exit(0)


if __name__ == '__main__':
    sys.exit(main())
