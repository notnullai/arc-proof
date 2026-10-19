"""Stream PX4 EKF state as 56-byte records to stdout (binary), with a GCS
heartbeat every second so PX4 keeps its datalink. Runs until --seconds
elapse. --tee writes a copy of every record to a file.

  python tools/px4/ekf_stream.py --seconds 60 --tee run.bin | px4_bridge ... | ...

This is a separate process on purpose: the record path from PX4 to the
bridge is then an OS pipe with no Python thread contention in it.
"""
import argparse
import os
import struct
import sys
import time

from pymavlink import mavutil

REC = struct.Struct('<Q12f')
REC2 = struct.Struct('<Q12fQQ4f')   # v2 (Track F): + att_time_us, gyro_time_us, gx, gy, gz, baro_alt
REC3 = struct.Struct('<Q12fQQ7f')   # v3: v2 + ax, ay, az (HIGHRES_IMU accelerometer, body FRD, m/s^2)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', type=int, default=14550)
    ap.add_argument('--keep-recording', action='store_true',
                    help='G6 kill runs: when the consumer dies, stop forwarding but keep teeing the true stream (direct writes, no thread)')
    ap.add_argument('--survive-sink-loss', action='store_true',
                    help='G6 bridge-loss runs: forward through a bounded queue (drop when full) and keep teeing the true stream after the consumer dies or hangs')
    ap.add_argument('--seconds', type=float, default=60.0)
    ap.add_argument('--tee', default=None)
    ap.add_argument('--stall-at', type=float, default=None, help='stop forwarding records at this wall offset (s)')
    ap.add_argument('--stall-len', type=float, default=1.0)
    ap.add_argument('--stalls', default=None, help='several dropouts: "at:len,at:len" (wall seconds)')
    ap.add_argument('--stall-kind', default='full', choices=['full', 'ekf', 'pos'],
                    help='full: no records at all; ekf: attitude/position/velocity frozen, rates fresh; pos: horizontal position/velocity frozen (GPS loss), attitude, rates and altitude fresh')
    ap.add_argument('--record', default='v1', choices=['v1', 'v2', 'v3'],
                    help='v2 (88 bytes): appends the ATTITUDE timestamp, the raw gyro and baro altitude (HIGHRES_IMU, else ATTITUDE rates and NaN) with their timestamp; v3 (100 bytes): v2 + the raw accelerometer; an ekf stall freezes the attitude timestamp too')
    ap.add_argument('--gap-log', default=None,
                    help='write every LOCAL_POSITION_NED arrival gap over --gap-log-ms here as "wall_time wall_gap_ms sim_gap_ms"')
    ap.add_argument('--gap-log-ms', type=float, default=20.0)
    args = ap.parse_args()
    gap_f = open(args.gap_log, 'w') if args.gap_log else None
    last_arr = None
    v2 = args.record in ('v2', 'v3')
    v3 = args.record == 'v3'
    RECX = REC3 if v3 else REC2
    stalls = []
    if args.stall_at is not None:
        stalls.append((args.stall_at, args.stall_len))
    if args.stalls:
        for part in args.stalls.split(','):
            a, l = part.split(':')
            stalls.append((float(a), float(l)))
    out = sys.stdout.buffer
    sink_q = None
    if args.survive_sink_loss:
        import queue
        import threading
        sink_q = queue.Queue(maxsize=200)
        sink_dead = threading.Event()

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
    gcs = mavutil.mavlink_connection('udp:0.0.0.0:%d' % args.port, source_system=255, source_component=198)
    if gcs.wait_heartbeat(timeout=60) is None:
        sys.stderr.write('EKF_STREAM: no heartbeat\n')
        return 1
    if v2:
        # ask for HIGHRES_IMU at 50 Hz (id 105); harmless if PX4 already streams it
        gcs.mav.command_long_send(gcs.target_system, gcs.target_component,
                                  mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, 105, 20000, 0, 0, 0, 0, 0)
    tee = open(args.tee, 'wb') if args.tee else None
    att = None
    imu = None
    imu_seen = 0
    frozen = None
    n = 0
    t0 = time.time()
    nb = t0
    try:
        while time.time() - t0 < args.seconds:
            if time.time() >= nb:
                gcs.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                nb += 1.0
            m = gcs.recv_match(type=['ATTITUDE', 'LOCAL_POSITION_NED'] + (['HIGHRES_IMU'] if v2 else []),
                               blocking=True, timeout=0.5)
            if m is None:
                continue
            if m.get_type() == 'ATTITUDE':
                att = m
                continue
            if m.get_type() == 'HIGHRES_IMU':
                imu = m
                imu_seen += 1
                continue
            if att is None:
                continue
            t_us = int(m.time_boot_ms) * 1000
            if gap_f is not None:
                t_arr = time.time()
                if last_arr is not None and (t_arr - last_arr[0]) * 1000.0 > args.gap_log_ms:
                    gap_f.write('%.3f %.1f %.1f\n' % (t_arr, (t_arr - last_arr[0]) * 1000.0, (t_us - last_arr[1]) / 1000.0))
                    gap_f.flush()
                last_arr = (t_arr, t_us)
            att_t = int(att.time_boot_ms) * 1000
            # raw body rates: the IMU stream when PX4 sends one, else the ATTITUDE rates
            # (then gyro freshness is attitude freshness and the fallback cannot engage)
            nan = float('nan')
            if imu is not None:
                gyro = (int(imu.time_usec), imu.xgyro, imu.ygyro, imu.zgyro, imu.pressure_alt)
                acc = (imu.xacc, imu.yacc, imu.zacc)
            else:
                gyro = (att_t, att.rollspeed, att.pitchspeed, att.yawspeed, nan)
                acc = (nan, nan, nan)
            if v3:
                gyro = gyro + acc
            fields = [t_us, att.roll, att.pitch, att.yaw, att.rollspeed, att.pitchspeed, att.yawspeed,
                      m.x, m.y, m.z, m.vx, m.vy, m.vz]
            tail = [att_t] + list(gyro)
            true_rec = RECX.pack(*(fields + tail)) if v2 else REC.pack(*fields)
            if tee:
                tee.write(true_rec)   # the tee holds the true stream; stalls apply to what is forwarded
            now = time.time() - t0
            stalled = any(a <= now < a + l for a, l in stalls)
            if stalled and args.stall_kind == 'full':
                continue   # simulated sensor dropout: records still recorded, not forwarded
            if stalled:
                if frozen is None:
                    frozen = (att.roll, att.pitch, att.yaw, m.x, m.y, m.z, m.vx, m.vy, m.vz, att_t)
                if args.stall_kind == 'ekf':
                    fields = [t_us, frozen[0], frozen[1], frozen[2], att.rollspeed, att.pitchspeed, att.yawspeed,
                              frozen[3], frozen[4], frozen[5], frozen[6], frozen[7], frozen[8]]
                    tail = [frozen[9]] + list(gyro)   # attitude timestamp frozen with the attitude; gyro fresh
                else:   # pos: GPS loss - horizontal position/velocity frozen, attitude, rates, altitude fresh
                    fields = [t_us, att.roll, att.pitch, att.yaw, att.rollspeed, att.pitchspeed, att.yawspeed,
                              frozen[3], frozen[4], m.z, frozen[6], frozen[7], m.vz]
            else:
                frozen = None
            rec = true_rec if not stalled else (RECX.pack(*(fields + tail)) if v2 else REC.pack(*fields))
            if sink_q is not None:
                if not sink_dead.is_set():
                    try:
                        sink_q.put_nowait(rec)
                    except Exception:
                        pass   # consumer hung: drop, keep recording
                n += 1
                continue
            if out is None:
                n += 1
                continue   # consumer gone (--keep-recording): keep teeing the true stream
            try:
                out.write(rec)
                out.flush()
            except (BrokenPipeError, OSError):
                if args.keep_recording:
                    out = None
                    continue
                break
            n += 1
    finally:
        if tee:
            tee.close()
        try:
            if out is not None:
                out.close()
        except OSError:
            pass
    sys.stderr.write('EKF_STREAM: %d records in %.1fs%s\n' % (
        n, time.time() - t0, (' gyro=%s' % ('HIGHRES_IMU(%d)' % imu_seen if imu_seen else 'ATTITUDE rates')) if v2 else ''))
    return 0


if __name__ == '__main__':
    sys.exit(main())
