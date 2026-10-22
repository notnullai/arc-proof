"""C4 scenario runner: one scenario, one control arm, on a fresh PX4 +
Gazebo x500 container; writes the EKF record stream, the bridge log and a
metrics JSON, and appends a Markdown row.

  python tools/px4/run_scenario.py --arm px4|reflex|pid|residual
         --scenario hover|wind|motor_loss [--seconds 60] [--alt 1.0]
         [--out runs/] [--tag name] [--no-restart] [--hover 769]
         [--wind-n 3.0] [--wind-every 6] [--wind-len 1.0] [--wind-from 15]
         [--fail-motor 1] [--fail-at 30] [--gains k=v,...] [--residual-gain 16384]
         [--fault raw|stabilised]

G6 handover: [--standby px4 --handover-ms 200 --watchdog-clock sim|wall --takeover warm|cold]
[--scenario bridge_loss --loss-kind kill|hang --loss-at 30]
[--arc-side host|container --bridge-linux bin/linux-x86_64/px4_bridge]
(container = the Arc side next to PX4 as on a companion computer: its own PX4
MAVLink link, the static Linux bridge and the motor publisher on local pipes
inside the container; docs/bridge_loss_method.md).

Rig: [--rig mavlink|direct] [--witness]. mavlink (default) = the records come
from PX4's MAVLink stream (ekf_stream.py, 50 Hz, rates 12-16 ms older than the
record) and the publisher re-sends the newest command on a 250 Hz grid.
direct (needs --arc-side container) = one record per Gazebo IMU sample
(gz_imu_stream.py: rates, gyro and accelerometer from Gazebo's IMU topic,
PX4's attitude propagated to the sample, PX4's position, velocity and baro),
every bridge tick forwarded and published the moment it arrives
(gz_motor_pub.py --immediate). PX4 stays the estimator and the fallback in
both. --witness logs Gazebo's IMU, motor command and motor-failure topics on
the sim clock (gz_witness.py, <tag>.witness.txt) for tools/px4/rig_latency.py.
--record-clock own (direct rig): the bridge runs a record's own tick on its
arrival instead of when the next record comes (the stamps rise strictly).
--physics-hz N: the gz world's physics step and the x500's IMU at N Hz (stock
250; both files read from the image and bind-mounted, physics_sdf below).

Arms: px4 = PX4's own controller (px4-owns; takeoff via MAVLink, hold);
reflex / pid / residual = Arc bridge arms (arc-owns, PX4 estimator only).
Scenarios: hover (nothing); wind = lateral force pulses on the model via
/world/default/wrench/persistent + /wrench/clear; motor_loss = gz
MotorFailure topic at --fail-at. All numbers are scenario statistics on
float physics (not bit-exact); the Arc tick stays deterministic for a
given record stream (bridge hash in the JSON).
"""
import argparse
import json
import math
import os
import re
import shlex
import struct
import subprocess
import sys
import threading
import time

from pymavlink import mavutil

from mavparam import set_int_param, set_float_param
import flight_default
from gap_attribution import attribute_gaps, FLIGHT_FROM_S

HERE = os.path.dirname(os.path.abspath(__file__))
REC = struct.Struct('<Q12f')
REC2 = struct.Struct('<Q12fQQ4f')   # v2 record (Track F)
REC3 = struct.Struct('<Q12fQQ7f')   # v3 record (Track F fix: + accelerometer)
IMAGE = 'px4io/px4-sitl-gazebo:latest'
TILT_FAULT = 0.7854


def default_bridge():
    """The prebuilt, stripped bridge shipped under bin/ for this host."""
    root = os.path.abspath(os.path.join(HERE, '..', '..'))
    if os.name == 'nt':
        return os.path.join(root, 'bin', 'windows-x64', 'px4_bridge.exe')
    return os.path.join(root, 'bin', 'linux-x86_64', 'px4_bridge')


def sh(cmd, **kw):
    return subprocess.run(cmd, capture_output=True, text=True, **kw)


X500_SDF = '/opt/px4-gazebo/share/gz/models/x500/model.sdf'


def motor_scaled_sdf(scale, out_dir):
    """G10: the gz x500 with every rotor's motorConstant x scale (thrust per rotor speed, so the maximum
    thrust; reaction torque follows thrust). Read from the image, written next to the run, bind-mounted
    over the model. scale 1 = the stock model, nothing mounted."""
    r = sh(['docker', 'run', '--rm', '--entrypoint', 'cat', IMAGE, X500_SDF])
    if r.returncode != 0 or 'motorConstant' not in r.stdout:
        raise SystemExit('cannot read %s from %s' % (X500_SDF, IMAGE))
    txt = re.sub(r'<motorConstant>([0-9.eE+-]+)</motorConstant>',
                 lambda m: '<motorConstant>%.6e</motorConstant>' % (float(m.group(1)) * scale), r.stdout)
    path = os.path.abspath(os.path.join(out_dir, 'x500_motor_x%.2f.sdf' % scale))
    with open(path, 'w', newline='\n') as f:
        f.write(txt)
    return path


X500_BASE_SDF = '/opt/px4-gazebo/share/gz/models/x500_base/model.sdf'
WORLD_SDF = '/opt/px4-gazebo/share/gz/worlds/default.sdf'


def physics_sdf(hz, out_dir):
    """G12: the stock world with a 1/hz physics step (real-time update rate hz, so still real time) and the
    x500's IMU sensor at hz. What changes for PX4 (measured at 1000, uorb top): sensor_gyro,
    vehicle_angular_velocity and actuator_motors run at hz instead of 250 (IMU_GYRO_RATEMAX is 800 in this
    image), so PX4's rate loop, as the G6 fallback too, runs hz/250 times as often; IMU_INTEG_RATE (250),
    EKF2's prediction (EKF2_PREDICT_US 10 ms), baro (50 Hz), magnetometer (100 Hz), GPS and the motor model
    are unchanged; Gazebo does hz/250 times the steps. Returns [(host path, image path)] to bind-mount; hz 250 = the stock files, nothing mounted."""
    if hz == 250:
        return []
    out = []
    for src, sub, name in [
            (WORLD_SDF, lambda t: re.sub(r'<real_time_update_rate>[0-9.]+</real_time_update_rate>',
                                         '<real_time_update_rate>%d</real_time_update_rate>' % hz,
                                         re.sub(r'<max_step_size>[0-9.eE+-]+</max_step_size>',
                                                '<max_step_size>%g</max_step_size>' % (1.0 / hz), t)),
             'world_default_%dhz.sdf' % hz),
            (X500_BASE_SDF, lambda t: re.sub(r'(<sensor name="imu_sensor" type="imu">.*?<update_rate>)[0-9.]+(</update_rate>)',
                                             lambda m: '%s%d%s' % (m.group(1), hz, m.group(2)), t, count=1, flags=re.S),
             'x500_base_imu_%dhz.sdf' % hz)]:
        r = sh(['docker', 'run', '--rm', '--entrypoint', 'cat', IMAGE, src])
        if r.returncode != 0:
            raise SystemExit('cannot read %s from %s' % (src, IMAGE))
        txt = sub(r.stdout)
        if txt == r.stdout:
            raise SystemExit('physics_sdf: nothing to change in %s' % src)
        path = os.path.abspath(os.path.join(out_dir, name))
        with open(path, 'w', newline='\n') as f:
            f.write(txt)
        out.append((path, src))
    return out


def restart_container(name, model_sdf=None, extra_mounts=()):
    sh(['docker', 'rm', '-f', name])
    # Docker Desktop (Windows, macOS) needs the published port; on Linux docker-proxy would take
    # host UDP 14550 from this harness, and PX4 reaches the host through host-gateway without it.
    ports = [] if sys.platform.startswith('linux') else ['-p', '14550:14550/udp']
    mounts = ['-v', '%s:%s:ro' % (model_sdf, X500_SDF)] if model_sdf else []
    for host, image_path in extra_mounts:
        mounts += ['-v', '%s:%s:ro' % (host, image_path)]
    r = sh(['docker', 'run', '-d', '--name', name, '--add-host=host.docker.internal:host-gateway',
            '-e', 'HEADLESS=1', '-e', 'PX4_SIM_MODEL=gz_x500'] + ports + mounts + [IMAGE])
    if r.returncode != 0:
        raise SystemExit('docker run failed: ' + r.stderr)


PX4_BIN = '/opt/px4-gazebo/bin'
CT_DIR = '/tmp/arc'          # in-container work dir (records, logs, binaries)
CT_PYLIB = '/tmp/arc/pylib'  # in-container copy of pymavlink (the image ships without it)
# Extra PX4 MAVLink instances inside the container, each sending to 127.0.0.1:
ARC_LINK = (14600, 14601)    # Arc's companion link: same stream config as the GCS link (normal mode)
WD_LINK = (14602, 14603)     # the handover watchdog's command link (minimal mode)


def stage_pymavlink(container):
    """Copy the host's pymavlink (pure Python: core modules, CRC, the all, common and
    ardupilotmega dialects) into the container, for the in-container streamer
    and the watchdog's MAVLink link."""
    import io
    import tarfile
    import pymavlink
    root = os.path.dirname(pymavlink.__file__)
    keep = {'all.py', 'ardupilotmega.py', 'common.py', 'minimal.py', '__init__.py'}

    def wanted(parts):
        if '__pycache__' in parts:
            return False
        if len(parts) == 1:
            return parts[0].endswith('.py')
        if parts[0] == 'generator':
            return len(parts) == 2 and parts[1] in ('__init__.py', 'mavcrc.py', 'mavparse.py')
        if parts[0] == 'dialects':
            return (len(parts) == 2 and parts[1] == '__init__.py') or \
                   (len(parts) == 3 and parts[1] in ('v10', 'v20') and parts[2] in keep)
        return False
    buf = io.BytesIO()
    with tarfile.open(fileobj=buf, mode='w') as t:
        for d, _, fs in os.walk(root):
            for f in fs:
                rel = os.path.relpath(os.path.join(d, f), root).replace('\\', '/')
                if wanted(rel.split('/')):
                    t.add(os.path.join(d, f), arcname='pymavlink/' + rel)
    r = subprocess.run(['docker', 'exec', '-i', container, 'sh', '-c',
                        'mkdir -p %s && tar xf - -C %s' % (CT_PYLIB, CT_PYLIB)], input=buf.getvalue(), capture_output=True)
    if r.returncode != 0:
        raise SystemExit('pymavlink staging failed: %s' % r.stderr.decode(errors='replace'))


# The image's GCS link stream rates (etc/init.d-posix/px4-rc.mavlink), copied to Arc's link so
# the record stream Arc sees is the same in both layouts (50 Hz LOCAL_POSITION_NED + ATTITUDE).
GCS_STREAMS = [('POSITION_TARGET_LOCAL_NED', 50), ('LOCAL_POSITION_NED', 50), ('GLOBAL_POSITION_INT', 50),
               ('ATTITUDE', 50), ('ATTITUDE_QUATERNION', 50), ('ATTITUDE_TARGET', 50), ('SERVO_OUTPUT_RAW_0', 50),
               ('RC_CHANNELS', 20), ('OPTICAL_FLOW_RAD', 10)]


def start_mavlink_link(container, link, mode=None, rate=4000000, streams=()):
    """Start one more PX4 MAVLink instance in the container: PX4 binds link[0] and
    sends to 127.0.0.1:link[1] (mode None = normal, as the GCS link)."""
    m = PX4_BIN + '/px4-mavlink'
    cmd = '%s start -x -t 127.0.0.1 -u %d -o %d -r %d -f%s' % (m, link[0], link[1], rate, (' -m ' + mode) if mode else '')
    for name, hz in streams:
        cmd += ' && %s stream -r %d -s %s -u %d' % (m, hz, name, link[0])
    r = sh(['docker', 'exec', container, 'sh', '-c', cmd])
    if r.returncode != 0:
        raise SystemExit('mavlink start failed: %s %s' % (r.stdout, r.stderr))


def gz_pub(container, topic, msgtype, payload):
    return sh(['docker', 'exec', container, 'gz', 'topic', '-t', topic, '-m', msgtype, '-p', payload])


class Mav:
    """One MAVLink connection. Before start_pump(): synchronous helpers
    (params, wait_global). After start_pump(): a reader thread owns recv,
    records EKF state to a file, forwards records to an optional sink, and
    routes COMMAND_ACKs to a queue for cmd()."""
    def __init__(self, port=14550, comp=196):
        self.gcs = mavutil.mavlink_connection('udp:0.0.0.0:%d' % port, source_system=255, source_component=comp)
        if self.gcs.wait_heartbeat(timeout=90) is None:
            raise SystemExit('no PX4 heartbeat')
        self.beat()
        self.pump = None
        self.stop = threading.Event()
        self.acks = []
        self.lock = threading.Lock()
        self.latest = None
        self.n = 0
        self.sink = None
        self.outbox = []

    def send(self, fn):
        """Run a send on the connection from the pump thread (pymavlink is not thread-safe)."""
        if self.pump is None:
            fn()
        else:
            with self.lock:
                self.outbox.append(fn)

    def beat(self):
        self.gcs.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)

    def wait_global(self, timeout=60):
        t0 = time.time()
        while time.time() - t0 < timeout:
            self.beat()
            if self.gcs.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1.0):
                return True
        return False

    def set_params(self, pairs):
        assert self.pump is None
        for name, v in pairs:
            if not set_int_param(self.gcs, name, v):
                raise SystemExit('param %s=%d not confirmed' % (name, v))

    def start_pump(self, path, sink=None):
        self.sink = sink
        self.pump = threading.Thread(target=self._pump, args=(path,), daemon=True)
        self.pump.start()

    def _pump(self, path):
        att = None
        next_beat = time.time()
        with open(path, 'wb') as f:
            while not self.stop.is_set():
                if time.time() >= next_beat:
                    self.beat()
                    next_beat += 1.0
                with self.lock:
                    pending = self.outbox
                    self.outbox = []
                for fn in pending:
                    fn()
                m = self.gcs.recv_match(type=['ATTITUDE', 'LOCAL_POSITION_NED', 'COMMAND_ACK', 'STATUSTEXT'],
                                        blocking=True, timeout=0.3)
                if m is None:
                    continue
                t = m.get_type()
                if t == 'ATTITUDE':
                    att = m
                elif t == 'LOCAL_POSITION_NED' and att is not None:
                    rec = REC.pack(int(m.time_boot_ms) * 1000, att.roll, att.pitch, att.yaw,
                                   att.rollspeed, att.pitchspeed, att.yawspeed, m.x, m.y, m.z, m.vx, m.vy, m.vz)
                    f.write(rec)
                    self.n += 1
                    self.latest = REC.unpack(rec)
                    if self.sink:
                        try:
                            self.sink(rec)
                        except (BrokenPipeError, OSError, ValueError):
                            self.sink = None
                elif t == 'COMMAND_ACK':
                    with self.lock:
                        self.acks.append((m.command, m.result))
                elif t == 'STATUSTEXT':
                    txt = m.text if isinstance(m.text, str) else m.text.decode(errors='replace')
                    if 'Preflight' not in txt:
                        print('PX4: %s' % txt.strip(), flush=True)

    def cmd(self, c, p1=0, p2=0, p3=0, p4=0, p5=0, p6=0, p7=0, tries=6, want_accept=False):
        """Retry until an ACK (or, with want_accept, until ACCEPTED; a
        TEMPORARILY_REJECTED right after boot means the health checks are
        still settling, so keep trying). Returns the last result."""
        last = None
        for _ in range(tries):
            with self.lock:
                self.acks = []
            self.send(lambda: self.gcs.mav.command_long_send(
                self.gcs.target_system, self.gcs.target_component, c, 0, p1, p2, p3, p4, p5, p6, p7))
            dl = time.time() + 2.0
            while time.time() < dl:
                if self.pump is None:
                    m = self.gcs.recv_match(type='COMMAND_ACK', blocking=True, timeout=0.2)
                    hit = [m.result] if (m is not None and m.command == c) else []
                else:
                    with self.lock:
                        hit = [r for (cc, r) in self.acks if cc == c]
                    time.sleep(0.05)
                if hit:
                    last = hit[0]
                    if not want_accept or last == 0:
                        return last
                    time.sleep(1.0)
                    break
        return last

    def finish(self):
        self.stop.set()
        if self.pump:
            self.pump.join(timeout=5)


def metrics_from_records(path, alt, t_dist, wind_windows, rec=REC):
    d = open(path, 'rb').read()
    n = len(d) // rec.size
    rows = [rec.unpack_from(d, i * rec.size) for i in range(n)]
    if not rows:
        return {'records': 0}
    t0 = rows[0][0]
    T = [(r[0] - t0) / 1e6 for r in rows]
    Z = [r[9] for r in rows]
    tilt = [max(abs(r[1]), abs(r[2])) for r in rows]
    xy = [math.hypot(r[7], r[8]) for r in rows]
    m = {'records': n, 'sim_s': T[-1]}
    airborne_i = next((i for i, z in enumerate(Z) if z < -0.5 * alt), None)
    m['takeoff_s'] = None if airborne_i is None else round(T[airborne_i], 2)
    if airborne_i is None:
        m['airborne'] = False
        return m
    m['airborne'] = True

    def window(i0, i1):
        if i1 <= i0:
            return None
        ae = [abs(-Z[i] - alt) for i in range(i0, i1)]
        alts = [-Z[i] for i in range(i0, i1)]
        mean = sum(alts) / len(alts)
        return {'alt_rms': round(math.sqrt(sum(a * a for a in ae) / len(ae)), 3),
                'alt_max_err': round(max(ae), 3),
                'alt_mean': round(mean, 3),
                'alt_jitter_rms': round(math.sqrt(sum((a - mean) ** 2 for a in alts) / len(alts)), 3),
                'xy_max': round(max(xy[i0:i1]), 3),
                'tilt_max_deg': round(math.degrees(max(tilt[i0:i1])), 1)}

    t_settle = T[airborne_i] + 8.0
    i_settle = next((i for i, t in enumerate(T) if t >= t_settle), n)
    i_dist = next((i for i, t in enumerate(T) if t >= t_dist), n) if t_dist else n
    m['settled'] = window(i_settle, i_dist)
    m['disturbed'] = window(i_dist, n) if t_dist else None

    # survival after airborne: first tilt fault or ground impact
    fault_i = next((i for i in range(airborne_i, n) if tilt[i] > TILT_FAULT), None)
    impact_i = next((i for i in range(airborne_i + 50, n) if Z[i] > -0.10), None)
    end_i = min(x for x in (fault_i, impact_i, n) if x is not None)
    m['tilt_fault_s'] = None if fault_i is None else round(T[fault_i], 2)
    m['ground_impact_s'] = None if impact_i is None else round(T[impact_i], 2)
    m['survival_s'] = round(T[min(end_i, n - 1)] - T[airborne_i], 2)
    m['survived_run'] = fault_i is None and impact_i is None

    # recovery after each wind pulse: seconds from pulse end until 2 s continuously in band
    recs = []
    for (ws, we) in wind_windows:
        i_end = next((i for i, t in enumerate(T) if t >= we), None)
        if i_end is None:
            continue
        run = 0
        rec = None
        for i in range(i_end, n):
            inb = xy[i] < 0.25 and abs(-Z[i] - alt) < 0.15
            run = run + 1 if inb else 0
            if run >= 100:            # 100 records ~ 2 s at 50 Hz
                rec = T[i] - we - 2.0
                break
        recs.append(None if rec is None else round(max(rec, 0.0), 2))
    if recs:
        m['wind_recovery_s'] = recs
        m['wind_recovery_max_s'] = max((r for r in recs if r is not None), default=None)
        m['wind_unrecovered'] = sum(1 for r in recs if r is None)
    return m


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--arm', required=True, choices=['px4', 'reflex', 'pid', 'residual'])
    ap.add_argument('--scenario', required=True, choices=['hover', 'wind', 'motor_loss', 'dropout', 'random', 'bridge_loss'])
    ap.add_argument('--seed', type=int, default=1, help='random scenario: schedule seed')
    ap.add_argument('--stall-at', type=float, default=30.0)
    ap.add_argument('--stall-len', type=float, default=1.0)
    ap.add_argument('--stall-kind', default='full', choices=['full', 'ekf', 'pos'])
    ap.add_argument('--seconds', type=float, default=60.0)
    ap.add_argument('--alt', type=float, default=1.0)
    ap.add_argument('--out', default='runs')
    ap.add_argument('--tag', default=None)
    ap.add_argument('--container', default='px4gz')
    ap.add_argument('--no-restart', action='store_true')
    ap.add_argument('--settle', type=float, default=20.0, help='seconds after first global position before flying (EKF settle)')
    ap.add_argument('--hover', type=float, default=769.0)
    ap.add_argument('--wind-n', type=float, default=3.0)
    ap.add_argument('--wind-every', type=float, default=6.0)
    ap.add_argument('--wind-len', type=float, default=1.0)
    ap.add_argument('--wind-from', type=float, default=15.0)
    ap.add_argument('--fail-motor', type=int, default=1)
    ap.add_argument('--fail-at', type=float, default=30.0)
    ap.add_argument('--rotor-loss', default=None,
                    help='G10: rotor-loss brain for the bridge (docs/rotor_loss.md), e.g. models/prior_x500_rotor_loss.arc002; '
                         'a detected rotor failure hands the vehicle to it; default off')
    ap.add_argument('--rotor-loss-detect-only', action='store_true', help='G10: detect and count, never hand over')
    ap.add_argument('--motor-scale', type=float, default=1.0,
                    help='G10: x500 variant with motorConstant x this (more thrust margin); 1 = the stock model')
    ap.add_argument('--gains', default='')
    ap.add_argument('--residual-gain', type=int, default=16384)
    ap.add_argument('--plasticity', default=None, choices=['on', 'off'],
                    help='default: off for the reflex arm on its default flight brain, on otherwise')
    ap.add_argument('--brain', default=None,
                    help='reflex arm brain (default: the flight default in tools/px4/flight_default.py, candidate v5 at '
                         '832 bits, plasticity off; models/prior_x500.arc002 at 480 bits is the learning demonstration; '
                         '"untrained" = the hand-wired DIRECT prior of Track C; any path = that file, give --bits)')
    ap.add_argument('--bits', type=int, default=None)
    ap.add_argument('--fault', default=None, choices=[None, 'raw', 'stabilised', 'live'])
    ap.add_argument('--att-fallback', default='none', choices=['none', 'gyro', 'imu'],
                    help='Track F: v3 records (attitude timestamp, raw gyro, baro, accelerometer) and the bridge fallback: gyro = attitude propagated with the gyro while the estimator attitude is stale; imu = gyro + altitude/vertical velocity from a baro + accelerometer complementary filter')
    ap.add_argument('--standby', default='none', choices=['none', 'px4'],
                    help='G6: PX4 (disarmed estimator) is the fallback controller: when Arc goes stale the motor publisher routes PX4 outputs, arms it in the air and switches it to Hold')
    ap.add_argument('--handover-ms', type=float, default=200.0)
    ap.add_argument('--takeover', default='warm', choices=['warm', 'cold'],
                    help='--standby px4: warm = PX4 takes over from the live state (gz_motor_pub.py: hover thrust '
                         'from the last commands, Takeoff entered from Offboard); cold = arm + Takeoff (before 2026-10-21)')
    ap.add_argument('--loss-at', type=float, default=30.0, help='bridge_loss: seconds into the run')
    ap.add_argument('--loss-kind', default='kill', choices=['kill', 'hang'],
                    help='bridge_loss: kill = process killed (crash), hang = process suspended (alive, silent)')
    ap.add_argument('--bridge', default=default_bridge(),
                    help='bridge binary (default: the prebuilt one under bin/ for this host)')
    ap.add_argument('--arc-side', default='host', choices=['host', 'container'],
                    help='where the Arc side (record streamer + bridge) runs: host = on this machine, records over '
                         'UDP from the container and commands back through a docker exec pipe; container = next to '
                         'PX4 as on a companion computer: its own PX4 MAVLink link, the Linux bridge and the motor '
                         'publisher joined by local pipes inside the container')
    ap.add_argument('--bridge-linux', default=os.path.join(HERE, '..', '..', 'bin', 'linux-x86_64', 'px4_bridge'),
                    help='--arc-side container: the static Linux x86-64 bridge copied into the container')
    ap.add_argument('--gap-log-ms', type=float, default=20.0, help='log every command gap over this (ms)')
    ap.add_argument('--rig', default='mavlink', choices=['mavlink', 'direct'],
                    help='sensor and command path: mavlink = PX4 MAVLink records + 250 Hz publisher grid (default); '
                         'direct = Gazebo IMU records + immediate publish (needs --arc-side container)')
    ap.add_argument('--record-clock', default='next', choices=['next', 'own'],
                    help="the bridge's record clock: next = a record's tick runs once a later record arrives (default); "
                         "own = the tick at a record's own stamp runs on its arrival (stamps strictly rise; G12)")
    ap.add_argument('--physics-hz', type=int, default=250,
                    help='gz physics step and x500 IMU rate (stock 250; e.g. 1000 = 1 ms step, IMU at 1 kHz; G12)')
    ap.add_argument('--sensor-delay-ms', type=float, default=0.0,
                    help='direct rig only: extra sensor delay, records held this long in sim time (G10 delay sweep)')
    ap.add_argument('--witness', action='store_true',
                    help='log Gazebo IMU, motor command and motor-failure topics on the sim clock (<tag>.witness.txt)')
    ap.add_argument('--watchdog-clock', default='sim', choices=['sim', 'wall'],
                    help='--standby px4: time base of the --handover-ms watchdog (sim = Gazebo sim time, the plant clock)')
    args = ap.parse_args()
    if args.sensor_delay_ms > 0 and args.rig != 'direct':
        raise SystemExit('--sensor-delay-ms needs --rig direct')
    direct = args.rig == 'direct'
    if direct and (args.arc_side != 'container' or args.arm == 'px4'):
        ap.error('--rig direct needs an Arc arm with --arc-side container')
    if direct and args.scenario in ('dropout', 'random'):
        ap.error('--rig direct: record dropouts are not modelled (use the mavlink rig for dropout and random)')

    if args.arm == 'reflex' and args.brain is None:
        # Flight default (owner, 2026-10-18): candidate v5, compiled, plasticity off (tools/px4/flight_default.py).
        args.brain = flight_default.BRAIN
        if args.bits is None:
            args.bits = flight_default.BITS
        if args.plasticity is None:
            args.plasticity = flight_default.PLASTICITY
    elif args.brain == 'untrained':
        args.brain = None
    if args.plasticity is None:
        args.plasticity = 'on'
    random_events = []
    if args.scenario == 'random':
        import random as _r
        rr = _r.Random(args.seed)
        for _ in range(rr.randint(2, 4)):
            kind = rr.choice(['wind', 'wind', 'dropout'])
            at = rr.uniform(12.0, 50.0)
            if kind == 'wind':
                ang = rr.uniform(0, 6.2832); n = rr.uniform(1.0, 4.0)
                random_events.append({'kind': 'wind', 'at': round(at, 2), 'len': round(rr.uniform(0.5, 1.5), 2),
                                      'fx': round(n * math.cos(ang), 2), 'fy': round(n * math.sin(ang), 2)})
            else:
                random_events.append({'kind': 'dropout', 'at': round(at, 2), 'len': round(rr.uniform(0.1, 0.4), 2)})
        random_events.sort(key=lambda e: e['at'])
        print('SCENARIO: random schedule seed %d: %s' % (args.seed, json.dumps(random_events)), flush=True)
    tag = args.tag or ('%s_%s_%s' % (args.scenario, args.arm, time.strftime('%H%M%S')))
    os.makedirs(args.out, exist_ok=True)
    base = os.path.join(args.out, tag)
    if not args.no_restart:
        restart_container(args.container, motor_scaled_sdf(args.motor_scale, args.out) if args.motor_scale != 1.0 else None,
                          physics_sdf(args.physics_hz, args.out))
        print('SCENARIO: container restarted', flush=True)
    mav = Mav()
    if not mav.wait_global():
        raise SystemExit('EKF never reported a global position')
    t_gp = time.time()
    mav.set_params([('COM_RC_IN_MODE', 4), ('NAV_RCL_ACT', 0), ('NAV_DLL_ACT', 0), ('COM_ARM_WO_GPS', 1)])
    if args.arm == 'px4':
        if not set_float_param(mav.gcs, 'MIS_TAKEOFF_ALT', args.alt):
            raise SystemExit('MIS_TAKEOFF_ALT not confirmed')
    if args.standby == 'px4':
        # PX4 fallback (G6): no spool-up delay when armed in the air, and a takeoff target
        # above the hover height so Takeoff climbs and then holds
        for name, v in (('COM_SPOOLUP_TIME', 0.0), ('MPC_TKO_RAMP_T', 0.0), ('MIS_TAKEOFF_ALT', args.alt + 1.0)):
            if not set_float_param(mav.gcs, name, v):
                raise SystemExit('param %s not confirmed' % name)
    funcs = [101, 102, 103, 104] if args.arm == 'px4' else [0, 0, 0, 0]
    if args.standby == 'px4' and args.arm != 'px4':
        # Standby keep-alive (SITL profile): Constant_Min on the unused ESC channel 5. With every channel
        # disabled PX4's output mixer runs at its 300 ms low rate and applies new routing only on that tick
        # (0-300 ms of the takeover); with one channel set it runs at 250 Hz. Channel 1 stays unset, so
        # gz_bridge publishes nothing (docs/bridge_loss_method.md).
        funcs = funcs + [1]
    mav.set_params([('SIM_GZ_EC_FUNC%d' % i, v) for i, v in enumerate(funcs, start=1)])
    print('SCENARIO: mode %s' % ('px4-owns' if args.arm == 'px4' else 'arc-owns'), flush=True)

    while time.time() - t_gp < args.settle:
        mav.beat()
        mav.gcs.recv_match(blocking=True, timeout=0.5)
    print('SCENARIO: EKF settle %.0fs done' % args.settle, flush=True)

    if args.arm == 'px4':
        nan = float('nan')
        r1 = mav.cmd(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, nan, nan, nan, nan, tries=20, want_accept=True)
        r2 = mav.cmd(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1, 0, tries=30, want_accept=True)
        print('SCENARIO: px4 takeoff ack=%s arm ack=%s' % (r1, r2), flush=True)
    # Hand the MAVLink port to the streamer process: the record path is then
    # streamer -> bridge -> container publisher over OS pipes only.
    mav.gcs.close()
    in_ct = args.arc_side == 'container' and args.arm != 'px4'
    if args.arm != 'px4':
        sh(['docker', 'exec', args.container, 'mkdir', '-p', CT_DIR])
        for f in ['gz_motor_pub.py', 'stall_probe.py'] + (['ekf_stream.py'] if in_ct else []) +                 (['gz_imu_stream.py'] if direct else []) + (['gz_witness.py'] if args.witness else []):
            subprocess.run(['docker', 'cp', os.path.join(HERE, f), '%s:%s/%s' % (args.container, CT_DIR, f)], check=True)
        if in_ct or args.standby == 'px4':
            stage_pymavlink(args.container)
        if in_ct:
            if not os.path.exists(args.bridge_linux):
                raise SystemExit('no Linux bridge at %s (--bridge-linux)' % args.bridge_linux)
            subprocess.run(['docker', 'cp', args.bridge_linux, '%s:%s/px4_bridge' % (args.container, CT_DIR)], check=True)
            if args.brain:
                subprocess.run(['docker', 'cp', args.brain, '%s:%s/brain.arc002' % (args.container, CT_DIR)], check=True)
            if args.rotor_loss:
                subprocess.run(['docker', 'cp', args.rotor_loss, '%s:%s/rotor_loss.arc002' % (args.container, CT_DIR)], check=True)
            start_mavlink_link(args.container, ARC_LINK, streams=GCS_STREAMS)
        if args.standby == 'px4':
            start_mavlink_link(args.container, WD_LINK, mode='minimal', rate=100000)
    inj = None
    if args.scenario in ('wind', 'motor_loss', 'random'):
        subprocess.run(['docker', 'cp', os.path.join(HERE, 'gz_inject.py'), '%s:/tmp/gz_inject.py' % args.container], check=True)
        inj = subprocess.Popen(['docker', 'exec', '-i', args.container, 'python3', '/tmp/gz_inject.py'],
                               stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
        ready = inj.stdout.readline().strip()
        print('SCENARIO: injector %s' % ready, flush=True)

    def inject(cmd):
        inj.stdin.write(cmd + '\n')
        inj.stdin.flush()
        ack = inj.stdout.readline().strip()
        print('SCENARIO: inject %s -> %s' % (cmd, ack), flush=True)
    stream_secs = args.seconds + 2.0
    # Streamer and bridge options; paths depend on where the Arc side runs.
    rec_path = CT_DIR + '/rec.bin' if in_ct else base + '.bin'
    sargs = ['--seconds', str(stream_secs), '--tee', rec_path]
    # v3 records also when the rotor-loss mode is on: it propagates the attitude from its own timestamp
    # (a v1 record's attitude is the newest ATTITUDE message, up to a record period older than the record)
    v2 = args.att_fallback != 'none' or bool(args.rotor_loss) or direct
    if v2:
        sargs += ['--record', 'v3']
    if args.scenario == 'bridge_loss':
        # kill: direct writes, keep recording after the pipe breaks; hang: a bounded queue so a
        # suspended consumer cannot block the recorder (one extra thread on the record path)
        sargs += ['--keep-recording'] if args.loss_kind == 'kill' else ['--survive-sink-loss']
    if args.scenario == 'dropout' and args.arm != 'px4':
        sargs += ['--stall-at', str(args.stall_at), '--stall-len', str(args.stall_len), '--stall-kind', args.stall_kind]
    if args.scenario == 'random' and args.arm != 'px4':
        # the streamer launches ~settle+1 s before the scenario clock starts; the offsets are
        # relative to its own start, so shift by the measured gap below
        drops = [e for e in random_events if e['kind'] == 'dropout']
        if drops:
            sargs += ['--stalls', ','.join('%.2f:%.2f' % (e['at'] + 1.0, e['len']) for e in drops), '--stall-kind', args.stall_kind]
    streamer = bridge = pub = None
    pub_lines = []   # (host wall time, line) as the motor publisher prints them

    def read_pub():
        for raw in pub.stdout:
            pub_lines.append((time.time(), raw.decode(errors='replace').rstrip()))
    if args.arm != 'px4':
        # direct rig: every tick to the publisher (a record's ticks run in one burst; the last is the newest)
        bargs = ['--hover', str(args.hover), '--alt', str(args.alt), '--every', '1' if direct else '4',
                 '--control', args.arm, '--log', CT_DIR + '/bridge.csv' if in_ct else base + '.csv',
                 '--residual-gain', str(args.residual_gain)]
        if args.plasticity != 'on':
            bargs += ['--plasticity', args.plasticity]
        if args.brain:
            bargs += ['--brain', CT_DIR + '/brain.arc002' if in_ct else args.brain]
        if args.rotor_loss:
            bargs += ['--rotor-loss', CT_DIR + '/rotor_loss.arc002' if in_ct else args.rotor_loss]
            if args.rotor_loss_detect_only:
                bargs += ['--rotor-loss-detect-only', '1']
            if not direct:
                # the MAVLink rig paces actuation by the records (a record's ticks run in one burst and the
                # publisher sends the newest command): the detector models it (docs/rotor_loss.md, under
                # sensor delay); on the direct rig every tick's command is published and acts
                bargs += ['--actuation-per-record', '1']
        if args.bits:
            bargs += ['--bits', str(args.bits)]
        if args.gains:
            bargs += ['--gains', args.gains]
        if args.fault:
            bargs += ['--fault', args.fault]
        if v2:
            bargs += ['--record', 'v3', '--att-fallback', args.att_fallback]
        if args.record_clock != 'next':
            bargs += ['--record-clock', args.record_clock]
        pargs = ['python3', '-u', CT_DIR + '/gz_motor_pub.py', '--stdin',
                 '--gap-log', CT_DIR + '/gaps.txt', '--gap-log-ms', str(args.gap_log_ms)]
        if direct:
            pargs += ['--immediate']
        if args.standby == 'px4':
            pargs += ['--handover-ms', str(args.handover_ms), '--mav', 'udp:127.0.0.1:%d' % WD_LINK[1],
                      '--watchdog-clock', args.watchdog_clock, '--takeover', args.takeover]
        pyenv = 'PYTHONPATH=' + CT_PYLIB
        if in_ct:
            # One pipeline inside the container: PX4 link -> streamer -> bridge -> motor publisher.
            q = lambda xs: ' '.join(shlex.quote(x) for x in xs)
            # stall_probe.py runs beside it as a witness for machine-wide stalls; the streamer logs record arrival gaps.
            witness = ('(python3 gz_witness.py --seconds %.0f --out witness.txt &) && ' % stream_secs) if args.witness else ''
            streamer_py = 'gz_imu_stream.py' if direct else 'ekf_stream.py'
            if direct:
                sargs += ['--lat-log', 'rec_lat.txt']
                if args.sensor_delay_ms > 0:
                    sargs += ['--delay-ms', str(args.sensor_delay_ms)]
            pipe = ('cd %s && (python3 stall_probe.py --seconds %.0f --out stalls.txt --over-ms %s &) && %s'
                    '%s python3 -u %s --port %d %s --gap-log stream_gaps.txt --gap-log-ms %s 2>stream.err | '
                    './px4_bridge %s 2>bridge.err | %s %s'
                    % (CT_DIR, stream_secs, args.gap_log_ms, witness, pyenv, streamer_py, ARC_LINK[1], q(sargs),
                       args.gap_log_ms, q(bargs), pyenv, q(pargs)))
            pub = subprocess.Popen(['docker', 'exec', args.container, 'sh', '-c', pipe],
                                   stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        else:
            # the same witnesses as the container layout: stall probe in the container, arrival gaps here
            sh(['docker', 'exec', '-d', args.container, 'python3', CT_DIR + '/stall_probe.py', '--seconds',
                '%.0f' % stream_secs, '--out', CT_DIR + '/stalls.txt', '--over-ms', str(args.gap_log_ms)])
            if args.witness:
                sh(['docker', 'exec', '-d', '-w', CT_DIR, args.container, 'python3', 'gz_witness.py', '--seconds',
                    '%.0f' % stream_secs, '--out', 'witness.txt'])
            streamer = subprocess.Popen([sys.executable, os.path.join(HERE, 'ekf_stream.py')] + sargs +
                                        ['--gap-log', base + '.stream_gaps.txt', '--gap-log-ms', str(args.gap_log_ms)],
                                        stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            bridge = subprocess.Popen([args.bridge] + bargs, stdin=streamer.stdout, stdout=subprocess.PIPE,
                                      stderr=subprocess.PIPE)
            pub = subprocess.Popen(['docker', 'exec', '-i', args.container, 'env', pyenv] + pargs,
                                   stdin=bridge.stdout, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            streamer.stdout.close()
            bridge.stdout.close()
        threading.Thread(target=read_pub, daemon=True).start()
    else:
        streamer = subprocess.Popen([sys.executable, os.path.join(HERE, 'ekf_stream.py')] + sargs,
                                    stdout=subprocess.PIPE, stderr=subprocess.PIPE)

        # px4 flies itself; the streamer only records (drain its stdout).
        def drain():
            while True:
                b = streamer.stdout.read(REC.size * 50)
                if not b:
                    break
        threading.Thread(target=drain, daemon=True).start()

    hover_speed = None

    t0 = time.time()
    wind_windows = []
    next_wind = args.wind_from if args.scenario == 'wind' else None
    wind_off = None
    failed = False
    px4_sampled = False
    t_dist = {'hover': None, 'wind': args.wind_from, 'motor_loss': args.fail_at, 'dropout': args.stall_at,
              'random': (random_events[0]['at'] if random_events else None), 'bridge_loss': args.loss_at}[args.scenario]
    lost = False
    while time.time() - t0 < args.seconds:
        now = time.time() - t0
        if args.arm == 'px4' and not px4_sampled and now > 12.0:
            r = sh(['docker', 'exec', args.container, 'sh', '-c',
                    'timeout 2 gz topic -e -t /x500_0/command/motor_speed 2>/dev/null | grep "^velocity" | awk \'{s+=$2;n++} END {if(n) print s/n}\''])
            try:
                hover_speed = float(r.stdout.strip().splitlines()[-1])
            except (ValueError, IndexError):
                hover_speed = None
            px4_sampled = True
        if args.scenario == 'random':
            for e in random_events:
                if e['kind'] == 'wind' and not e.get('fired') and now >= e['at']:
                    inject('wind %f %f 0' % (e['fx'], e['fy']))
                    e['fired'] = True; e['off_at'] = now + e['len']
                    wind_windows.append([now, None])
                if e['kind'] == 'wind' and e.get('fired') and not e.get('cleared') and now >= e['off_at']:
                    inject('clear'); e['cleared'] = True
                    for w in wind_windows:
                        if w[1] is None: w[1] = now
        if next_wind is not None and now >= next_wind:
            inject('wind %f %f 0' % (args.wind_n, args.wind_n * 0.5))
            wind_off = now + args.wind_len
            wind_windows.append([now, None])
            next_wind += args.wind_every
            print('SCENARIO: wind pulse at %.1fs' % now, flush=True)
        if wind_off is not None and now >= wind_off:
            inject('clear')
            wind_windows[-1][1] = now
            wind_off = None
        if args.scenario == 'bridge_loss' and pub is not None and not lost and now >= args.loss_at:
            if in_ct:
                sh(['docker', 'exec', args.container, 'pkill', '-KILL' if args.loss_kind == 'kill' else '-STOP',
                    '-x', 'px4_bridge'])
            elif args.loss_kind == 'kill':
                bridge.kill()
            else:
                import psutil
                psutil.Process(bridge.pid).suspend()
            lost = True
            print('SCENARIO: bridge %s at %.1fs' % ('killed' if args.loss_kind == 'kill' else 'suspended', now), flush=True)
        if args.scenario == 'motor_loss' and not failed and now >= args.fail_at:
            inject('fail %d' % args.fail_motor)
            failed = True
            print('SCENARIO: motor %d failed at %.1fs' % (args.fail_motor, now), flush=True)
        time.sleep(0.05)
    if wind_off is not None:
        inject('clear')
        wind_windows[-1][1] = time.time() - t0
    if inj is not None:
        try:
            inj.stdin.write('quit\n')
            inj.stdin.flush()
        except (BrokenPipeError, OSError, ValueError):
            pass
        inj.wait(timeout=10)

    stats = {}
    bridge_err = ''
    serr = ''
    if streamer is not None:
        try:
            streamer.wait(timeout=stream_secs + 10)
        except subprocess.TimeoutExpired:
            streamer.kill()
        serr = streamer.stderr.read().decode(errors='replace').strip()
    elif in_ct:
        if lost and args.loss_kind == 'hang':
            sh(['docker', 'exec', args.container, 'pkill', '-KILL', '-x', 'px4_bridge'])
        try:
            pub.wait(timeout=stream_secs + 30)
        except subprocess.TimeoutExpired:
            sh(['docker', 'exec', args.container, 'pkill', '-KILL', '-f', CT_DIR])
            pub.kill()
        serr = sh(['docker', 'exec', args.container, 'cat', CT_DIR + '/stream.err']).stdout.strip()
        bridge_err = sh(['docker', 'exec', args.container, 'cat', CT_DIR + '/bridge.err']).stdout
        for src, dst in (('rec.bin', base + '.bin'), ('bridge.csv', base + '.csv'),
                         ('stream_gaps.txt', base + '.stream_gaps.txt'), ('stalls.txt', base + '.stalls.txt')) +                         ((('rec_lat.txt', base + '.lat.txt'),) if direct else ()):
            sh(['docker', 'cp', '%s:%s/%s' % (args.container, CT_DIR, src), dst])
    if serr:
        print('SCENARIO: streamer: %s' % serr.splitlines()[-1], flush=True)
    if not os.path.exists(base + '.bin'):
        print('SCENARIO: no record file (streamer failed): %s' % serr, flush=True)
        return 3
    handover = None
    handover_t = None
    gaps = None
    if pub is not None:
        if bridge is not None:
            if lost and args.loss_kind == 'hang':
                bridge.kill()
            bridge.wait(timeout=30)
            bridge_err = bridge.stderr.read().decode(errors='replace')
        try:
            pub.wait(timeout=15)
        except subprocess.TimeoutExpired:
            pub.kill()
        time.sleep(0.2)   # let the reader thread take the last lines
        sh(['docker', 'cp', '%s:%s/gaps.txt' % (args.container, CT_DIR), base + '.gaps.txt'])
        if args.witness:
            sh(['docker', 'cp', '%s:%s/witness.txt' % (args.container, CT_DIR), base + '.witness.txt'])
        if not in_ct:
            sh(['docker', 'cp', '%s:%s/stalls.txt' % (args.container, CT_DIR), base + '.stalls.txt'])
        if args.standby == 'px4':
            # PX4's own flight log (ulog, tools/px4/takeover_trace.py): what its controllers did in the takeover
            r = sh(['docker', 'exec', args.container, 'sh', '-c',
                    'ls -t $(find / -name "*.ulg" 2>/dev/null) 2>/dev/null | head -1'])
            ulg = r.stdout.strip()
            if ulg:
                sh(['docker', 'cp', '%s:%s' % (args.container, ulg), base + '.ulg'])
        with open(base + '.pub.txt', 'w') as pf:   # the motor publisher's whole output (host wall time, line)
            pf.writelines('%.3f %s\n' % tl for tl in pub_lines)
        for t_line, line in pub_lines:
            if 'HANDOVER' in line:
                handover = line.strip()
                handover_t = round(t_line - t0, 2)
                print('SCENARIO: %s (host t=%.2fs)' % (handover, handover_t), flush=True)
            elif 'input gap' in line:
                print('SCENARIO: %s' % line.strip(), flush=True)
            elif 'command gaps:' in line:
                print('SCENARIO: %s' % line.strip(), flush=True)
                gm = re.search(r'(\d+) lines, (\d+) over (\d+) ms, worst ([\d.]+) ms \(sim time in the logged gaps: '
                               r'worst (-?[\d.]+) ms', line)
                if gm:
                    gaps = {'lines': int(gm.group(1)), 'over_ms': float(gm.group(3)), 'n_over': int(gm.group(2)),
                            'worst_ms': float(gm.group(4)), 'worst_sim_ms_in_gaps': float(gm.group(5))}
            elif 'no PX4 heartbeat' in line or 'Traceback' in line or 'Error' in line:
                print('SCENARIO: publisher: %s' % line.strip(), flush=True)
        if gaps is not None:
            # in flight = from 3 s (spin-up and lift-off: takeoff_s is 1-2 s) to the end of the window
            gaps.update(attribute_gaps(base, t0 + args.seconds, t0 + FLIGHT_FROM_S))
            print('SCENARIO: gap attribution %s' % json.dumps(gaps), flush=True)
        for line in bridge_err.splitlines():
            if 'PX4_BRIDGE_STATS' in line:
                for kv in line.split()[1:]:
                    k, v = kv.split('=', 1)
                    stats[k] = v
    else:
        mav = Mav()
        mav.cmd(mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, float('nan'), float('nan'), float('nan'), 0)
        mav.gcs.close()

    # Record timestamps are sim time; convert wall-based wind windows to sim offsets via the first record.
    m = metrics_from_records(base + '.bin', args.alt, t_dist, [(a, b) for a, b in wind_windows if b is not None],
                             REC3 if v2 else REC)
    result = {'tag': tag, 'arm': args.arm, 'scenario': args.scenario, 'seconds': args.seconds, 'alt': args.alt,
              'hover_rad_s': args.hover, 'px4_hover_mean_rad_s': hover_speed, 'gains': args.gains,
              'residual_gain': args.residual_gain, 'plasticity': args.plasticity, 'fault_policy': args.fault,
              'brain': args.brain, 'bits': args.bits, 'att_fallback': args.att_fallback,
              'streamer': serr.splitlines()[-1] if serr else '',
              'wind': {'n': args.wind_n, 'every': args.wind_every, 'len': args.wind_len, 'from': args.wind_from,
                       'windows': wind_windows} if args.scenario == 'wind' else None,
              'motor_loss': {'motor': args.fail_motor, 'at': args.fail_at} if args.scenario == 'motor_loss' else None,
              'rotor_loss': {'brain': args.rotor_loss, 'detect_only': args.rotor_loss_detect_only} if args.rotor_loss else None,
              'motor_scale': args.motor_scale, 'record': 'v3' if v2 else 'v1',
              'dropout': {'at': args.stall_at, 'len': args.stall_len, 'kind': args.stall_kind} if args.scenario == 'dropout' else None,
              'random_stall_kind': args.stall_kind if args.scenario == 'random' else None,
              'random_schedule': random_events if args.scenario == 'random' else None, 'seed': args.seed,
              'standby': args.standby, 'handover_ms': args.handover_ms if args.standby == 'px4' else None,
              'watchdog_clock': args.watchdog_clock if args.standby == 'px4' else None,
              'takeover': args.takeover if args.standby == 'px4' else None,
              'bridge_loss': {'at': args.loss_at, 'kind': args.loss_kind} if args.scenario == 'bridge_loss' else None,
              'handover': handover, 'handover_host_t_s': handover_t, 'arc_side': args.arc_side if args.arm != 'px4' else None,
              # a takeover before the loss (or, without a loss, inside the flight window) is a false takeover
              'false_takeover': (handover_t is not None and handover_t < (args.loss_at if args.scenario == 'bridge_loss'
                                                                          else args.seconds)) if args.standby == 'px4' else None,
              'command_gaps': gaps, 'rig': args.rig, 'record_clock': args.record_clock, 'physics_hz': args.physics_hz, 'sensor_delay_ms': args.sensor_delay_ms, 'actuation_per_record': bool(args.rotor_loss) and not direct,
              'bridge': stats, 'metrics': m}
    with open(base + '.json', 'w') as f:
        json.dump(result, f, indent=1)
    s = m.get('settled') or {}
    d = m.get('disturbed') or {}
    row = '| %s | %s | %s | %s | %s | %s | %s | %s | %s | %s | %s | %s | %s |' % (
        args.scenario, args.arm, m.get('takeoff_s'), s.get('alt_mean'), s.get('alt_jitter_rms'), s.get('xy_max'), s.get('tilt_max_deg'),
        d.get('alt_max_err'), d.get('xy_max'), d.get('tilt_max_deg'),
        m.get('survival_s'), m.get('tilt_fault_s') or m.get('ground_impact_s') or '-', stats.get('fault_ticks', '-'))
    with open(os.path.join(args.out, 'results.md'), 'a') as f:
        f.write(row + '\n')
    print('SCENARIO RESULT ' + json.dumps(result['metrics']), flush=True)
    print(row, flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
