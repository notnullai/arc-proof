"""Sensor-to-command latency and the true catch, from a run's Gazebo witness (gz_witness.py,
<tag>.witness.txt). Same measurement for both rigs (run_scenario.py --rig mavlink|direct).

  python tools/px4/rig_latency.py <run.json> [...]      Markdown table to stdout

Per rotor-failure run, on Gazebo's sim clock and from Gazebo's own topics only:
  onset      the first IMU sample after the failure command whose body angular acceleration exceeds
             20 rad/s^2 (the summarizer's threshold, here on the true 250 Hz gyro)
  cut        the first motor command after the onset with the rotor opposite the failed one at or below
             half its last-second mean (the opposite rotor's thrust going: the pre-emptive cut or the mode)
  latency    cut - onset; the command acts from the next physics step (+4 ms)
  first move the first motor command after the onset that differs from the pre-onset command pattern by
             more than 5 % on any rotor (the first reaction of any kind)
  true tilt  largest tilt in the first 1.5 s after the onset, from Gazebo's model pose (50 Hz) carried
             between pose samples with the true gyro (the 'catch' of docs/rotor_loss.md 4, from the truth
             instead of the estimator's attitude)
  peak rate  largest |(p, q)| of the true gyro in the first 0.3 s (the tip-over, before the spin; rad/s)
The failed rotor is the gz MotorFailure index - 1; opposite = the diagonal rotor of the x500 (0-1, 2-3).
"""
import json
import math
import os
import sys

OPP = {0: 1, 1: 0, 2: 3, 3: 2}


def qmul(a, b):
    return (a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
            a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
            a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
            a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0])


def tilt_deg(q):
    # angle between body z and world z: cos = 1 - 2(qx^2 + qy^2)
    c = 1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2])
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def load(path):
    I, M, P, F = [], [], [], []
    opener = open
    if path.endswith('.gz'):
        import gzip
        opener = lambda p: gzip.open(p, 'rt')
    with opener(path) as f:
        for line in f:
            p = line.split()
            if not p:
                continue
            try:
                if p[0] == 'I' and len(p) == 5:
                    I.append((float(p[1]), float(p[2]), float(p[3]), float(p[4])))
                elif p[0] == 'M' and len(p) == 6:
                    M.append((float(p[1]), float(p[2]), float(p[3]), float(p[4]), float(p[5])))
                elif p[0] == 'P' and len(p) == 9:
                    P.append(tuple(float(x) for x in p[1:]))
                elif p[0] == 'F' and len(p) == 3:
                    F.append((float(p[1]), int(p[2])))
            except ValueError:
                pass   # a line cut by the end of the run
    return I, M, P, F


def analyse(json_path):
    r = json.load(open(json_path))
    base = json_path[:-5]
    wp = base + '.witness.txt'
    if not os.path.exists(wp):
        wp += '.gz'
    if not os.path.exists(wp):
        return None
    I, M, P, F = load(wp)
    ml = r.get('motor_loss') or {}
    res = {'tag': r.get('tag'), 'rig': r.get('rig', 'mavlink'), 'alt': r.get('alt')}
    if not F or not ml or int(ml.get('motor', 0)) < 1:
        return res
    failed = int(ml['motor']) - 1
    opp = OPP[failed]
    t_cmd = F[0][0]
    onset = None
    for k in range(1, len(I)):
        if I[k][0] < t_cmd - 0.2:
            continue
        dt = I[k][0] - I[k - 1][0]
        if dt <= 0:
            continue
        a = math.hypot(I[k][1] - I[k - 1][1], I[k][2] - I[k - 1][2]) / dt
        if a > 20.0:
            onset = I[k][0]
            break
    if onset is None:
        return res
    res['onset_s'] = round(onset, 3)
    pre = [m for m in M if onset - 1.0 <= m[0] < onset - 0.02]
    if pre:
        mean_opp = sum(m[1 + opp] for m in pre) / len(pre)
        cut = next((m[0] for m in M if m[0] >= onset and m[1 + opp] <= 0.5 * mean_opp), None)
        res['cut_ms'] = None if cut is None else round((cut - onset) * 1000.0, 1)
        mean = [sum(m[1 + i] for m in pre) / len(pre) for i in range(4)]
        lo = [min(m[1 + i] for m in pre) for i in range(4)]
        hi = [max(m[1 + i] for m in pre) for i in range(4)]
        mv = next((m[0] for m in M if m[0] >= onset and
                   any(m[1 + i] < lo[i] - 0.05 * mean[i] or m[1 + i] > hi[i] + 0.05 * mean[i] for i in range(4))), None)
        res['move_ms'] = None if mv is None else round((mv - onset) * 1000.0, 1)
    # true attitude: pose samples, carried between them with the true gyro (body FLU, quaternion w x y z)
    win = [i for i in I if onset - 0.05 <= i[0] <= onset + 1.5]
    poses = [p for p in P if onset - 0.1 <= p[0] <= onset + 1.5]
    if poses and win:
        tilts = []
        pk = 0
        q = None
        tq = None
        for s in win:
            while pk < len(poses) and poses[pk][0] <= s[0]:
                q = (poses[pk][4], poses[pk][5], poses[pk][6], poses[pk][7])
                tq = poses[pk][0]
                pk += 1
            if q is None:
                continue
            dt = s[0] - tq
            if dt > 0:
                wx, wy, wz = s[1], s[2], s[3]
                h = 0.5 * dt
                q = qmul(q, (1.0, wx * h, wy * h, wz * h))
                n = math.sqrt(sum(c * c for c in q))
                q = tuple(c / n for c in q)
                tq = s[0]
            if s[0] >= onset:
                tilts.append(tilt_deg(q))
        if tilts:
            res['true_tilt'] = round(max(tilts), 1)
    rates = [math.hypot(s[1], s[2]) for s in I if onset <= s[0] <= onset + 0.3]
    if rates:
        res['peak_rate'] = round(max(rates), 2)
    return res


def main(argv):
    print('| run | rig | alt m | onset s (sim) | first command change ms | opposite rotor cut ms | true catch tilt deg (1.5 s) | peak roll/pitch rate rad/s (0.3 s) |')
    print('|---|---|---|---|---|---|---|---|')
    for p in argv:
        a = analyse(p)
        if a is None:
            continue
        print('| %s | %s | %s | %s | %s | %s | %s | %s |' % (
            a['tag'], a['rig'], a.get('alt'), a.get('onset_s', '-'), a.get('move_ms', '-'), a.get('cut_ms', '-'),
            a.get('true_tilt', '-'), a.get('peak_rate', '-')))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
