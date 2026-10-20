"""G10 rotor-loss runs on Gazebo (docs/rotor_loss.md): one table row per run from the run's JSON and
its recorded estimator stream (<tag>.bin, the records the bridge flew on).

    python tools/px4/summarize_rotor_loss.py runs/g10/*.json

Columns: arm (flight default alone, or with the rotor-loss mode), motor scale, failed motor, altitude
at the failure, detection / confirmation latency (bridge stats, sim time), rotor confirmed, outcome
(held to the end, or touchdown), lowest altitude after the failure, touchdown vertical speed, max
tilt and mean spin after the failure, max horizontal drift from the failure point.
Standard library only."""
import json
import math
import os
import struct
import sys

REC = struct.Struct('<Q12f')
REC3 = struct.Struct('<Q12fQQ7f')


def summarize(path):
    with open(path) as f:
        r = json.load(f)
    base = path[:-5]
    rec = REC3 if r.get('att_fallback') not in (None, 'none') or r.get('record') == 'v3' else REC
    d = open(base + '.bin', 'rb').read()
    rows = [rec.unpack_from(d, i * rec.size) for i in range(len(d) // rec.size)]
    if not rows:
        return None
    t0 = rows[0][0]
    T = [(x[0] - t0) / 1e6 for x in rows]
    b = r.get('bridge') or {}
    ml = r.get('motor_loss') or {}
    # the failure onset, from the records: the first record whose body angular acceleration exceeds
    # 20 rad/s^2 (a lost rotor makes ~57 within a record or two; hover noise reached 10 and the largest
    # gust-only residual 16.5 in the hermetic gusts), searched after the
    # station was reached; the scheduled time is wall clock and only bounds the search from below
    def accel(i):
        dt = (rows[i][0] - rows[i - 1][0]) / 1e6
        return math.hypot(rows[i][4] - rows[i - 1][4], rows[i][5] - rows[i - 1][5]) / dt if dt > 0 else 0.0
    t_lo = max(0.0, float(ml.get('at', 0)) - 8.0)
    t_fail = next((T[i] for i in range(1, len(rows)) if T[i] >= t_lo and accel(i) > 20.0), None)
    if t_fail is None:
        return {'tag': r.get('tag'), 'none': True}
    i0 = next((i for i, t in enumerate(T) if t >= t_fail), len(rows) - 1)
    alt = lambda i: -rows[i][9]
    tilt = lambda i: math.degrees(math.acos(max(-1.0, min(1.0, math.cos(rows[i][1]) * math.cos(rows[i][2])))))
    after = range(i0, len(rows))
    td = next((i for i in after if alt(i) < 0.10 and i > i0 + 5), None)
    end = td if td is not None else len(rows) - 1
    seg = range(i0, end + 1)
    td_speed = None
    if td is not None:
        td_speed = max(rows[i][12] for i in range(max(i0, td - 10), td + 1))   # NED vz down positive
    spin = [abs(rows[i][6]) for i in seg]
    x0, y0 = rows[i0][7], rows[i0][8]
    det = lambda k: (int(b[k]) - t0) / 1e6 if b.get(k) and int(b[k]) > 0 else None
    return {
        'tag': r.get('tag'), 'arm': 'v5 + rotor-loss' if r.get('rotor_loss') else 'v5 alone',
        'motor_scale': r.get('motor_scale', 1.0), 'motor': ml.get('motor'),
        'alt_fail': round(alt(i0), 2),
        'fail_s': t_fail,
        'detect_s': det('rl_first_detect_us'), 'confirm_s': det('rl_confirm_us'),
        'rl_rotor': int(b['rl_rotor']) if b.get('rl_rotor') not in (None, '') else None,
        'rl_aborts': b.get('rl_aborts'),
        'outcome': 'held' if td is None else 'touchdown',
        'alt_min': round(min(alt(i) for i in seg), 2),
        'td_speed': None if td_speed is None else round(td_speed, 2),
        'tilt_max': round(max(tilt(i) for i in seg), 1),
        # the catch: the first 1.5 s after the onset, while PX4's attitude still follows its rates (spin < 13 rad/s)
        'tilt_catch': round(max(tilt(i) for i in seg if T[i] <= t_fail + 1.5), 1),
        'spin_mean': round(sum(spin) / len(spin), 1) if spin else None,
        'drift_max': round(max(math.hypot(rows[i][7] - x0, rows[i][8] - y0) for i in seg), 2),
        'flown_after_s': round(T[end] - T[i0], 1),
        'hold20': hold20(rows, T, end, x0, y0) if td is None else None,
    }


def hold20(rows, T, end, x0, y0):
    # the hold, last 20 s of a held run: estimator altitude range and horizontal distance from the failure point
    last = [i for i in range(end + 1) if T[i] >= T[end] - 20.0]
    alts = [-rows[i][9] for i in last]
    return '%.1f-%.1f m, xy %.1f m' % (min(alts), max(alts), max(math.hypot(rows[i][7] - x0, rows[i][8] - y0) for i in last))


def main(argv):
    print('| run | arm | motor x | failed motor (gz index + 1) | alt at failure m | onset s | detected after ms | confirmed after ms | rotor | outcome | '
          'lowest alt m | touchdown m/s | max tilt deg | catch tilt deg (1.5 s) | mean spin rad/s | max drift m | flown after s | last 20 s |')
    print('|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|')
    for p in argv:
        s = summarize(p)
        if not s:
            print('| %s | no records |' % os.path.basename(p))
            continue
        if s.get('none'):
            print('| %s | no rotor failure in the records (control flight) |' % s['tag'])
            continue
        ms = lambda x: '-' if x is None or s['fail_s'] is None else '%.0f' % ((x - s['fail_s']) * 1000)
        print('| %s | %s | %.2f | %s | %s | %s | %s | %s | %s | %s | %s | %s | %s | %s | %s | %s | %s | %s |' % (
            s['tag'], s['arm'], s['motor_scale'], s['motor'], s['alt_fail'],
            '-' if s['fail_s'] is None else '%.2f' % s['fail_s'], ms(s['detect_s']), ms(s['confirm_s']),
            '-' if s['rl_rotor'] in (None, -1) else s['rl_rotor'], s['outcome'], s['alt_min'],
            '-' if s['td_speed'] is None else s['td_speed'], s['tilt_max'], s['tilt_catch'], s['spin_mean'], s['drift_max'], s['flown_after_s'],
            s['hold20'] or '-'))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
