"""Aggregate run_scenario.py JSON results into a Markdown table.

  python tools/px4/summarize_runs.py runs/m3 [--out docs/px4_sitl_results.md]

Groups by (scenario, arm, fault_policy); reports each metric as
"value" for one run or "min..max" across repeats. Columns:
  takeoff s | hover alt mean / jitter rms | station max (settled) | tilt max (settled)
  | disturbed: alt max err / station max / tilt max | wind recovery max s (unrecovered)
  | survival s | first fault/impact s | bridge fault ticks | dropout ticks | max wall stall ms
"""
import argparse
import glob
import json
import os
import sys


def fmt(vals, nd=2):
    vals = [v for v in vals if v is not None]
    if not vals:
        return '-'
    lo, hi = min(vals), max(vals)
    if isinstance(lo, str):
        return lo if lo == hi else '%s..%s' % (lo, hi)
    if lo == hi:
        return ('%.' + str(nd) + 'f') % lo
    return ('%.' + str(nd) + 'f..%.' + str(nd) + 'f') % (lo, hi)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('dir')
    ap.add_argument('--out', default=None)
    args = ap.parse_args()
    groups = {}
    for path in sorted(glob.glob(os.path.join(args.dir, '*.json'))):
        d = json.load(open(path))
        variant = d.get('fault_policy') or ('stabilised' if d['arm'] in ('pid', 'residual') else 'raw')
        if d.get('plasticity') == 'off':
            variant += ', plasticity off'
        if d['arm'] == 'residual' and d.get('residual_gain', 16384) != 16384:
            variant += ', gain %.2f' % (d['residual_gain'] / 65536.0)
        if d.get('gains'):
            variant += ', ' + d['gains']
        if d.get('brain'):
            variant += ', ' + os.path.basename(d['brain'])
            if d.get('bits'):
                variant += ' (%d bits)' % d['bits']   # 480 = frame v2 prior, 512 = frame v3 (Track G)
        if d.get('dropout'):
            variant += ', stall %.1f s %s' % (d['dropout']['len'], d['dropout'].get('kind', 'full'))
        if d.get('random_schedule') is not None:
            variant += ', seed %s' % d.get('seed')
            if d.get('random_stall_kind') and d['random_stall_kind'] != 'full':
                variant += ' (%s dropouts)' % d['random_stall_kind']
        if d.get('att_fallback') in ('gyro', 'imu'):
            variant += ', %s fallback' % d['att_fallback']
        key = (d['scenario'], d['arm'], variant)
        groups.setdefault(key, []).append(d)
    order = {'hover': 0, 'wind': 1, 'motor_loss': 2, 'dropout': 3, 'random': 4}
    arms = {'px4': 0, 'pid': 1, 'residual': 2, 'reflex': 3}
    lines = ['| scenario | arm | fault policy | n | takeoff s | alt mean m | alt jitter rms m | station max m | tilt max ° | dist. alt err max m | dist. station max m | dist. tilt max ° | wind recovery max s (unrec.) | survival s | first fault / impact s | fault ticks | dropout ticks | max wall stall ms |',
             '|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|']
    for key in sorted(groups, key=lambda k: (order[k[0]], arms[k[1]], k[2])):
        runs = groups[key]
        m = [r['metrics'] for r in runs]
        s = [x.get('settled') or {} for x in m]
        dd = [x.get('disturbed') or {} for x in m]
        b = [r.get('bridge') or {} for r in runs]
        rec = [x.get('wind_recovery_max_s') for x in m]
        unrec = [x.get('wind_unrecovered') for x in m]
        rec_s = fmt(rec)
        if any(u for u in unrec if u):
            rec_s += ' (%s)' % fmt([u or 0 for u in unrec], 0)
        first = [x.get('tilt_fault_s') or x.get('ground_impact_s') for x in m]
        lines.append('| %s | %s | %s | %d | %s | %s | %s | %s | %s | %s | %s | %s | %s | %s | %s | %s | %s | %s |' % (
            key[0], key[1], key[2] if key[1] != 'px4' else '-', len(runs),
            fmt([x.get('takeoff_s') for x in m]),
            fmt([x.get('alt_mean') for x in s], 3), fmt([x.get('alt_jitter_rms') for x in s], 3),
            fmt([x.get('xy_max') for x in s], 3), fmt([x.get('tilt_max_deg') for x in s], 1),
            fmt([x.get('alt_max_err') for x in dd], 3), fmt([x.get('xy_max') for x in dd], 3),
            fmt([x.get('tilt_max_deg') for x in dd], 1), rec_s,
            fmt([x.get('survival_s') for x in m], 1), fmt(first),
            fmt([int(x['fault_ticks']) if 'fault_ticks' in x else None for x in b], 0),
            fmt([int(x['dropout_ticks']) if 'dropout_ticks' in x else None for x in b], 0),
            fmt([int(x['max_wall_tick_us']) / 1000.0 if 'max_wall_tick_us' in x else None for x in b], 1)))
    text = '\n'.join(lines) + '\n'
    if args.out:
        with open(args.out, 'w', encoding='utf-8', newline='\n') as f:
            f.write(text)
    sys.stdout.buffer.write(text.encode('utf-8'))
    return 0


if __name__ == '__main__':
    sys.exit(main())
