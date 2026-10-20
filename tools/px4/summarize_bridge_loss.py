"""G6 bridge-loss matrix summary: per run, survival, handover latency, worst
altitude loss and tilt in the takeover window, station after takeover; with
the G6 transport fields (runs from 2026-10-19): false takeover, worst command
gap in the flight window, machine-wide stalls among the gaps, worst gap outside
them (tools/px4/run_scenario.py attribute_gaps)."""
import glob
import json
import math
import os
import re
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gap_attribution import attribute_gaps, FLIGHT_FROM_S  # noqa: E402

R = struct.Struct('<Q12f')
R3 = struct.Struct('<Q12fQQ7f')
D = sys.argv[1]
rows = []
for jf in sorted(glob.glob(os.path.join(D, '*.json'))):
    tag = os.path.basename(jf)[:-5]
    j = json.load(open(jf))
    lp = os.path.join(D, tag + '.log')
    log = open(lp if os.path.exists(lp) else os.path.join(D, tag + '.out'), errors='replace').read()
    m = re.search(r'HANDOVER to PX4 \((\w+), (\d+) ms since last command, route\+arm\+takeoff (\d+) ms', log)
    bp = os.path.join(D, tag + '.bin')
    if os.path.exists(bp):
        b = open(bp, 'rb').read()
    else:   # evidence kept compressed (the direct rig's 250 Hz records)
        import gzip
        b = gzip.open(bp + '.gz', 'rb').read()
    rs = R3 if j.get('record') == 'v3' else R   # the first 13 fields are the same in both layouts
    recs = [rs.unpack_from(b, i)[:13] for i in range(0, len(b) - rs.size + 1, rs.size)]
    t0 = recs[0][0]
    loss = (j.get('bridge_loss') or {}).get('at')
    # The loss in sim time: the bridge's last logged tick (its per-tick log stops when it is killed or
    # suspended). The scheduled time is wall clock, and machine-wide stalls put sim time behind it (two
    # ~1 s stalls in the G11 runs moved these windows across the loss); wall time only without the log.
    cp = os.path.join(D, tag + '.csv')
    if loss and (os.path.exists(cp) or os.path.exists(cp + '.gz')):
        import gzip
        with (open(cp) if os.path.exists(cp) else gzip.open(cp + '.gz', 'rt')) as cf:
            last = None
            for last in cf:
                pass
        try:
            loss = (int(last.split(',')[0]) - t0) / 1e6
        except (AttributeError, ValueError):
            pass
    kind = (j.get('bridge_loss') or {}).get('kind', '-')
    pre = [r for r in recs if loss and loss - 3.0 <= (r[0] - t0) / 1e6 < loss - 0.5]
    win = [r for r in recs if loss and loss - 0.5 <= (r[0] - t0) / 1e6 <= loss + 4.0]
    post = [r for r in recs if loss and loss + 6.0 <= (r[0] - t0) / 1e6]
    def tilt(r):
        return math.degrees(math.acos(max(-1.0, min(1.0, math.cos(r[1]) * math.cos(r[2])))))
    alt_pre = sum(-r[9] for r in pre) / len(pre) if pre else float('nan')
    alt_min = min(-r[9] for r in win) if win else float('nan')
    tilt_max = max(tilt(r) for r in win) if win else float('nan')
    x0 = sum(r[7] for r in pre) / len(pre) if pre else 0.0
    y0 = sum(r[8] for r in pre) / len(pre) if pre else 0.0
    drift = max(math.hypot(r[7] - x0, r[8] - y0) for r in win + post) if (win or post) else float('nan')
    met = j['metrics']
    g = {}
    mt = re.search(r'HANDOVER to PX4 .* at ([0-9.]+) \(host t=([0-9.]+)s\)', log)
    if mt:   # recompute from the gap files: in flight = 3 s after the pipeline started to the window end
        t0w = float(mt.group(1)) - float(mt.group(2))
        g = attribute_gaps(os.path.join(D, tag), t0w + j['seconds'], t0w + FLIGHT_FROM_S)
    rows.append((tag, j['arm'], kind, j.get('standby'), met.get('survived_run'), met.get('survival_s'),
                 m.group(1) if m else '-', int(m.group(2)) if m else None, int(m.group(3)) if m else None,
                 alt_pre, alt_min, alt_pre - alt_min if pre and win else float('nan'), tilt_max, drift,
                 j.get('false_takeover', '-'), g.get('worst_ms_in_flight', '-'), g.get('machine_stalls_in_flight', '-'),
                 g.get('worst_ms_outside_machine_stalls_in_flight', '-'), g.get('worst_ms_added_by_arc_side_in_flight', '-'),
                 g.get('worst_ms_in_window', '-')))
print('| run | arm | loss | fallback | survived | survival s | trigger | stale ms | route+arm+takeoff ms | alt before m | alt min m | alt lost m | tilt max deg | drift max m | false takeover | worst gap in flight ms | machine stalls in flight | worst gap in flight outside stalls ms | worst gap in flight added by the Arc side ms | worst gap incl. spin-up ms |')
print('|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|')
for r in rows:
    print('| %s | %s | %s | %s | %s | %s | %s | %s | %s | %.2f | %.2f | %.2f | %.1f | %.2f | %s | %s | %s | %s | %s | %s |' % r)
