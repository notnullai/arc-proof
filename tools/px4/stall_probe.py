"""Scheduling-stall witness for the G6 transport measurements: sleeps 2 ms in a
loop and logs every wake-up later than --over-ms as "wall_time late_ms". Run it
next to the Arc pipeline (own process, nothing shared): a command-stream gap
that this probe sees at the same moment was a stall of the whole machine (VM,
container or host), not of the pipe or of one process.

  python3 stall_probe.py --seconds 52 --out stalls.txt [--over-ms 20]
"""
import argparse
import time


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--over-ms', type=float, default=20.0)
    args = ap.parse_args()
    t_end = time.time() + args.seconds
    worst = 0.0
    n = 0
    with open(args.out, 'w') as f:
        last = time.time()
        while last < t_end:
            time.sleep(0.002)
            now = time.time()
            late = (now - last) * 1000.0 - 2.0
            worst = max(worst, late)
            if late > args.over_ms:
                n += 1
                f.write('%.3f %.1f\n' % (now, late))
                f.flush()
            last = now
        f.write('# worst %.1f ms, %d over %.0f ms\n' % (worst, n, args.over_ms))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
