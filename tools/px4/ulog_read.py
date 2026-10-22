"""Minimal ULog reader (PX4 flight log, docs.px4.io "ULog File Format"): the primitive fields of the
logged topics (nested types flattened as parent.field), as {topic: {field: [values]}} with 'timestamp' in microseconds. Used to see what PX4's
controllers did in a G6 takeover (PX4 logs from arming, which in the fallback is the takeover).
Usage: python tools/px4/ulog_read.py LOG.ulg topic[.field,field] ...  (prints a table every 20 ms of log)."""
import struct
import sys

PRIM = {'int8_t': 'b', 'uint8_t': 'B', 'int16_t': 'h', 'uint16_t': 'H', 'int32_t': 'i', 'uint32_t': 'I',
        'int64_t': 'q', 'uint64_t': 'Q', 'float': 'f', 'double': 'd', 'bool': '?', 'char': 'c'}


def read(path, want=None):
    b = open(path, 'rb').read()
    if b[:7] != b'ULog\x01\x12\x35':
        raise SystemExit('%s: not a ULog file' % path)
    fmts, subs, out = {}, {}, {}
    layouts = {}

    def layout(name, prefix='', base=0):   # [(field, struct char, count, offset)], nested types as a.b
        key = (name, prefix, base)
        if key in layouts:
            return layouts[key]
        res, off = [], base
        for f in fmts[name]:
            t, fname = f.split(' ')
            n = 1
            if '[' in t:
                t, n = t[:-1].split('[')
                n = int(n)
            if t in PRIM:
                if not fname.startswith('_padding'):
                    res.append((prefix + fname, PRIM[t], n, off))
                off += struct.calcsize('<' + PRIM[t]) * n
            else:
                sz = layout_size(t)
                for k in range(n):
                    res += layout(t, '%s%s%s.' % (prefix, fname, '[%d]' % k if n > 1 else ''), off + k * sz)
                off += sz * n
        layouts[key] = res
        return res

    def layout_size(name):
        s = 0
        for f in fmts[name]:
            t, _ = f.split(' ')
            n = 1
            if '[' in t:
                t, n = t[:-1].split('[')
                n = int(n)
            s += (struct.calcsize('<' + PRIM[t]) if t in PRIM else layout_size(t)) * n
        return s

    i = 16
    while i + 3 <= len(b):
        size, typ = struct.unpack_from('<HB', b, i)
        p = b[i + 3:i + 3 + size]
        i += 3 + size
        if typ == ord('F'):
            name, fields = p.decode(errors='replace').split(':', 1)
            fmts[name] = [f for f in fields.split(';') if f]
        elif typ == ord('A'):
            multi, mid = struct.unpack_from('<BH', p, 0)
            name = p[3:].decode(errors='replace')
            if want is None or name in want:
                subs[mid] = (name if multi == 0 else '%s_%d' % (name, multi), name)
        elif typ == ord('D'):
            mid = struct.unpack_from('<H', p, 0)[0]
            if mid not in subs:
                continue
            key, name = subs[mid]
            d = out.setdefault(key, {})
            for fname, c, n, off in layout(name):
                if n == 1:
                    v = struct.unpack_from('<' + c, p, 2 + off)[0]
                else:
                    v = struct.unpack_from('<%d%s' % (n, c), p, 2 + off)
                d.setdefault(fname, []).append(v)
    return out


def main():
    path, specs = sys.argv[1], sys.argv[2:]
    want = {s.split('.')[0] for s in specs}
    data = read(path, want)
    cols = []
    for s in specs:
        t, _, fl = s.partition('.')
        if t not in data:
            print('# no topic %s' % t)
            continue
        for f in (fl.split(',') if fl else [k for k in data[t] if k != 'timestamp']):
            cols.append((t, f))
    t0 = min(data[t]['timestamp'][0] for t, _ in cols)
    t1 = max(data[t]['timestamp'][-1] for t, _ in cols)
    print('t_ms ' + ' '.join('%s.%s' % c for c in cols))
    idx = {t: 0 for t, _ in cols}
    for ts in range(t0, t1, 20000):
        row = ['%5d' % ((ts - t0) / 1000)]
        for t, f in cols:
            tt = data[t]['timestamp']
            while idx[t] + 1 < len(tt) and tt[idx[t] + 1] <= ts:
                idx[t] += 1
            v = data[t][f][idx[t]]
            row.append(('%.3f' % v) if isinstance(v, float) else str(v))
        print(' '.join(row))


if __name__ == '__main__':
    main()
