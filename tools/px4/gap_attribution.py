"""G6 command-gap attribution (standard library only). The motor publisher logs
every command gap (<base>.gaps.txt: wall_time gap_ms sim_gap_ms), the record
streamer every record-arrival gap (<base>.stream_gaps.txt: wall_time gap_ms
sim_gap_ms) and the stall probe every late wake-up (<base>.stalls.txt:
wall_time late_ms). A command gap is a machine-wide stall when the probe woke
late inside it, and upstream when PX4's own record stream had the same hole;
what is neither was added by the Arc side and its pipes."""

FLIGHT_FROM_S = 3.0   # in-flight figures start this long after the pipeline starts (spin-up, lift-off)


def _rows(path):
    try:
        with open(path) as f:
            return [[float(x) for x in l.split()] for l in f if l.strip() and not l.startswith('#')]
    except OSError:
        return None


def attribute_gaps(base, t_end=None, t_flight=None):
    """Split the motor publisher's command gaps (<base>.gaps.txt: wall_time gap_ms sim_gap_ms) into
    machine-wide stalls (the stall probe, <base>.stalls.txt, woke late inside the same interval) and
    the rest; add the worst record-arrival gap at the streamer (<base>.stream_gaps.txt). Gaps ending
    after t_end (wall time: the end of the flight window, when the streamer winds down) are left out.
    With t_flight (wall time after spin-up and lift-off), the in-flight figures are given as well."""
    out = {}
    g = _rows(base + '.gaps.txt')
    st = _rows(base + '.stalls.txt')
    sg = _rows(base + '.stream_gaps.txt')
    if g is None:
        return out
    if t_end is not None:
        g = [r for r in g if r[0] <= t_end]
        st = [r for r in st if r[0] <= t_end] if st is not None else None
        sg = [r for r in sg if r[0] <= t_end] if sg is not None else None

    def stalled(r):
        t1, t0 = r[0], r[0] - r[1] / 1000.0
        return st is not None and any(t0 - 0.005 <= s[0] <= t1 + 0.005 and s[1] > 20.0 for s in st)

    def upstream(r):   # the streamer saw the same silence in the PX4 record stream (ends together, as long)
        return sg is not None and any(abs(x[0] - r[0]) <= 0.010 and x[1] >= r[1] - 10.0 for x in sg)

    def stats(rows, sfx):
        out['worst_ms' + sfx] = max([r[1] for r in rows] or [0.0])
        if st is not None:
            out['machine_stalls' + sfx] = len([r for r in rows if stalled(r)])
            out['worst_ms_outside_machine_stalls' + sfx] = max([r[1] for r in rows if not stalled(r)] or [0.0])
            # what the Arc side and its pipes added: neither a machine stall nor a gap in PX4's own stream
            out['worst_ms_added_by_arc_side' + sfx] = max([r[1] for r in rows if not stalled(r) and not upstream(r)]
                                                          or [0.0])
    stats(g, '_in_window')
    out['worst_sim_ms'] = max([r[2] for r in g if len(r) > 2] or [0.0])
    if st is not None:
        out['machine_stalls'] = out['machine_stalls_in_window']
        out['worst_ms_outside_machine_stalls'] = out['worst_ms_outside_machine_stalls_in_window']
        out['worst_stall_probe_ms'] = max([s[1] for s in st] or [0.0])
    if t_flight is not None:
        stats([r for r in g if r[0] - r[1] / 1000.0 >= t_flight], '_in_flight')
    if sg is not None:
        out['worst_stream_arrival_ms'] = max([r[1] for r in sg] or [0.0])
    return out
