"""G6 takeover trace from PX4's flight log (ulog): PX4's state from the arm at handover onwards, to see
what its controllers start from (nav = vehicle_status.nav_state, tko = takeoff_status, logged at a low
rate, hte = hover-thrust estimate, sp = position-controller setpoint, traj = trajectory setpoint, z/vz =
local position, NED). Usage: python tools/px4/takeover_trace.py LOG.ulg [ms_after_arm]"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ulog_read  # noqa: E402


def main():
    d = ulog_read.read(sys.argv[1])
    span = int(sys.argv[2]) if len(sys.argv) > 2 else 3000
    vs = d['vehicle_status']
    ta = next(t for t, a in zip(vs['timestamp'], vs['arming_state']) if a == 2)

    def at(topic, field, ts, k=None):
        if topic not in d:   # not in this log (the log is copied while PX4 still writes it)
            return float('nan')
        tt = d[topic]['timestamp']
        i = max(0, sum(1 for t in tt if t <= ts) - 1)
        v = d[topic][field][i]
        return v[k] if k is not None else v

    print('arm at %.3f s (log time)' % (ta / 1e6))
    print('  ms nav tko landed maybe gc  hte    thr_z  sp_z   sp_vz  traj_z traj_vz   z      vz')
    for ms in list(range(-40, 600, 20)) + list(range(600, span, 100)):
        ts = ta + ms * 1000
        L = lambda f: at('vehicle_land_detected', f, ts)  # noqa: E731
        print('%5d %3d %3d %d %d %d %.3f %6.3f %6.2f %6.2f %6.2f %6.2f %6.2f %6.2f' % (
            ms, at('vehicle_status', 'nav_state', ts), at('takeoff_status', 'takeoff_state', ts),
            L('landed'), L('maybe_landed'), L('ground_contact'), at('hover_thrust_estimate', 'hover_thrust', ts),
            at('vehicle_thrust_setpoint', 'xyz', ts, 2), at('vehicle_local_position_setpoint', 'z', ts),
            at('vehicle_local_position_setpoint', 'vz', ts), at('trajectory_setpoint', 'position', ts, 2),
            at('trajectory_setpoint', 'velocity', ts, 2), at('vehicle_local_position', 'z', ts),
            at('vehicle_local_position', 'vz', ts)))


if __name__ == '__main__':
    main()
