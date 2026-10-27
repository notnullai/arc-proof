"""Measure PX4 routing latency (PARAM_SET FUNC1..4 -> gz_bridge status routed), disarmed.
argv: link_port keepalive_func(0 = none) trials"""
import re, subprocess, sys, time
sys.path.insert(0, '/tmp/arc')
from gz_motor_pub import Px4Link, ROUTED
port, ka, n = int(sys.argv[1]), int(sys.argv[2]), int(sys.argv[3])
L = Px4Link('udp:127.0.0.1:%d' % port)
gzb = '/opt/px4-gazebo/bin/px4-gz_bridge'
print('ka set', L.set_int_params([('SIM_GZ_EC_FUNC5', ka)]))
out = []
for k in range(n):
    L.set_int_params([('SIM_GZ_EC_FUNC%d' % i, 0) for i in range(1, 5)])
    time.sleep(2.5 + 0.037 * k)
    t = time.time()
    L.set_int_params([('SIM_GZ_EC_FUNC%d' % i, 100 + i) for i in range(1, 5)])
    tp = time.time()
    polls = 0
    while polls < 200:
        r = subprocess.run([gzb, 'status'], capture_output=True, text=True)
        if len(ROUTED.findall(r.stdout)) >= 4:
            break
        polls += 1
    out.append((round((tp - t) * 1000, 1), polls, round((time.time() - t) * 1000)))
print(out)
L.set_int_params([('SIM_GZ_EC_FUNC%d' % i, 0) for i in range(1, 6)])
