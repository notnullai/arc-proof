"""Persistent disturbance injector, runs inside the PX4 Gazebo container.
Keeps its gz-transport publishers alive (one-shot `gz topic -p` exits
before discovery completes and loses messages), reads commands on stdin:

  fail <n>            publish Int32 n on /model/x500_0/motor_failure/motor_number
  wind <fx> <fy> <fz> persistent EntityWrench force on the model (N)
  clear               clear the persistent wrench
  quit

Each command is published 3 times, 50 ms apart, and acknowledged on
stdout with "OK <cmd>".
"""
import sys
import time

from gz.transport13 import Node
from gz.msgs10.int32_pb2 import Int32
from gz.msgs10.entity_wrench_pb2 import EntityWrench
from gz.msgs10.entity_pb2 import Entity

MODEL = 'x500_0'


def main():
    node = Node()
    pub_fail = node.advertise('/model/%s/motor_failure/motor_number' % MODEL, Int32)
    pub_wrench = node.advertise('/world/default/wrench/persistent', EntityWrench)
    pub_clear = node.advertise('/world/default/wrench/clear', Entity)
    time.sleep(1.0)   # discovery
    print('READY', flush=True)
    for line in sys.stdin:
        parts = line.split()
        if not parts:
            continue
        cmd = parts[0]
        if cmd == 'quit':
            break
        for _ in range(3):
            if cmd == 'fail' and len(parts) >= 2:
                m = Int32()
                m.data = int(parts[1])
                pub_fail.publish(m)
            elif cmd == 'wind' and len(parts) >= 4:
                m = EntityWrench()
                m.entity.name = MODEL
                m.entity.type = Entity.MODEL
                m.wrench.force.x = float(parts[1])
                m.wrench.force.y = float(parts[2])
                m.wrench.force.z = float(parts[3])
                pub_wrench.publish(m)
            elif cmd == 'clear':
                m = Entity()
                m.name = MODEL
                m.type = Entity.MODEL
                pub_clear.publish(m)
            else:
                print('ERR %s' % line.strip(), flush=True)
                break
            time.sleep(0.05)
        else:
            print('OK %s' % line.strip(), flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
