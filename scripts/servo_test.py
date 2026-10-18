#!/usr/bin/env python3
"""Move ONE joint gently through ros2_control and record command vs measured position.

Run inside the robot container while bringup runs with torque (gait:=false):
  python3 servo_test.py l_down [amplitude_deg] [out.csv]

Sequence: hold 1 s -> sine +-A at 0.5 Hz (2 cycles, ramped) -> steps +3/0/-3/0 deg (1 s each)
-> hold. All other joints are held at their start position.
Safety: aborts (all targets := measured, i.e. limp) if the tracking error of the tested joint
exceeds 15 deg or any other joint moves more than 5 deg.
"""
import csv
import math
import sys
import time

import rclpy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

JOINTS = ['l_up', 'l_mid', 'l_down', 'r_up', 'r_mid', 'r_down']


def main():
    joint = sys.argv[1]
    amp = math.radians(float(sys.argv[2]) if len(sys.argv) > 2 else 5.0)
    out = sys.argv[3] if len(sys.argv) > 3 else f'/tmp/servo_test_{joint}.csv'
    k = JOINTS.index(joint)

    rclpy.init()
    n = rclpy.create_node('servo_test')
    pub = n.create_publisher(Float64MultiArray, 'position_controller/commands', 10)
    st = {}

    def on_js(m):
        idx = {nm: i for i, nm in enumerate(m.name)}
        st['q'] = [m.position[idx[j]] for j in JOINTS]
        st['t'] = time.monotonic()
        st['stamp'] = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
    n.create_subscription(JointState, 'joint_states', on_js, 10)

    t0 = time.monotonic()
    while 'q' not in st:
        rclpy.spin_once(n, timeout_sec=0.1)
        if time.monotonic() - t0 > 5:
            sys.exit('no joint_states')
    q0 = list(st['q'])
    print('start pose [deg]:', [round(math.degrees(v), 1) for v in q0])

    def target(t):
        if t < 1.0:
            return 0.0
        if t < 5.0:
            s = t - 1.0
            return amp * min(1.0, s) * math.sin(2 * math.pi * 0.5 * s)
        steps = [math.radians(3), 0.0, -math.radians(3), 0.0]
        i = int(t - 5.0)
        return steps[i] if i < len(steps) else 0.0

    rows, T, aborted = [], 10.0, None
    t_start = time.monotonic()
    next_tick = t_start
    while True:
        t = time.monotonic() - t_start
        if t > T:
            break
        for _ in range(20):
            rclpy.spin_once(n, timeout_sec=0.0)
        cmd = list(q0)
        cmd[k] = q0[k] + target(t)
        q = st['q']
        err = abs(q[k] - cmd[k])
        others = max(abs(q[i] - q0[i]) for i in range(6) if i != k)
        if t > 0.3 and (err > math.radians(15) or others > math.radians(5)):
            aborted = (f'tracking error {math.degrees(err):.1f} deg, '
                       f'other joints moved {math.degrees(others):.1f} deg')
            pub.publish(Float64MultiArray(data=list(q)))
            break
        pub.publish(Float64MultiArray(data=cmd))
        now = n.get_clock().now().nanoseconds * 1e-9
        rows.append((round(t, 4), cmd[k] - q0[k], q[k] - q0[k], round(now - st['stamp'], 4)))
        next_tick += 0.01
        time.sleep(max(0.0, next_tick - time.monotonic()))

    if aborted is None:
        for _ in range(50):
            pub.publish(Float64MultiArray(data=q0))
            time.sleep(0.01)
    with open(out, 'w') as f:
        w = csv.writer(f)
        w.writerow(['t', 'cmd_rel', 'meas_rel', 'meas_age'])
        w.writerows(rows)
    print('ABORTED: ' + aborted if aborted else f'done, {len(rows)} samples -> {out}')
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
