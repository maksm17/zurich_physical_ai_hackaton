#!/usr/bin/env python3
"""Show the REAL robot in RViz on the laptop: /joint_states streamed from the RPi over SSH
(DDS discovery does not cross Tailscale), republished locally for robot_state_publisher.

  scripts/rviz_robot.sh            # starts relay + robot_state_publisher + RViz
"""
import json
import subprocess
import sys

import rclpy
from sensor_msgs.msg import JointState

REMOTE = r'''
import json, sys, time, rclpy
from sensor_msgs.msg import JointState
rclpy.init(); n = rclpy.create_node("js_stream"); last = [0.0]
def cb(m):
    if time.time() - last[0] < 0.03: return
    last[0] = time.time()
    print(json.dumps([list(m.name), [round(p, 4) for p in m.position]]), flush=True)
n.create_subscription(JointState, "joint_states", cb, 10)
rclpy.spin(n)
'''


def main():
    host = sys.argv[1] if len(sys.argv) > 1 else 'rpi'
    cmd = ['ssh', '-o', 'ConnectTimeout=8', host,
           'docker exec -i biped bash -c "source /ws/install/setup.bash && python3 -u -"']
    proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True)
    proc.stdin.write(REMOTE)
    proc.stdin.close()

    rclpy.init()
    node = rclpy.create_node('joint_relay')
    pub = node.create_publisher(JointState, 'joint_states', 10)
    node.get_logger().info(f'Relaying /joint_states from {host} ...')
    try:
        for line in proc.stdout:
            if not rclpy.ok():
                break
            try:
                names, pos = json.loads(line)
            except ValueError:
                continue
            msg = JointState()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.name, msg.position = names, pos
            pub.publish(msg)
    except KeyboardInterrupt:
        pass
    finally:
        proc.terminate()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
