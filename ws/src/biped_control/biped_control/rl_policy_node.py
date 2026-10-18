"""RL policy as a drop-in replacement for gait_node (same topics), runs on the RPi with numpy.

  ros2 run biped_control rl_policy_node --ros-args -p policy:=/path/policy.npz
  ros2 param set /rl_policy enabled true         # stand -> policy
  ros2 topic pub -r 10 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.05}}"
"""
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64MultiArray

from biped_control.rl_policy import (ACTION_SCALE, CMD_MAX, CTRL_DT, JOINTS, LIMIT, REF,
                                     build_obs, gravity_in_frame, NumpyPolicy, reference)


class RlPolicyNode(Node):

    def __init__(self):
        super().__init__('rl_policy')
        self.declare_parameter('policy', '')
        self.declare_parameter('enabled', False)
        self.declare_parameter('cmd', 0.06)            # forward speed when no cmd_vel [m/s]
        self.declare_parameter('startup_time', 2.0)

        self.policy = NumpyPolicy(self.get_parameter('policy').value)
        self.pub = self.create_publisher(Float64MultiArray, 'position_controller/commands', 10)
        self.create_subscription(JointState, 'joint_states', self.on_js, 10)
        self.create_subscription(Imu, 'imu/data', self.on_imu, 10)
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd, 10)

        self.q = self.dq = self.grav = self.gyro = None
        self.q_start = None
        self.t = 0.0
        self.phi = 0.0
        self.last_action = np.zeros(6)
        self.cmd_vel = None
        self.cmd_time = None
        self.create_timer(CTRL_DT, self.tick)
        self.get_logger().info('Waiting for joint_states and imu/data ...')

    def on_js(self, msg):
        idx = {n: i for i, n in enumerate(msg.name)}
        if all(j in idx for j in JOINTS):
            self.q = np.array([msg.position[idx[j]] for j in JOINTS])
            self.dq = (np.array([msg.velocity[idx[j]] for j in JOINTS])
                       if len(msg.velocity) == len(msg.name) else np.zeros(6))

    def on_imu(self, msg):
        o = msg.orientation
        self.grav = gravity_in_frame(o.w, o.x, o.y, o.z)
        w = msg.angular_velocity
        self.gyro = np.array([w.x, w.y, w.z])

    def on_cmd(self, msg):
        self.cmd_vel = float(np.clip(msg.linear.x, 0.0, CMD_MAX))
        self.cmd_time = self.get_clock().now()

    def command(self):
        if self.cmd_time is not None:
            if (self.get_clock().now() - self.cmd_time).nanoseconds < 5e8:
                return self.cmd_vel
        return float(np.clip(self.get_parameter('cmd').value, 0.0, CMD_MAX))

    def tick(self):
        if self.q is None or self.grav is None:
            return
        cmd = self.command()
        if self.q_start is None:
            self.q_start = self.q.copy()
        T = self.get_parameter('startup_time').value
        if self.t < T or not self.get_parameter('enabled').value:
            # move to the reference pose at phase 0 and hold it
            self.t = min(self.t + CTRL_DT, T)
            a = self.t / T
            a = a * a * (3 - 2 * a)
            self.publish(self.q_start + a * (reference(0.0, cmd) - self.q_start))
            self.phi = 0.0
            self.last_action = np.zeros(6)
            return

        self.phi = (self.phi + 2 * np.pi * REF['freq'] * CTRL_DT) % (2 * np.pi)
        obs = build_obs(self.grav, self.gyro, self.q, self.dq, self.last_action, self.phi, cmd)
        action = self.policy(obs)
        self.last_action = action
        self.publish(reference(self.phi, cmd) + ACTION_SCALE * action)

    def publish(self, q):
        self.pub.publish(Float64MultiArray(data=np.clip(q, -LIMIT, LIMIT).tolist()))


def main():
    rclpy.init()
    try:
        rclpy.spin(RlPolicyNode())
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
