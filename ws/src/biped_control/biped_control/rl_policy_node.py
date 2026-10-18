"""RL policy as a drop-in replacement for gait_node (same topics), runs on the RPi with numpy.

  ros2 run biped_control rl_policy_node --ros-args -p policy:=/path/policy.npz
  ros2 param set /rl_policy enabled true         # stand -> policy
Safety: tilt > fall_angle_deg for fall_time -> halt (servo targets = measured, policy off);
ros2 service call /rl/estop std_srvs/srv/Trigger ; re-enable with enabled true
  ros2 topic pub -r 10 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.05}, angular: {z: 0.2}}"
"""
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from rclpy.parameter import Parameter
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger

from biped_control.rl_policy import (CMD_MAX, CTRL_DT, JOINTS, LIMIT,
                                     build_obs, gravity_in_frame, NumpyPolicy, reference, WZ_MAX)


class RlPolicyNode(Node):

    def __init__(self):
        super().__init__('rl_policy')
        self.declare_parameter('policy', '')
        self.declare_parameter('enabled', False)
        self.declare_parameter('cmd', 0.06)
        self.declare_parameter('startup_time', 2.0)
        self.declare_parameter('fall_angle_deg', 40.0)
        self.declare_parameter('fall_time', 0.3)
        self.declare_parameter('imu_yaw', 3.14159265)

        self.policy = NumpyPolicy(self.get_parameter('policy').value)
        self.ref = self.policy.ref
        self.get_logger().info(f'policy reference {self.ref}, action scale '
                               f'{self.policy.action_scale}, trained for foot cut {self.policy.cut} m')
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
        self.create_service(Trigger, 'rl/estop', self.on_estop)
        self.state_pub = self.create_publisher(String, 'gait/state', 10)
        self.halted = None
        self.tilt_since = None
        self.create_timer(CTRL_DT, self.tick)
        self.get_logger().info('Waiting for joint_states and imu/data ...')

    def halt(self, reason):
        if self.halted is None:
            self.get_logger().error(f'HALT ({reason}): policy off, servos limp. '
                                    'ros2 param set /rl_policy enabled true to restart')
        self.halted = reason
        self.set_parameters([Parameter('enabled', value=False)])

    def on_estop(self, req, res):
        self.halt('estop')
        res.success, res.message = True, 'halted'
        return res

    def on_js(self, msg):
        idx = {n: i for i, n in enumerate(msg.name)}
        if all(j in idx for j in JOINTS):
            self.q = np.array([msg.position[idx[j]] for j in JOINTS])
            self.dq = (np.array([msg.velocity[idx[j]] for j in JOINTS])
                       if len(msg.velocity) == len(msg.name) else np.zeros(6))

    def on_imu(self, msg):
        o = msg.orientation
        w = msg.angular_velocity
        y = self.get_parameter('imu_yaw').value
        c, s = np.cos(y), np.sin(y)
        rz = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
        self.grav = rz @ gravity_in_frame(o.w, o.x, o.y, o.z)
        self.gyro = rz @ np.array([w.x, w.y, w.z])

    def on_cmd(self, msg):
        self.cmd_vel = (float(np.clip(msg.linear.x, 0.0, CMD_MAX)),
                        float(np.clip(msg.angular.z, -WZ_MAX, WZ_MAX)))
        self.cmd_time = self.get_clock().now()

    def command(self):
        if self.cmd_time is not None:
            if (self.get_clock().now() - self.cmd_time).nanoseconds < 5e8:
                return self.cmd_vel
        return float(np.clip(self.get_parameter('cmd').value, 0.0, CMD_MAX)), 0.0

    def tick(self):
        if self.q is None or self.grav is None:
            return
        cmd, wz = self.command()
        tilt = np.degrees(np.arccos(np.clip(-self.grav[2], -1.0, 1.0)))
        if self.halted is None and tilt > self.get_parameter('fall_angle_deg').value:
            self.tilt_since = self.tilt_since or self.get_clock().now()
            if (self.get_clock().now() - self.tilt_since).nanoseconds * 1e-9 > \
                    self.get_parameter('fall_time').value:
                self.halt(f'fall, tilt {tilt:.0f} deg')
        else:
            self.tilt_since = None
        if self.halted is not None:
            if self.get_parameter('enabled').value:
                self.halted, self.q_start, self.t = None, None, 0.0
            else:
                self.publish(self.q)
                self.state_pub.publish(String(data=f'halted:{self.halted}'))
                return
        self.state_pub.publish(String(data='walking' if self.get_parameter('enabled').value
                                      and self.t >= self.get_parameter('startup_time').value
                                      else 'standing'))
        if self.q_start is None:
            self.q_start = self.q.copy()
        T = self.get_parameter('startup_time').value
        if self.t < T or not self.get_parameter('enabled').value:
            self.t = min(self.t + CTRL_DT, T)
            a = self.t / T
            a = a * a * (3 - 2 * a)
            self.publish(self.q_start + a * (reference(0.0, cmd, self.ref) - self.q_start))
            self.phi = 0.0
            self.last_action = np.zeros(6)
            return

        self.phi = (self.phi + 2 * np.pi * self.ref['freq'] * CTRL_DT) % (2 * np.pi)
        obs = build_obs(self.grav, self.gyro, self.q, self.dq, self.last_action, self.phi, cmd, wz)
        action = self.policy(obs)
        self.last_action = action
        self.publish(reference(self.phi, cmd, self.ref) + self.policy.action_scale * action)

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
