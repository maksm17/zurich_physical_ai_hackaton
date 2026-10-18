"""MuJoCo simulation that stands in for the real robot (ros2_control + MPU6050 + Madgwick).

  sub  position_controller/commands  Float64MultiArray  [l_up, l_mid, l_down, r_up, r_mid, r_down]
  pub  joint_states, imu/data, sim/base_pose, TF world -> base_link
  srv  sim/reset

So gait_node runs unchanged:  ros2 launch biped_sim sim.launch.py
"""
import os
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, TransformStamped
import mujoco
import mujoco.viewer
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

from biped_sim.model import build_model, JOINTS, SERVO
from biped_sim.servo import REAL, ServoModel


class SimNode(Node):

    def __init__(self):
        super().__init__('mujoco_sim')
        default_xacro = os.path.join(
            get_package_share_directory('biped_description'), 'urdf', 'biped.xacro')
        self.declare_parameter('xacro', default_xacro)
        self.declare_parameter('viewer', True)
        self.declare_parameter('publish_rate', 100.0)
        self.declare_parameter('realtime_factor', 1.0)
        self.declare_parameter('flat_sole', True)
        self.declare_parameter('sole_size', [0.04, 0.045, 0.004])
        self.declare_parameter('foot_cut', 0.0)
        self.declare_parameter('servo_model', True)
        for k, v in REAL.items():
            self.declare_parameter('servo_' + k, v)
        for k, v in SERVO.items():
            self.declare_parameter('servo.' + k, v)

        servo = {k: self.get_parameter('servo.' + k).value for k in SERVO}
        sole = (list(self.get_parameter('sole_size').value)
                if self.get_parameter('flat_sole').value else None)
        self.model, _ = build_model(self.get_parameter('xacro').value, servo, sole,
                                    cut=self.get_parameter('foot_cut').value)
        self.data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, self.data)
        m = self.model
        self.qadr = np.array([m.joint(j).qposadr[0] for j in JOINTS])
        self.vadr = np.array([m.joint(j).dofadr[0] for j in JOINTS])
        self.act = np.array([m.actuator(j).id for j in JOINTS])
        self.sens = {n: (m.sensor(n).adr[0], m.sensor(n).dim[0])
                     for n in ('imu_quat', 'imu_gyro', 'imu_acc')}
        self.get_logger().info(
            f'MuJoCo model: {m.body_subtreemass[1]:.3f} kg, servo {servo}')

        self.ctrl = np.zeros(len(JOINTS))
        self.servo = None
        if self.get_parameter('servo_model').value:
            sp = {k: self.get_parameter('servo_' + k).value for k in REAL}
            self.servo = ServoModel(len(JOINTS), dt=m.opt.timestep, **sp)
            self.servo.reset(self.data.qpos[self.qadr])
            self.get_logger().info(f'servo model {sp}')
        self.reset_requested = False

        self.js_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self.pose_pub = self.create_publisher(PoseStamped, 'sim/base_pose', 10)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(
            Float64MultiArray, 'position_controller/commands', self.on_cmd, 10)
        self.create_service(Trigger, 'sim/reset', self.on_reset)

    def on_cmd(self, msg):
        if len(msg.data) != len(JOINTS):
            self.get_logger().warn('expected 6 commands', throttle_duration_sec=2.0)
            return
        self.ctrl[:] = msg.data

    def on_reset(self, req, res):
        self.reset_requested = True
        res.success, res.message = True, 'reset'
        return res

    def sensor(self, name):
        adr, dim = self.sens[name]
        return self.data.sensordata[adr:adr + dim]

    def publish(self):
        d = self.data
        stamp = self.get_clock().now().to_msg()

        js = JointState()
        js.header.stamp = stamp
        js.name = JOINTS
        js.position = d.qpos[self.qadr].tolist()
        js.velocity = d.qvel[self.vadr].tolist()
        self.js_pub.publish(js)

        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = 'imu_link'
        q = self.sensor('imu_quat')
        imu.orientation.w, imu.orientation.x, imu.orientation.y, imu.orientation.z = q
        g = self.sensor('imu_gyro')
        imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = g
        a = self.sensor('imu_acc')
        imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z = a
        self.imu_pub.publish(imu)

        ps = PoseStamped()
        ps.header.stamp = stamp
        ps.header.frame_id = 'world'
        p, q = d.qpos[0:3], d.qpos[3:7]
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = p
        ps.pose.orientation.w, ps.pose.orientation.x = q[0], q[1]
        ps.pose.orientation.y, ps.pose.orientation.z = q[2], q[3]
        self.pose_pub.publish(ps)

        tf = TransformStamped()
        tf.header = ps.header
        tf.child_frame_id = 'base_link'
        tf.transform.translation.x, tf.transform.translation.y = p[0], p[1]
        tf.transform.translation.z = p[2]
        tf.transform.rotation = ps.pose.orientation
        self.tf.sendTransform(tf)

    def run(self, executor, viewer=None):
        m, d = self.model, self.data
        pub_dt = 1.0 / self.get_parameter('publish_rate').value
        rtf = self.get_parameter('realtime_factor').value
        t_wall0, t_sim0 = time.monotonic(), d.time
        next_pub = 0.0
        while rclpy.ok() and (viewer is None or viewer.is_running()):
            if self.reset_requested:
                self.reset_requested = False
                mujoco.mj_resetData(m, d)
                mujoco.mj_forward(m, d)
                if self.servo is not None:
                    self.servo.reset(d.qpos[self.qadr])
                t_wall0, t_sim0, next_pub = time.monotonic(), d.time, 0.0
            executor.spin_once(timeout_sec=0.0)
            target = t_sim0 + (time.monotonic() - t_wall0) * rtf
            if target - d.time > 0.02:
                t_sim0 -= target - d.time - 0.02
                target = d.time + 0.02
            while d.time < target:
                if self.servo is not None:
                    d.ctrl[self.act] = self.servo.update(self.ctrl, d.qpos[self.qadr])
                else:
                    d.ctrl[self.act] = self.ctrl
                mujoco.mj_step(m, d)
            if d.time >= next_pub:
                self.publish()
                next_pub = d.time + pub_dt
                if viewer is not None:
                    viewer.sync()
            time.sleep(0.001)


def main():
    rclpy.init()
    node = SimNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        if node.get_parameter('viewer').value:
            v = mujoco.viewer.launch_passive(node.model, node.data)
            v.cam.distance, v.cam.elevation, v.cam.lookat[2] = 0.9, -15, 0.12
            node.run(executor, v)
        else:
            node.run(executor)
    except KeyboardInterrupt:
        pass
    rclpy.try_shutdown()
    os._exit(0)


if __name__ == '__main__':
    main()
