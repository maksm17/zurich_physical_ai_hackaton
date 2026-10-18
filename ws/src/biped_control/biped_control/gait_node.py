"""Feedforward gait generator for the 6-DOF biped (+ optional IMU feedback).

Per leg: up = hip roll (x), mid = hip pitch (y), down = ankle pitch (y).

Gait (semantic angles, then mapped to joints with `sem_sign`):
    roll   = A_roll * sin(phi)                 both hips, rocks weight onto one foot
    p_L    = A_step * k_L * sin(phi + phase)   swing leg forward while the other is loaded
    p_R    = -A_step * k_R * sin(phi + phase)
    ankle  = -ankle_ratio * p                  keep the sole parallel to the ground
k_L = forward + turn, k_R = forward - turn.

Instead of sines you can play a CSV trajectory (feedforward computed offline, e.g. Crocoddyl):
header `t,l_up,l_mid,l_down,r_up,r_mid,r_down`, played in a loop.

Run / tune live:
    ros2 param set /gait walk true
    ros2 param set /gait step_amp 0.2
    ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.5}}"
    ros2 service call /gait/zero_imu std_srvs/srv/Trigger
"""
import csv
import math

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger

JOINTS = ['l_up', 'l_mid', 'l_down', 'r_up', 'r_mid', 'r_down']


def quat_to_roll_pitch(q):
    roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
    return roll, pitch


class GaitNode(Node):

    def __init__(self):
        super().__init__('gait')
        p = self.declare_parameter
        p('rate', 100.0)
        p('limit', 0.43)
        p('startup_time', 2.0)          # ramp from current pose to stand pose
        p('ramp_time', 1.0)             # gait amplitude fade in / out
        # semantic -> joint sign, order = JOINTS (from URDF axes: r_mid axis is -y)
        p('sem_sign', [1.0, 1.0, 1.0, 1.0, -1.0, 1.0])
        p('trim', [0.0] * 6)            # stand pose offsets per joint [rad]
        # feedforward gait
        p('walk', False)
        p('forward', 1.0)               # -1..1, used when no cmd_vel is coming
        p('turn', 0.0)                  # -1..1
        p('freq', 1.2)                  # [Hz] one full L+R cycle
        p('roll_amp', 0.10)             # [rad]
        p('step_amp', 0.25)             # [rad]
        p('phase_deg', -90.0)           # step lag behind roll
        p('ankle_ratio', 1.0)
        p('ankle_bias', 0.0)            # lean forward/back [rad]
        p('trajectory_file', '')        # CSV instead of sines
        # cmd_vel scaling
        p('max_vx', 1.0)
        p('max_wz', 1.0)
        p('cmd_tau', 0.8)               # [s] smoothing of forward/turn changes
        # IMU feedback (0 = pure feedforward)
        p('k_roll', 0.0)
        p('k_pitch', 0.0)
        p('max_correction', 0.15)
        p('imu_roll_sign', 1.0)
        p('imu_pitch_sign', 1.0)
        p('imu_swap_axes', False)
        # safety: above this tilt stop and go limp (targets = measured positions)
        p('fall_angle_deg', 40.0)

        self.dt = 1.0 / self.get_parameter('rate').value
        self.pub = self.create_publisher(Float64MultiArray, 'position_controller/commands', 10)
        self.create_subscription(JointState, 'joint_states', self.on_joint_states, 10)
        self.create_subscription(Imu, 'imu/data', self.on_imu, 10)
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10)
        self.create_service(Trigger, 'gait/zero_imu', self.on_zero_imu)
        self.create_service(Trigger, 'gait/estop', self.on_estop)
        self.create_service(Trigger, 'gait/resume', self.on_resume)
        self.halted = None  # None or reason ('fall', 'estop')

        self.q_meas = None
        self.q_start = None
        self.t = 0.0
        self.phi = 0.0
        self.env = 0.0
        self.imu_rp = None
        self.imu_ref = (0.0, 0.0)
        self.imu_ref_set = False
        self.cmd = None
        self.cmd_time = None
        self.traj = None
        self.traj_file = ''
        self.fwd_f = 0.0
        self.turn_f = 0.0

        self.create_timer(self.dt, self.tick)
        self.get_logger().info('Waiting for /joint_states ...')

    # ---------------------------------------------------------------- inputs
    def on_joint_states(self, msg):
        idx = {n: i for i, n in enumerate(msg.name)}
        if all(j in idx for j in JOINTS):
            self.q_meas = np.array([msg.position[idx[j]] for j in JOINTS])

    def on_imu(self, msg):
        r, pt = quat_to_roll_pitch(msg.orientation)
        if self.get_parameter('imu_swap_axes').value:
            r, pt = pt, r
        self.imu_rp = (r * self.get_parameter('imu_roll_sign').value,
                       pt * self.get_parameter('imu_pitch_sign').value)

    def on_cmd_vel(self, msg):
        self.cmd = (msg.linear.x / self.get_parameter('max_vx').value,
                    msg.angular.z / self.get_parameter('max_wz').value)
        self.cmd_time = self.get_clock().now()

    def on_zero_imu(self, req, res):
        if self.imu_rp is None:
            res.success, res.message = False, 'no IMU data on imu/data'
        else:
            self.imu_ref = self.imu_rp
            self.imu_ref_set = True
            res.success, res.message = True, f'IMU reference roll/pitch = {self.imu_ref}'
        return res

    def halt(self, reason):
        if self.halted is None:
            self.get_logger().error(f'HALT ({reason}): walking stopped, servos limp. '
                                    'ros2 service call /gait/resume std_srvs/srv/Trigger')
        self.halted = reason
        self.set_parameters([Parameter('walk', value=False)])
        self.env = 0.0
        self.phi = 0.0

    def on_estop(self, req, res):
        self.halt('estop')
        res.success, res.message = True, 'halted'
        return res

    def on_resume(self, req, res):
        # stand up again smoothly from wherever the robot is now
        self.halted = None
        self.imu_ref_set = False  # re-taken when standing again
        self.q_start = None
        self.t = 0.0
        res.success, res.message = True, 'standing up'
        return res

    # ---------------------------------------------------------------- gait
    def load_traj(self, path):
        with open(path) as f:
            rows = list(csv.DictReader(f))
        t = np.array([float(r['t']) for r in rows])
        q = np.array([[float(r.get(j, 0.0)) for j in JOINTS] for r in rows])
        self.get_logger().info(f'Loaded trajectory {path}: {len(t)} samples, {t[-1]:.2f} s')
        return t, q

    def walk_command(self):
        """Returns (walking, forward, turn)."""
        if self.cmd_time is not None:
            age = (self.get_clock().now() - self.cmd_time).nanoseconds * 1e-9
            if age < 0.5:
                fwd, turn = self.cmd
                return abs(fwd) > 0.05 or abs(turn) > 0.05, fwd, turn
        g = self.get_parameter
        return g('walk').value, g('forward').value, g('turn').value

    def gait_offsets(self, fwd, turn):
        """Joint offsets around the stand pose (already scaled by envelope)."""
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        path = g('trajectory_file')
        if path:
            if path != self.traj_file:
                self.traj, self.traj_file = self.load_traj(path), path
            t, q = self.traj
            tt = self.phi / (2 * math.pi) * t[-1] % t[-1]
            return self.env * np.array([np.interp(tt, t, q[:, i]) for i in range(6)])

        kl = float(np.clip(fwd + turn, -1.0, 1.0))
        kr = float(np.clip(fwd - turn, -1.0, 1.0))
        s_roll = math.sin(self.phi)
        s_step = math.sin(self.phi + math.radians(g('phase_deg')))
        roll = g('roll_amp') * s_roll
        p_l = g('step_amp') * kl * s_step
        p_r = -g('step_amp') * kr * s_step
        a_l = -g('ankle_ratio') * p_l
        a_r = -g('ankle_ratio') * p_r
        return self.env * np.array([roll, p_l, a_l, roll, p_r, a_r])

    def imu_correction(self):
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        if self.imu_rp is None or (g('k_roll') == 0.0 and g('k_pitch') == 0.0):
            return np.zeros(6)
        c = g('max_correction')
        er = self.imu_rp[0] - self.imu_ref[0]
        ep = self.imu_rp[1] - self.imu_ref[1]
        dr = float(np.clip(-g('k_roll') * er, -c, c))
        dp = float(np.clip(-g('k_pitch') * ep, -c, c))
        return np.array([dr, 0.0, dp, dr, 0.0, dp])

    def tick(self):
        if self.q_meas is None:
            return
        # tilt relative to the reference taken while standing (IMU may be mounted at an angle);
        # not checked until that reference exists
        if self.imu_rp is not None and self.halted is None and self.imu_ref_set:
            er = self.imu_rp[0] - self.imu_ref[0]
            ep = self.imu_rp[1] - self.imu_ref[1]
            tilt = math.degrees(math.acos(min(1.0, math.cos(er) * math.cos(ep))))
            if tilt > self.get_parameter('fall_angle_deg').value:
                self.halt(f'fall, tilt {tilt:.0f} deg')
        if self.halted is not None:
            # target = measured position: servos stop pushing against the ground
            self.pub.publish(Float64MultiArray(data=self.q_meas.tolist()))
            return
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        sem_sign = np.array(g('sem_sign'))
        stand = np.array(g('trim'))

        # 1) smooth transition from wherever the robot is to the stand pose
        if self.q_start is None:
            self.q_start = self.q_meas.copy()
            self.get_logger().info('Moving to stand pose')
        T = g('startup_time')
        if self.t < T:
            self.t += self.dt
            a = min(self.t / T, 1.0)
            a = a * a * (3 - 2 * a)  # smoothstep
            self.publish(self.q_start + a * (stand - self.q_start))
            if self.t >= T:
                if self.imu_rp is not None:
                    self.imu_ref = self.imu_rp
                    self.imu_ref_set = True
                self.get_logger().info('Standing. ros2 param set /gait walk true')
            return

        # 2) gait envelope: fade in when walking, fade out and reset phase when stopping
        walking, fwd, turn = self.walk_command()
        # abrupt amplitude changes mid-step tip the robot over -> low-pass them
        a = min(1.0, self.dt / max(g('cmd_tau'), 1e-3))
        self.fwd_f += a * (fwd - self.fwd_f)
        self.turn_f += a * (turn - self.turn_f)
        fwd, turn = self.fwd_f, self.turn_f
        rate = self.dt / max(g('ramp_time'), 1e-3)
        self.env = min(1.0, self.env + rate) if walking else max(0.0, self.env - rate)
        if self.env > 0.0:
            self.phi = (self.phi + 2 * math.pi * g('freq') * self.dt) % (2 * math.pi)
        else:
            self.phi = 0.0

        ff = self.gait_offsets(fwd, turn)
        if not g('trajectory_file'):  # sines are semantic, CSV is already in joint space
            ff = sem_sign * ff
        self.publish(stand + ff + sem_sign * self.imu_correction())

    def publish(self, q):
        lim = self.get_parameter('limit').value
        self.pub.publish(Float64MultiArray(data=np.clip(q, -lim, lim).tolist()))


def main():
    rclpy.init()
    try:
        rclpy.spin(GaitNode())
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
