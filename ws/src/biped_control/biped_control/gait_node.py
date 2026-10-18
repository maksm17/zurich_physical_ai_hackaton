"""Feedforward gait generator for the 6-DOF biped (+ optional IMU feedback).

Per leg: up = hip roll (x), mid = hip pitch (y), down = ankle pitch (y).

Gait (semantic angles, then mapped to joints with `sem_sign`):
    roll   = A_roll * sin(phi)                 both hips, rocks weight onto one foot
    p_L    = A_step * k_L * sin(phi + phase)   swing leg forward while the other is loaded
    p_R    = -A_step * k_R * sin(phi + phase)
    ankle  = -ankle_ratio * p                  keep the sole parallel to the ground
k_L = forward * (1 + turn), k_R = forward * (1 - turn).

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
from std_msgs.msg import Float64MultiArray, String
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
        p('startup_time', 2.0)
        p('ramp_time', 1.0)
        p('sem_sign', [1.0, 1.0, 1.0, 1.0, -1.0, 1.0])
        p('trim', [0.0] * 6)
        p('walk', False)
        p('forward', 1.0)
        p('turn', 0.0)
        p('freq', 1.2)
        p('roll_amp', 0.10)
        p('step_amp', 0.25)
        p('phase_deg', -90.0)
        p('ankle_ratio', 1.0)
        p('ankle_bias', 0.0)
        p('trajectory_file', '')
        p('max_vx', 1.0)
        p('max_wz', 1.0)
        p('cmd_tau', 0.8)
        p('fwd_min', -1.0)
        p('fwd_max', 1.0)
        p('turn_min', -1.0)
        p('turn_max', 1.0)
        p('k_roll', 0.0)
        p('k_pitch', 0.0)
        p('max_correction', 0.15)
        p('pitch_tau', 0.0)
        p('adaptive', False)
        p('k_phase', 3.0)
        p('k_freq', 0.3)
        p('freq_adapt_max', 0.2)
        p('k_step_pitch', 4.0)
        p('auto_trim', False)
        p('ki_trim', 0.4)
        p('trim_tau', 1.0)
        p('auto_trim_limit', 0.04)
        p('imu_roll_sign', 1.0)
        p('imu_pitch_sign', 1.0)
        p('imu_swap_axes', False)
        p('fall_angle_deg', 40.0)
        p('fall_time', 0.3)
        p('imu_timeout', 0.2)

        self.dt = 1.0 / self.get_parameter('rate').value
        self.pub = self.create_publisher(Float64MultiArray, 'position_controller/commands', 10)
        self.create_subscription(JointState, 'joint_states', self.on_joint_states, 10)
        self.create_subscription(Imu, 'imu/data', self.on_imu, 10)
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10)
        self.create_service(Trigger, 'gait/zero_imu', self.on_zero_imu)
        self.create_service(Trigger, 'gait/estop', self.on_estop)
        self.create_service(Trigger, 'gait/resume', self.on_resume)
        self.halted = None

        self.q_meas = None
        self.q_start = None
        self.t = 0.0
        self.phi = 0.0
        self.env = 0.0
        self.imu_rp = None
        self.imu_ref = (0.0, 0.0)
        self.imu_ref_set = False
        self.imu_time = None
        self.tilt_since = None
        self.pitch_f = 0.0
        self.auto_ankle = 0.0
        self.ep_f = 0.0
        self.gyro_roll = 0.0
        self.psi_off = None
        self.freq_add = 0.0
        self.pitch_step_f = 0.0
        self.cmd = None
        self.cmd_time = None
        self.traj = None
        self.traj_file = ''
        self.kl_f = 0.0
        self.kr_f = 0.0
        self.roll_f = self.get_parameter('roll_amp').value
        self.step_f = self.get_parameter('step_amp').value
        self.legs = None
        self.legs_time = None
        self.state = 'starting'
        self.state_pub = self.create_publisher(String, 'gait/state', 10)
        self.create_subscription(Float64MultiArray, 'gait/leg_gains', self.on_leg_gains, 10)
        self.create_timer(0.2, self.publish_state)

        self.create_timer(self.dt, self.tick)
        self.get_logger().info('Waiting for /joint_states ...')

    def on_joint_states(self, msg):
        idx = {n: i for i, n in enumerate(msg.name)}
        if all(j in idx for j in JOINTS):
            self.q_meas = np.array([msg.position[idx[j]] for j in JOINTS])

    def imu_fresh(self):
        return (self.imu_time is not None and
                (self.get_clock().now() - self.imu_time).nanoseconds * 1e-9
                < self.get_parameter('imu_timeout').value)

    def on_imu(self, msg):
        self.imu_time = self.get_clock().now()
        r, pt = quat_to_roll_pitch(msg.orientation)
        if self.get_parameter('imu_swap_axes').value:
            r, pt = pt, r
        self.imu_rp = (r * self.get_parameter('imu_roll_sign').value,
                       pt * self.get_parameter('imu_pitch_sign').value)
        wx = msg.angular_velocity.y if self.get_parameter('imu_swap_axes').value \
            else msg.angular_velocity.x
        self.gyro_roll = wx * self.get_parameter('imu_roll_sign').value

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
        self.state = f'halted:{reason}'
        self.set_parameters([Parameter('walk', value=False)])
        self.env = 0.0
        self.phi = 0.0

    def on_estop(self, req, res):
        self.halt('estop')
        res.success, res.message = True, 'halted'
        return res

    def on_resume(self, req, res):
        self.halted = None
        self.state = 'starting'
        self.imu_ref_set = False
        self.q_start = None
        self.t = 0.0
        res.success, res.message = True, 'standing up'
        return res

    def load_traj(self, path):
        with open(path) as f:
            rows = list(csv.DictReader(f))
        t = np.array([float(r['t']) for r in rows])
        q = np.array([[float(r.get(j, 0.0)) for j in JOINTS] for r in rows])
        self.get_logger().info(f'Loaded trajectory {path}: {len(t)} samples, {t[-1]:.2f} s')
        return t, q

    def walk_command(self):
        """Returns (walking, k_left, k_right, roll_amp).

        Priority: gait/leg_gains (direct per-leg step gains, e.g. turning arcs from nav_fsm)
        > cmd_vel (forward/steer, clipped to the safe ranges, `turn` param kept as trim)
        > parameters walk/forward/turn."""
        g = lambda n: self.get_parameter(n).value
        now = self.get_clock().now()
        if self.legs_time is not None and (now - self.legs_time).nanoseconds < 5e8:
            kl, kr, roll = self.legs
            return abs(kl) > 0.05 or abs(kr) > 0.05, kl, kr, roll
        walking, fwd, turn = g('walk'), g('forward'), g('turn')
        if self.cmd_time is not None and (now - self.cmd_time).nanoseconds < 5e8:
            fwd, steer = self.cmd
            walking = abs(fwd) > 0.05 or abs(steer) > 0.05
            turn = g('turn') + steer
        fwd = float(np.clip(fwd, g('fwd_min'), g('fwd_max')))
        turn = float(np.clip(turn, g('turn_min'), g('turn_max')))
        return walking, fwd * (1 + turn), fwd * (1 - turn), g('roll_amp')

    def publish_state(self):
        if rclpy.ok():
            self.state_pub.publish(String(data=self.state))

    def on_leg_gains(self, msg):
        g = self.get_parameter
        d = list(msg.data)
        if len(d) < 2:
            return
        roll = d[2] if len(d) > 2 else g('roll_amp').value
        self.legs = (float(np.clip(d[0], -1, 1)), float(np.clip(d[1], -1, 1)),
                     float(np.clip(roll, 0.0, 0.15)))
        self.legs_time = self.get_clock().now()

    def gait_offsets(self, kl, kr, roll_amp):
        """Joint offsets around the stand pose (already scaled by envelope)."""
        g = lambda n: self.get_parameter(n).value
        path = g('trajectory_file')
        if path:
            if path != self.traj_file:
                self.traj, self.traj_file = self.load_traj(path), path
            t, q = self.traj
            tt = self.phi / (2 * math.pi) * t[-1] % t[-1]
            return self.env * np.array([np.interp(tt, t, q[:, i]) for i in range(6)])

        s = self.step_scale()
        kl, kr = float(np.clip(kl * s, -1.5, 1.5)), float(np.clip(kr * s, -1.5, 1.5))
        s_roll = math.sin(self.phi)
        s_step = math.sin(self.phi + math.radians(g('phase_deg')))
        roll = roll_amp * s_roll
        p_l = self.step_f * kl * s_step
        p_r = -self.step_f * kr * s_step
        a_l = -g('ankle_ratio') * p_l
        a_r = -g('ankle_ratio') * p_r
        return self.env * np.array([roll, p_l, a_l, roll, p_r, a_r])

    def phase_rate(self):
        """d(phi)/dt: nominal, or entrained to the measured body sway when adaptive."""
        g = lambda n: self.get_parameter(n).value
        w = 2 * math.pi * g('freq') * (1.0 + self.freq_add)
        if not (g('adaptive') and self.imu_rp is not None and self.imu_ref_set
                and self.imu_fresh() and self.env > 0.5):
            return w
        r = self.imu_rp[0] - self.imu_ref[0]
        amp = math.hypot(r, self.gyro_roll / w)
        if amp < math.radians(0.5):
            return w
        psi = math.atan2(r * w, self.gyro_roll)
        d = math.atan2(math.sin(psi - self.phi), math.cos(psi - self.phi))
        if self.psi_off is None:
            self.psi_off = d
        self.psi_off += self.dt / 5.0 * math.atan2(math.sin(d - self.psi_off), math.cos(d - self.psi_off))
        e = math.atan2(math.sin(d - self.psi_off), math.cos(d - self.psi_off))
        lim = g('freq_adapt_max')
        self.freq_add = float(np.clip(self.freq_add + g('k_freq') * math.sin(e) * self.dt / g('freq'),
                                      -lim, lim))
        self.get_logger().info(f'adaptive: phase err {math.degrees(e):+.0f} deg, freq '
                               f'{g("freq") * (1 + self.freq_add):.2f} Hz', throttle_duration_sec=2.0)
        return w + g('k_phase') * math.sin(e)

    def step_scale(self):
        """Longer steps when pitching forward, shorter when pitching back (adaptive only)."""
        g = lambda n: self.get_parameter(n).value
        if not (g('adaptive') and self.imu_rp is not None and self.imu_ref_set and self.imu_fresh()):
            return 1.0
        ep = self.imu_rp[1] - self.imu_ref[1]
        self.pitch_step_f += self.dt / 0.3 * (ep - self.pitch_step_f)
        return float(np.clip(1.0 + g('k_step_pitch') * self.pitch_step_f, 0.5, 1.5))

    def imu_correction(self):
        g = lambda n: self.get_parameter(n).value
        if (self.imu_rp is None or not self.imu_fresh() or not self.imu_ref_set
                or (g('k_roll') == 0.0 and g('k_pitch') == 0.0)):
            return np.zeros(6)
        c = g('max_correction')
        er = self.imu_rp[0] - self.imu_ref[0]
        ep = self.imu_rp[1] - self.imu_ref[1]
        tau = g('pitch_tau')
        if tau > 0:
            self.ep_f += self.dt / tau * (ep - self.ep_f)
            ep = self.ep_f
        dr = float(np.clip(-g('k_roll') * er, -c, c))
        dp = float(np.clip(-g('k_pitch') * ep, -c, c))
        return np.array([dr, 0.0, dp, dr, 0.0, dp])

    def tick(self):
        if self.q_meas is None:
            return
        standing_up_done = self.t >= self.get_parameter('startup_time').value
        if self.imu_rp is None and standing_up_done:
            self.get_logger().error(
                'NO IMU DATA on imu/data: no fall detection and no IMU feedback '
                '(check MPU6050 wiring / mpu6050_node)', throttle_duration_sec=5.0)
        elif (self.imu_rp is not None and not self.imu_ref_set and standing_up_done
              and self.env == 0.0 and self.halted is None):
            self.imu_ref, self.imu_ref_set = self.imu_rp, True
            self.get_logger().info(f'IMU reference taken while standing: {self.imu_ref}')
        if self.imu_rp is not None and self.halted is None and self.imu_ref_set:
            er = self.imu_rp[0] - self.imu_ref[0]
            ep = self.imu_rp[1] - self.imu_ref[1]
            tilt = math.degrees(math.acos(min(1.0, math.cos(er) * math.cos(ep))))
            if tilt > self.get_parameter('fall_angle_deg').value and self.imu_fresh():
                now = self.get_clock().now()
                self.tilt_since = self.tilt_since or now
                if (now - self.tilt_since).nanoseconds * 1e-9 > self.get_parameter('fall_time').value:
                    self.halt(f'fall, tilt {tilt:.0f} deg')
            else:
                self.tilt_since = None
        if self.halted is not None:
            self.pub.publish(Float64MultiArray(data=self.q_meas.tolist()))
            return
        g = lambda n: self.get_parameter(n).value
        sem_sign = np.array(g('sem_sign'))
        stand = np.array(g('trim'), dtype=float)
        if g('auto_trim'):
            if self.imu_rp is not None and self.imu_ref_set and self.imu_fresh() and self.env > 0.5:
                ep = self.imu_rp[1] - self.imu_ref[1]
                self.pitch_f += self.dt / max(g('trim_tau'), 1e-3) * (ep - self.pitch_f)
                lim = g('auto_trim_limit')
                self.auto_ankle = float(np.clip(self.auto_ankle + g('ki_trim') * self.pitch_f * self.dt,
                                                -lim, lim))
                self.get_logger().info(f'auto-trim ankle {math.degrees(self.auto_ankle):+.2f} deg '
                                       f'(avg pitch {math.degrees(self.pitch_f):+.1f} deg)',
                                       throttle_duration_sec=2.0)
            stand[2] += self.auto_ankle
            stand[5] += self.auto_ankle

        if self.q_start is None:
            self.q_start = self.q_meas.copy()
            self.get_logger().info('Moving to stand pose')
        T = g('startup_time')
        if self.t < T:
            self.t += self.dt
            a = min(self.t / T, 1.0)
            a = a * a * (3 - 2 * a)
            self.publish(self.q_start + a * (stand - self.q_start))
            if self.t >= T:
                if self.imu_rp is not None:
                    self.imu_ref = self.imu_rp
                    self.imu_ref_set = True
                self.get_logger().info('Standing. ros2 param set /gait walk true')
            return

        walking, kl, kr, roll_amp = self.walk_command()
        a = min(1.0, self.dt / max(g('cmd_tau'), 1e-3))
        self.kl_f += a * (kl - self.kl_f)
        self.kr_f += a * (kr - self.kr_f)
        self.roll_f += a * (roll_amp - self.roll_f)
        self.step_f += a * (g('step_amp') - self.step_f)
        rate = self.dt / max(g('ramp_time'), 1e-3)
        self.env = min(1.0, self.env + rate) if walking else max(0.0, self.env - rate)
        if self.env > 0.0:
            self.phi = (self.phi + self.phase_rate() * self.dt) % (2 * math.pi)
        else:
            self.phi = 0.0
            self.psi_off = None
            self.freq_add = 0.0

        ff = self.gait_offsets(self.kl_f, self.kr_f, self.roll_f)
        self.state = 'walking' if self.env > 0 else 'standing'
        if not g('trajectory_file'):
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
