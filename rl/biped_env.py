"""Gymnasium env: biped in MuJoCo, sim2real-oriented (feet: filed CAD / CAD dome / flat / rocker).

Observation (only what the real robot measures; layout = biped_control.rl_policy.build_obs):
  gravity in IMU frame (3), gyro (3), joint pos (6), joint vel (6), last action (6),
  gait clock sin/cos (2), forward speed command (1), yaw rate command (1)          = 28
Action: residual joint targets (6) in [-1, 1] * ACTION_SCALE added to the feedforward
reference gait (same as gait_node), sent at 50 Hz.
"""
import os
import sys

import gymnasium as gym
import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'ws', 'src', 'biped_sim'))
sys.path.insert(0, os.path.join(HERE, '..', 'ws', 'src', 'biped_control'))
from biped_sim.model import build_model, JOINTS, SERVO
from biped_sim.servo import REAL, ServoModel
from biped_control.rl_policy import (ACTION_SCALE, build_obs, CMD_MAX, CTRL_DT,
                                     LIMIT, REF, reference, WZ_MAX)

URDF = os.path.join(HERE, 'biped.urdf')
PUSH_FORCE = 0.25


class BipedEnv(gym.Env):
    metadata = {'render_modes': []}

    def __init__(self, sole=None, cut=0.0, ref=None, randomize=True, max_steps=1000,
                 seed=None, servo_model=True, action_scale=ACTION_SCALE, physics_dt=0.002,
                 turn_cmd=True):
        self.model, _ = build_model(URDF, sole=sole, cut=cut)
        self.model.opt.timestep = physics_dt
        self.turn_cmd = turn_cmd
        self.data = mujoco.MjData(self.model)
        self.ref = dict(ref or REF)
        self.action_scale = action_scale
        m = self.model
        self.n_sub = int(round(CTRL_DT / m.opt.timestep))
        self.qadr = np.array([m.joint(j).qposadr[0] for j in JOINTS])
        self.vadr = np.array([m.joint(j).dofadr[0] for j in JOINTS])
        self.act = np.array([m.actuator(j).id for j in JOINTS])
        self.site = m.site('imu').id
        self.gyro_adr = m.sensor('imu_gyro').adr[0]
        self.randomize = randomize
        self.max_steps = max_steps
        self.nominal = dict(mass=m.body_mass.copy(), friction=m.geom_friction.copy())
        self.observation_space = gym.spaces.Box(-np.inf, np.inf, (28,), np.float32)
        self.action_space = gym.spaces.Box(-1.0, 1.0, (6,), np.float32)
        self.rng = np.random.default_rng(seed)
        self.servo = ServoModel(6, dt=m.opt.timestep, **REAL) if servo_model else None

    def _randomize(self):
        m, rng, n = self.model, self.rng, self.nominal
        if not self.randomize:
            kp, kv, tmax = SERVO['kp'], SERVO['kv'], SERVO['max_torque']
            m.body_mass[:] = n['mass']
            m.geom_friction[:] = n['friction']
            self.delay, self.q_offset, self.push_force = 0, np.zeros(6), 0.0
            self.lean = 0.0
            self.imu_lag, self.joint_lag = 1, 1
            if self.servo is not None:
                self.servo.set(**REAL)
        else:
            kp, kv, tmax = rng.uniform(5, 12), rng.uniform(0.05, 0.3), rng.uniform(1.8, 3.0)
            m.body_mass[:] = n['mass'] * rng.uniform(0.85, 1.2, n['mass'].shape)
            m.geom_friction[:] = n['friction']
            m.geom_friction[:, 0] = rng.uniform(0.4, 1.2)
            m.dof_damping[self.vadr] = rng.uniform(0.02, 0.1, 6)
            m.dof_frictionloss[self.vadr] = rng.uniform(0.0, 0.03, 6)
            self.delay = 0
            self.imu_lag = int(rng.integers(0, 3))
            self.joint_lag = int(rng.integers(0, 2))
            if self.servo is not None:
                self.servo.set(delay=rng.uniform(0.02, 0.07), tau=rng.uniform(0.005, 0.03),
                               deadband=rng.uniform(0.003, 0.012))
            self.q_offset = rng.uniform(-0.03, 0.03, 6)
            self.lean = rng.uniform(-0.03, 0.03)
            self.push_force = PUSH_FORCE
        for a in self.act:
            m.actuator_gainprm[a, 0] = kp
            m.actuator_biasprm[a, 1] = -kp
            m.actuator_biasprm[a, 2] = -kv
            m.actuator_forcerange[a] = [-tmax, tmax]

    def reset(self, seed=None, options=None):
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        rng = self.rng
        self._randomize()
        m, d = self.model, self.data
        mujoco.mj_resetData(m, d)
        yaw = rng.uniform(-np.pi, np.pi)
        d.qpos[3:7] = [np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)]
        d.qpos[self.qadr] += rng.uniform(-0.03, 0.03, 6)
        mujoco.mj_forward(m, d)
        if self.servo is not None:
            self.servo.reset(d.qpos[self.qadr])
        if self.randomize:
            self.cmd = rng.uniform(0.0, CMD_MAX)
            self.wz_cmd = 0.0 if (rng.random() < 0.4 or not self.turn_cmd) else \
                rng.uniform(-WZ_MAX, WZ_MAX)
            self.phi = rng.uniform(0, 2 * np.pi)
        else:
            self.cmd, self.wz_cmd, self.phi = 0.06, 0.0, 0.0
        if options:
            self.cmd = options.get('cmd', self.cmd)
            self.wz_cmd = options.get('wz', self.wz_cmd)
        self.last_action = np.zeros(6)
        self.queue = [np.zeros(6)] * (self.delay + 1)
        self.steps = 0
        self.wz_sum = 0.0
        self.vx_sum = 0.0
        self.start_xy = d.qpos[:2].copy()
        self.heading0 = yaw
        self.next_push = rng.uniform(2, 5)
        self.sense_hist = []
        return self._obs(), {}

    def _obs(self):
        d, rng = self.data, self.rng
        R = d.site_xmat[self.site].reshape(3, 3)
        grav = R.T @ np.array([0, 0, -1.0])
        gyro = d.sensordata[self.gyro_adr:self.gyro_adr + 3].copy()
        q = d.qpos[self.qadr] + self.q_offset
        dq = d.qvel[self.vadr].copy()
        self.sense_hist.append((grav, gyro, q, dq))
        self.sense_hist = self.sense_hist[-3:]
        grav, gyro = self.sense_hist[max(0, len(self.sense_hist) - 1 - self.imu_lag)][:2]
        q, dq = self.sense_hist[max(0, len(self.sense_hist) - 1 - self.joint_lag)][2:]
        if self.randomize:
            grav = grav + rng.normal(0, 0.02, 3)
            gyro = gyro + rng.normal(0, 0.05, 3)
            q = q + rng.normal(0, 0.01, 6)
            dq = dq + rng.normal(0, 0.1, 6)
        return build_obs(grav, gyro, q, dq, self.last_action, self.phi, self.cmd, self.wz_cmd)

    def step(self, action):
        m, d, rng = self.model, self.data, self.rng
        action = np.clip(action, -1, 1)
        self.phi = (self.phi + 2 * np.pi * self.ref['freq'] * CTRL_DT) % (2 * np.pi)
        target = reference(self.phi, self.cmd, self.ref) + self.action_scale * action
        target[2] += self.lean
        target[5] += self.lean
        target = np.clip(target, -LIMIT, LIMIT)
        if self.servo is None:
            self.queue.append(target)
            d.ctrl[self.act] = self.queue.pop(0)

        t = self.steps * CTRL_DT
        d.xfrc_applied[1, :] = 0
        if self.push_force > 0 and t >= self.next_push:
            if t < self.next_push + 0.1:
                ang = rng.uniform(0, 2 * np.pi)
                d.xfrc_applied[1, :2] = self.push_force * rng.uniform(0.3, 1) * np.array(
                    [np.cos(ang), np.sin(ang)])
            else:
                self.next_push = t + rng.uniform(2, 5)
        for _ in range(self.n_sub):
            if self.servo is not None:
                d.ctrl[self.act] = self.servo.update(target, d.qpos[self.qadr])
            mujoco.mj_step(m, d)
        self.steps += 1

        R = d.xmat[1].reshape(3, 3)
        yaw = np.arctan2(R[1, 0], R[0, 0])
        c, s = np.cos(yaw), np.sin(yaw)
        vx = c * d.qvel[0] + s * d.qvel[1]
        vy = -s * d.qvel[0] + c * d.qvel[1]
        w = d.qvel[3:6]
        upright = R[2, 2]
        self.wz_sum += w[2]
        self.vx_sum += vx
        tau = d.actuator_force[self.act]

        penalty = (1.0 * (1 - upright)
                   + 0.01 * (w[0] ** 2 + w[1] ** 2)
                   + 0.005 * np.sum((action - self.last_action) ** 2)
                   + 0.05 * np.sum(action ** 2)
                   + 0.0005 * np.sum(tau ** 2))
        reward = (1.0
                  + 1.5 * np.exp(-((vx - self.cmd) ** 2 + vy ** 2) / 0.04 ** 2)
                  + 1.0 * np.exp(-(w[2] - self.wz_cmd) ** 2 / 0.08 ** 2)
                  - min(penalty, 0.9))
        self.last_action = action
        fell = upright < np.cos(np.radians(45))
        truncated = self.steps >= self.max_steps
        info = {}
        if fell or truncated:
            dxy = d.qpos[:2] - self.start_xy
            info['dist'] = float(np.cos(self.heading0) * dxy[0] + np.sin(self.heading0) * dxy[1])
            info['survived'] = float(not fell)
            info['yaw_err'] = float(abs(self.wz_sum / self.steps - self.wz_cmd))
            info['vx_err'] = float(abs(self.vx_sum / self.steps - self.cmd))
        return self._obs(), float(reward), bool(fell), bool(truncated), info
