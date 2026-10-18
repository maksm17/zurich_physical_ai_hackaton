"""Gymnasium env: biped with the original rounded CAD feet, MuJoCo, sim2real-oriented.

Observation (only what the real robot measures):
  gravity in IMU frame (3), gyro (3), joint pos (6), joint vel (6), last action (6),
  gait clock sin/cos (2), forward velocity command (1)                         = 27
Action: residual joint targets (6) in [-1, 1] * ACTION_SCALE added to a sine stepping
reference (the same family as biped_control/gait_node), sent at 50 Hz like the ROS loop.
"""
import os
import sys

import gymnasium as gym
import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'ws', 'src', 'biped_sim'))
from biped_sim.model import build_model, JOINTS, SERVO  # noqa: E402

URDF = os.path.join(HERE, 'biped.urdf')
sys.path.insert(0, os.path.join(HERE, '..', 'ws', 'src', 'biped_control'))
# single source of truth shared with the robot-side node
from biped_control.rl_policy import (ACTION_SCALE, CMD_MAX, CTRL_DT, LIMIT, REF,  # noqa: E402
                                     reference)
CMD_RANGE = (0.0, CMD_MAX)
PUSH_FORCE = 0.15


class BipedEnv(gym.Env):
    metadata = {'render_modes': []}

    def __init__(self, sole=None, randomize=True, max_steps=1000, seed=None):
        self.model, _ = build_model(URDF, sole=sole)
        self.data = mujoco.MjData(self.model)
        m = self.model
        self.n_sub = int(round(CTRL_DT / m.opt.timestep))
        self.qadr = np.array([m.joint(j).qposadr[0] for j in JOINTS])
        self.vadr = np.array([m.joint(j).dofadr[0] for j in JOINTS])
        self.act = np.array([m.actuator(j).id for j in JOINTS])
        self.site = m.site('imu').id
        g = m.sensor('imu_gyro')
        self.gyro_adr = g.adr[0]
        self.randomize = randomize
        self.max_steps = max_steps
        self.nominal = dict(mass=m.body_mass.copy(), friction=m.geom_friction.copy(),
                            damping=m.dof_damping.copy(), frictionloss=m.dof_frictionloss.copy())
        self.observation_space = gym.spaces.Box(-np.inf, np.inf, (27,), np.float32)
        self.action_space = gym.spaces.Box(-1.0, 1.0, (6,), np.float32)
        self.np_random_ = np.random.default_rng(seed)

    # ------------------------------------------------------------------ randomization
    def _randomize(self):
        m, rng = self.model, self.np_random_
        n = self.nominal
        if not self.randomize:
            kp, kv, tmax = SERVO['kp'], SERVO['kv'], SERVO['max_torque']
            m.body_mass[:] = n['mass']
            m.geom_friction[:] = n['friction']
            self.delay, self.q_offset, self.push_force = 1, np.zeros(6), 0.0
        else:
            kp, kv, tmax = rng.uniform(5, 12), rng.uniform(0.05, 0.3), rng.uniform(1.8, 3.0)
            m.body_mass[:] = n['mass'] * rng.uniform(0.85, 1.2, n['mass'].shape)
            m.geom_friction[:] = n['friction']
            m.geom_friction[:, 0] = rng.uniform(0.4, 1.2)
            m.dof_damping[self.vadr] = rng.uniform(0.02, 0.1, 6)
            m.dof_frictionloss[self.vadr] = rng.uniform(0.0, 0.03, 6)
            self.delay = int(rng.integers(0, 2))            # 0-20 ms
            self.q_offset = rng.uniform(-0.03, 0.03, 6)      # calibration error
            self.push_force = PUSH_FORCE
        for a in self.act:
            m.actuator_gainprm[a, 0] = kp
            m.actuator_biasprm[a, 1] = -kp
            m.actuator_biasprm[a, 2] = -kv
            m.actuator_forcerange[a] = [-tmax, tmax]

    # ------------------------------------------------------------------ gym API
    def reset(self, seed=None, options=None):
        if seed is not None:
            self.np_random_ = np.random.default_rng(seed)
        rng = self.np_random_
        self._randomize()
        m, d = self.model, self.data
        mujoco.mj_resetData(m, d)
        yaw = rng.uniform(-np.pi, np.pi)
        d.qpos[3:7] = [np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)]
        d.qpos[self.qadr] += rng.uniform(-0.03, 0.03, 6)
        mujoco.mj_forward(m, d)
        self.cmd = rng.uniform(*CMD_RANGE) if self.randomize else CMD_RANGE[1] * 0.75
        self.phi = rng.uniform(0, 2 * np.pi) if self.randomize else 0.0
        self.last_action = np.zeros(6)
        self.queue = [np.zeros(6)] * (self.delay + 1)
        self.steps = 0
        self.start_xy = d.qpos[:2].copy()
        self.heading0 = yaw
        self.next_push = rng.uniform(2, 5)
        return self._obs(), {}

    def _obs(self):
        d, rng = self.data, self.np_random_
        R = d.site_xmat[self.site].reshape(3, 3)
        grav = R.T @ np.array([0, 0, -1.0])
        gyro = d.sensordata[self.gyro_adr:self.gyro_adr + 3]
        q = d.qpos[self.qadr] + self.q_offset
        dq = d.qvel[self.vadr]
        if self.randomize:
            grav = grav + rng.normal(0, 0.02, 3)
            gyro = gyro + rng.normal(0, 0.05, 3)
            q = q + rng.normal(0, 0.01, 6)
            dq = dq + rng.normal(0, 0.1, 6)
        return np.concatenate([grav, gyro, q, 0.1 * dq, self.last_action,
                               [np.sin(self.phi), np.cos(self.phi)], [self.cmd / CMD_RANGE[1]]]
                              ).astype(np.float32)

    def step(self, action):
        m, d, rng = self.model, self.data, self.np_random_
        action = np.clip(action, -1, 1)
        self.phi = (self.phi + 2 * np.pi * REF['freq'] * CTRL_DT) % (2 * np.pi)
        target = np.clip(reference(self.phi, self.cmd) + ACTION_SCALE * action, -LIMIT, LIMIT)
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
            mujoco.mj_step(m, d)
        self.steps += 1

        # base velocity in heading frame
        R = d.xmat[1].reshape(3, 3)
        yaw = np.arctan2(R[1, 0], R[0, 0])
        c, s = np.cos(yaw), np.sin(yaw)
        vx = c * d.qvel[0] + s * d.qvel[1]
        vy = -s * d.qvel[0] + c * d.qvel[1]
        w = d.qvel[3:6]  # body frame
        upright = R[2, 2]
        tau = d.actuator_force[self.act]

        # alive bonus must dominate: with net-negative steps PPO learns to fall early
        penalty = (1.0 * (1 - upright)
                   + 0.01 * (w[0] ** 2 + w[1] ** 2) + 0.02 * w[2] ** 2
                   + 0.005 * np.sum((action - self.last_action) ** 2)
                   + 0.0005 * np.sum(tau ** 2))
        reward = (1.0
                  + 0.5 * np.exp(-((vx - self.cmd) ** 2 + vy ** 2) / 0.03 ** 2)
                  - min(penalty, 0.9))
        self.last_action = action
        fell = upright < np.cos(np.radians(45))
        truncated = self.steps >= self.max_steps
        info = {}
        if fell or truncated:
            dxy = d.qpos[:2] - self.start_xy
            info['dist'] = float(np.cos(self.heading0) * dxy[0] + np.sin(self.heading0) * dxy[1])
            info['survived'] = float(not fell)
        return self._obs(), float(reward), bool(fell), bool(truncated), info
