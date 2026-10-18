"""RL policy inference in plain numpy (no torch) + the gait reference it was trained with.

Must match rl/biped_env.py (observation layout, reference, scaling).
"""
import numpy as np

JOINTS = ['l_up', 'l_mid', 'l_down', 'r_up', 'r_mid', 'r_down']
SEM = np.array([1, 1, 1, 1, -1, 1.0])
LIMIT = 0.43
CTRL_DT = 0.02
ACTION_SCALE = 0.25
# rounded feet: only roll-free small fast steps survive (see biped_sim experiments)
REF = dict(freq=1.8, roll=0.0, step=0.05, phase_deg=-90.0)
CMD_MAX = 0.08


def reference(phi, cmd):
    s = min(1.0, cmd / CMD_MAX)
    r = REF['roll'] * np.sin(phi)
    p = REF['step'] * (0.3 + 0.7 * s) * np.sin(phi + np.radians(REF['phase_deg']))
    return SEM * np.array([r, p, -p, r, -p, p])


def gravity_in_frame(qw, qx, qy, qz):
    """World gravity direction (0,0,-1) expressed in the IMU frame, from its orientation."""
    # third row of R(q) = world z-axis expressed in the body frame
    zx = 2 * (qx * qz - qw * qy)
    zy = 2 * (qy * qz + qw * qx)
    zz = 1 - 2 * (qx * qx + qy * qy)
    return -np.array([zx, zy, zz])


def build_obs(grav, gyro, q, dq, last_action, phi, cmd):
    return np.concatenate([grav, gyro, q, 0.1 * np.asarray(dq), last_action,
                           [np.sin(phi), np.cos(phi)], [cmd / CMD_MAX]]).astype(np.float32)


class NumpyPolicy:
    """SB3 MlpPolicy actor (ELU hidden layers) + VecNormalize obs statistics."""

    def __init__(self, path):
        z = np.load(path)
        self.layers = []
        i = 0
        while f'mlp_extractor_policy_net_{i}_weight' in z:
            self.layers.append((z[f'mlp_extractor_policy_net_{i}_weight'],
                                z[f'mlp_extractor_policy_net_{i}_bias']))
            i += 2  # activations sit between the Linear layers
        self.out = (z['action_net_weight'], z['action_net_bias'])
        self.mean, self.var = z['obs_mean'], z['obs_var']
        self.clip, self.eps = float(z['clip_obs']), float(z['epsilon'])

    def __call__(self, obs):
        x = np.clip((obs - self.mean) / np.sqrt(self.var + self.eps), -self.clip, self.clip)
        for w, b in self.layers:
            x = w @ x + b
            x = np.where(x > 0, x, np.expm1(np.minimum(x, 0)))  # ELU
        return np.clip(self.out[0] @ x + self.out[1], -1.0, 1.0)
