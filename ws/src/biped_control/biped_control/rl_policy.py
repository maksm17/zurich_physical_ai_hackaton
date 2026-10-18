"""RL policy inference in plain numpy (no torch) + the gait reference it was trained with.

Must match rl/biped_env.py (observation layout, reference, scaling).
"""
import numpy as np

JOINTS = ['l_up', 'l_mid', 'l_down', 'r_up', 'r_mid', 'r_down']
SEM = np.array([1, 1, 1, 1, -1, 1.0])
LIMIT = 0.43
CTRL_DT = 0.02
ACTION_SCALE = 0.25
REF = dict(freq=1.8, roll=0.08, step=0.25, phase_deg=-90.0)
CMD_MAX = 0.08
WZ_MAX = 0.3


def reference(phi, cmd, ref=None):
    ref = ref or REF
    s = min(1.0, cmd / CMD_MAX)
    r = ref['roll'] * np.sin(phi)
    p = ref['step'] * (0.3 + 0.7 * s) * np.sin(phi + np.radians(ref['phase_deg']))
    return SEM * np.array([r, p, -p, r, -p, p])


def gravity_in_frame(qw, qx, qy, qz):
    """World gravity direction (0,0,-1) expressed in the IMU frame, from its orientation."""
    zx = 2 * (qx * qz - qw * qy)
    zy = 2 * (qy * qz + qw * qx)
    zz = 1 - 2 * (qx * qx + qy * qy)
    return -np.array([zx, zy, zz])


def build_obs(grav, gyro, q, dq, last_action, phi, cmd, wz_cmd):
    return np.concatenate([grav, gyro, q, 0.1 * np.asarray(dq), last_action,
                           [np.sin(phi), np.cos(phi)], [cmd / CMD_MAX, wz_cmd / WZ_MAX]]
                          ).astype(np.float32)


class NumpyPolicy:
    """SB3 MlpPolicy actor (ELU hidden layers) + VecNormalize obs statistics."""

    def __init__(self, path):
        z = np.load(path)
        self.layers = []
        i = 0
        while f'mlp_extractor_policy_net_{i}_weight' in z:
            self.layers.append((z[f'mlp_extractor_policy_net_{i}_weight'],
                                z[f'mlp_extractor_policy_net_{i}_bias']))
            i += 2
        self.out = (z['action_net_weight'], z['action_net_bias'])
        self.mean, self.var = z['obs_mean'], z['obs_var']
        self.clip, self.eps = float(z['clip_obs']), float(z['epsilon'])
        self.ref = (dict(zip(('freq', 'roll', 'step', 'phase_deg'), map(float, z['ref'])))
                    if 'ref' in z else dict(REF))
        self.cut = float(z['cut']) if 'cut' in z else None
        self.action_scale = float(z['action_scale']) if 'action_scale' in z else ACTION_SCALE
        self.sole = [float(v) for v in z['sole']] if 'sole' in z and float(z['sole'][0]) > 0 else None

    def __call__(self, obs):
        x = np.clip((obs - self.mean) / np.sqrt(self.var + self.eps), -self.clip, self.clip)
        for w, b in self.layers:
            x = w @ x + b
            x = np.where(x > 0, x, np.expm1(np.minimum(x, 0)))
        return np.clip(self.out[0] @ x + self.out[1], -1.0, 1.0)
