"""Evaluate a trained policy with the numpy inference used on the robot.

  .venv-rl/bin/python rl/eval.py round_v1                 # stats, nominal + randomized physics
  .venv-rl/bin/python rl/eval.py round_v1 --viewer        # watch it (run outside the snap VS Code
                                                          # terminal or use scripts/sim.sh env fix)
  .venv-rl/bin/python rl/eval.py round_v1 --check         # numpy vs torch output match
"""
import argparse
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..', 'ws', 'src', 'biped_control'))
from biped_env import BipedEnv
from biped_control.rl_policy import NumpyPolicy


def episode(env, policy, seed, viewer=None, options=None):
    obs, _ = env.reset(seed=seed, options=options)
    total = 0.0
    while True:
        obs, r, term, trunc, info = env.step(policy(obs))
        total += r
        if viewer is not None:
            viewer.sync()
            time.sleep(0.02)
            if not viewer.is_running():
                return None
        if term or trunc:
            return env.steps * 0.02, info['dist'], total


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('run')
    ap.add_argument('--episodes', type=int, default=10)
    ap.add_argument('--viewer', action='store_true')
    ap.add_argument('--check', action='store_true')
    ap.add_argument('--flat', action='store_true')
    ap.add_argument('--wz', type=float, default=0.0, help='yaw rate command for --viewer')
    a = ap.parse_args()
    run = os.path.join(HERE, 'runs', a.run)
    policy = NumpyPolicy(os.path.join(run, 'policy.npz'))
    sole = policy.sole or ([0.04, 0.045, 0.004] if a.flat else None)
    cut = policy.cut if policy.cut is not None else 0.0
    kw = dict(sole=sole, cut=cut, ref=policy.ref, action_scale=policy.action_scale)

    if a.check:
        import torch
        from stable_baselines3 import PPO
        ckpts = sorted(f for f in os.listdir(run) if f.endswith('.zip'))
        model = PPO.load(os.path.join(run, ckpts[-1]), device='cpu')
        rng = np.random.default_rng(0)
        obs = rng.normal(size=(5, policy.mean.shape[0])).astype(np.float32)
        x = np.clip((obs - policy.mean) / np.sqrt(policy.var + policy.eps), -10, 10)
        with torch.no_grad():
            t = model.policy.get_distribution(torch.tensor(x)).distribution.mean.numpy()
        n = np.array([policy(o) for o in obs])
        print('checkpoint', ckpts[-1], 'max |numpy - torch| =', np.abs(np.clip(t, -1, 1) - n).max())
        print('(policy.npz is written every 1M steps, checkpoints every 2M: small mismatch is ok '
              'if they come from different steps)')
        return

    if a.viewer:
        import mujoco.viewer
        env = BipedEnv(**kw, randomize=False)
        with mujoco.viewer.launch_passive(env.model, env.data) as v:
            v.cam.distance, v.cam.elevation = 0.9, -15
            for s in range(100):
                res = episode(env, policy, s, v, dict(wz=a.wz))
                if res is None:
                    break
                print(f'episode {s}: {res[0]:.1f} s, {res[1]:+.3f} m')
        return

    for name, rand, opts in (('nominal', False, dict(cmd=0.06, wz=0.0)),
                             ('randomized', True, None),
                             ('turn left', False, dict(cmd=0.04, wz=0.25)),
                             ('turn right', False, dict(cmd=0.04, wz=-0.25))):
        env = BipedEnv(**kw, randomize=rand)
        res = []
        for s in range(a.episodes):
            r = episode(env, policy, 100 + s, options=opts)
            res.append((*r, env.wz_sum / env.steps))
        res = np.array(res)
        print(f'{name:10s}: survive {res[:, 0].mean():5.1f} s (max 20), '
              f'full {np.mean(res[:, 0] >= 19.99) * 100:3.0f}%, '
              f'forward {res[:, 1].mean():+.2f} m, mean yaw rate {res[:, 3].mean():+.2f} rad/s')


if __name__ == '__main__':
    main()
