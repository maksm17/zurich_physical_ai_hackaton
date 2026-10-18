"""PPO training for BipedEnv.

  .venv-rl/bin/python rl/train.py --steps 30000000 --name round_v1
Outputs in rl/runs/<name>/: progress.csv, checkpoints, final model, policy.npz (numpy, for the RPi).
"""
import argparse
import os
import sys

import numpy as np
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback, CheckpointCallback
from stable_baselines3.common.logger import configure
from functools import partial

from stable_baselines3.common.vec_env import VecMonitor, VecNormalize

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from biped_env import BipedEnv  # noqa: E402
from vec import BatchedSubprocVecEnv  # noqa: E402


class EpisodeStatsCallback(BaseCallback):
    """Logs mean forward distance and survival rate of finished episodes."""

    def _on_rollout_end(self):
        buf = self.model.ep_info_buffer
        if buf:
            self.logger.record('biped/dist_mean', float(np.mean([e['dist'] for e in buf])))
            self.logger.record('biped/survived', float(np.mean([e['survived'] for e in buf])))

    def _on_step(self):
        return True


class SaveNormCallback(BaseCallback):
    def __init__(self, path, every):
        super().__init__()
        self.path, self.every, self.last = path, every, 0

    def _on_step(self):
        if self.num_timesteps - self.last >= self.every:
            self.last = self.num_timesteps
            self.training_env.save(os.path.join(self.path, 'vecnormalize.pkl'))
            export_numpy(self.model, self.training_env, os.path.join(self.path, 'policy.npz'))
        return True


def export_numpy(model, venv, path):
    """MLP weights + observation normalization -> npz; inference is a few matmuls in numpy."""
    sd = model.policy.state_dict()
    arrays = {k.replace('.', '_'): v.cpu().numpy() for k, v in sd.items()
              if k.startswith('mlp_extractor.policy_net') or k.startswith('action_net')}
    arrays['obs_mean'] = venv.obs_rms.mean.astype(np.float32)
    arrays['obs_var'] = venv.obs_rms.var.astype(np.float32)
    arrays['clip_obs'] = np.float32(venv.clip_obs)
    arrays['epsilon'] = np.float32(venv.epsilon)
    np.savez(path, **arrays)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--steps', type=int, default=30_000_000)
    ap.add_argument('--workers', type=int, default=7)
    ap.add_argument('--per-worker', type=int, default=6)
    ap.add_argument('--name', default='round_v1')
    ap.add_argument('--flat', action='store_true', help='flat box soles instead of CAD feet')
    a = ap.parse_args()

    out = os.path.join(HERE, 'runs', a.name)
    os.makedirs(out, exist_ok=True)
    torch.set_num_threads(4)  # PPO update in the main process
    sole = [0.04, 0.045, 0.004] if a.flat else None

    n_envs = a.workers * a.per_worker
    venv = BatchedSubprocVecEnv(
        [partial(BipedEnv, sole=sole, seed=1000 + i) for i in range(n_envs)], a.workers)
    venv = VecMonitor(venv, info_keywords=('dist', 'survived'))
    venv = VecNormalize(venv, norm_obs=True, norm_reward=True, clip_obs=10.0, gamma=0.99)

    model = PPO('MlpPolicy', venv, n_steps=256, batch_size=n_envs * 64, n_epochs=5,
                learning_rate=3e-4, gamma=0.99, gae_lambda=0.95, clip_range=0.2,
                ent_coef=0.0, vf_coef=0.5, max_grad_norm=0.5,
                policy_kwargs=dict(net_arch=dict(pi=[128, 128], vf=[256, 256]),
                                   activation_fn=torch.nn.ELU, log_std_init=-1.0),
                device='cpu', verbose=0)
    model.set_logger(configure(out, ['stdout', 'csv']))
    cbs = [CheckpointCallback(2_000_000 // n_envs, out, name_prefix='ckpt'),
           SaveNormCallback(out, 1_000_000), EpisodeStatsCallback()]
    model.learn(total_timesteps=a.steps, callback=cbs)
    model.save(os.path.join(out, 'final'))
    venv.save(os.path.join(out, 'vecnormalize.pkl'))
    export_numpy(model, venv, os.path.join(out, 'policy.npz'))
    print('done ->', out)


if __name__ == '__main__':
    main()
