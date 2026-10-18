"""VecEnv with several envs per worker process: one pipe round-trip per worker per step
instead of per env (SubprocVecEnv IPC dominated: ~1k steps/s per env)."""
import multiprocessing as mp

import numpy as np
from stable_baselines3.common.vec_env.base_vec_env import VecEnv


def _worker(remote, make_fns):
    envs = [f() for f in make_fns]
    while True:
        cmd, data = remote.recv()
        if cmd == 'step':
            obs, rews, dones, infos = [], [], [], []
            for env, a in zip(envs, data):
                o, r, term, trunc, info = env.step(a)
                done = term or trunc
                if done:
                    info['terminal_observation'] = o
                    info['TimeLimit.truncated'] = trunc and not term
                    o, _ = env.reset()
                obs.append(o)
                rews.append(r)
                dones.append(done)
                infos.append(info)
            remote.send((np.stack(obs), np.array(rews, np.float32), np.array(dones), infos))
        elif cmd == 'reset':
            remote.send(np.stack([env.reset()[0] for env in envs]))
        elif cmd == 'close':
            remote.close()
            return


class BatchedSubprocVecEnv(VecEnv):

    def __init__(self, make_fns, n_workers):
        chunks = [list(c) for c in np.array_split(np.array(make_fns, dtype=object), n_workers)]
        self.sizes = [len(c) for c in chunks]
        ctx = mp.get_context('fork')
        self.remotes, self.procs = [], []
        for c in chunks:
            parent, child = ctx.Pipe()
            p = ctx.Process(target=_worker, args=(child, c), daemon=True)
            p.start()
            child.close()
            self.remotes.append(parent)
            self.procs.append(p)
        probe = make_fns[0]()
        super().__init__(len(make_fns), probe.observation_space, probe.action_space)
        probe.close()

    def reset(self):
        for r in self.remotes:
            r.send(('reset', None))
        return np.concatenate([r.recv() for r in self.remotes])

    def step_async(self, actions):
        i = 0
        for r, n in zip(self.remotes, self.sizes):
            r.send(('step', actions[i:i + n]))
            i += n

    def step_wait(self):
        res = [r.recv() for r in self.remotes]
        obs = np.concatenate([x[0] for x in res])
        rews = np.concatenate([x[1] for x in res])
        dones = np.concatenate([x[2] for x in res])
        infos = [i for x in res for i in x[3]]
        return obs, rews, dones, infos

    def close(self):
        for r in self.remotes:
            r.send(('close', None))
        for p in self.procs:
            p.join(timeout=2)

    def get_attr(self, attr_name, indices=None):
        return [None] * self.num_envs

    def set_attr(self, attr_name, value, indices=None):
        pass

    def env_method(self, method_name, *args, indices=None, **kwargs):
        return [None] * self.num_envs

    def env_is_wrapped(self, wrapper_class, indices=None):
        return [False] * self.num_envs

    def seed(self, seed=None):
        return [None] * self.num_envs
