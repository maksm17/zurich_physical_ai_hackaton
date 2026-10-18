"""Training progress:  python3 rl/status.py [run_name]"""
import csv
import os
import sys

name = sys.argv[1] if len(sys.argv) > 1 else 'filed_v1'
path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'runs', name, 'progress.csv')
rows = list(csv.DictReader(open(path)))
keys = ['time/total_timesteps', 'time/fps', 'rollout/ep_len_mean', 'rollout/ep_rew_mean',
        'biped/survived', 'biped/vx_err', 'biped/yaw_err']
print(f'{"steps":>10} {"fps":>6} {"ep_len":>7} {"ep_rew":>8} {"surv%":>6} {"vx_err":>7} {"yaw_err":>7}')
for r in rows[::max(1, len(rows) // 12)] + [rows[-1]]:
    v = [float(r.get(k) or 'nan') for k in keys]
    print(f'{v[0]:10.0f} {v[1]:6.0f} {v[2]:7.1f} {v[3]:8.1f} {100*v[4]:6.1f} {v[5]:7.3f} {v[6]:7.3f}')
print(f'ep_len 1000 = survives 20 s; {len(rows)} updates logged')
