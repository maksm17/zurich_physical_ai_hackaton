#!/usr/bin/env python3
"""Analyse a robot recording (scripts/robot.sh record / record-stop).

  python3 scripts/analyze_bag.py bags/run_123456      (needs ROS 2 Jazzy sourced, re-execs itself)

Prints the gait state timeline, body tilt (IMU converted to base axes: the board is mounted
rotated 180 deg about z), the fall (time, direction) and how well the servos tracked the
commands; writes <bag>/plot.png.
"""
import math
import os
import sys

if 'ROS_DISTRO' not in os.environ:
    os.execvp('bash', ['bash', '-c', 'source /opt/ros/jazzy/setup.bash && exec python3 "$0" "$@"',
                       os.path.abspath(__file__)] + sys.argv[1:])

import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py

JOINTS = ['l_up', 'l_mid', 'l_down', 'r_up', 'r_mid', 'r_down']
IMU_SIGN = -1.0


def read(path):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path, storage_id=''),
           rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    out = {k: [] for k in types}
    while r.has_next():
        topic, data, t = r.read_next()
        out[topic].append((t * 1e-9, deserialize_message(data, get_message(types[topic]))))
    return out


def rp(o):
    r = math.atan2(2 * (o.w * o.x + o.y * o.z), 1 - 2 * (o.x * o.x + o.y * o.y))
    p = math.asin(max(-1, min(1, 2 * (o.w * o.y - o.z * o.x))))
    return math.degrees(r) * IMU_SIGN, math.degrees(p) * IMU_SIGN


def main():
    path = sys.argv[1].rstrip('/')
    d = read(path)
    t0 = min(v[0][0] for v in d.values() if v)
    print(f'== {path}: ' + ', '.join(f'{k} {len(v)}' for k, v in d.items() if v))

    for topic in ('/gait/state', '/nav/state'):
        last = None
        for t, m in d.get(topic, []):
            s = m.data.split()[0] if topic == '/nav/state' else m.data
            if s != last:
                print(f'  {t - t0:6.2f}s  {topic.split("/")[1]:4s} -> {m.data}')
                last = s

    imu = d.get('/imu/data', [])
    if imu:
        ti = np.array([t - t0 for t, _ in imu])
        tilt = np.array([rp(m.orientation) for _, m in imu])
        stand = [t - t0 for t, m in d.get('/gait/state', []) if m.data == 'standing']
        ref_t = stand[0] if stand else ti[0]
        ref = tilt[np.searchsorted(ti, ref_t).clip(0, len(ti) - 1)]
        rel = tilt - ref
        k = int(np.argmax(np.abs(rel).max(axis=1)))
        print(f'  body tilt reference (standing) roll {ref[0]:+.1f} pitch {ref[1]:+.1f} deg')
        print(f'  max tilt at {ti[k]:.2f}s: roll {rel[k, 0]:+.1f} deg '
              f'({"right side down" if rel[k, 0] > 0 else "left side down"}), '
              f'pitch {rel[k, 1]:+.1f} deg ({"forward" if rel[k, 1] > 0 else "backward"})')
        halt = [t - t0 for t, m in d.get('/gait/state', []) if m.data.startswith('halted')]
        if halt:
            j = np.searchsorted(ti, halt[0] - 1.0)
            print(f'  last second before halt ({halt[0]:.2f}s), every 0.2 s [roll, pitch]:')
            for i in range(j, min(len(ti), np.searchsorted(ti, halt[0]) + 1), 20):
                print(f'     {ti[i]:6.2f}s  roll {rel[i, 0]:+6.1f}  pitch {rel[i, 1]:+6.1f}')

    cmd, js = d.get('/position_controller/commands', []), d.get('/joint_states', [])
    if cmd and js:
        tc = np.array([t - t0 for t, _ in cmd]); qc = np.degrees([list(m.data) for _, m in cmd])
        tj = np.array([t - t0 for t, _ in js])
        names = list(js[0][1].name)
        qj = np.degrees([[m.position[names.index(j)] for j in JOINTS] for _, m in js])
        print('  joint     cmd range [deg]      rms err   max err  (meas vs cmd 80 ms earlier)')
        for i, j in enumerate(JOINTS):
            ref = np.interp(tj - 0.08, tc, qc[:, i])
            e = qj[:, i] - ref
            print(f'  {j:7s} {qc[:, i].min():+7.1f}..{qc[:, i].max():+6.1f}   {np.sqrt(np.mean(e**2)):6.2f}  '
                  f'{np.abs(e).max():7.2f}')
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(4, 1, figsize=(12, 11), sharex=True)
        if imu:
            ax[0].plot(ti, rel[:, 0], label='roll (+ right side down)')
            ax[0].plot(ti, rel[:, 1], label='pitch (+ forward)')
            ax[0].axhline(40, ls=':', c='r'); ax[0].axhline(-40, ls=':', c='r')
            ax[0].set_ylabel('body tilt [deg]'); ax[0].legend(loc='upper left')
        if cmd and js:
            for a, idx in zip(ax[1:], ((0, 3), (1, 4), (2, 5))):
                for i in idx:
                    a.plot(tc, qc[:, i], '--', label=f'{JOINTS[i]} cmd')
                    a.plot(tj, qj[:, i], label=f'{JOINTS[i]} meas')
                a.set_ylabel('[deg]'); a.legend(loc='upper left', ncol=4)
        for t, m in d.get('/gait/state', []):
            pass
        ax[-1].set_xlabel('time [s]')
        fig.tight_layout(); fig.savefig(os.path.join(path, 'plot.png'), dpi=90)
        print(f'  plot: {os.path.join(path, "plot.png")}')
    except Exception as e:
        print(f'  (no plot: {e})')


if __name__ == '__main__':
    main()
