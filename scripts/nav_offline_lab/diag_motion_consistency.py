#!/usr/bin/python3
"""Diagnóstico de consistencia de movimiento (solo lectura).

Contrasta la odometría con una señal independiente: el cambio entre scans
consecutivos del LiDAR (/scan). Si el robot está quieto, los scans deberían
ser casi idénticos. También analiza estampas y saltos de map->odom.
"""
import json
import os
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa

sys.path.insert(0, os.path.dirname(__file__))
from common import OUT, open_bag, iter_msgs, yaw_from_q  # noqa
from sensor_msgs.msg import LaserScan  # noqa
from tf2_msgs.msg import TFMessage  # noqa
from geometry_msgs.msg import Twist  # noqa

con = open_bag()
t0 = con.execute('SELECT MIN(timestamp) FROM messages').fetchone()[0] * 1e-9

# --- scans: diferencia mediana entre scans separados 1 s (10 scans) ---
ts, R = [], []
for tb, m in iter_msgs(con, '/scan', LaserScan):
    r = np.asarray(m.ranges, dtype=np.float32)
    r[~np.isfinite(r) | (r < m.range_min) | (r > 12.0)] = np.nan
    ts.append(tb * 1e-9 - t0); R.append(r)
ts, R = np.array(ts), np.array(R)
lag = 10
d = np.abs(R[lag:] - R[:-lag])
both = np.isfinite(d)
scan_change = np.where(both.sum(1) > 30, np.nanmedian(np.where(both, d, np.nan), axis=1), np.nan)
tc = ts[lag:]
mean_range = np.nanmedian(R, axis=1)

# --- map->odom: stamp vs receive, saltos ---
mo = []
for tb, m in iter_msgs(con, '/tf', TFMessage):
    for t in m.transforms:
        if t.header.frame_id == 'map':
            s = t.header.stamp.sec + t.header.stamp.nanosec * 1e-9
            mo.append((tb * 1e-9 - t0, s - t0, t.transform.translation.x,
                       t.transform.translation.y, yaw_from_q(t.transform.rotation)))
mo = np.array(mo)
recv, st = mo[:, 0], mo[:, 1]
stamp_back = np.diff(st) < 0
step = np.hypot(np.diff(mo[:, 2]), np.diff(mo[:, 3]))
dst = np.diff(st)

# --- odom moving flag para comparar ---
traj = np.loadtxt(os.path.join(OUT, 'real_trajectory.csv'), delimiter=',', skiprows=1)
todom = traj[:, 1] - t0
v, w = traj[:, 8], traj[:, 9]
moving = (np.abs(v) > 0.01) | (np.abs(w) > 0.02)
mov_at_scan = np.interp(tc, todom, moving.astype(float)) > 0.5

cmd = np.array([(tb * 1e-9 - t0, m.linear.x, m.angular.z) for tb, m in iter_msgs(con, '/cmd_vel', Twist)])

thr = 0.02  # 2 cm mediana de cambio a 1 s
res = {
    'scan_change_median_m_when_odom_static': float(np.nanmedian(scan_change[~mov_at_scan])),
    'scan_change_p95_m_when_odom_static': float(np.nanpercentile(scan_change[~mov_at_scan], 95)),
    'scan_change_median_m_when_odom_moving': float(np.nanmedian(scan_change[mov_at_scan])) if mov_at_scan.any() else None,
    'scan_changed_gt_2cm_s_total': float(np.sum(scan_change > thr) * 0.1),
    'scan_changed_gt_2cm_while_odom_static_s': float(np.sum((scan_change > thr) & ~mov_at_scan) * 0.1),
    'scan_changed_gt_5cm_while_odom_static_s': float(np.sum((scan_change > 0.05) & ~mov_at_scan) * 0.1),
    'map_odom_msgs': int(len(mo)),
    'map_odom_stamp_minus_recv_median_s': float(np.median(st - recv)),
    'map_odom_stamp_minus_recv_min_max_s': [float((st - recv).min()), float((st - recv).max())],
    'map_odom_stamp_goes_backwards_count': int(stamp_back.sum()),
    'map_odom_stamp_repeated_count': int((dst == 0).sum()),
    'map_odom_step_gt_1cm': int((step > 0.01).sum()),
    'map_odom_step_gt_5cm': int((step > 0.05).sum()),
    'map_odom_step_max_m': float(step.max()),
    'map_odom_step_sum_m': float(step.sum()),
}
# intervalos (1 s) con scan cambiando y odom quieta
bad = (scan_change > 0.05) & ~mov_at_scan
edges = np.diff(np.r_[0, bad.astype(int), 0])
starts, ends = np.where(edges == 1)[0], np.where(edges == -1)[0] - 1
res['intervals_scan_moving_odom_static'] = [[round(float(tc[a]), 1), round(float(tc[b]), 1)]
                                            for a, b in zip(starts, ends) if tc[b] - tc[a] >= 2.0][:40]
print(json.dumps(res, indent=1))
with open(os.path.join(OUT, 'motion_consistency.json'), 'w') as f:
    json.dump(res, f, indent=1)

fig, ax = plt.subplots(4, 1, figsize=(14, 11), sharex=True)
ax[0].plot(tc, scan_change, lw=0.4); ax[0].set_ylabel('Δscan med. 1 s [m]'); ax[0].set_ylim(0, 0.5)
ax[0].set_title('Cambio del LiDAR (señal independiente de odom)')
ax[1].plot(todom, v, lw=0.5, label='odom v'); ax[1].plot(todom, w, lw=0.5, label='odom w')
if len(cmd):
    ax[1].plot(cmd[:, 0], cmd[:, 1], 'r.', ms=3, label='cmd v')
    ax[1].plot(cmd[:, 0], cmd[:, 2], 'm.', ms=3, label='cmd w')
ax[1].legend(loc='upper right'); ax[1].set_ylabel('vel')
ax[2].plot(ts, mean_range, lw=0.5); ax[2].set_ylabel('rango mediano [m]')
ax[3].plot(recv, np.hypot(mo[:, 2], mo[:, 3]), lw=0.5); ax[3].set_ylabel('|map->odom| [m]')
ax[3].set_xlabel('t desde inicio bag [s]')
for a in ax:
    a.grid(alpha=0.3)
fig.tight_layout(); fig.savefig(os.path.join(OUT, 'motion_consistency.png'), dpi=100)
