#!/usr/bin/python3
"""Diagnóstico adicional (solo lectura): actualización de pose odom, distancia
en frame map robusta (decimada), tracking cmd_vel fresco vs odom."""
import json, os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(__file__))
from common import OUT, open_bag, iter_msgs  # noqa
from geometry_msgs.msg import Twist  # noqa

tr = np.loadtxt(os.path.join(OUT, 'real_trajectory.csv'), delimiter=',', skiprows=1)
t, ox, oy, oyaw, mx, my, myaw, v, w = tr[:, 1], tr[:, 2], tr[:, 3], tr[:, 4], tr[:, 5], tr[:, 6], tr[:, 7], tr[:, 8], tr[:, 9]
moving = (np.abs(v) > 0.01) | (np.abs(w) > 0.02)
res = {}
# 1) ¿cambia la pose odom en cada mensaje mientras hay movimiento?
dpos = np.hypot(np.diff(ox), np.diff(oy)) + np.abs(np.diff(oyaw))
m = moving[1:]
res['odom_pose_unchanged_while_twist_moving_ratio'] = float((dpos[m] == 0).mean())
chg = np.where((dpos > 0) & m)[0]
res['odom_pose_update_interval_s_median_while_moving'] = float(np.median(np.diff(t[chg]))) if len(chg) > 2 else None
# integración del twist (unicycle) vs pose odom publicada
dt = np.diff(t)
th = np.cumsum(np.r_[oyaw[0], w[:-1] * dt])
ix = np.cumsum(np.r_[0, v[:-1] * np.cos(th[:-1]) * dt]); iy = np.cumsum(np.r_[0, v[:-1] * np.sin(th[:-1]) * dt])
res['twist_integrated_distance_m'] = float(np.sum(np.abs(v[:-1]) * dt))
res['twist_integrated_yaw_abs_rad'] = float(np.sum(np.abs(w[:-1]) * dt))
res['twist_integrated_end_xy_yaw'] = [float(ix[-1]), float(iy[-1]), float(np.angle(np.exp(1j * th[-1])))]
# 2) distancia map decimada (1 s y 2 s) para filtrar jitter de map->odom
for step in (50, 100):
    res[f'distance_map_decimated_{step // 50}s_m'] = float(np.sum(np.hypot(np.diff(mx[::step]), np.diff(my[::step]))))
    res[f'distance_odom_decimated_{step // 50}s_m'] = float(np.sum(np.hypot(np.diff(ox[::step]), np.diff(oy[::step]))))
# 3) tracking con comando fresco (<0.15 s)
con = open_bag()
cmd = np.array([(ts * 1e-9, c.linear.x, c.angular.z) for ts, c in iter_msgs(con, '/cmd_vel', Twist)])
j = np.searchsorted(cmd[:, 0], t, side='right') - 1
ok = j >= 0; jj = np.clip(j, 0, None); age = t - cmd[jj, 0]
cmd_dt = np.diff(cmd[:, 0])
res['cmd_interarrival_s_median_p90_max'] = [float(np.median(cmd_dt)), float(np.percentile(cmd_dt, 90)), float(cmd_dt.max())]
for lo, hi in ((0.0, 0.15), (0.15, 0.3), (0.3, 0.6), (0.6, 2.0)):
    sel = ok & (age >= lo) & (age < hi) & ((cmd[jj, 1] != 0) | (cmd[jj, 2] != 0))
    if sel.sum():
        res[f'nonzero_cmd_age_{lo}-{hi}s'] = {
            'samples': int(sel.sum()),
            'odom_v_over_cmd_v_median': float(np.median(v[sel & (cmd[jj, 1] != 0)] / cmd[jj, 1][sel & (cmd[jj, 1] != 0)])) if (sel & (cmd[jj, 1] != 0)).any() else None,
            'odom_w_over_cmd_w_median': float(np.median(w[sel & (cmd[jj, 2] != 0)] / cmd[jj, 2][sel & (cmd[jj, 2] != 0)])) if (sel & (cmd[jj, 2] != 0)).any() else None,
            'moving_ratio': float(moving[sel].mean())}
# 4) ventanas de movimiento
d = np.diff(np.r_[0, moving.astype(int), 0]); s, e = np.where(d == 1)[0], np.where(d == -1)[0] - 1
t0 = t[0]
res['motion_segments_rel_s'] = [[round(float(t[a] - t0), 2), round(float(t[b] - t0), 2)] for a, b in zip(s, e)]
res['motion_time_before_2700s'] = float(sum(t[b] - t[a] for a, b in zip(s, e) if t[a] - t0 < 2700))
res['distance_odom_before_2700s_m'] = float(np.sum(np.hypot(np.diff(ox), np.diff(oy))[(t[1:] - t0) < 2700]))
print(json.dumps(res, indent=1))
json.dump(res, open(os.path.join(OUT, 'odom_tracking.json'), 'w'), indent=1)
