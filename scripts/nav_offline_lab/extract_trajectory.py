#!/usr/bin/python3
"""FASE 3 — trayectoria real desde el bag (solo lectura).

- odom: /odom (odom->base_link, 50 Hz)
- map : T(map->odom) de /tf (Cartographer) compuesto con odom->base_link,
        tomando el map->odom más reciente con stamp <= stamp odom.
Salida: real_trajectory.csv (todas las muestras de /odom), real_trajectory.png,
        trajectory_metrics.json, cmd_vs_odom.png
"""
import json
import os
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa

sys.path.insert(0, os.path.dirname(__file__))
from common import OUT, open_bag, iter_msgs, yaw_from_q, load_map, classify  # noqa
from nav_msgs.msg import Odometry  # noqa
from tf2_msgs.msg import TFMessage  # noqa
from geometry_msgs.msg import Twist  # noqa

V_STOP, W_STOP, STOP_MIN_S = 0.01, 0.02, 1.0

con = open_bag()


def stamp(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


odom = np.array([(stamp(m.header), m.pose.pose.position.x, m.pose.pose.position.y,
                  yaw_from_q(m.pose.pose.orientation), m.twist.twist.linear.x,
                  m.twist.twist.linear.y, m.twist.twist.angular.z)
                 for _, m in iter_msgs(con, '/odom', Odometry)])
mo = []
for _, m in iter_msgs(con, '/tf', TFMessage):
    for t in m.transforms:
        if t.header.frame_id == 'map' and t.child_frame_id == 'odom':
            tr = t.transform
            mo.append((stamp(t.header), tr.translation.x, tr.translation.y, yaw_from_q(tr.rotation)))
mo = np.array(mo)
mo = mo[np.argsort(mo[:, 0], kind='stable')]
cmd = np.array([(ts * 1e-9, m.linear.x, m.angular.z) for ts, m in iter_msgs(con, '/cmd_vel', Twist)])
bag_t0 = con.execute('SELECT MIN(timestamp) FROM messages').fetchone()[0] * 1e-9

t, ox, oy, oyaw, vx, vy, wz = odom.T
# componer map->odom (último conocido)
idx = np.searchsorted(mo[:, 0], t, side='right') - 1
has_map = idx >= 0
idx = np.clip(idx, 0, None)
tx, ty, th = mo[idx, 1], mo[idx, 2], mo[idx, 3]
mx = tx + np.cos(th) * ox - np.sin(th) * oy
my = ty + np.sin(th) * ox + np.cos(th) * oy
myaw = np.arctan2(np.sin(th + oyaw), np.cos(th + oyaw))
mo_age = t - mo[idx, 0]

trel = t - t[0]
with open(os.path.join(OUT, 'real_trajectory.csv'), 'w') as f:
    f.write('t_rel_s,stamp_s,odom_x,odom_y,odom_yaw,map_x,map_y,map_yaw,v_x,w_z,map_odom_age_s\n')
    for i in range(len(t)):
        f.write(f'{trel[i]:.4f},{t[i]:.6f},{ox[i]:.5f},{oy[i]:.5f},{oyaw[i]:.5f},'
                f'{mx[i]:.5f},{my[i]:.5f},{myaw[i]:.5f},{vx[i]:.5f},{wz[i]:.5f},{mo_age[i]:.4f}\n')

# ---------- métricas ----------
dt = np.diff(t)
step_odom = np.hypot(np.diff(ox), np.diff(oy))
step_map = np.hypot(np.diff(mx), np.diff(my))
dyaw = np.abs(np.angle(np.exp(1j * np.diff(oyaw))))
moving = (np.abs(vx) > V_STOP) | (np.abs(wz) > W_STOP)


def segments(mask):
    """Devuelve lista (i0, i1) de tramos True."""
    d = np.diff(np.r_[0, mask.astype(int), 0])
    return list(zip(np.where(d == 1)[0], np.where(d == -1)[0] - 1))


mov_segs = segments(moving)
first_mov = mov_segs[0][0] if mov_segs else None
last_mov = mov_segs[-1][1] if mov_segs else None
stop_segs = [(a, b) for a, b in segments(~moving)
             if first_mov is not None and a > first_mov and b < last_mov and t[b] - t[a] >= STOP_MIN_S]
# velocidad por diferenciación (ventana 0.2 s) para contrastar el twist
k = 10
v_diff = np.hypot(ox[k:] - ox[:-k], oy[k:] - oy[:-k]) / (t[k:] - t[:-k])

act = slice(first_mov, last_mov + 1) if first_mov is not None else slice(0, 0)
active_dur = t[last_mov] - t[first_mov] if first_mov is not None else 0.0
moving_time = float(np.sum(dt[moving[:-1]]))
dist_odom = float(step_odom.sum())
metrics = {
    'n_samples': int(len(t)),
    'bag_duration_s': float(t[-1] - t[0]),
    'first_motion_rel_s': float(t[first_mov] - t[0]) if first_mov is not None else None,
    'last_motion_rel_s': float(t[last_mov] - t[0]) if last_mov is not None else None,
    'active_window_s': float(active_dur),
    'moving_time_s': moving_time,
    'distance_odom_m': dist_odom,
    'distance_map_m': float(step_map.sum()),
    'mean_speed_active_window_mps': dist_odom / active_dur if active_dur else None,
    'mean_speed_while_moving_mps': dist_odom / moving_time if moving_time else None,
    'v_max_twist_mps': float(np.abs(vx).max()),
    'v_p99_twist_mps': float(np.percentile(np.abs(vx[act]), 99)),
    'v_max_diff_0.2s_mps': float(v_diff.max()),
    'vy_max_twist_mps': float(np.abs(vy).max()),
    'w_max_twist_rps': float(np.abs(wz).max()),
    'w_p99_twist_rps': float(np.percentile(np.abs(wz[act]), 99)),
    'yaw_accumulated_abs_rad': float(dyaw.sum()),
    'yaw_accumulated_abs_turns': float(dyaw.sum() / (2 * np.pi)),
    'yaw_net_rad': float(np.unwrap(oyaw)[-1] - np.unwrap(oyaw)[0]),
    'n_motion_segments': len(mov_segs),
    'n_stops_ge_1s_inside_active_window': len(stop_segs),
    'stop_durations_s_summary': ([float(np.min([t[b] - t[a] for a, b in stop_segs])),
                                  float(np.median([t[b] - t[a] for a, b in stop_segs])),
                                  float(np.max([t[b] - t[a] for a, b in stop_segs]))] if stop_segs else None),
    'map_odom_samples': int(len(mo)),
    'map_odom_age_s_median_max': [float(np.median(mo_age[has_map])), float(np.max(mo_age[has_map]))],
    'odom_before_first_map_odom': int((~has_map).sum()),
    'odom_start_pose': [float(ox[0]), float(oy[0]), float(oyaw[0])],
    'odom_end_pose': [float(ox[-1]), float(oy[-1]), float(oyaw[-1])],
    'map_start_pose': [float(mx[0]), float(my[0]), float(myaw[0])],
    'map_end_pose': [float(mx[-1]), float(my[-1]), float(myaw[-1])],
    'map_bbox': [float(mx.min()), float(mx.max()), float(my.min()), float(my.max())],
}
# deriva odom vs map: corrección total aplicada por Cartographer (map->odom) inicio vs fin
metrics['map_odom_first'] = mo[0, 1:].tolist()
metrics['map_odom_last'] = mo[-1, 1:].tolist()
metrics['map_odom_translation_range_m'] = float(np.hypot(mo[:, 1] - mo[0, 1], mo[:, 2] - mo[0, 2]).max())
metrics['map_odom_yaw_range_rad'] = float(np.abs(np.angle(np.exp(1j * (mo[:, 3] - mo[0, 3])))).max())

# ---------- cmd_vel vs odom ----------
cmp = {}
if len(cmd):
    cmd_rel = cmd[:, 0] - t[0]
    cmp['n_cmd'] = int(len(cmd))
    cmp['cmd_window_rel_s'] = [float(cmd_rel[0]), float(cmd_rel[-1])]
    cmp['cmd_distinct'] = sorted({(round(a, 3), round(b, 3)) for a, b in cmd[:, 1:]})
    # Hipótesis a contrastar: la base mantiene el último comando (sin timeout).
    # Se compara el twist de odom con el último cmd vigente.
    j = np.searchsorted(cmd[:, 0], t, side='right') - 1
    valid = j >= 0
    cv = np.where(valid, cmd[np.clip(j, 0, None), 1], np.nan)
    cw = np.where(valid, cmd[np.clip(j, 0, None), 2], np.nan)
    # excluir 0.5 s tras cada cambio de comando (transitorio)
    since = t - cmd[np.clip(j, 0, None), 0]
    steady = valid & (since > 0.5)
    ev, ew = vx - cv, wz - cw
    cmp['steady_samples'] = int(steady.sum())
    cmp['v_err_mean_median_p95'] = [float(np.nanmean(ev[steady])), float(np.nanmedian(ev[steady])),
                                    float(np.nanpercentile(np.abs(ev[steady]), 95))]
    cmp['w_err_mean_median_p95'] = [float(np.nanmean(ew[steady])), float(np.nanmedian(ew[steady])),
                                    float(np.nanpercentile(np.abs(ew[steady]), 95))]
    # ratio odom/cmd por tipo de comando
    per = {}
    for a, b in cmp['cmd_distinct']:
        sel = steady & np.isclose(cv, a, atol=1e-3) & np.isclose(cw, b, atol=1e-3)
        if sel.sum() > 25:
            per[f'v={a},w={b}'] = {'samples': int(sel.sum()), 'dur_s': float(sel.sum() / 50.0),
                                   'odom_v_median': float(np.median(vx[sel])),
                                   'odom_w_median': float(np.median(wz[sel]))}
    cmp['per_command'] = per
    # movimiento fuera del efecto de cmd_vel grabado
    no_cmd = ~valid
    cmp['moving_before_first_cmd_s'] = float(np.sum(dt[(moving & no_cmd)[:-1]]))
    last_zero = cmd[(cmd[:, 1] == 0) & (cmd[:, 2] == 0)]
    after_zero = np.zeros_like(moving)
    if len(last_zero):
        after_zero = valid & np.isclose(cv, 0) & np.isclose(cw, 0) & (since > 1.0)
    cmp['moving_while_last_cmd_zero_s'] = float(np.sum(dt[(moving & after_zero)[:-1]]))
    # latencia: primer cmd no nulo -> primer moving
    nz = np.where((cmd[:, 1] != 0) | (cmd[:, 2] != 0))[0]
    if len(nz):
        t_cmd = cmd[nz[0], 0]
        after = np.where(moving & (t >= t_cmd))[0]
        cmp['first_nonzero_cmd_rel_s'] = float(t_cmd - t[0])
        cmp['latency_first_cmd_to_motion_s'] = float(t[after[0]] - t_cmd) if len(after) else None
metrics['cmd_vs_odom'] = cmp

with open(os.path.join(OUT, 'trajectory_metrics.json'), 'w') as f:
    json.dump(metrics, f, indent=1)
print(json.dumps(metrics, indent=1))

# ---------- figuras ----------
meta, pgm = load_map()
occ = classify(pgm['arr'], meta)
res_, (x0, y0) = meta['resolution'], meta['origin'][:2]
extent = [x0, x0 + pgm['w'] * res_, y0, y0 + pgm['h'] * res_]
shade = np.where(occ == 100, 0.0, np.where(occ == 0, 1.0, 0.75))

fig, ax = plt.subplots(1, 2, figsize=(16, 7.5))
ax[0].imshow(shade, cmap='gray', vmin=0, vmax=1, extent=extent, origin='upper')
sc = ax[0].scatter(mx[::10], my[::10], c=trel[::10], s=1.5, cmap='viridis')
ax[0].plot(mx[0], my[0], 'go', ms=8, label='inicio')
ax[0].plot(mx[-1], my[-1], 'rs', ms=8, label='fin')
ax[0].set_title('Trayectoria frame map (map->odom Cartographer ∘ /odom) sobre map.pgm')
ax[0].set_xlabel('x [m]'); ax[0].set_ylabel('y [m]'); ax[0].legend(loc='upper right')
plt.colorbar(sc, ax=ax[0], label='t [s]')
ax[1].plot(ox, oy, 'b-', lw=0.6, label='odom (sin corrección)')
ax[1].plot(mx, my, 'r-', lw=0.6, alpha=0.7, label='map (corregida SLAM)')
ax[1].set_aspect('equal'); ax[1].grid(alpha=0.3); ax[1].legend()
ax[1].set_title('odom vs map'); ax[1].set_xlabel('x [m]'); ax[1].set_ylabel('y [m]')
fig.tight_layout(); fig.savefig(os.path.join(OUT, 'real_trajectory.png'), dpi=110)

fig, ax = plt.subplots(3, 1, figsize=(14, 9), sharex=True)
ax[0].plot(trel, vx, lw=0.5, label='odom v_x')
ax[1].plot(trel, wz, lw=0.5, label='odom w_z')
if len(cmd):
    ax[0].step(cmd[:, 0] - t[0], cmd[:, 1], where='post', color='r', lw=0.8, label='cmd_vel v_x')
    ax[1].step(cmd[:, 0] - t[0], cmd[:, 2], where='post', color='r', lw=0.8, label='cmd_vel w_z')
ax[2].plot(mo[:, 0] - t[0], np.hypot(mo[:, 1] - mo[0, 1], mo[:, 2] - mo[0, 2]), lw=0.6,
           label='|Δ map->odom| [m]')
for a in ax:
    a.grid(alpha=0.3); a.legend(loc='upper right')
ax[0].set_ylabel('m/s'); ax[1].set_ylabel('rad/s'); ax[2].set_ylabel('m'); ax[2].set_xlabel('t [s]')
fig.tight_layout(); fig.savefig(os.path.join(OUT, 'cmd_vs_odom.png'), dpi=100)
