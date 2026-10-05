#!/usr/bin/python3
"""FASE 2 — auditoría del dataset (solo lectura). Imprime JSON con hallazgos."""
import json
import os
import sys
from collections import Counter, defaultdict

import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
from common import DATASET, BAG_DB, OUT, open_bag, topics, iter_msgs, load_map, classify  # noqa

res = {}

# ---------------- MAPA ----------------
meta, pgm = load_map()
m = {'yaml': meta, 'pgm_w': pgm['w'], 'pgm_h': pgm['h'], 'maxval': pgm['maxval'],
     'payload_bytes': pgm['payload_bytes'], 'expected_payload': pgm['w'] * pgm['h']}
m['size_m'] = [pgm['w'] * meta['resolution'], pgm['h'] * meta['resolution']]
o = meta['origin']
m['extent_x'] = [o[0], o[0] + m['size_m'][0]]
m['extent_y'] = [o[1], o[1] + m['size_m'][1]]
m['unique_values'] = {int(k): int(v) for k, v in zip(*np.unique(pgm['arr'], return_counts=True))}
m['image_exists'] = os.path.exists(os.path.join(DATASET, meta['image']))
res['map'] = m

# ---------------- PBSTREAM ----------------
pb = os.path.join(DATASET, 'map.pbstream')
with open(pb, 'rb') as f:
    head = f.read(16)
res['pbstream'] = {'size': os.path.getsize(pb), 'head_hex': head.hex()}
# Formato Cartographer: uint64 magic 0x7b1d1f7b5bf501db (little endian) + chunks gzip
res['pbstream']['magic_ok'] = head[:8] == (0x7b1d1f7b5bf501db).to_bytes(8, 'little')
# chunk 1: uint64 size + gzip data (1f8b)
res['pbstream']['first_chunk_size'] = int.from_bytes(head[8:16], 'little')
with open(pb, 'rb') as f:
    f.seek(16)
    res['pbstream']['first_chunk_gzip'] = f.read(2) == b'\x1f\x8b'
# recorrer cadena de chunks para verificar que termina exactamente en EOF
n_chunks, pos, size = 0, 8, res['pbstream']['size']
with open(pb, 'rb') as f:
    while pos < size:
        f.seek(pos)
        ln = int.from_bytes(f.read(8), 'little')
        f.seek(pos + 8)
        if f.read(2) != b'\x1f\x8b':
            break
        pos += 8 + ln
        n_chunks += 1
res['pbstream']['chunks'] = n_chunks
res['pbstream']['chain_ends_at_eof'] = pos == size

# ---------------- ROSBAG ----------------
con = open_bag()
tp = topics(con)
bag = {}
for name, (tid, typ) in sorted(tp.items()):
    ts = np.array([r[0] for r in con.execute(
        'SELECT timestamp FROM messages WHERE topic_id=? ORDER BY timestamp', (tid,))], dtype=np.int64)
    d = {'type': typ, 'count': int(ts.size)}
    if ts.size:
        d['t0'] = int(ts[0]); d['t1'] = int(ts[-1])
        d['span_s'] = (ts[-1] - ts[0]) / 1e9
    if ts.size > 1:
        dt = np.diff(ts) / 1e9
        d['hz_mean'] = (ts.size - 1) / d['span_s']
        d['hz_median'] = 1.0 / float(np.median(dt)) if np.median(dt) > 0 else None
        d['dt_max_s'] = float(dt.max())
        d['gaps_gt_0.5s'] = int((dt > 0.5).sum())
        d['gaps_gt_1s'] = int((dt > 1.0).sum())
    bag[name] = d
bag_t0 = min(v['t0'] for v in bag.values() if 't0' in v)
bag_t1 = max(v['t1'] for v in bag.values() if 't1' in v)
res['bag'] = {'topics': bag, 't0': bag_t0, 't1': bag_t1, 'duration_s': (bag_t1 - bag_t0) / 1e9,
              'db_size': os.path.getsize(BAG_DB)}

# ---- contenido de mensajes clave ----
from sensor_msgs.msg import LaserScan, Imu  # noqa
from nav_msgs.msg import Odometry  # noqa
from tf2_msgs.msg import TFMessage  # noqa
from geometry_msgs.msg import Twist  # noqa

for sname in ('/scan', '/scan_filtered'):
    if sname not in tp:
        continue
    valid, first = [], None
    for i, (ts, msg) in enumerate(iter_msgs(con, sname, LaserScan)):
        if first is None:
            first = {'frame': msg.header.frame_id, 'angle_min': msg.angle_min, 'angle_max': msg.angle_max,
                     'angle_inc': msg.angle_increment, 'range_min': msg.range_min,
                     'range_max': msg.range_max, 'n_ranges': len(msg.ranges),
                     'n_intensities': len(msg.intensities)}
        if i % 50 == 0:
            r = np.asarray(msg.ranges)
            ok = np.isfinite(r) & (r >= msg.range_min) & (r <= msg.range_max)
            valid.append(ok.mean())
    first['valid_ratio_sampled_mean'] = float(np.mean(valid))
    first['valid_ratio_sampled_min'] = float(np.min(valid))
    bag[sname]['content'] = first

# odom: frames + stamp vs bag time
frames, lag = set(), []
for i, (ts, msg) in enumerate(iter_msgs(con, '/odom', Odometry)):
    frames.add((msg.header.frame_id, msg.child_frame_id))
    if i % 100 == 0:
        lag.append(ts / 1e9 - (msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9))
bag['/odom']['content'] = {'frames': sorted(frames), 'recv_minus_stamp_s_median': float(np.median(lag)),
                           'recv_minus_stamp_s_max': float(np.max(lag))}

imu_frames, acc_norm = set(), []
for i, (ts, msg) in enumerate(iter_msgs(con, '/imu', Imu)):
    if i % 200 == 0:
        imu_frames.add(msg.header.frame_id)
        a = msg.linear_acceleration
        acc_norm.append(np.sqrt(a.x ** 2 + a.y ** 2 + a.z ** 2))
bag['/imu']['content'] = {'frames': sorted(imu_frames), 'acc_norm_median': float(np.median(acc_norm))}

pairs = Counter()
for ts, msg in iter_msgs(con, '/tf', TFMessage):
    for t in msg.transforms:
        pairs[(t.header.frame_id, t.child_frame_id)] += 1
bag['/tf']['content'] = {'pairs': {f'{a}->{b}': c for (a, b), c in sorted(pairs.items())}}
static = []
for ts, msg in iter_msgs(con, '/tf_static', TFMessage):
    for t in msg.transforms:
        tr = t.transform.translation
        static.append({'parent': t.header.frame_id, 'child': t.child_frame_id,
                       'xyz': [round(tr.x, 4), round(tr.y, 4), round(tr.z, 4)]})
bag['/tf_static']['content'] = static

cmd = [(ts, m.linear.x, m.linear.y, m.angular.z) for ts, m in iter_msgs(con, '/cmd_vel', Twist)]
if cmd:
    c = np.array(cmd, dtype=float)
    bag['/cmd_vel']['content'] = {'vx_range': [c[:, 1].min(), c[:, 1].max()],
                                  'vy_range': [c[:, 2].min(), c[:, 2].max()],
                                  'wz_range': [c[:, 3].min(), c[:, 3].max()],
                                  'n_zero': int(((c[:, 1] == 0) & (c[:, 3] == 0)).sum())}

try:
    from limo_msgs.msg import LimoStatus
    st = [(ts, m.vehicle_state, m.control_mode, m.battery_voltage, m.error_code, m.motion_mode)
          for ts, m in iter_msgs(con, '/limo_status', LimoStatus)]
    s = np.array(st, dtype=float)
    bag['/limo_status']['content'] = {
        'vehicle_state': sorted(set(s[:, 1].astype(int).tolist())),
        'control_mode': sorted(set(s[:, 2].astype(int).tolist())),
        'battery_v_first_last_min_max': [s[0, 3], s[-1, 3], s[:, 3].min(), s[:, 3].max()],
        'error_codes': sorted(set(s[:, 4].astype(int).tolist())),
        'motion_mode': sorted(set(s[:, 5].astype(int).tolist()))}
except ImportError as e:
    bag['/limo_status']['content'] = {'error': f'limo_msgs no disponible: {e}'}

with open(os.path.join(OUT, 'dataset_audit.json'), 'w') as f:
    json.dump(res, f, indent=1, default=str)
print(json.dumps(res, indent=1, default=str))
