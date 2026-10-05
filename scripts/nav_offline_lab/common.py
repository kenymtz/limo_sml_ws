"""Utilidades comunes NAVIGATION_OFFLINE_LAB_R1 (solo lectura, offline).

El bag se abre con sqlite en modo read-only + immutable: no se crean
-journal/-wal ni se modifica el original.
"""
import os
import sqlite3

import numpy as np
import yaml

WS = os.path.expanduser('~/limo_sml_ws')
DATASET = os.path.join(WS, 'results/maps/arena_20260918_1900')
BAG_DB = os.path.join(DATASET, 'bag/bag_0.db3')
OUT = os.path.join(WS, 'results/navigation_offline_lab')

# Footprint real (CLAUDE.md) + padding
FOOTPRINT = np.array([[-0.17, -0.16], [-0.17, 0.16], [0.22, 0.16], [0.22, -0.16]])
PADDING = 0.02


def open_bag():
    return sqlite3.connect(f'file:{BAG_DB}?mode=ro&immutable=1', uri=True)


def topics(con):
    return {name: (tid, typ) for tid, name, typ in
            con.execute('SELECT id, name, type FROM topics')}


def iter_msgs(con, topic, cls):
    from rclpy.serialization import deserialize_message
    tid = topics(con)[topic][0]
    for ts, data in con.execute(
            'SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp', (tid,)):
        yield ts, deserialize_message(data, cls)


def load_map():
    with open(os.path.join(DATASET, 'map.yaml')) as f:
        meta = yaml.safe_load(f)
    img = read_pgm(os.path.join(DATASET, meta['image']))
    return meta, img


def read_pgm(path):
    with open(path, 'rb') as f:
        data = f.read()
    # cabecera P5: magic, width height, maxval (sin comentarios esperados, se toleran)
    tokens, idx = [], 0
    while len(tokens) < 4:
        while data[idx:idx + 1].isspace():
            idx += 1
        if data[idx:idx + 1] == b'#':
            idx = data.index(b'\n', idx) + 1
            continue
        start = idx
        while not data[idx:idx + 1].isspace():
            idx += 1
        tokens.append(data[start:idx])
    idx += 1
    assert tokens[0] == b'P5', tokens[0]
    w, h, maxval = int(tokens[1]), int(tokens[2]), int(tokens[3])
    arr = np.frombuffer(data[idx:idx + w * h], dtype=np.uint8)
    return {'w': w, 'h': h, 'maxval': maxval, 'payload_bytes': len(data) - idx,
            'arr': arr.reshape(h, w) if arr.size == w * h else None}


def classify(img_arr, meta):
    """Clasificación igual a nav2_map_server (trinary). Devuelve 0 libre, 100 ocupado, -1 desconocido."""
    p = img_arr.astype(float) / 255.0
    occ = p if meta.get('negate', 0) else 1.0 - p
    out = np.full(img_arr.shape, -1, dtype=np.int8)
    out[occ > meta['occupied_thresh']] = 100
    out[occ < meta['free_thresh']] = 0
    return out


def yaw_from_q(q):
    return np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
