#!/usr/bin/python3
"""Helper de save_map_checked.sh. Subcomandos:

  snapshot  --topic /map --out f.npz [--timeout 15]
      Toma UNA instantánea del OccupancyGrid (QoS transient_local). Solo suscribe.
  load-yaml --raw raw.yaml --out map.yaml --free 0.10
      Genera el yaml de CARGA a partir del yaml bruto del saver (solo cambia free_thresh).
  check     --pgm map.pgm --yaml map.yaml --snap a.npz [--snap b.npz] --save-free 0.25 --save-occ 0.65
      Verifica que el PGM guardado corresponde celda a celda a alguna instantánea
      (misma resolución, dims, origin; -1 -> 205; <=free -> 254; >=occ -> 0).
      exit 0 OK, 2 FAIL.
"""
import argparse
import json
import os
import sys
import time

import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from validate_ros_map import read_image  # noqa


def cmd_snapshot(a):
    import rclpy
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from nav_msgs.msg import OccupancyGrid
    rclpy.init()
    n = rclpy.create_node('save_map_checked_snapshot')
    got = {}
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
    n.create_subscription(OccupancyGrid, a.topic, lambda m: got.setdefault('m', m), qos)
    t = time.time()
    while 'm' not in got and time.time() - t < a.timeout:
        rclpy.spin_once(n, timeout_sec=0.5)
    if 'm' not in got:
        print(f'FAIL: no se recibió OccupancyGrid en {a.topic} en {a.timeout}s', file=sys.stderr)
        return 2
    m = got['m']
    grid = np.array(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
    q = m.info.origin.orientation
    yaw = float(np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2)))
    np.savez_compressed(a.out, grid=grid, resolution=m.info.resolution,
                        origin=np.array([m.info.origin.position.x, m.info.origin.position.y, yaw]),
                        frame_id=m.header.frame_id,
                        stamp=m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
    v = grid.ravel()
    print(json.dumps({'topic': a.topic, 'w': int(m.info.width), 'h': int(m.info.height),
                      'res': float(m.info.resolution), 'frame': m.header.frame_id,
                      'unknown': int((v == -1).sum()), 'min': int(v.min()), 'max': int(v.max())}))
    return 0


def cmd_load_yaml(a):
    raw = open(a.raw).read()
    meta = yaml.safe_load(raw)
    lines = []
    for ln in raw.splitlines():
        if ln.strip().startswith('free_thresh:'):
            ln = f'free_thresh: {a.free}'
        lines.append(ln)
    if not any(ln.startswith('free_thresh:') for ln in lines):
        lines.append(f'free_thresh: {a.free}')
    with open(a.out, 'w') as f:
        f.write('\n'.join(lines) + '\n')
    new = yaml.safe_load(open(a.out))
    changed = {k for k in set(meta) | set(new) if meta.get(k) != new.get(k)}
    if changed - {'free_thresh'}:
        print(f'FAIL: cambiaron campos no esperados: {changed}', file=sys.stderr)
        return 2
    print(json.dumps({'raw_free_thresh': meta.get('free_thresh'), 'load_free_thresh': new['free_thresh']}))
    return 0


def expected_pgm(grid, save_free, save_occ):
    """Réplica de nav2 map_io saveMapToFile (trinary), validada en provenance_experiment."""
    free_i, occ_i = int(np.rint(save_free * 100)), int(np.rint(save_occ * 100))
    px = np.full(grid.shape, 205, np.uint8)
    valid = (grid >= 0) & (grid <= 100)
    px[valid & (grid <= free_i)] = 254
    px[valid & (grid >= occ_i)] = 0
    return np.flipud(px)  # la imagen tiene la fila 0 arriba (y máximo)


def cmd_check(a):
    meta = yaml.safe_load(open(a.yaml))
    arr, _ = read_image(os.path.join(os.path.dirname(os.path.abspath(a.yaml)), meta['image'])
                        if not os.path.isabs(meta['image']) else meta['image'])
    res = {'pgm': a.pgm, 'yaml': a.yaml, 'snapshots': []}
    ok_any = False
    for s in a.snap:
        z = np.load(s)
        g = z['grid']
        r = {'snap': s, 'dims_ok': list(arr.shape) == list(g.shape),
             'res_ok': abs(float(z['resolution']) - float(meta['resolution'])) < 1e-6,
             'origin_ok': bool(np.allclose(np.array(meta['origin'][:2], float), z['origin'][:2], atol=1e-3)),
             'snap_unknown_cells': int((g == -1).sum())}
        if r['dims_ok']:
            exp = expected_pgm(g, a.save_free, a.save_occ)
            diff = exp != arr
            r['pixel_mismatch'] = int(diff.sum())
            r['unknown_in_snapshot_saved_as_205'] = bool((np.flipud(g) == -1)[arr != 205].sum() == 0)
        r['match'] = bool(r['dims_ok'] and r['res_ok'] and r['origin_ok'] and r.get('pixel_mismatch', 1) == 0
                          and r.get('unknown_in_snapshot_saved_as_205', False))
        ok_any |= r['match']
        res['snapshots'].append(r)
    res['status'] = 'OK' if ok_any else 'FAIL'
    if not ok_any:
        res['hint'] = ('El PGM no coincide con ninguna instantánea: el mapa pudo cambiar durante el guardado '
                       '(SLAM activo) o resolución/origin incoherentes. Detener actualizaciones del mapa y repetir.')
    print(json.dumps(res, indent=1))
    return 0 if ok_any else 2


def main():
    ap = argparse.ArgumentParser()
    sp = ap.add_subparsers(dest='cmd', required=True)
    p = sp.add_parser('snapshot'); p.add_argument('--topic', default='/map'); p.add_argument('--out', required=True)
    p.add_argument('--timeout', type=float, default=15.0)
    p = sp.add_parser('load-yaml'); p.add_argument('--raw', required=True); p.add_argument('--out', required=True)
    p.add_argument('--free', type=float, required=True)
    p = sp.add_parser('check'); p.add_argument('--pgm', required=True); p.add_argument('--yaml', required=True)
    p.add_argument('--snap', action='append', required=True)
    p.add_argument('--save-free', type=float, required=True); p.add_argument('--save-occ', type=float, required=True)
    a = ap.parse_args()
    return {'snapshot': cmd_snapshot, 'load-yaml': cmd_load_yaml, 'check': cmd_check}[a.cmd](a)


if __name__ == '__main__':
    sys.exit(main())
