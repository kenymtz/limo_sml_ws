#!/usr/bin/python3
"""Audita (solo lectura) yaml de mapas: umbrales, pgm referenciado, histograma de
píxeles y cómo los clasificaría nav2_map_server (trinary) con los umbrales del yaml
vs con free_thresh 0.196. Uso: audit_map_yamls.py <yaml> [<yaml> ...]"""
import hashlib
import json
import os
import sys

import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(__file__))
from common import read_pgm  # noqa


def nav2_counts(arr, negate, occ_t, free_t):
    p = arr.astype(float) / 255.0
    occ = p if negate else 1.0 - p
    o = occ > occ_t
    f = (~o) & (occ < free_t)
    return {'free': int(f.sum()), 'occupied': int(o.sum()), 'unknown': int((~o & ~f).sum())}


rows = []
for y in sys.argv[1:]:
    r = {'yaml': y}
    try:
        meta = yaml.safe_load(open(y))
    except Exception as e:  # noqa
        r['error'] = f'yaml ilegible: {e}'; rows.append(r); continue
    r.update({k: meta.get(k) for k in ('image', 'mode', 'resolution', 'origin', 'negate',
                                        'occupied_thresh', 'free_thresh')})
    img = meta.get('image', '')
    path = img if os.path.isabs(img) else os.path.join(os.path.dirname(y), img)
    r['image_path'] = path
    r['image_exists'] = os.path.exists(path)
    if r['image_exists'] and path.endswith('.pgm'):
        pg = read_pgm(path)
        a = pg['arr']
        r['pgm_sha256_12'] = hashlib.sha256(open(path, 'rb').read()).hexdigest()[:12]
        r['size_px'] = [pg['w'], pg['h']]
        r['size_m'] = [round(pg['w'] * meta['resolution'], 3), round(pg['h'] * meta['resolution'], 3)]
        v, c = np.unique(a, return_counts=True)
        r['pixel_hist'] = {int(k): int(n) for k, n in zip(v, c)}
        r['px205'] = int((a == 205).sum())
        r['nav2_yaml_thresh'] = nav2_counts(a, meta.get('negate', 0), meta['occupied_thresh'], meta['free_thresh'])
        r['nav2_free_0.196'] = nav2_counts(a, meta.get('negate', 0), meta['occupied_thresh'], 0.196)
        r['semantic'] = {'free': int((a == 254).sum()), 'occupied': int((a == 0).sum()),
                         'unknown': int(((a != 254) & (a != 0)).sum())}
        n = a.size
        r['unknown_lost_pct'] = round(100 * (r['semantic']['unknown'] - r['nav2_yaml_thresh']['unknown']) / n, 2)
        r['affected'] = r['nav2_yaml_thresh']['unknown'] < r['semantic']['unknown']
    rows.append(r)
print(json.dumps(rows, indent=1))
