#!/usr/bin/python3
"""NAV-OFFLINE-R3 (suplementario) — variante del MAPA SINTÉTICO con UNKNOWN en contacto con libre.
NO modifica synthetic_arena/. NO representa el entorno físico.

Cambios respecto a synthetic_arena.pgm (todo pasa a 205 = UNKNOWN, "no observado"):
  G1, G2 : huecos de 0.60 m en la pared superior, a ambos lados del divisor (x∈[3.60,4.20] y [5.82,6.42])
  U1     : zona interna no observada 0.90 × 0.90 m, elegida automáticamente:
           máximo nº de rutas baseline (paths_B.json) que la cruzan, sin ningún start/goal a < 0.45 m.
Salidas: results/navigation_offline_lab/synthetic_arena_unknown/
"""
import json
import os
import shutil
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from validate_ros_map import read_image  # noqa

LAB = os.path.expanduser('~/limo_sml_ws/results/navigation_offline_lab')
SRC = os.path.join(LAB, 'synthetic_arena')
OUT = os.path.join(LAB, 'synthetic_arena_unknown')
RES, OX = 0.03, -0.81
UNK = 205

arr, _ = read_image(os.path.join(SRC, 'synthetic_arena.pgm'))
g = np.flipud(arr).copy()                          # fila 0 = y mínimo
geo = json.load(open(os.path.join(SRC, 'synthetic_arena_geometry.json')))


def c(v):
    return int(round((v - OX) / RES))


def rect(x0, x1, y0, y1, val):
    g[c(y0):c(y1), c(x0):c(x1)] = val


# huecos en la pared superior (pared en y ∈ [9.99, 10.08])
gaps = {'G1': [3.60, 4.20], 'G2': [5.82, 6.42]}
for x0, x1 in gaps.values():
    rect(x0, x1, 9.99, 10.08, UNK)

# zona interna no observada: búsqueda de la mejor ubicación
pairs = json.load(open(os.path.join(LAB, 'synthetic_benchmark_pairs.json')))['pairs']
paths = [np.array(r['path'])[:, :2] for r in json.load(open(os.path.join(LAB, 'planner_runs/paths_B.json')))['results']]
pts = np.array([p['start'][:2] for p in pairs] + [p['goal'][:2] for p in pairs])
S = 0.90                     # lado de U1 (m)
EXCL = 0.45                  # distancia mínima a cualquier start/goal (> r_circ B 0.29 + 0.15)
best = None
for x0 in np.arange(0.30, 9.99 - S - 0.3, 0.09):
    for y0 in np.arange(0.30, 9.99 - S - 0.3, 0.09):
        x1, y1 = x0 + S, y0 + S
        if 4.0 < x1 and x0 < 6.0:                   # no tocar la zona del divisor/corredores
            continue
        dx = np.maximum(0, np.maximum(x0 - pts[:, 0], pts[:, 0] - x1))
        dy = np.maximum(0, np.maximum(y0 - pts[:, 1], pts[:, 1] - y1))
        if np.hypot(dx, dy).min() < EXCL:
            continue
        # no solapar obstáculos (celdas libres en todo el cuadro)
        if (g[c(y0):c(y1), c(x0):c(x1)] != 254).any():
            continue
        n = sum(bool(((P[:, 0] > x0) & (P[:, 0] < x1) & (P[:, 1] > y0) & (P[:, 1] < y1)).any()) for P in paths)
        if best is None or n > best[0]:
            best = (n, round(x0 / RES) * RES, round(y0 / RES) * RES)
n_cross, ux0, uy0 = best
rect(ux0, ux0 + S, uy0, uy0 + S, UNK)
u1 = {'x': [round(ux0, 3), round(ux0 + S, 3)], 'y': [round(uy0, 3), round(uy0 + S, 3)],
      'baseline_B_paths_crossing': int(n_cross)}

os.makedirs(OUT, exist_ok=True)
img = np.flipud(g)
with open(os.path.join(OUT, 'synthetic_arena_unknown.pgm'), 'wb') as f:
    f.write(f'P5\n{img.shape[1]} {img.shape[0]}\n255\n'.encode()); f.write(img.tobytes())
with open(os.path.join(OUT, 'synthetic_arena_unknown.yaml'), 'w') as f:
    f.write('# MAPA SINTETICO (variante R3 con UNKNOWN en contacto con libre) - NO representa el entorno fisico\n'
            'image: synthetic_arena_unknown.pgm\nmode: trinary\nresolution: 0.03\norigin: [-0.81, -0.81, 0]\n'
            'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.10\n')
geo2 = dict(geo)
geo2['unknown_additions'] = {'gaps_top_wall': gaps, 'U1_internal': u1}
geo2['disclaimer'] = 'VARIANTE SINTETICA R3. No representa el entorno fisico.'
json.dump(geo2, open(os.path.join(OUT, 'synthetic_arena_unknown_geometry.json'), 'w'), indent=1)

ext = [OX, OX + g.shape[1] * RES, OX, OX + g.shape[0] * RES]
shade = np.where(img == 254, 1.0, np.where(img == 0, 0.0, 0.7))
fig, ax = plt.subplots(figsize=(9, 9))
ax.imshow(shade, cmap='gray', vmin=0, vmax=1, extent=ext, origin='upper')
for k, (x0, x1) in gaps.items():
    ax.annotate(k, ((x0 + x1) / 2, 10.3), color='tab:red', ha='center')
ax.add_patch(plt.Rectangle((ux0, uy0), S, S, fill=False, ec='tab:red', lw=1.5))
ax.annotate('U1 (no observado)', (ux0 + S / 2, uy0 + S / 2), color='tab:red', ha='center', va='center')
ax.set_title('synthetic_arena_unknown — VARIANTE SINTÉTICA R3\n(gris = UNKNOWN; huecos G1/G2 y zona U1 añadidos)')
fig.tight_layout(); fig.savefig(os.path.join(OUT, 'synthetic_arena_unknown.png'), dpi=100)
print(json.dumps(geo2['unknown_additions'], indent=1))
