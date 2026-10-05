#!/usr/bin/python3
"""FASE 7a — pares start/goal para el benchmark sobre el MAPA SINTÉTICO (determinista, seed 42).
Válido = footprints A y B (con padding) sin colisión en start y goal con su yaw.
Salida: results/navigation_offline_lab/synthetic_benchmark_pairs.json
"""
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from footprint_eval import GridMap, FOOTPRINTS, pad_poly, footprint_clearance  # noqa

LAB = os.path.expanduser('~/limo_sml_ws/results/navigation_offline_lab')
gm = GridMap(os.path.join(LAB, 'synthetic_arena/synthetic_arena.yaml'))
geo = json.load(open(os.path.join(LAB, 'synthetic_arena/synthetic_arena_geometry.json')))
rng = np.random.default_rng(42)
PAD = {F: pad_poly(s['poly'], s['padding']) for F, s in FOOTPRINTS.items()}
MIN_CLEAR = 0.03


def valid(p):
    p = np.array([p])
    return all(footprint_clearance(gm, p, PAD[F])[0] > MIN_CLEAR for F in PAD)


def with_yaw(x, y, yaw=None, tries=50):
    for _ in range(tries):
        th = rng.uniform(-np.pi, np.pi) if yaw is None else yaw
        if valid((x, y, th)):
            return [round(x, 3), round(y, 3), round(float(th), 4)]
        if yaw is not None:
            return None
    return None


def rand_free():
    while True:
        x, y = rng.uniform(0.3, 9.7, 2)
        p = with_yaw(x, y)
        if p:
            return p


pairs = []


def add(cat, s, g):
    if s and g:
        pairs.append({'test_id': f'T{len(pairs) + 1:03d}', 'category': cat, 'start': s, 'goal': g})
    else:
        print('descartado', cat, s, g)


# cortas (0.6–1.5 m)
while sum(p['category'] == 'short' for p in pairs) < 10:
    s = rand_free()
    ang, d = rng.uniform(-np.pi, np.pi), rng.uniform(0.6, 1.5)
    g = with_yaw(s[0] + d * np.cos(ang), s[1] + d * np.sin(ang))
    if g and 0.2 < g[0] < 9.8 and 0.2 < g[1] < 9.8:
        add('short', s, g)
# largas (cruzan el divisor, >= 6 m en línea recta)
while sum(p['category'] == 'long' for p in pairs) < 10:
    s, g = rand_free(), rand_free()
    if (s[0] - 5) * (g[0] - 5) < 0 and np.hypot(s[0] - g[0], s[1] - g[1]) >= 6.0:
        add('long', s, g)
# alrededor de obstáculos
around = [((1.0, 7.0), (3.1, 7.0)), ((2.0, 6.0), (2.0, 8.0)), ((1.8, 4.0), (3.2, 4.0)), ((2.5, 3.2), (2.5, 4.8)),
          ((6.7, 3.9), (8.3, 3.9)), ((7.5, 2.5), (7.5, 5.3)), ((8.0, 7.5), (8.9, 7.5)), ((7.5, 6.1), (7.5, 7.2)),
          ((8.0, 8.0), (9.3, 6.0)), ((6.0, 9.2), (9.2, 5.8))]
for s, g in around:
    add('around_obstacle', with_yaw(*s), with_yaw(*g))
# corredores: extremos a 0.36 m de la entrada, orientaciones variadas
for name in ('WIDE', 'N1', 'N2'):
    cd = geo['corridors'][name]
    yc = float(np.mean(cd['y']))
    xl, xr = cd['x'][0] - 0.36, cd['x'][1] + 0.36
    offs = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] if name != 'WIDE' else [0, 0.3, -0.3, 0, 0.2, -0.2, 0, 0]
    yaws = [(0, 0), (np.pi, np.pi), (0, np.pi), (np.pi / 2, np.pi / 2), (-np.pi / 4, np.pi / 4),
            (np.pi, 0), (3 * np.pi / 4, -np.pi / 2), (0, -np.pi / 2)]
    for k, ((ys, yg), o) in enumerate(zip(yaws, offs)):
        a, b = (xl, xr) if k % 2 == 0 else (xr, xl)
        add(f'corridor_{name}', with_yaw(a, yc + o, ys), with_yaw(b, yc + o, yg))
# aleatorias
while sum(p['category'] == 'random' for p in pairs) < 10:
    s, g = rand_free(), rand_free()
    if np.hypot(s[0] - g[0], s[1] - g[1]) > 1.0:
        add('random', s, g)

json.dump({'map': 'synthetic_arena (SINTETICO)', 'seed': 42, 'min_start_goal_clearance_m': MIN_CLEAR,
           'pairs': pairs}, open(os.path.join(LAB, 'synthetic_benchmark_pairs.json'), 'w'), indent=1)
from collections import Counter  # noqa
print(len(pairs), Counter(p['category'] for p in pairs))
