#!/usr/bin/python3
"""FASE 7b — evaluación offline de las rutas del planner Nav2 real sobre el MAPA SINTÉTICO.

Modelo de ejecución (SUPUESTO, no medido): el robot gira en el sitio en el start hasta el
rumbo inicial, sigue la ruta con rumbo tangente (ventana 0.10 m) y gira en el sitio en el goal.
Swept footprint = todas esas poses (traslación ≤ 1 cm, rotación ≤ 5°).

Clasificación por footprint (polígono SIN padding, obstáculo = ocupado ∪ unknown):
  COLLISION : clearance mínima ≤ 0 (contacto/solape, resolución ±0.5 cm)
  MARGINAL  : 0 < clearance < padding_X + MARGIN_LOC
  PASS      : clearance ≥ padding_X + MARGIN_LOC
MARGIN_LOC = 0.05 m: reserva para error de localización+tracking (valor de trabajo, NO medido).
"""
import csv
import json
import os
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from footprint_eval import GridMap, FOOTPRINTS, footprint_clearance, raster_kernel  # noqa

LAB = os.path.expanduser('~/limo_sml_ws/results/navigation_offline_lab')
RUNS = os.path.join(LAB, 'planner_runs')
MARGIN_LOC = 0.05
HEAD_WIN = 0.10
gm = GridMap(os.path.join(LAB, 'synthetic_arena/synthetic_arena.yaml'))
geo = json.load(open(os.path.join(LAB, 'synthetic_arena/synthetic_arena_geometry.json')))


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def rot_seq(x, y, a0, a1, step=np.deg2rad(5)):
    d = wrap(a1 - a0)
    n = max(1, int(np.ceil(abs(d) / step)))
    return np.array([[x, y, a0 + d * k / n] for k in range(n + 1)])


def densify(P, step=0.01):
    out = [P[0]]
    for a, b in zip(P[:-1], P[1:]):
        L = np.hypot(*(b - a))
        n = max(1, int(np.ceil(L / step)))
        out.extend(a + (b - a) * (k / n) for k in range(1, n + 1))
    return np.array(out)


def headings(P):
    d = np.r_[0, np.cumsum(np.hypot(*np.diff(P, axis=0).T))]
    h = np.zeros(len(P))
    for i in range(len(P)):
        j = np.searchsorted(d, d[i] + HEAD_WIN)
        if j >= len(P):
            j = len(P) - 1
        k = i
        if j == i:                                 # final: usar rumbo previo
            k = max(0, np.searchsorted(d, d[i] - HEAD_WIN) - 1); j = i
        v = P[j] - P[k]
        h[i] = np.arctan2(v[1], v[0]) if np.hypot(*v) > 1e-6 else (h[i - 1] if i else 0.0)
    return h


def passage_used(P):
    sel = (P[:, 0] > 4.95) & (P[:, 0] < 5.04)
    used = set()
    for y in P[sel, 1]:
        for name, cd in geo['corridors'].items():
            if cd['y'][0] <= y <= cd['y'][1]:
                used.add(name)
    return '+'.join(sorted(used)) if used else 'none'


def classify(c, pad):
    return 'COLLISION' if c <= 0 else ('MARGINAL' if c < pad + MARGIN_LOC else 'PASS')


rows, swept_store = [], {}
for cfg in ('A', 'B'):
    data = json.load(open(os.path.join(RUNS, f'paths_{cfg}.json')))
    for r in data['results']:
        s, g = np.array(r['start']), np.array(r['goal'])
        row = {'test_id': r['test_id'], 'category': r['category'], 'planner_config': cfg,
               'start_x': s[0], 'start_y': s[1], 'start_yaw': s[2], 'goal_x': g[0], 'goal_y': g[1], 'goal_yaw': g[2],
               'status': r['status'], 'path_found': r['status'] == 'SUCCEEDED' and len(r['path']) > 1,
               'planning_time_s': r['planning_time_s'], 'wall_time_s': round(r['wall_time_s'], 4),
               'poses': len(r['path'])}
        if not row['path_found']:
            rows.append(row); continue
        P = np.array(r['path'])[:, :2]
        L = float(np.sum(np.hypot(*np.diff(P, axis=0).T)))
        cc = gm.clearance_at(P)
        D = densify(P)
        H = headings(D)
        trans = np.c_[D, H]
        rs = rot_seq(D[0, 0], D[0, 1], s[2], H[0])
        rg = rot_seq(D[-1, 0], D[-1, 1], H[-1], g[2])
        row.update(path_length_m=round(L, 4), straight_dist_m=round(float(np.hypot(*(g[:2] - s[:2]))), 4),
                   path_ratio=round(L / max(1e-6, np.hypot(*(g[:2] - s[:2]))), 3),
                   start_err_m=round(float(np.hypot(*(P[0] - s[:2]))), 4),
                   goal_err_m=round(float(np.hypot(*(P[-1] - g[:2]))), 4),
                   minimum_clearance=round(float(cc.min()), 4), mean_clearance=round(float(cc.mean()), 4),
                   p05_clearance=round(float(np.percentile(cc, 5)), 4), passage=passage_used(P))
        for F, spec in FOOTPRINTS.items():
            ct = footprint_clearance(gm, trans, spec['poly'])
            cs = footprint_clearance(gm, rs, spec['poly'])
            cgl = footprint_clearance(gm, rg, spec['poly'])
            m_all = float(min(ct.min(), cs.min(), cgl.min()))
            row[f'{F}_min_fp_clearance_m'] = round(m_all, 4)
            row[f'{F}_min_fp_clearance_translation_m'] = round(float(ct.min()), 4)
            row[f'{F}_min_fp_clearance_rotations_m'] = round(float(min(cs.min(), cgl.min())), 4)
            row[f'{F}_class'] = classify(m_all, spec['padding'])
            row[f'{F}_class_translation_only'] = classify(float(ct.min()), spec['padding'])
            row[f'{F}_worst_xy'] = ('%.2f,%.2f' % tuple(trans[np.argmin(ct), :2]))
        swept_store[(cfg, r['test_id'])] = (np.r_[rs, trans, rg], P)
        rows.append(row)

cols = ['test_id', 'category', 'planner_config', 'start_x', 'start_y', 'start_yaw', 'goal_x', 'goal_y', 'goal_yaw',
        'path_found', 'status', 'planning_time_s', 'wall_time_s', 'path_length_m', 'straight_dist_m', 'path_ratio',
        'poses', 'start_err_m', 'goal_err_m', 'minimum_clearance', 'mean_clearance', 'p05_clearance', 'passage',
        'A_min_fp_clearance_m', 'A_min_fp_clearance_translation_m', 'A_min_fp_clearance_rotations_m', 'A_class',
        'A_class_translation_only', 'A_worst_xy',
        'B_min_fp_clearance_m', 'B_min_fp_clearance_translation_m', 'B_min_fp_clearance_rotations_m', 'B_class',
        'B_class_translation_only', 'B_worst_xy']
with open(os.path.join(LAB, 'synthetic_planner_benchmark.csv'), 'w', newline='') as f:
    w = csv.DictWriter(f, fieldnames=cols); w.writeheader()
    for r in rows:
        w.writerow({k: r.get(k, '') for k in cols})

# ---------- resumen ----------
from collections import Counter, defaultdict  # noqa
summ = {'n_routes': len(rows) // 2, 'n_plans': len(rows), 'margin_loc_m': MARGIN_LOC, 'by_config': {}}
for cfg in ('A', 'B'):
    R = [r for r in rows if r['planner_config'] == cfg]
    ok = [r for r in R if r['path_found']]
    pt = np.array([r['planning_time_s'] for r in ok])
    d = {'plans': len(R), 'found': len(ok), 'success_pct': round(100 * len(ok) / len(R), 2),
         'planning_time_s_median_p95_max': [round(float(np.median(pt)), 4), round(float(np.percentile(pt, 95)), 4),
                                           round(float(pt.max()), 4)],
         'goal_err_max_m': max(r['goal_err_m'] for r in ok),
         'center_min_clearance_min_m': min(r['minimum_clearance'] for r in ok),
         'path_ratio_median': float(np.median([r['path_ratio'] for r in ok]))}
    for F in ('A', 'B'):
        d[f'fp{F}_class'] = dict(Counter(r[f'{F}_class'] for r in ok))
        d[f'fp{F}_class_translation_only'] = dict(Counter(r[f'{F}_class_translation_only'] for r in ok))
    by_cat = defaultdict(dict)
    for cat in sorted({r['category'] for r in ok}):
        rc = [r for r in ok if r['category'] == cat]
        by_cat[cat] = {'n': len(rc), 'passages': dict(Counter(r['passage'] for r in rc)),
                       'A': dict(Counter(r['A_class'] for r in rc)), 'B': dict(Counter(r['B_class'] for r in rc)),
                       'path_len_median': round(float(np.median([r['path_length_m'] for r in rc])), 3),
                       'center_min_clear_min': round(min(r['minimum_clearance'] for r in rc), 3)}
    d['by_category'] = by_cat
    summ['by_config'][cfg] = d
json.dump(summ, open(os.path.join(LAB, 'synthetic_planner_benchmark_summary.json'), 'w'), indent=1)
print(json.dumps(summ, indent=1))

# ---------- figuras ----------
ext = [gm.origin[0], gm.origin[0] + gm.w * gm.res, gm.origin[1], gm.origin[1] + gm.h * gm.res]
base = np.where(gm.cls == 0, 1.0, np.where(gm.cls == 100, 0.0, 0.75))
colors = {'PASS': 'tab:green', 'MARGINAL': 'tab:orange', 'COLLISION': 'tab:red'}
fig, axs = plt.subplots(2, 2, figsize=(16, 16))
for i, cfg in enumerate(('A', 'B')):
    for j, F in enumerate(('A', 'B')):
        ax = axs[i, j]
        ax.imshow(base, cmap='gray', vmin=0, vmax=1, origin='lower', extent=ext)
        for r in rows:
            if r['planner_config'] != cfg or not r['path_found']:
                continue
            P = swept_store[(cfg, r['test_id'])][1]
            ax.plot(P[:, 0], P[:, 1], '-', color=colors[r[f'{F}_class']], lw=1.0, alpha=0.8)
        cnt = summ['by_config'][cfg][f'fp{F}_class']
        ax.set_title(f'Planner con costmap {cfg} — clasificación footprint {F}: {cnt}', fontsize=10)
        ax.set_xlim(-0.3, 10.3); ax.set_ylim(-0.3, 10.3)
fig.suptitle('Benchmark NavFn real sobre MAPA SINTÉTICO (verde PASS · naranja MARGINAL · rojo COLLISION)\n'
             'NO es evidencia de desempeño físico', fontsize=12)
fig.tight_layout(); fig.savefig(os.path.join(LAB, 'synthetic_benchmark_paths.png'), dpi=80)

# swept footprint de ejemplo: una ruta por categoría de corredor, ambos footprints
fig, axs = plt.subplots(2, 3, figsize=(18, 12))
for j, cat in enumerate(('corridor_WIDE', 'corridor_N1', 'corridor_N2')):
    for i, cfg in enumerate(('A', 'B')):
        ax = axs[i, j]
        r = next(r for r in rows if r['planner_config'] == cfg and r['category'] == cat and r['path_found'])
        poses, P = swept_store[(cfg, r['test_id'])]
        sw = np.zeros(gm.obst_f.shape, bool)
        F = cfg
        for x, y, th in poses[::2]:
            K = raster_kernel(FOOTPRINTS[F]['poly'], th, gm.fres)
            n = K.shape[0] // 2
            rr, cc = int((y - gm.origin[1]) / gm.fres), int((x - gm.origin[0]) / gm.fres)
            sw[rr - n:rr + n + 1, cc - n:cc + n + 1] |= K
        rgb = np.ones(gm.obst_f.shape + (3,)); rgb[gm.obst_f] = 0; rgb[sw] = [0.4, 0.6, 1.0]
        rgb[sw & gm.obst_f] = [1, 0, 0]
        ax.imshow(rgb, origin='lower', extent=[gm.origin[0], gm.origin[0] + gm.obst_f.shape[1] * gm.fres,
                                               gm.origin[1], gm.origin[1] + gm.obst_f.shape[0] * gm.fres])
        ax.plot(P[:, 0], P[:, 1], 'k-', lw=0.8)
        xs, ys = P[:, 0], P[:, 1]
        ax.set_xlim(min(xs.min(), 3.5) - 0.4, max(xs.max(), 6.5) + 0.4)
        ax.set_ylim(min(ys.min(), 1.0) - 0.4 if cat != 'corridor_N2' else ys.min() - 0.5, ys.max() + 0.5)
        ax.set_aspect('equal')
        ax.set_title(f'{r["test_id"]} {cat} planner {cfg}, swept footprint {F}\n'
                     f'paso={r["passage"]} L={r["path_length_m"]} m clase {F}={r[f"{F}_class"]} '
                     f'(min {r[f"{F}_min_fp_clearance_m"]} m)', fontsize=9)
fig.suptitle('Swept footprint (azul; rojo = solape) — MAPA SINTÉTICO', fontsize=12)
fig.tight_layout(); fig.savefig(os.path.join(LAB, 'synthetic_swept_examples.png'), dpi=80)
