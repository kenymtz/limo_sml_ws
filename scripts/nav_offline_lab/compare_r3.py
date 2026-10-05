#!/usr/bin/python3
"""NAV-OFFLINE-R3 — comparación baseline (R2) vs config experimental R3 (offline).

Corridas (results/navigation_offline_lab/planner_runs/paths_<TAG>.json):
  Mapa sintético (repetición exacta, mismos 64 pares):
    A        costmap footprint A, track_unknown_space sin definir (UNKNOWN libre)   [baseline R2]
    B        costmap footprint B, track_unknown_space sin definir (UNKNOWN libre)   [baseline R2]
    B_rerun  = B re-ejecutado (control de determinismo)
    R3       costmap footprint B, track_unknown_space: true (UNKNOWN bloqueado)
  Mapa suplementario synthetic_arena_unknown (UNKNOWN en contacto con libre):
    SUP_B    = config B     SUP_R3 = config R3
Evaluación: path_eval.py (misma lógica que R2). Obstáculo para clearance/clases = ocupado ∪ UNKNOWN.
Salidas: r3_comparison.csv, r3_comparison_summary.json, r3_*.png
"""
import csv
import json
import os
import sys
from collections import Counter

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from footprint_eval import GridMap  # noqa
from path_eval import evaluate_result, path_deviation  # noqa

LAB = os.path.expanduser('~/limo_sml_ws/results/navigation_offline_lab')
RUNS = os.path.join(LAB, 'planner_runs')
SAME_TOL = 0.01          # rutas "idénticas" si Hausdorff < 1 cm

MAPS = {
    'synthetic': (os.path.join(LAB, 'synthetic_arena/synthetic_arena.yaml'),
                  os.path.join(LAB, 'synthetic_arena/synthetic_arena_geometry.json')),
    'supplementary': (os.path.join(LAB, 'synthetic_arena_unknown/synthetic_arena_unknown.yaml'),
                      os.path.join(LAB, 'synthetic_arena_unknown/synthetic_arena_unknown_geometry.json')),
}
COMPARISONS = [  # (nombre, mapa, antes, después, qué aísla)
    ('determinism_B_vs_B_rerun', 'synthetic', 'B', 'B_rerun', 'control: misma config dos veces'),
    ('unknown_only_B_vs_R3', 'synthetic', 'B', 'R3', 'solo track_unknown_space (footprint B en ambos)'),
    ('footprint_and_unknown_A_vs_R3', 'synthetic', 'A', 'R3', 'footprint A->B + track_unknown_space'),
    ('SUP_unknown_only_B_vs_R3', 'supplementary', 'SUP_B', 'SUP_R3', 'solo track_unknown_space, mapa con UNKNOWN expuesto'),
]


def load(tag):
    p = os.path.join(RUNS, f'paths_{tag}.json')
    return {r['test_id']: r for r in json.load(open(p))['results']} if os.path.exists(p) else None


evals, paths = {}, {}
for mname, (y, gj) in MAPS.items():
    if not os.path.exists(y):
        continue
    gm = GridMap(y); geo = json.load(open(gj))
    for tag in {c[2] for c in COMPARISONS if c[1] == mname} | {c[3] for c in COMPARISONS if c[1] == mname}:
        data = load(tag)
        if data is None:
            continue
        for tid, r in data.items():
            row, _, P = evaluate_result(r, gm, geo, tag)
            evals[(tag, tid)] = row; paths[(tag, tid)] = P

rows, summary = [], {'same_tol_m': SAME_TOL, 'comparisons': {}}
for name, mname, a, b, what in COMPARISONS:
    if (a, 'T001') not in evals or (b, 'T001') not in evals:
        summary['comparisons'][name] = 'NO EJECUTADA'
        continue
    tids = sorted({t for (tag, t) in evals if tag == a})
    cmp_rows = []
    for t in tids:
        ea, eb = evals[(a, t)], evals[(b, t)]
        r = {'comparison': name, 'map': mname, 'test_id': t, 'category': ea['category'],
             'before_cfg': a, 'after_cfg': b, 'found_before': ea['path_found'], 'found_after': eb['path_found']}
        if ea['path_found'] and eb['path_found']:
            dev = path_deviation(paths[(a, t)], paths[(b, t)])
            r.update(route_change='same' if dev < SAME_TOL else 'changed', hausdorff_m=round(dev, 4),
                     len_before=ea['path_length_m'], len_after=eb['path_length_m'],
                     len_diff_m=round(eb['path_length_m'] - ea['path_length_m'], 4),
                     passage_before=ea['passage'], passage_after=eb['passage'],
                     unknown_cells_before=ea['cells_in_unknown'], unknown_cells_after=eb['cells_in_unknown'],
                     minclr_before=ea['minimum_clearance'], minclr_after=eb['minimum_clearance'],
                     meanclr_before=ea['mean_clearance'], meanclr_after=eb['mean_clearance'],
                     p05_before=ea['p05_clearance'], p05_after=eb['p05_clearance'])
            for F in ('A', 'B'):
                r[f'{F}_class_before'] = ea[f'{F}_class']; r[f'{F}_class_after'] = eb[f'{F}_class']
                r[f'{F}_fpclr_before'] = ea[f'{F}_min_fp_clearance_m']; r[f'{F}_fpclr_after'] = eb[f'{F}_min_fp_clearance_m']
        elif ea['path_found'] and not eb['path_found']:
            r.update(route_change='disappeared', status_after=eb['status'], passage_before=ea['passage'],
                     unknown_cells_before=ea['cells_in_unknown'], B_class_before=ea['B_class'])
        elif eb['path_found'] and not ea['path_found']:
            r.update(route_change='new', status_before=ea['status'], passage_after=eb['passage'])
        else:
            r.update(route_change='none_both')
        cmp_rows.append(r)
    rows.extend(cmp_rows)
    both = [r for r in cmp_rows if r['route_change'] in ('same', 'changed')]
    ch = [r for r in both if r['route_change'] == 'changed']
    s = {'what': what, 'before': a, 'after': b, 'pairs': len(cmp_rows),
         'found_before': sum(r['found_before'] for r in cmp_rows), 'found_after': sum(r['found_after'] for r in cmp_rows),
         'route_change': dict(Counter(r['route_change'] for r in cmp_rows)),
         'passage_transitions': dict(Counter(f"{r['passage_before']}->{r['passage_after']}"
                                             for r in both if r['passage_before'] != r['passage_after'])),
         'paths_through_unknown_before': sum(1 for r in cmp_rows if r.get('unknown_cells_before', 0) > 0),
         'paths_through_unknown_after': sum(1 for r in cmp_rows if r.get('unknown_cells_after', 0) > 0)}
    if both:
        s['total_len_before_m'] = round(sum(r['len_before'] for r in both), 3)
        s['total_len_after_m'] = round(sum(r['len_after'] for r in both), 3)
        s['len_diff_changed'] = ({'n': len(ch), 'sum_m': round(sum(r['len_diff_m'] for r in ch), 3),
                                  'min_m': min(r['len_diff_m'] for r in ch), 'max_m': max(r['len_diff_m'] for r in ch),
                                  'max_hausdorff_m': max(r['hausdorff_m'] for r in ch)} if ch else None)
        s['center_min_clearance_min_before_after'] = [min(r['minclr_before'] for r in both),
                                                      min(r['minclr_after'] for r in both)]
        s['center_mean_clearance_avg_before_after'] = [round(np.mean([r['meanclr_before'] for r in both]), 4),
                                                       round(np.mean([r['meanclr_after'] for r in both]), 4)]
        for F in ('A', 'B'):
            s[f'fp{F}_class_before'] = dict(Counter(r[f'{F}_class_before'] for r in both))
            s[f'fp{F}_class_after'] = dict(Counter(r[f'{F}_class_after'] for r in both))
            s[f'fp{F}_class_transitions'] = dict(Counter(f"{r[f'{F}_class_before']}->{r[f'{F}_class_after']}"
                                                         for r in both if r[f'{F}_class_before'] != r[f'{F}_class_after']))
        s['by_category_len_median_before_after'] = {
            cat: [round(float(np.median([r['len_before'] for r in both if r['category'] == cat])), 3),
                  round(float(np.median([r['len_after'] for r in both if r['category'] == cat])), 3)]
            for cat in sorted({r['category'] for r in both})}
    s['changed_tests'] = [{k: r.get(k) for k in ('test_id', 'category', 'route_change', 'hausdorff_m', 'len_before',
                                                  'len_after', 'len_diff_m', 'passage_before', 'passage_after',
                                                  'unknown_cells_before', 'unknown_cells_after', 'B_class_before',
                                                  'B_class_after', 'B_fpclr_before', 'B_fpclr_after', 'status_after')}
                          for r in cmp_rows if r['route_change'] not in ('same',)]
    summary['comparisons'][name] = s

cols = sorted({k for r in rows for k in r}, key=lambda k: (k not in ('comparison', 'map', 'test_id', 'category'), k))
with open(os.path.join(LAB, 'r3_comparison.csv'), 'w', newline='') as f:
    w = csv.DictWriter(f, fieldnames=cols); w.writeheader(); w.writerows(rows)
json.dump(summary, open(os.path.join(LAB, 'r3_comparison_summary.json'), 'w'), indent=1, default=str)
print(json.dumps({k: ({kk: vv for kk, vv in v.items() if kk != 'changed_tests'} if isinstance(v, dict) else v)
                  for k, v in summary['comparisons'].items()}, indent=1, default=str))

# ---------- figuras ----------
for name, mname, a, b, what in COMPARISONS:
    if not isinstance(summary['comparisons'].get(name), dict):
        continue
    gm = GridMap(MAPS[mname][0])
    ext = [gm.origin[0], gm.origin[0] + gm.w * gm.res, gm.origin[1], gm.origin[1] + gm.h * gm.res]
    base = np.where(gm.cls == 0, 1.0, np.where(gm.cls == 100, 0.0, 0.75))
    fig, axs = plt.subplots(1, 2, figsize=(18, 9))
    for ax, tag, lab in ((axs[0], a, 'ANTES'), (axs[1], b, 'DESPUÉS')):
        ax.imshow(base, cmap='gray', vmin=0, vmax=1, origin='lower', extent=ext)
        for (tg, tid), P in paths.items():
            if tg != tag or P is None:
                continue
            rr = next(r for r in rows if r['comparison'] == name and r['test_id'] == tid)
            col = 'tab:red' if rr['route_change'] in ('changed', 'disappeared', 'new') else 'tab:green'
            ax.plot(P[:, 0], P[:, 1], '-', color=col, lw=1.4 if col == 'tab:red' else 0.7, alpha=0.85)
        ax.set_title(f'{lab}: {tag}  (rojo = ruta que cambia, verde = idéntica)', fontsize=10)
        ax.set_xlim(ext[0], ext[1]); ax.set_ylim(ext[2], ext[3])
    fig.suptitle(f'{name} — {what}\nMAPA SINTÉTICO ({mname}); no es evidencia física', fontsize=11)
    fig.tight_layout(); fig.savefig(os.path.join(LAB, f'r3_{name}.png'), dpi=75); plt.close(fig)
