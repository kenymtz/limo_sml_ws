#!/usr/bin/python3
"""FASE 5 — análisis de footprints A y B sobre el MAPA SINTÉTICO (geometría pura, offline).

Espacio de configuraciones (C-space) en grid de 1 cm, 72 orientaciones (5°).
  free_any : existe alguna orientación libre (condición necesaria para estar ahí)
  free_all : libre en todas las orientaciones (puede girar en el sitio)
Variantes: raw (polígono) y padded (polígono + padding, como nav2 padFootprint).
No decide cuál footprint es físicamente correcto.
"""
import json
import os
import sys

import numpy as np
from scipy import ndimage as ndi
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from footprint_eval import GridMap, FOOTPRINTS, pad_poly, cspace_free, footprint_clearance  # noqa

LAB = os.path.expanduser('~/limo_sml_ws/results/navigation_offline_lab')
SYN = os.path.join(LAB, 'synthetic_arena')
OUT = os.path.join(LAB, 'footprint_analysis_r2')
os.makedirs(OUT, exist_ok=True)
gm = GridMap(os.path.join(SYN, 'synthetic_arena.yaml'))
geo = json.load(open(os.path.join(SYN, 'synthetic_arena_geometry.json')))
YAWS = np.deg2rad(np.arange(0, 360, 5))
k0, k180 = 0, 36
A_CELL = gm.fres ** 2

res = {'map': 'synthetic_arena (SINTETICO)', 'grid_m': gm.fres, 'n_yaws': len(YAWS), 'footprints': {}}
masks = {}
for F, spec in FOOTPRINTS.items():
    for var in ('raw', 'padded'):
        poly = spec['poly'] if var == 'raw' else pad_poly(spec['poly'], spec['padding'])
        any_ = np.zeros(gm.obst_f.shape, bool)
        all_ = np.ones(gm.obst_f.shape, bool)
        slices = {}
        for k, yaw in enumerate(YAWS):
            fr = cspace_free(gm, poly, [yaw])[0]
            any_ |= fr; all_ &= fr
            if k in (k0, k180):
                slices[k] = fr
        key = f'{F}_{var}'
        masks[key] = {'any': any_, 'all': all_, 'yaw0': slices[k0], 'yaw180': slices[k180]}
        w = poly[:, 1].max() - poly[:, 1].min(); L = poly[:, 0].max() - poly[:, 0].min()
        res['footprints'][key] = {
            'poly': poly.round(4).tolist(), 'width_m': round(w, 3), 'length_m': round(L, 3),
            'r_inscribed_m': round(float(np.min(np.abs(np.r_[poly[:, 0], poly[:, 1]]))), 3),
            'r_circumscribed_m': round(float(np.hypot(poly[:, 0], poly[:, 1]).max()), 3),
            'area_free_any_m2': round(any_.sum() * A_CELL, 3),
            'area_free_all_m2': round(all_.sum() * A_CELL, 3)}
        print(key, res['footprints'][key]['area_free_any_m2'], res['footprints'][key]['area_free_all_m2'], flush=True)

free_center = gm.edt_f > 0
res['area_center_free_m2'] = round(free_center.sum() * A_CELL, 3)
for var in ('raw', 'padded'):
    a, b = masks[f'A_{var}'], masks[f'B_{var}']
    res[f'A_only_{var}'] = {'any_m2': round((a['any'] & ~b['any']).sum() * A_CELL, 3),
                            'all_m2': round((a['all'] & ~b['all']).sum() * A_CELL, 3),
                            'B_only_any_m2': round((b['any'] & ~a['any']).sum() * A_CELL, 3)}

# ---- corredores: ¿se pueden atravesar con yaw 0 / 180 (eje del corredor)? ----
def fine(x, y):
    return int((y - gm.origin[1]) / gm.fres), int((x - gm.origin[0]) / gm.fres)


corr = {}
for name, cd in geo['corridors'].items():
    yc = float(np.mean(cd['y']))
    pL, pR = fine(cd['x'][0] - 0.40, yc), fine(cd['x'][1] + 0.40, yc)
    r0, c0 = fine(cd['x'][0] - 0.6, cd['y'][0] - 0.5); r1, c1 = fine(cd['x'][1] + 0.6, cd['y'][1] + 0.5)
    corr[name] = {'width_m': cd['width_m']}
    for key, m in masks.items():
        ok = []
        for sl in ('yaw0', 'yaw180'):
            sub = m[sl][r0:r1, c0:c1]
            lab, _ = ndi.label(sub)
            a_, b_ = lab[pL[0] - r0, pL[1] - c0], lab[pR[0] - r0, pR[1] - c0]
            ok.append(bool(a_ and a_ == b_))
        fw = res['footprints'][key]['width_m']
        corr[name][key] = {'passable_yaw0': ok[0], 'passable_yaw180': ok[1],
                           'lateral_margin_each_side_m': round((cd['width_m'] - fw) / 2, 4)}
res['corridors'] = corr

# ---- autocomprobación C-space vs clearance directa ----
rng = np.random.default_rng(0)
agree = {}
for key in ('A_raw', 'B_raw'):
    F = key[0]
    poly = FOOTPRINTS[F]['poly']
    n = 3000
    xy = rng.uniform([0, 0], [9.99, 9.99], (n, 2))
    k = rng.integers(0, len(YAWS), n)
    cl = footprint_clearance(gm, np.c_[xy, YAWS[k]], poly)
    # celdas de C-space para yaw k
    fr = np.array([cspace_free(gm, poly, [YAWS[kk]])[0][fine(x, y)] for (x, y), kk in zip(xy[:300], k[:300])])
    decided = np.abs(cl[:300]) > 0.015             # fuera de la banda de cuantización
    agree[key] = {'samples': int(decided.sum()),
                  'agreement_pct': round(100 * float(((cl[:300] > 0) == fr)[decided].mean()), 2)}
res['self_check_cspace_vs_clearance'] = agree

json.dump(res, open(os.path.join(OUT, 'footprint_analysis.json'), 'w'), indent=1)
np.savez_compressed(os.path.join(OUT, 'cspace_masks.npz'),
                    **{f'{k}_{s}': v[s] for k, v in masks.items() for s in ('any', 'all')})
print(json.dumps({k: v for k, v in res.items() if k != 'footprints'}, indent=1))

# ---- figuras ----
ext = [gm.origin[0], gm.origin[0] + gm.obst_f.shape[1] * gm.fres,
       gm.origin[1], gm.origin[1] + gm.obst_f.shape[0] * gm.fres]


def show(ax, img, title):
    ax.imshow(img, origin='lower', extent=ext, interpolation='nearest')
    ax.set_title(title, fontsize=9); ax.set_xlim(-0.2, 10.2); ax.set_ylim(-0.2, 10.2)


fig, axs = plt.subplots(2, 3, figsize=(18, 12))
for ax, key in zip(axs[0], ('A_padded', 'B_padded')):
    m = masks[key]
    rgb = np.ones(gm.obst_f.shape + (3,)) * 0.15
    rgb[m['any']] = [1.0, 0.8, 0.3]; rgb[m['all']] = [0.3, 0.8, 0.3]; rgb[gm.obst_f] = [0, 0, 0]
    show(ax, rgb, f'{key}: verde = libre en todo yaw (gira en sitio) · naranja = solo algunos yaw')
rgb = np.ones(gm.obst_f.shape + (3,))
rgb[~masks['A_padded']['any']] = [0.85, 0.85, 0.85]
rgb[masks['A_padded']['any'] & ~masks['B_padded']['any']] = [0.9, 0.1, 0.1]
rgb[masks['A_padded']['all'] & ~masks['B_padded']['all'] & masks['B_padded']['any']] = [1.0, 0.6, 0.0]
rgb[gm.obst_f] = [0, 0, 0]
show(axs[0, 2], rgb, 'rojo: A (padded) cabe con algún yaw y B no · naranja: A gira en sitio y B no')
for ax, name in zip(axs[1], ('WIDE', 'N1', 'N2')):
    cd = geo['corridors'][name]
    rgb = np.ones(gm.obst_f.shape + (3,))
    rgb[masks['A_padded']['yaw0']] = [0.6, 0.85, 1.0]
    rgb[masks['B_padded']['yaw0']] = [0.2, 0.5, 0.9]
    rgb[gm.obst_f] = [0, 0, 0]
    ax.imshow(rgb, origin='lower', extent=ext, interpolation='nearest')
    ax.set_xlim(cd['x'][0] - 0.8, cd['x'][1] + 0.8); ax.set_ylim(cd['y'][0] - 0.8, cd['y'][1] + 0.8)
    c = corr[name]
    ax.set_title(f'{name} ({cd["width_m"]} m), yaw 0 — celeste: solo A_pad · azul: B_pad\n'
                 f'A_pad pasa={c["A_padded"]["passable_yaw0"]} (margen {c["A_padded"]["lateral_margin_each_side_m"]} m) · '
                 f'B_pad pasa={c["B_padded"]["passable_yaw0"]} (margen {c["B_padded"]["lateral_margin_each_side_m"]} m) · '
                 f'B_raw pasa={c["B_raw"]["passable_yaw0"]}', fontsize=8)
fig.suptitle('Footprints A/B sobre MAPA SINTÉTICO — análisis geométrico, no evidencia física', fontsize=12)
fig.tight_layout(); fig.savefig(os.path.join(OUT, 'footprint_cspace.png'), dpi=90)
