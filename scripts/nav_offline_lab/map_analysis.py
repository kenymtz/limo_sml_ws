#!/usr/bin/python3
"""FASE 4 — análisis inicial del mapa (solo lectura del mapa original).

Clasificación trinary idéntica a nav2_map_server. Clearance vía transformada
de distancia euclídea. Dos escenarios:
  OPT : obstáculo = ocupado (desconocido transitable, optimista)
  CONS: obstáculo = ocupado ∪ desconocido (conservador)
Radios del footprint real (CLAUDE.md) + padding 0.02.
"""
import json
import os
import sys

import numpy as np
from scipy import ndimage as ndi
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa

sys.path.insert(0, os.path.dirname(__file__))
from common import OUT, FOOTPRINT, PADDING, load_map, classify  # noqa

MODE = sys.argv[1] if len(sys.argv) > 1 else 'semantic'
meta, pgm = load_map()
if MODE == 'semantic':
    # Semántica de map_saver trinary: 254 libre, 0 ocupado, resto (205) desconocido
    occ = np.full(pgm['arr'].shape, -1, np.int8)
    occ[pgm['arr'] == 254] = 0
    occ[pgm['arr'] == 0] = 100
else:
    # 'loaded': réplica de nav2_map_server con los umbrales del yaml
    occ = classify(pgm['arr'], meta)
SUFFIX = '' if MODE == 'semantic' else '_as_loaded'
res_ = meta['resolution']
x0, y0 = meta['origin'][:2]
H, W = occ.shape
FREE, OCC, UNK = occ == 0, occ == 100, occ == -1
N = occ.size
R_CIR = float(np.max(np.hypot(FOOTPRINT[:, 0], FOOTPRINT[:, 1]))) + PADDING
R_INS = 0.16 + PADDING  # distancia mínima centro->borde (lado lateral 0.16)
EIGHT = np.ones((3, 3), bool)

out = {'mode': MODE, 'resolution': res_, 'size_px': [W, H], 'r_inscribed_m': R_INS, 'r_circumscribed_m': R_CIR}
out['pct_free'] = 100 * FREE.sum() / N
out['pct_occupied'] = 100 * OCC.sum() / N
out['pct_unknown'] = 100 * UNK.sum() / N
out['area_free_m2'] = float(FREE.sum() * res_ ** 2)
out['area_occupied_m2'] = float(OCC.sum() * res_ ** 2)

# componentes libres
lab, n = ndi.label(FREE, structure=EIGHT)
sizes = np.bincount(lab.ravel())[1:]
order = np.argsort(sizes)[::-1]
main = lab == (order[0] + 1)
out['free_components'] = int(n)
out['free_components_top5_m2'] = [float(sizes[i] * res_ ** 2) for i in order[:5]]
out['main_free_pct_of_free'] = float(100 * sizes[order[0]] / FREE.sum())
out['free_components_lt_0.01m2'] = int((sizes * res_ ** 2 < 0.01).sum())
ys, xs = np.where(main)
out['main_free_bbox_m'] = [float(x0 + xs.min() * res_), float(x0 + (xs.max() + 1) * res_),
                           float(y0 + (H - 1 - ys.max()) * res_), float(y0 + (H - ys.min()) * res_)]

# frontera del componente principal: vecinos ocupados vs desconocidos
dil = ndi.binary_dilation(main, structure=EIGHT) & ~main
out['main_boundary_cells'] = int(dil.sum())
out['main_boundary_pct_occupied'] = float(100 * (dil & OCC).sum() / dil.sum())
out['main_boundary_pct_unknown'] = float(100 * (dil & UNK).sum() / dil.sum())

# desconocidos internos: componentes de desconocido que no tocan el borde de la imagen
ulab, un = ndi.label(UNK, structure=EIGHT)
border = np.unique(np.r_[ulab[0], ulab[-1], ulab[:, 0], ulab[:, -1]])
usz = np.bincount(ulab.ravel())
internal = [i for i in range(1, un + 1) if i not in set(border.tolist())]
isz = np.array([usz[i] for i in internal]) if internal else np.array([0])
out['unknown_components'] = int(un)
out['unknown_internal_components'] = len(internal)
out['unknown_internal_area_m2'] = float(isz.sum() * res_ ** 2)
out['unknown_internal_largest_m2'] = float(isz.max() * res_ ** 2)
out['unknown_internal_ge_0.05m2'] = int((isz * res_ ** 2 >= 0.05).sum())
# desconocido interno rodeado mayormente por libre del componente principal
internal_mask = np.isin(ulab, internal) if internal else np.zeros_like(UNK)

# clearance
scen = {}
for name, obst in (('OPT_occupied_only', OCC), ('CONS_occupied_plus_unknown', OCC | UNK)):
    dist = ndi.distance_transform_edt(~obst) * res_
    d_free = dist[main]
    s = {'clearance_main_free_median_m': float(np.median(d_free)),
         'clearance_main_free_p90_m': float(np.percentile(d_free, 90)),
         'clearance_main_free_max_m': float(d_free.max())}
    for rn, r in (('inscribed', R_INS), ('circumscribed', R_CIR)):
        ok = main & (dist >= r)
        lb, k = ndi.label(ok, structure=EIGHT)
        sz = np.bincount(lb.ravel())[1:]
        s[f'center_ok_{rn}_area_m2'] = float(ok.sum() * res_ ** 2)
        s[f'center_ok_{rn}_components'] = int(k)
        s[f'center_ok_{rn}_largest_m2'] = float(sz.max() * res_ ** 2) if k else 0.0
    # cuellos de botella: celdas con r_ins <= d < r_cir que conectan >=2 componentes "circumscribed"
    okc = main & (dist >= R_CIR)
    lc, kc = ndi.label(okc, structure=EIGHT)
    narrow = main & (dist >= R_INS) & (dist < R_CIR)
    ln, kn = ndi.label(narrow, structure=EIGHT)
    bott = []
    for i in range(1, kn + 1):
        m = ln == i
        touch = np.unique(lc[ndi.binary_dilation(m, structure=EIGHT)])
        touch = touch[touch > 0]
        big = [t for t in touch if (lc == t).sum() * res_ ** 2 >= 0.05]
        if len(big) >= 2:
            yy, xx = np.where(m)
            bott.append({'center_xy_m': [round(float(x0 + xx.mean() * res_), 2),
                                         round(float(y0 + (H - 1 - yy.mean()) * res_), 2)],
                         'min_clearance_m': round(float(dist[m].min()), 3),
                         'cells': int(m.sum()), 'connects': len(big)})
    s['bottlenecks'] = bott
    s['_dist'] = dist
    scen[name] = s

# trayectoria real: clearance del mapa en las posiciones visitadas (sanidad mapa vs realidad)
traj_path = os.path.join(OUT, 'real_trajectory.csv')
if os.path.exists(traj_path):
    tr = np.loadtxt(traj_path, delimiter=',', skiprows=1, usecols=(5, 6))[::10]
    cx = ((tr[:, 0] - x0) / res_).astype(int)
    cy = (H - 1 - ((tr[:, 1] - y0) / res_)).astype(int)
    inside = (cx >= 0) & (cx < W) & (cy >= 0) & (cy < H)
    cls = occ[cy[inside], cx[inside]]
    dO = scen['OPT_occupied_only']['_dist'][cy[inside], cx[inside]]
    out['trajectory_check'] = {
        'samples': int(inside.sum()),
        'pct_on_free': float(100 * (cls == 0).mean()), 'pct_on_unknown': float(100 * (cls == -1).mean()),
        'pct_on_occupied': float(100 * (cls == 100).mean()),
        'min_clearance_occupied_m': float(dO.min()),
        'pct_clearance_lt_r_inscribed': float(100 * (dO < R_INS).mean())}

for k, s in scen.items():
    out[k] = {kk: vv for kk, vv in s.items() if kk != '_dist'}
with open(os.path.join(OUT, f'map_analysis{SUFFIX}.json'), 'w') as f:
    json.dump(out, f, indent=1)
print(json.dumps(out, indent=1))

# ---------- figura ----------
extent = [x0, x0 + W * res_, y0, y0 + H * res_]
fig, ax = plt.subplots(1, 3, figsize=(21, 7))
rgb = np.zeros((H, W, 3))
rgb[FREE] = [1, 1, 1]; rgb[UNK] = [0.72, 0.72, 0.72]; rgb[OCC] = [0, 0, 0]
rgb[internal_mask.astype(bool)] = [1.0, 0.75, 0.3]
rgb[FREE & ~main] = [0.6, 0.8, 1.0]
ax[0].imshow(rgb, extent=extent, origin='upper')
ax[0].set_title('Clasificación (blanco libre main, azul libre aislado,\nnaranja desconocido interno, gris desconocido, negro ocupado)')
if os.path.exists(traj_path):
    ax[0].plot(tr[:, 0], tr[:, 1], 'r-', lw=0.8, label='trayectoria (map)')
    ax[0].legend(loc='upper right')
for a, key in zip(ax[1:], scen):
    d = np.where(main, scen[key]['_dist'], np.nan)
    im = a.imshow(d, extent=extent, origin='upper', cmap='viridis', vmin=0, vmax=1.0)
    a.contour(np.flipud(main & (scen[key]['_dist'] >= R_CIR)), levels=[0.5], colors='lime', linewidths=0.8,
              extent=extent)
    a.contour(np.flipud(main & (scen[key]['_dist'] >= R_INS)), levels=[0.5], colors='orange', linewidths=0.6,
              extent=extent)
    for b in scen[key]['bottlenecks']:
        a.plot(*b['center_xy_m'], 'rx', ms=8)
    a.set_title(f'Clearance {key}\nnaranja: ≥ r_ins {R_INS:.2f} m, verde: ≥ r_circ {R_CIR:.3f} m, x = cuello')
    plt.colorbar(im, ax=a, fraction=0.046, label='m')
for a in ax:
    a.set_xlabel('x [m]'); a.set_ylabel('y [m]')
fig.tight_layout(); fig.savefig(os.path.join(OUT, f'map_overview{SUFFIX}.png'), dpi=100)
