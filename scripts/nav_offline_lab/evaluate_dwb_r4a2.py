#!/usr/bin/python3
"""NAV-OFFLINE-R4-A2 — BASELINE (BaseObstacle) vs EXPERIMENTAL (ObstacleFootprint): seguridad del footprint.

Verdad de terreno OFFLINE (solo evaluación, nunca control): geometría del mapa sintético (PGM, celdas ocupadas)
y footprint REAL sin padding [-0.17,0.22]x[-0.16,0.16]. Cada celda ocupada se representa por 9 puntos
(centro, esquinas, medios de lado) -> error de clearance <= ~res/4 (0.0075 m).

Por caso y variante:
  pose_clear   clearance con signo del footprint real en la pose actual (<0 = penetración)
  traj_clear   mínimo sobre las poses de la trayectoria ELEGIDA (solo aceptados)
  center_clear distancia del centro al obstáculo más cercano en la trayectoria elegida
Clasificación (positivo = DWB rechaza):
  TN aceptado y traj_clear >= 0       FN (inseguro) aceptado y traj_clear < 0
  TP rechazado y pose_clear < 0       FP (falsa alarma) rechazado y pose_clear >= 0   (FP_within_padding: 0<=pose_clear<0.02; FP_strict: >=0.02)
"centro pasa, robot no": aceptado, celda central < INSCRIBED en el costmap (criterio BaseObstacle) y traj_clear < 0.
"""
import argparse
import collections
import csv
import json
import os

import numpy as np
import yaml
from scipy.spatial import cKDTree

LAB = os.path.expanduser('~/limo_sml_ws/results/navigation_offline_lab')
FP_REAL = (-0.17, 0.22, -0.16, 0.16)
PADDING = 0.02
R_INSCRIBED_PADDED = 0.16 + PADDING
LETHAL = 254


def load_map(yaml_path):
    m = yaml.safe_load(open(yaml_path))
    pgm = os.path.join(os.path.dirname(yaml_path), m['image'])
    with open(pgm, 'rb') as f:
        assert f.readline().strip() == b'P5'
        line = f.readline()
        while line.startswith(b'#'):
            line = f.readline()
        w, h = map(int, line.split())
        maxv = int(f.readline())
        img = np.frombuffer(f.read(w * h), dtype=np.uint8).reshape(h, w)
    p = (maxv - img.astype(float)) / maxv if not m.get('negate', 0) else img.astype(float) / maxv
    occ = p > m['occupied_thresh']
    res, (ox, oy, _) = m['resolution'], m['origin']
    rows, cols = np.nonzero(occ)
    cx = ox + (cols + 0.5) * res
    cy = oy + (h - rows - 0.5) * res
    offs = np.array([[0, 0], [-1, -1], [-1, 1], [1, -1], [1, 1], [0, -1], [0, 1], [-1, 0], [1, 0]]) * (res / 2)
    pts = (np.stack([cx, cy], 1)[:, None, :] + offs[None]).reshape(-1, 2)
    return pts, res, int(occ.sum())


class Clearance:
    def __init__(self, pts):
        self.pts = pts
        self.tree = cKDTree(pts)

    def pose(self, x, y, th, radius=0.6):
        idx = self.tree.query_ball_point([x, y], radius)
        if not idx:
            return radius, radius
        p = self.pts[idx] - [x, y]
        c, s = np.cos(th), np.sin(th)
        px, py = c * p[:, 0] + s * p[:, 1], -s * p[:, 0] + c * p[:, 1]
        xmin, xmax, ymin, ymax = FP_REAL
        dx = np.maximum.reduce([xmin - px, np.zeros_like(px), px - xmax])
        dy = np.maximum.reduce([ymin - py, np.zeros_like(py), py - ymax])
        outside = np.hypot(dx, dy)
        inside = -np.minimum.reduce([px - xmin, xmax - px, py - ymin, ymax - py])
        d = np.where((dx == 0) & (dy == 0), inside, outside)
        return float(d.min()), float(np.hypot(px, py).min())


def load_eval(fn):
    out = {}
    for line in open(fn):
        r = json.loads(line)
        if 'case' in r:
            out[r['case']] = r
    return out


def classify(r, clr):
    x, y, th = r['pose']
    pc, _ = clr.pose(x, y, th)
    d = dict(pose_clear=pc, status=r['status'], exc_msg=r['exc_msg'].split('\n')[-1].strip()[:80],
             cmd_vx=r['cmd'][0], cmd_wz=r['cmd'][1], best_fp_lethal=(r['best_footprint_max_cost'] or 0) >= LETHAL,
             legal_fp_collide=r['legal_footprint_collide'], t_ms=r['times_ms'][0], n_legal=r['n_legal'],
             center_cost=r['center_cost'])
    if r['status'] == 'ACCEPTED' and r.get('best_traj'):
        tc, cc = zip(*(clr.pose(*p) for p in r['best_traj']))
        d['traj_clear'], d['center_clear'] = min(tc), min(cc)
        d['cls'] = 'FN' if d['traj_clear'] < 0 else 'TN'
        # "centro pasa" con el criterio de DWB/BaseObstacle: celda central < INSCRIBED (253) en el costmap.
        # (en la baseline toda trayectoria aceptada lo cumple por construcción; en la pose se verifica center_cost)
        d['center_ok_robot_not'] = (r['center_cost'] is not None and r['center_cost'] < 253) and d['traj_clear'] < 0
        d['center_ok_robot_margin'] = (r['center_cost'] is not None and r['center_cost'] < 253) and d['traj_clear'] < PADDING
        d['margin_violation'] = d['traj_clear'] < PADDING
    else:
        d['traj_clear'] = d['center_clear'] = None
        d['cls'] = 'TP' if pc < 0 else 'FP'
        d['center_ok_robot_not'] = False
        d['center_ok_robot_margin'] = False
        d['margin_violation'] = False
    return d


def stats(v):
    v = [x for x in v if x is not None]
    if not v:
        return None
    return dict(n=len(v), min=min(v), p5=float(np.percentile(v, 5)), p50=float(np.median(v)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--base', required=True)
    ap.add_argument('--exp', required=True)
    ap.add_argument('--index', required=True)
    ap.add_argument('--map', default=os.path.join(LAB, 'synthetic_arena/synthetic_arena.yaml'))
    ap.add_argument('--r4a', default=os.path.join(LAB, 'dwb_r4a/eval_full_R4A.jsonl'))
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    pts, res, nocc = load_map(a.map)
    clr = Clearance(pts)
    idx = {c['case']: c for c in json.load(open(a.index))['cases']}
    B, E = load_eval(a.base), load_eval(a.exp)
    cases = [c for c in idx if c in B and c in E]
    rows = []
    for c in cases:
        b, e = classify(B[c], clr), classify(E[c], clr)
        row = dict(case=c, test=idx[c]['test'], kind=idx[c]['kind'], category=idx[c]['category'],
                   pose_clear=b['pose_clear'])
        for tag, d in (('b', b), ('e', e)):
            for k, v in d.items():
                if k != 'pose_clear':
                    row[f'{tag}_{k}'] = v
        row['same_cmd'] = B[c]['cmd'] == E[c]['cmd']
        rows.append(row)
    os.makedirs(a.out, exist_ok=True)
    with open(os.path.join(a.out, 'r4a2_cases.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)

    S = dict(n_cases=len(rows), map=a.map, occupied_cells=nocc, res=res, footprint_real=FP_REAL, padding=PADDING)
    # reproducibilidad: baseline A2 vs R4-A
    if os.path.exists(a.r4a):
        R = load_eval(a.r4a)
        S['baseline_vs_R4A'] = dict(compared=sum(c in R for c in cases),
                                    cmd_diff=sum(1 for c in cases if c in R and R[c]['cmd'] != B[c]['cmd']),
                                    status_diff=sum(1 for c in cases if c in R and R[c]['status'] != B[c]['status']))
    S['cmd_changed_base_vs_exp'] = sum(not r['same_cmd'] for r in rows)
    for tag, name in (('b', 'baseline'), ('e', 'experimental')):
        V = {}
        for kind in ('ALL', 'START', 'ALONG', 'PERT_LAT', 'PERT_YAW'):
            rr = [r for r in rows if kind == 'ALL' or r['kind'] == kind]
            cls = collections.Counter(r[f'{tag}_cls'] for r in rr)
            V[kind] = dict(
                n=len(rr), accepted=sum(r[f'{tag}_status'] == 'ACCEPTED' for r in rr),
                rejected=sum(r[f'{tag}_status'] != 'ACCEPTED' for r in rr),
                TN=cls['TN'], FN_unsafe=cls['FN'], TP=cls['TP'], FP=cls['FP'],
                FP_within_padding=sum(1 for r in rr if r[f'{tag}_cls'] == 'FP' and r['pose_clear'] < PADDING),
                FP_strict=sum(1 for r in rr if r[f'{tag}_cls'] == 'FP' and r['pose_clear'] >= PADDING),
                unsafe_margin=sum(1 for r in rr if r[f'{tag}_margin_violation']),
                unsafe_costmap_padded=sum(r[f'{tag}_best_fp_lethal'] for r in rr if r[f'{tag}_status'] == 'ACCEPTED'),
                center_ok_robot_not=sum(r[f'{tag}_center_ok_robot_not'] for r in rr),
                center_ok_robot_margin=sum(r[f'{tag}_center_ok_robot_margin'] for r in rr),
                center_geom_dist_unsafe=stats([r[f'{tag}_center_clear'] for r in rr if r[f'{tag}_cls'] == 'FN']),
                n_legal=stats([r[f'{tag}_n_legal'] for r in rr]),
                margin_violation_lt_padding=sum(r[f'{tag}_margin_violation'] for r in rr),
                cases_with_legal_fp_collision=sum(r[f'{tag}_legal_fp_collide'] > 0 for r in rr),
                traj_clear=stats([r[f'{tag}_traj_clear'] for r in rr]),
                t_ms=stats([r[f'{tag}_t_ms'] for r in rr]),
                reject_reasons=dict(collections.Counter(r[f'{tag}_exc_msg'] for r in rr
                                                        if r[f'{tag}_status'] != 'ACCEPTED')),
                FP_pose_clear=stats([r['pose_clear'] for r in rr if r[f'{tag}_cls'] == 'FP']),
                TP_pose_clear=stats([r['pose_clear'] for r in rr if r[f'{tag}_cls'] == 'TP']),
            )
        cat = {}
        for r in rows:
            c = cat.setdefault(r['category'], collections.Counter())
            c['n'] += 1
            c['acc'] += r[f'{tag}_status'] == 'ACCEPTED'
            c[r[f'{tag}_cls']] += 1
            c['pose_contact'] += r['pose_clear'] < 0
            c['n_legal_sum'] += r[f'{tag}_n_legal']
        V['by_category'] = {k: dict(v) for k, v in cat.items()}
        # rutas: aceptada si START y todas las muestras ALONG nominales aceptadas
        tests = collections.defaultdict(lambda: dict(start=None, along=[], pert=[], unsafe=0))
        for r in rows:
            t = tests[r['test']]
            ok = r[f'{tag}_status'] == 'ACCEPTED'
            if r['kind'] == 'START':
                t['start'] = ok
            elif r['kind'] == 'ALONG':
                t['along'].append(ok)
            else:
                t['pert'].append(ok)
            t['unsafe'] += r[f'{tag}_cls'] == 'FN'
        V['routes'] = dict(
            n=len(tests), nominal_accepted=sum(1 for t in tests.values() if t['start'] and all(t['along'])),
            with_any_pert_reject=sum(1 for t in tests.values() if not all(t['pert'])),
            with_any_unsafe=sum(1 for t in tests.values() if t['unsafe'] > 0))
        V['routes_with_unsafe'] = sorted(k for k, t in tests.items() if t['unsafe'] > 0)
        S[name] = V
    S['pose_contact_cases'] = sum(r['pose_clear'] < 0 for r in rows)
    S['pose_clear_lt_padding'] = sum(r['pose_clear'] < PADDING for r in rows)
    bu, eu = S['baseline']['ALL']['FN_unsafe'], S['experimental']['ALL']['FN_unsafe']
    S['improvement_pct'] = 100.0 * (bu - eu) / bu if bu else None
    json.dump(S, open(os.path.join(a.out, 'r4a2_summary.json'), 'w'), indent=1, default=str)

    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(1, 2, figsize=(14, 4.5))
        for tag, name, col in (('b', 'BASELINE BaseObstacle', 'tab:red'), ('e', 'EXPERIMENTAL ObstacleFootprint', 'tab:blue')):
            v = [r[f'{tag}_traj_clear'] for r in rows if r[f'{tag}_traj_clear'] is not None]
            ax[0].hist(np.clip(v, -0.05, 0.4), bins=90, alpha=0.5, color=col, label=f'{name} (n={len(v)})')
        ax[0].axvline(0, color='k', lw=1)
        ax[0].axvline(PADDING, color='k', ls=':', lw=1, label='padding 0.02')
        ax[0].set_xlabel('clearance mínima del footprint REAL en la trayectoria elegida (m)')
        ax[0].set_yscale('log')
        ax[0].legend(fontsize=8)
        n1 = [r for r in rows if r['category'] == 'corridor_N1' and r['kind'] != 'START']
        for tag, mk, col in (('b', 'o', 'tab:red'), ('e', 'x', 'tab:blue')):
            acc = [r for r in n1 if r[f'{tag}_status'] == 'ACCEPTED']
            rej = [r for r in n1 if r[f'{tag}_status'] != 'ACCEPTED']
            ax[1].scatter([r['pose_clear'] for r in acc], [r[f'{tag}_traj_clear'] for r in acc], marker=mk, c=col,
                          s=18, alpha=0.7, label=f'{tag}: aceptado ({len(acc)})')
            ax[1].scatter([r['pose_clear'] for r in rej], [-0.06] * len(rej), marker='v', c=col, s=30,
                          label=f'{tag}: rechazado ({len(rej)}) [y=-0.06]')
        ax[1].axhline(0, color='k', lw=1)
        ax[1].axvline(0, color='k', lw=1)
        ax[1].set_xlabel('clearance footprint real en la pose (m)')
        ax[1].set_ylabel('clearance mín. trayectoria elegida (m)')
        ax[1].set_title('corridor_N1 (ALONG + perturbaciones)')
        ax[1].legend(fontsize=7)
        fig.tight_layout()
        fig.savefig(os.path.join(a.out, 'r4a2_clearance.png'), dpi=110)
    except Exception as ex:  # noqa
        print('figura no generada:', ex)
    print(json.dumps({k: S[k] for k in ('n_cases', 'baseline_vs_R4A', 'cmd_changed_base_vs_exp', 'pose_contact_cases',
                                        'improvement_pct')}, default=str))
    for name in ('baseline', 'experimental'):
        print(name, json.dumps(S[name]['ALL'], default=str))
        print(name, 'routes', S[name]['routes'], S[name]['routes_with_unsafe'])


if __name__ == '__main__':
    main()
