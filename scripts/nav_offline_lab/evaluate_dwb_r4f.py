#!/usr/bin/python3
"""NAV-OFFLINE-R4-F — general_goal_checker.yaw_goal_tolerance 0.25 (R4-E 0.15, existente) vs 0.20 / 0.15 / 0.10.

Variantes (mismas 384 corridas, FollowPath.xy_goal_tolerance 0.15 y checker xy 0.25 en todas):
  Y025 = R4-E 0.15  dwb_r4e/sim_<S>_015.jsonl  (baseline, NO re-ejecutada)
  Y020/Y015/Y010    dwb_r4f/sim_<S>_<tag>.jsonl
Criterio de "correcto" FIJO para todas (independiente del checker): SUCCEEDED, sin contacto, pose REAL ≤ 0.25 m y ≤ 0.25 rad
(evaluate_dwb_r4d.classify). Assert: clases de Y025 == dwb_r4e/r4e_runs.csv (T015).
"""
import collections
import csv
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from evaluate_dwb_r4a2 import Clearance, load_map, LAB  # noqa: E402
from evaluate_dwb_r4b import run_metrics, sign_changes  # noqa: E402
from evaluate_dwb_r4d import classify, extra  # noqa: E402
from evaluate_dwb_r4e import near_goal, pct, summ, SCEN, NARROW, V_STOP, W_ROT  # noqa: E402

OUT = os.path.join(LAB, 'dwb_r4f')
SRC = {'Y025': ('dwb_r4e', '015', 0.25), 'Y020': ('dwb_r4f', '020', 0.20), 'Y015': ('dwb_r4f', '015', 0.15),
       'Y010': ('dwb_r4f', '010', 0.10)}
VARS = list(SRC)
NEW = VARS[1:]
FP_TOL = 0.15
TOL = 0.25
BLOCK = ('OSCILACION', 'TIMEOUT', 'OTRO_SIN_TRAYECT')


def osc_split(x):
    """inversiones de ω antes / después de empezar a girar en el lugar dentro de la ventana de RotateToGoal"""
    c = np.array(x['ctrl'], dtype=float)
    g = x['goal']
    d = np.hypot(c[:, 4] - g[0], c[:, 5] - g[1])
    if not (d <= 0.25).any():
        return dict(flips_pre_rot=None, flips_post_rot=None, retranslate_after_rot=None)
    i0 = int(np.argmax(d <= 0.25))
    rs = np.where((d <= FP_TOL) & (np.abs(c[:, 9]) < V_STOP) & (np.abs(c[:, 10]) >= W_ROT))[0]
    if not len(rs):
        return dict(flips_pre_rot=sign_changes(c[i0:, 10]), flips_post_rot=None, retranslate_after_rot=None)
    r0 = int(rs[0])
    return dict(flips_pre_rot=sign_changes(c[i0:r0, 10]), flips_post_rot=sign_changes(c[r0:, 10]),
                retranslate_after_rot=bool((np.abs(c[r0:, 9]) >= V_STOP).any()))


def subcls(r):
    """clase extendida: falso éxito por yaw/xy y GOAL_TOLERANCE por estado final"""
    c = r['cls']
    if c == 'OTRO_EXITO_APARENTE':
        return 'FALSE_YAW' if r['final_yaw_err'] > TOL and r['final_xy_err'] <= TOL else (
            'FALSE_XY' if r['final_yaw_err'] <= TOL else 'FALSE_XY_YAW')
    if c == 'GOAL_TOLERANCE':
        return f'GOAL_TOLERANCE[{r["status"]}]'
    return c


def main():
    pts, _, _ = load_map(os.path.join(LAB, 'synthetic_arena/synthetic_arena.yaml'))
    clr = Clearance(pts)
    paths = {r['test_id']: (np.array(r['path']), r['category'])
             for r in json.load(open(os.path.join(LAB, 'planner_runs/paths_R3.json')))['results']}
    seeds = {(s, ln.split()[1]): ln.split()[8] for s in SCEN
             for ln in open(os.path.join(LAB, 'dwb_r4b', f'runs_{s}.txt')) if ln.startswith('RUN')}
    rows = []
    partial = bool(os.environ.get('R4F_PARTIAL'))
    for var, (sub, suf, ytol) in SRC.items():
        for s in SCEN:
            fn = os.path.join(LAB, sub, f'sim_{s}_{suf}.jsonl')
            if not os.path.exists(fn):
                assert partial, fn
                continue
            L = [json.loads(z) for z in open(fn)]
            if len(L) != 65:
                assert partial, (fn, len(L))
                continue
            meta = L[0]['meta']
            assert abs(meta['goal_yaw'] - ytol) < 1e-12 and abs(meta['goal_xy'] - 0.25) < 1e-12, (fn, meta)
            for x in L[1:]:
                m = run_metrics(x, meta, paths[x['run']][0], clr)
                cls, trans, flips = classify(x, m)
                m.update(extra(x))
                m.update(near_goal(x, m, FP_TOL))
                m.update(osc_split(x))
                m.update(variant=var, yaw_tol=ytol, scenario=s, path_id=x['run'], seed=seeds[(s, x['run'])],
                         category=paths[x['run']][1], cls=cls, end_trans_25s=trans, end_flips_25s=flips)
                m['subcls'] = subcls(m)
                rows.append(m)
    have = [v for v in VARS if sum(r['variant'] == v for r in rows) == 384]
    rows = [r for r in rows if r['variant'] in have]
    print('variantes completas:', have)
    assert 'Y025' in have
    r4e = {(r['scenario'], r['path_id']): r['cls'] for r in csv.DictReader(open(os.path.join(LAB, 'dwb_r4e/r4e_runs.csv')))
           if r['variant'] == 'T015'}
    mism = [(r['scenario'], r['path_id']) for r in rows if r['variant'] == 'Y025' and r4e[(r['scenario'], r['path_id'])] != r['cls']]
    assert not mism, f'baseline != R4-E 0.15: {mism[:5]}'
    os.makedirs(OUT, exist_ok=True)
    first = ['variant', 'yaw_tol', 'scenario', 'path_id', 'seed', 'category', 'cls', 'subcls']
    keys = first + [k for k in rows[0] if k not in first + ['run']]
    with open(os.path.join(OUT, 'r4f_runs.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=keys, extrasaction='ignore')
        w.writeheader()
        w.writerows(rows)

    def summ_f(rr):
        s = summ(rr)
        sc = collections.Counter(r['subcls'] for r in rr)
        rx = [r for r in rr if r['reached_xy']]
        s.update(subcls=dict(sorted(sc.items())), false_yaw=sc.get('FALSE_YAW', 0) + sc.get('FALSE_XY_YAW', 0),
                 false_xy=sc.get('FALSE_XY', 0) + sc.get('FALSE_XY_YAW', 0), timeout=s['cls'].get('TIMEOUT', 0),
                 goal_tol_by_status=dict(collections.Counter(r['status'] for r in rr if r['cls'] == 'GOAL_TOLERANCE')),
                 w_flips_total=pct([r['w_sign_changes'] for r in rr]),
                 flips_pre_rot=pct([r['flips_pre_rot'] for r in rx]), flips_post_rot=pct([r['flips_post_rot'] for r in rx]),
                 retranslate_after_rot=int(sum(bool(r['retranslate_after_rot']) for r in rx)),
                 last_nz_w=pct([abs(r['last_nz_w']) for r in rr if r['status'] == 'SUCCEEDED']),
                 last_nz_v=pct([abs(r['last_nz_v']) for r in rr if r['status'] == 'SUCCEEDED']),
                 t_after_xy_fail=pct([r['t_after_xy'] for r in rr if r['cls'] == 'GOAL_TOLERANCE']))
        return s

    S = dict(variants={}, by_scenario={}, by_category={}, narrow={}, paired={})
    for v in have:
        rr = [r for r in rows if r['variant'] == v]
        S['variants'][v] = summ_f(rr)
        for s in SCEN:
            S['by_scenario'][f'{s}|{v}'] = summ_f([r for r in rr if r['scenario'] == s])
        for c in sorted({r['category'] for r in rr}):
            S['by_category'][f'{c}|{v}'] = summ_f([r for r in rr if r['category'] == c])
        for c in NARROW:
            S['narrow'][f'{c}|{v}'] = summ_f([r for r in rr if r['category'] == c])

    idx = {(r['variant'], r['scenario'], r['path_id']): r for r in rows}
    keys_run = sorted((s, k) for (v, s, k) in idx if v == 'Y025')
    ok = lambda v, s, k: idx[(v, s, k)]['cls'] == 'OK'  # noqa: E731
    base_false_yaw = [(s, k) for s, k in keys_run if idx[('Y025', s, k)]['subcls'] in ('FALSE_YAW', 'FALSE_XY_YAW')]
    base_ok = [(s, k) for s, k in keys_run if ok('Y025', s, k)]
    for v in [v for v in NEW if v in have]:
        lab = collections.Counter()
        for s, k in keys_run:
            a, b = ok('Y025', s, k), ok(v, s, k)
            lab['UNCHANGED_OK' if a and b else 'BROKEN' if a else 'FIXED' if b else 'UNCHANGED_FAIL'] += 1
        dest_fy = collections.Counter(idx[(v, s, k)]['subcls'] for s, k in base_false_yaw)
        dest_ok = collections.Counter(idx[(v, s, k)]['subcls'] for s, k in base_ok if not ok(v, s, k))
        trans = collections.Counter(f"{idx[('Y025', s, k)]['subcls']}->{idx[(v, s, k)]['subcls']}" for s, k in keys_run)
        both = [(s, k) for s, k in keys_run if ok('Y025', s, k) and ok(v, s, k)]
        S['paired'][v] = dict(
            n=len(keys_run), labels=dict(lab), transitions=dict(sorted(trans.items(), key=lambda z: -z[1])),
            base_false_yaw_n=len(base_false_yaw), base_false_yaw_dest=dict(dest_fy),
            base_false_yaw_to_ok=dest_fy.get('OK', 0),
            base_false_yaw_to_fail=sum(n for c, n in dest_fy.items() if c != 'OK' and not c.startswith('FALSE')),
            base_false_yaw_still_false=sum(n for c, n in dest_fy.items() if c.startswith('FALSE')),
            base_ok_broken=sum(dest_ok.values()), base_ok_broken_dest=dict(dest_ok),
            broken_list=sorted(f'{s}:{k}:{idx[(v, s, k)]["subcls"]}' for s, k in base_ok if not ok(v, s, k)),
            fixed_list=sorted(f'{s}:{k}:{idx[("Y025", s, k)]["subcls"]}' for s, k in keys_run if not ok('Y025', s, k) and ok(v, s, k)),
            contact_new=sorted(f'{s}:{k}' for s, k in keys_run if idx[(v, s, k)]['contact'] and not idx[('Y025', s, k)]['contact']),
            contact_removed=sorted(f'{s}:{k}' for s, k in keys_run if idx[('Y025', s, k)]['contact'] and not idx[(v, s, k)]['contact']),
            block_new=sorted(f'{s}:{k}:{idx[(v, s, k)]["cls"]}' for s, k in keys_run
                             if idx[(v, s, k)]['cls'] in BLOCK and idx[('Y025', s, k)]['cls'] not in BLOCK),
            block_removed=sorted(f'{s}:{k}' for s, k in keys_run if idx[('Y025', s, k)]['cls'] in BLOCK and idx[(v, s, k)]['cls'] not in BLOCK),
            same_clear_min=sum(1 for s, k in keys_run if idx[(v, s, k)]['clear_min'] == idx[('Y025', s, k)]['clear_min']),
            dclear=pct([idx[(v, s, k)]['clear_min'] - idx[('Y025', s, k)]['clear_min'] for s, k in keys_run]),
            dyaw_both_ok=pct([idx[(v, s, k)]['final_yaw_err'] - idx[('Y025', s, k)]['final_yaw_err'] for s, k in both]),
            dt_goal_both_ok=pct([idx[(v, s, k)]['t_goal'] - idx[('Y025', s, k)]['t_goal'] for s, k in both]),
        )
    json.dump(S, open(os.path.join(OUT, 'r4f_summary.json'), 'w'), indent=1, default=str)
    for v in have:
        x = S['variants'][v]
        print(f"{v}: correct {x['correct']} false {x['false_success']} false_yaw {x['false_yaw']} goaltol {x['goal_tol']} "
              f"{x['goal_tol_by_status']} timeout {x['timeout']} contacts {x['contacts']} bloq {x['bloqueos']} "
              f"yaw_p90 {x['yaw_true_reached']['p90']:.3f} t_goal_p50 {x['t_goal']['p50']:.1f}")
    for v, p in S['paired'].items():
        print(v, p['labels'], 'false_yaw', p['base_false_yaw_n'], '->', p['base_false_yaw_dest'], '| ok rotos ->',
              p['base_ok_broken_dest'], '| contact_new', p['contact_new'], 'block_new', p['block_new'])


if __name__ == '__main__':
    main()
