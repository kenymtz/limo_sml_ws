#!/usr/bin/python3
"""NAV-OFFLINE-R4-G1 — baseline replan_1hz (384) de la candidata R4-F y de la cadena R4-B..R4-E, vs fixed_plan.

Cadena (cada eslabón cambia una variable respecto del anterior):
  B_A  BaseObstacle           -> B_B  ObstacleFootprint        (pregunta 1)
  B_B  sin RotateToGoal       -> D    + RotateToGoal            (pregunta 2)
  D    FollowPath xy 0.25     -> E015 FollowPath xy 0.15        (pregunta 3)
  E015 yaw checker 0.25       -> F010 yaw 0.10 = CANDIDATA      (pregunta 4)
fixed_plan: archivos existentes (R4-B/D/E/F). replan_1hz: dwb_r4g1/sim_<S>_<TAG>.jsonl.
Clasificación idéntica a R4-C..R4-F (evaluate_dwb_r4d.classify; "correcto" = pose REAL ≤ 0.25 m / 0.25 rad, sin contacto).
Assert: las clases fixed recalculadas == r4c/r4d/r4e/r4f_runs.csv.
Uso: evaluate_dwb_r4g1.py [TAG,...]  (por defecto, todas las disponibles completas)
"""
import collections
import csv
import json
import math
import os
import sys
from math import comb

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from evaluate_dwb_r4a2 import Clearance, load_map, LAB  # noqa: E402
from evaluate_dwb_r4b import run_metrics, sign_changes  # noqa: E402
from evaluate_dwb_r4d import classify, extra  # noqa: E402
from evaluate_dwb_r4e import near_goal, pct, SCEN, NARROW  # noqa: E402
from evaluate_dwb_r4f import subcls  # noqa: E402

OUT = os.path.join(LAB, 'dwb_r4g1')
CHAIN = ['B_A', 'B_B', 'D', 'E015', 'F010']
FIXED = {'B_A': ('dwb_r4b', 'A'), 'B_B': ('dwb_r4b', 'B'), 'D': ('dwb_r4d', 'R'), 'E015': ('dwb_r4e', '015'), 'F010': ('dwb_r4f', '010')}
FP_TOL = {'B_A': None, 'B_B': None, 'D': 0.25, 'E015': 0.15, 'F010': 0.15}
NAME = {'B_A': 'R4-B A (BaseObstacle)', 'B_B': 'R4-B B (ObstacleFootprint)', 'D': 'R4-D (+RotateToGoal)',
        'E015': 'R4-E (FollowPath xy 0.15)', 'F010': 'R4-F (yaw 0.10) CANDIDATA'}
STEPS = [('B_A', 'B_B', '1 ObstacleFootprint'), ('B_B', 'D', '2 RotateToGoal'), ('D', 'E015', '3 FollowPath xy 0.15'),
         ('E015', 'F010', '4 yaw 0.10')]
BLOCK = ('OSCILACION', 'TIMEOUT', 'OTRO_SIN_TRAYECT')
REF_CSV = {'B_A': ('dwb_r4c/r4c_all_runs.csv', 'A'), 'B_B': ('dwb_r4c/r4c_all_runs.csv', 'B'), 'D': ('dwb_r4d/r4d_runs.csv', 'RTG'),
           'E015': ('dwb_r4e/r4e_runs.csv', 'T015'), 'F010': ('dwb_r4f/r4f_runs.csv', 'Y010')}


def osc_split(x, fp_tol):
    c = np.array(x['ctrl'], dtype=float)
    g = x['goal']
    d = np.hypot(c[:, 4] - g[0], c[:, 5] - g[1])
    if not (d <= 0.25).any():
        return dict(flips_pre_rot=None, flips_post_rot=None)
    i0 = int(np.argmax(d <= 0.25))
    tol = fp_tol if fp_tol is not None else 0.25
    rs = np.where((d <= tol) & (np.abs(c[:, 9]) < 0.03) & (np.abs(c[:, 10]) >= 0.1))[0]
    if not len(rs):
        return dict(flips_pre_rot=sign_changes(c[i0:, 10]), flips_post_rot=None)
    r0 = int(rs[0])
    return dict(flips_pre_rot=sign_changes(c[i0:r0, 10]), flips_post_rot=sign_changes(c[r0:, 10]))


def mcnemar(rows_a, rows_b, keys):
    b01 = sum(1 for k in keys if rows_a[k]['cls'] != 'OK' and rows_b[k]['cls'] == 'OK')
    b10 = sum(1 for k in keys if rows_a[k]['cls'] == 'OK' and rows_b[k]['cls'] != 'OK')
    n = b01 + b10
    p = min(1.0, 2 * sum(comb(n, i) for i in range(min(b01, b10) + 1)) / 2 ** n) if n else 1.0
    return dict(gained=b01, lost=b10, p=p)


def summ(rr):
    cl = collections.Counter(r['cls'] for r in rr)
    st = collections.Counter(r['status'] for r in rr)
    sc = collections.Counter(r['subcls'] for r in rr)
    succ = [r for r in rr if r['status'] == 'SUCCEEDED']
    rx = [r for r in rr if r['reached_xy']]
    return dict(
        n=len(rr), cls=dict(cl), status=dict(st), correct=cl.get('OK', 0), false_success=cl.get('OTRO_EXITO_APARENTE', 0),
        false_yaw=sc.get('FALSE_YAW', 0) + sc.get('FALSE_XY_YAW', 0), false_xy=sc.get('FALSE_XY', 0) + sc.get('FALSE_XY_YAW', 0),
        goal_tol=cl.get('GOAL_TOLERANCE', 0), abort_progress=st.get('ABORT_PROGRESS', 0),
        abort_progress_near_goal=sum(r['status'] == 'ABORT_PROGRESS' and r['reached_xy'] for r in rr),
        abort_progress_far=sum(r['status'] == 'ABORT_PROGRESS' and not r['reached_xy'] for r in rr),
        no_valid_traj=sum(r['status'] == 'ABORT_CONTROLLER' for r in rr), timeout_status=st.get('TIMEOUT', 0),
        oscilacion=cl.get('OSCILACION', 0), timeout=cl.get('TIMEOUT', 0), sin_trayect=cl.get('OTRO_SIN_TRAYECT', 0),
        tracking=cl.get('TRACKING', 0), bloqueos=sum(cl.get(c, 0) for c in BLOCK),
        contacts=int(sum(bool(r['contact']) for r in rr)), contact_time=float(sum(r['contact_time'] for r in rr)),
        margin_viol=int(sum(bool(r['margin_viol']) for r in rr)), clear_min=float(min(r['clear_min'] for r in rr)),
        clear_p05=float(np.percentile([r['clear_min'] for r in rr], 5)),
        xy_true_reached=pct([r['final_xy_err'] for r in rx]), yaw_true_reached=pct([r['final_yaw_err'] for r in rx]),
        xy_true_ok=pct([r['final_xy_err'] for r in rr if r['cls'] == 'OK']), yaw_true_ok=pct([r['final_yaw_err'] for r in rr if r['cls'] == 'OK']),
        t_goal=pct([r['t_goal'] for r in succ]), w_flips_total=pct([r['w_flips_total'] for r in rr]),
        w_flips_total_sum=int(sum(r['w_flips_total'] for r in rr)), w_flips_per_min=pct([r['w_sign_changes_per_min'] for r in rr]),
        w_flips_after_xy=pct([r['w_sign_changes_after_xy'] for r in rx]), flips_post_rot=pct([r['flips_post_rot'] for r in rx]),
        n_setplan=pct([r['n_setplan'] for r in rr]), n_plan_fail=int(sum(r['n_plan_fail'] for r in rr)),
        yaw_overshoot=pct([r.get('yaw_overshoot') for r in succ]), last_nz_rot_in_place=int(sum(r['last_nz_rot_in_place'] for r in rr)))


def main():
    pts, _, _ = load_map(os.path.join(LAB, 'synthetic_arena/synthetic_arena.yaml'))
    clr = Clearance(pts)
    paths = {r['test_id']: (np.array(r['path']), r['category'])
             for r in json.load(open(os.path.join(LAB, 'planner_runs/paths_R3.json')))['results']}
    want = sys.argv[1].split(',') if len(sys.argv) > 1 else CHAIN
    have = [t for t in want if all(os.path.exists(os.path.join(OUT, f'sim_{s}_{t}.jsonl')) and
                                   sum(1 for _ in open(os.path.join(OUT, f'sim_{s}_{t}.jsonl'))) == 65 for s in SCEN)]
    print('variantes con replan completo:', have)
    rows = []
    for tag in have:
        for mode, (sub, suf) in (('fixed', FIXED[tag]), ('replan', ('dwb_r4g1', tag))):
            for s in SCEN:
                L = [json.loads(z) for z in open(os.path.join(LAB, sub, f'sim_{s}_{suf}.jsonl'))]
                meta = L[0]['meta']
                for x in L[1:]:
                    m = run_metrics(x, meta, paths[x['run']][0], clr)
                    cls, trans, flips = classify(x, m)
                    m.update(extra(x))
                    m.update(near_goal(x, m, FP_TOL[tag]))
                    m.update(osc_split(x, FP_TOL[tag]))
                    m.update(tag=tag, mode=mode, scenario=s, path_id=x['run'], category=paths[x['run']][1], cls=cls)
                    m['subcls'] = subcls(m)
                    m['w_flips_total'] = sign_changes(np.array(x['ctrl'], float)[:, 10])
                    m['n_setplan'] = x.get('n_setplan', 1)
                    m['n_plan_fail'] = x.get('n_plan_fail', 0)
                    rows.append(m)
    # assert fixed == clases publicadas
    for tag in have:
        fn, var = REF_CSV[tag]
        ref = {(r['scenario'], r['path_id']): r['cls'] for r in csv.DictReader(open(os.path.join(LAB, fn))) if r['variant'] == var}
        mism = [(r['scenario'], r['path_id']) for r in rows if r['tag'] == tag and r['mode'] == 'fixed'
                and ref[(r['scenario'], r['path_id'])] != r['cls']]
        assert not mism, f'{tag} fixed != {fn}: {mism[:5]}'
    os.makedirs(OUT, exist_ok=True)
    first = ['tag', 'mode', 'scenario', 'path_id', 'category', 'cls', 'subcls', 'status']
    keys = first + [k for k in rows[0] if k not in first + ['run']]
    with open(os.path.join(OUT, 'r4g1_runs.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=keys, extrasaction='ignore')
        w.writeheader()
        w.writerows(rows)
    idx = {(r['tag'], r['mode'], r['scenario'], r['path_id']): r for r in rows}
    K = sorted({(s, k) for (_, _, s, k) in idx})
    S = dict(variants=have, summary={}, by_scenario={}, narrow={}, paired_fixed_vs_replan={}, steps={}, blocks46={})
    for tag in have:
        for mode in ('fixed', 'replan'):
            rr = [idx[(tag, mode, s, k)] for s, k in K]
            S['summary'][f'{tag}|{mode}'] = summ(rr)
            for sc in SCEN:
                S['by_scenario'][f'{tag}|{mode}|{sc}'] = summ([r for r in rr if r['scenario'] == sc])
            for c in NARROW + ('around_obstacle',):
                S['narrow'][f'{tag}|{mode}|{c}'] = summ([r for r in rr if r['category'] == c])
        A = {k: idx[(tag, 'fixed') + k] for k in K}
        B = {k: idx[(tag, 'replan') + k] for k in K}
        lab = collections.Counter('UNCHANGED_OK' if A[k]['cls'] == 'OK' and B[k]['cls'] == 'OK' else 'BROKEN' if A[k]['cls'] == 'OK'
                                  else 'FIXED' if B[k]['cls'] == 'OK' else 'UNCHANGED_FAIL' for k in K)
        trans = collections.Counter(f"{A[k]['subcls']}->{B[k]['subcls']}" for k in K if A[k]['subcls'] != B[k]['subcls'])
        both = [k for k in K if A[k]['cls'] == 'OK' and B[k]['cls'] == 'OK']
        S['paired_fixed_vs_replan'][tag] = dict(
            labels=dict(lab), same_class=sum(A[k]['subcls'] == B[k]['subcls'] for k in K), mcnemar=mcnemar(A, B, K),
            transitions=dict(sorted(trans.items(), key=lambda z: -z[1])),
            broken=sorted(f'{s}:{k}:{B[(s, k)]["subcls"]}' for s, k in K if A[(s, k)]['cls'] == 'OK' and B[(s, k)]['cls'] != 'OK'),
            fixed=sorted(f'{s}:{k}:{A[(s, k)]["subcls"]}' for s, k in K if A[(s, k)]['cls'] != 'OK' and B[(s, k)]['cls'] == 'OK'),
            contact_new=sorted(f'{s}:{k}' for s, k in K if B[(s, k)]['contact'] and not A[(s, k)]['contact']),
            contact_removed=sorted(f'{s}:{k}' for s, k in K if A[(s, k)]['contact'] and not B[(s, k)]['contact']),
            dt_goal_both_ok=pct([B[k]['t_goal'] - A[k]['t_goal'] for k in both]),
            dxy_both_ok=pct([B[k]['final_xy_err'] - A[k]['final_xy_err'] for k in both]),
            dyaw_both_ok=pct([B[k]['final_yaw_err'] - A[k]['final_yaw_err'] for k in both]),
            dflips=pct([B[k]['w_flips_total'] - A[k]['w_flips_total'] for k in K]),
            abort_progress_list=sorted(f'{s}:{k}:{"near" if B[(s, k)]["reached_xy"] else "far"}' for s, k in K
                                       if B[(s, k)]['status'] == 'ABORT_PROGRESS'))
    for a, b, name in STEPS:
        if a in have and b in have:
            st = {}
            for mode in ('fixed', 'replan'):
                A = {k: idx[(a, mode) + k] for k in K}
                B = {k: idx[(b, mode) + k] for k in K}
                sa, sb = S['summary'][f'{a}|{mode}'], S['summary'][f'{b}|{mode}']
                st[mode] = dict(mcnemar=mcnemar(A, B, K), correct=(sa['correct'], sb['correct']), contacts=(sa['contacts'], sb['contacts']),
                                margin_viol=(sa['margin_viol'], sb['margin_viol']), bloqueos=(sa['bloqueos'], sb['bloqueos']),
                                goal_tol=(sa['goal_tol'], sb['goal_tol']), false_success=(sa['false_success'], sb['false_success']),
                                false_yaw=(sa['false_yaw'], sb['false_yaw']), abort_progress_near=(sa['abort_progress_near_goal'], sb['abort_progress_near_goal']),
                                yaw_p90=(sa['yaw_true_reached']['p90'], sb['yaw_true_reached']['p90']),
                                xy_p50=(sa['xy_true_reached']['p50'], sb['xy_true_reached']['p50']),
                                flips_after_xy_mean=(sa['w_flips_after_xy']['mean'], sb['w_flips_after_xy']['mean']))
            S['steps'][name] = st
    if 'B_B' in have:
        blk = [k for k in K if idx[('B_B', 'fixed') + k]['cls'] in BLOCK]
        S['blocks46'] = dict(n=len(blk), list=[f'{s}:{k}' for s, k in blk])
        for tag in have:
            for mode in ('fixed', 'replan'):
                c = collections.Counter(idx[(tag, mode) + k]['cls'] for k in blk)
                S['blocks46'][f'{tag}|{mode}'] = dict(still_block=sum(c[x] for x in BLOCK), dest=dict(c))
    json.dump(S, open(os.path.join(OUT, 'r4g1_summary.json'), 'w'), indent=1, default=str)
    for tag in have:
        for mode in ('fixed', 'replan'):
            x = S['summary'][f'{tag}|{mode}']
            print(f"{tag:5s} {mode:6s} correct {x['correct']} false {x['false_success']} (yaw {x['false_yaw']}) goaltol {x['goal_tol']} "
                  f"AP {x['abort_progress']} (near {x['abort_progress_near_goal']}) novalid {x['no_valid_traj']} timeout {x['timeout_status']} "
                  f"contacts {x['contacts']} bloq {x['bloqueos']} flips {x['w_flips_total_sum']} setplan_p50 {x['n_setplan']['p50']}")
        p = S['paired_fixed_vs_replan'][tag]
        print('   paired', p['labels'], 'mcnemar', p['mcnemar'], 'contact_new', p['contact_new'], 'removed', p['contact_removed'])
    for name, st in S['steps'].items():
        print('STEP', name, {m: (v['correct'], v['mcnemar']) for m, v in st.items()})
    if S['blocks46']:
        print('blocks46', {k: v for k, v in S['blocks46'].items() if k not in ('list',)})


if __name__ == '__main__':
    main()
