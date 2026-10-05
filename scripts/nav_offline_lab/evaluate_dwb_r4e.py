#!/usr/bin/python3
"""NAV-OFFLINE-R4-E — FollowPath.xy_goal_tolerance (RotateToGoal) 0.25 (R4-D, existente) vs 0.20 / 0.15 / 0.10.

Variantes (todas con las mismas 384 corridas = 6 escenarios × 64 paths, mismas semillas):
  NORTG  = R4-B variante B (sin RotateToGoal)            dwb_r4b/sim_<S>_B.jsonl   (solo para la lista de 122/46 de R4-D)
  T025   = R4-D (RotateToGoal, FollowPath xy tol 0.25)   dwb_r4d/sim_<S>_R.jsonl   (baseline, NO re-ejecutada)
  T020/T015/T010 = R4-E                                  dwb_r4e/sim_<S>_<tag>.jsonl
Clasificación y métricas: idénticas a R4-C/R4-D (evaluate_dwb_r4d.classify / extra, evaluate_dwb_r4b.run_metrics).
Assert: las clases recalculadas de NORTG y T025 coinciden con dwb_r4d/r4d_runs.csv.
Métricas nuevas: pose final ESTIMADA, p95, tiempo cerca del goal (girando / detenido), distancia al goal al empezar a girar.
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
from evaluate_dwb_r4b import run_metrics  # noqa: E402
from evaluate_dwb_r4d import classify, extra, wrap, CLS  # noqa: E402

SCEN = ['S1_nominal', 'S2_ruido_xy', 'S3_ruido_yaw', 'S4_latencia_0p6', 'S5_combinado_medido', 'S6_combinado_lat0p6']
OUT = os.path.join(LAB, 'dwb_r4e')
SRC = {'NORTG': ('dwb_r4b', 'B', None), 'T025': ('dwb_r4d', 'R', 0.25), 'T020': ('dwb_r4e', '020', 0.20),
       'T015': ('dwb_r4e', '015', 0.15), 'T010': ('dwb_r4e', '010', 0.10)}
RTG_VARS = ['T025', 'T020', 'T015', 'T010']
NEW_VARS = ['T020', 'T015', 'T010']
NARROW = ('corridor_N1', 'corridor_N2')
V_STOP, W_ROT = 0.03, 0.1   # = trans_stopped_velocity / umbral de giro usado en R4-D


def near_goal(x, m, fp_tol):
    """pose final estimada y dinámica cerca del goal"""
    c = np.array(x['ctrl'], dtype=float)   # t x y th ex ey eth v w cv cw ok ms
    g = x['goal']
    fx = x['final_true']
    ox, oy, oth = c[-1, 4] - c[-1, 1], c[-1, 5] - c[-1, 2], wrap(c[-1, 6] - c[-1, 3])   # offset de localización final
    ex, ey, eth = fx[0] + ox, fx[1] + oy, fx[2] + oth
    d = dict(final_xy_err_est=float(math.hypot(ex - g[0], ey - g[1])),
             final_yaw_err_est=float(abs(wrap(eth - g[2]))),
             trig_xy_err_est=float(math.hypot(c[-1, 4] - g[0], c[-1, 5] - g[1])),
             trig_yaw_err_est=float(abs(wrap(c[-1, 6] - g[2]))))
    dt = float(np.median(np.diff(c[:, 0]))) if len(c) > 1 else 0.1
    d_est = np.hypot(c[:, 4] - g[0], c[:, 5] - g[1])
    near = d_est <= 0.25   # ventana del goal checker (igual en todas las variantes)
    cv, cw = np.abs(c[:, 9]), np.abs(c[:, 10])
    d.update(t_near_goal=float(near.sum() * dt),
             t_near_rotating=float((near & (cv < V_STOP) & (cw >= W_ROT)).sum() * dt),
             t_near_stopped=float((near & (cv < V_STOP) & (cw < W_ROT)).sum() * dt),
             t_near_translating=float((near & (cv >= V_STOP)).sum() * dt))
    # distancia (estimada y verdadera) al goal cuando RotateToGoal empieza a girar en el lugar dentro de su ventana
    rs = None
    if fp_tol is not None:
        inw = np.where((d_est <= fp_tol) & (cv < V_STOP) & (cw >= W_ROT))[0]
        rs = int(inw[0]) if len(inw) else None
    d['rot_start_xy_est'] = float(d_est[rs]) if rs is not None else None
    d['rot_start_xy_true'] = float(math.hypot(c[rs, 1] - g[0], c[rs, 2] - g[1])) if rs is not None else None
    return d


def pct(v):
    v = [a for a in v if a is not None and not (isinstance(a, float) and math.isnan(a))]
    if not v:
        return None
    return dict(n=len(v), min=float(np.min(v)), p50=float(np.median(v)), p90=float(np.percentile(v, 90)), p95=float(np.percentile(v, 95)),
                max=float(np.max(v)), mean=float(np.mean(v)))


def summ(rr):
    cl = collections.Counter(r['cls'] for r in rr)
    succ = [r for r in rr if r['status'] == 'SUCCEEDED']
    rx = [r for r in rr if r['reached_xy']]
    return dict(
        n=len(rr), cls={c: cl.get(c, 0) for c in CLS}, correct=cl.get('OK', 0), goal_tol=cl.get('GOAL_TOLERANCE', 0),
        false_success=cl.get('OTRO_EXITO_APARENTE', 0), succeeded=len(succ), oscilacion=cl.get('OSCILACION', 0),
        contacts=int(sum(r['contact'] for r in rr)), contact_time=float(sum(r['contact_time'] for r in rr)),
        margin_viol=int(sum(r['margin_viol'] for r in rr)), clear_min=float(min(r['clear_min'] for r in rr)),
        clear_min_p05=float(np.percentile([r['clear_min'] for r in rr], 5)),
        bloqueos=cl.get('OSCILACION', 0) + cl.get('TIMEOUT', 0) + cl.get('OTRO_SIN_TRAYECT', 0),
        xy_true_reached=pct([r['final_xy_err'] for r in rx]), xy_est_reached=pct([r['final_xy_err_est'] for r in rx]),
        yaw_true_reached=pct([r['final_yaw_err'] for r in rx]), yaw_est_reached=pct([r['final_yaw_err_est'] for r in rx]),
        xy_true_succ=pct([r['final_xy_err'] for r in succ]), xy_est_succ=pct([r['final_xy_err_est'] for r in succ]),
        yaw_true_succ=pct([r['final_yaw_err'] for r in succ]), yaw_est_succ=pct([r['final_yaw_err_est'] for r in succ]),
        xy_true_ok=pct([r['final_xy_err'] for r in rr if r['cls'] == 'OK']),
        yaw_true_ok=pct([r['final_yaw_err'] for r in rr if r['cls'] == 'OK']),
        t_goal=pct([r['t_goal'] for r in succ]), t_xy=pct([r['t_xy'] for r in rx]),
        t_goal_minus_xy=pct([r['t_goal'] - r['t_xy'] for r in succ if r['t_xy'] is not None]),
        t_near_goal=pct([r['t_near_goal'] for r in rx]), t_near_rotating=pct([r['t_near_rotating'] for r in rx]),
        t_near_stopped=pct([r['t_near_stopped'] for r in rx]), t_near_translating=pct([r['t_near_translating'] for r in rx]),
        rot_start_xy_est=pct([r['rot_start_xy_est'] for r in rx]), rot_start_xy_true=pct([r['rot_start_xy_true'] for r in rx]),
        w_flips_per_min=pct([r['w_sign_changes_per_min'] for r in rr]),
        w_flips_after_xy=pct([r['w_sign_changes_after_xy'] for r in rx]),
        yaw_overshoot=pct([r.get('yaw_overshoot') for r in succ]), w_at_trigger=pct([r['w_at_end'] for r in succ]),
        last_nz_rot_in_place=int(sum(r['last_nz_rot_in_place'] for r in rr)),
        dwb_ms=pct([r['dwb_ms_mean'] for r in rr]), n_fail_dwb=int(sum(r['n_fail'] for r in rr)),
    )


def main():
    pts, _, _ = load_map(os.path.join(LAB, 'synthetic_arena/synthetic_arena.yaml'))
    clr = Clearance(pts)
    paths = {r['test_id']: (np.array(r['path']), r['category'])
             for r in json.load(open(os.path.join(LAB, 'planner_runs/paths_R3.json')))['results']}
    rows, missing = [], []
    for var, (sub, suf, tol) in SRC.items():
        for s in SCEN:
            fn = os.path.join(LAB, sub, f'sim_{s}_{suf}.jsonl')
            if not os.path.exists(fn):
                missing.append(fn)
                continue
            L = [json.loads(z) for z in open(fn)]
            meta = L[0]['meta']
            if len(L) != 65 and os.environ.get('R4E_PARTIAL'):
                missing.append(fn)
                continue
            assert len(L) == 65, (fn, len(L))
            for x in L[1:]:
                m = run_metrics(x, meta, paths[x['run']][0], clr)
                cls, trans, flips = classify(x, m)
                m.update(extra(x))
                m.update(near_goal(x, m, tol))
                m.update(variant=var, fp_xy_tol=tol, scenario=s, category=paths[x['run']][1], cls=cls,
                         end_trans_25s=trans, end_flips_25s=flips, path_id=x['run'])
                rows.append(m)
    if os.environ.get('R4E_PARTIAL'):   # prueba con datos incompletos: solo variantes con los 6 escenarios
        have = {v for v in SRC if sum(r['variant'] == v for r in rows) == 384}
        rows = [r for r in rows if r['variant'] in have]
        for v in [v for v in SRC if v not in have]:
            SRC.pop(v)
            for L_ in (RTG_VARS, NEW_VARS):
                if v in L_:
                    L_.remove(v)
        print('PARCIAL, variantes completas:', sorted(have))
    else:
        assert not missing, missing
    # verificación: NORTG y T025 == clases publicadas en R4-D
    r4d = {({'BASE': 'NORTG', 'RTG': 'T025'}[r['variant']], r['scenario'], r['path_id']): r['cls']
           for r in csv.DictReader(open(os.path.join(LAB, 'dwb_r4d/r4d_runs.csv')))}
    mism = [(r['variant'], r['scenario'], r['path_id']) for r in rows if r['variant'] in ('NORTG', 'T025')
            and r4d[(r['variant'], r['scenario'], r['path_id'])] != r['cls']]
    assert not mism, f'no coincide con R4-D: {mism[:5]}'
    os.makedirs(OUT, exist_ok=True)
    first = ['variant', 'fp_xy_tol', 'scenario', 'path_id', 'category', 'cls']
    keys = first + [k for k in rows[0] if k not in first + ['run']]
    with open(os.path.join(OUT, 'r4e_runs.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=keys, extrasaction='ignore')
        w.writeheader()
        w.writerows(rows)

    S = dict(variants={}, by_scenario={}, by_category={}, narrow={}, paired={}, r4d_sets={})
    for var in SRC:
        rr = [r for r in rows if r['variant'] == var]
        S['variants'][var] = summ(rr)
        for s in SCEN:
            S['by_scenario'][f'{s}|{var}'] = summ([r for r in rr if r['scenario'] == s])
        for c in sorted({r['category'] for r in rr}):
            S['by_category'][f'{c}|{var}'] = summ([r for r in rr if r['category'] == c])
        for c in NARROW:
            S['narrow'][f'{c}|{var}'] = summ([r for r in rr if r['category'] == c])

    idx = {(r['variant'], r['scenario'], r['path_id']): r for r in rows}
    keys_run = sorted({(s, k) for (v, s, k) in idx if v == 'T025'})
    ok = lambda v, s, k: idx[(v, s, k)]['cls'] == 'OK'  # noqa: E731
    broken_by_r4d = [(s, k) for s, k in keys_run if ok('NORTG', s, k) and not ok('T025', s, k)]
    fixed_by_r4d = [(s, k) for s, k in keys_run if not ok('NORTG', s, k) and ok('T025', s, k)]
    assert len(broken_by_r4d) == 46 and len(fixed_by_r4d) == 122, (len(broken_by_r4d), len(fixed_by_r4d))
    S['r4d_sets'] = dict(broken_46=[f'{s}:{k}' for s, k in broken_by_r4d], fixed_122=[f'{s}:{k}' for s, k in fixed_by_r4d])
    for var in NEW_VARS:
        lab = collections.Counter()
        for s, k in keys_run:
            a, b = ok('T025', s, k), ok(var, s, k)
            lab['UNCHANGED_OK' if a and b else 'BROKEN' if a else 'FIXED' if b else 'UNCHANGED_FAIL'] += 1
        trans = collections.Counter(f"{idx[('T025', s, k)]['cls']}->{idx[(var, s, k)]['cls']}" for s, k in keys_run)
        rec46 = [(s, k) for s, k in broken_by_r4d if ok(var, s, k)]
        still46 = collections.Counter(idx[(var, s, k)]['cls'] for s, k in broken_by_r4d if not ok(var, s, k))
        rebroken122 = [(s, k) for s, k in fixed_by_r4d if not ok(var, s, k)]
        both_ok = [(s, k) for s, k in keys_run if ok('T025', s, k) and ok(var, s, k)]
        S['paired'][var] = dict(
            n=len(keys_run), labels=dict(lab), transitions=dict(sorted(trans.items(), key=lambda z: -z[1])),
            fixed_list=sorted(f'{s}:{k}:{idx[("T025", s, k)]["cls"]}' for s, k in keys_run if not ok('T025', s, k) and ok(var, s, k)),
            broken_list=sorted(f'{s}:{k}:{idx[(var, s, k)]["cls"]}' for s, k in keys_run if ok('T025', s, k) and not ok(var, s, k)),
            r4d_broken46_recovered=len(rec46), r4d_broken46_still_fail=dict(still46),
            r4d_fixed122_rebroken=len(rebroken122),
            r4d_fixed122_rebroken_list=sorted(f'{s}:{k}:{idx[(var, s, k)]["cls"]}' for s, k in rebroken122),
            contact_new=sorted(f'{s}:{k}' for s, k in keys_run if idx[(var, s, k)]['contact'] and not idx[('T025', s, k)]['contact']),
            contact_removed=sorted(f'{s}:{k}' for s, k in keys_run if idx[('T025', s, k)]['contact'] and not idx[(var, s, k)]['contact']),
            block_new=sorted(f'{s}:{k}:{idx[(var, s, k)]["cls"]}' for s, k in keys_run
                             if idx[(var, s, k)]['cls'] in ('OSCILACION', 'TIMEOUT', 'OTRO_SIN_TRAYECT')
                             and idx[('T025', s, k)]['cls'] not in ('OSCILACION', 'TIMEOUT', 'OTRO_SIN_TRAYECT')),
            block_removed=sorted(f'{s}:{k}:{idx[("T025", s, k)]["cls"]}' for s, k in keys_run
                                 if idx[('T025', s, k)]['cls'] in ('OSCILACION', 'TIMEOUT', 'OTRO_SIN_TRAYECT')
                                 and idx[(var, s, k)]['cls'] not in ('OSCILACION', 'TIMEOUT', 'OTRO_SIN_TRAYECT')),
            dxy_true_both_ok=pct([idx[(var, s, k)]['final_xy_err'] - idx[('T025', s, k)]['final_xy_err'] for s, k in both_ok]),
            dyaw_true_both_ok=pct([idx[(var, s, k)]['final_yaw_err'] - idx[('T025', s, k)]['final_yaw_err'] for s, k in both_ok]),
            dt_goal_both_ok=pct([idx[(var, s, k)]['t_goal'] - idx[('T025', s, k)]['t_goal'] for s, k in both_ok]),
            dclear_narrow=pct([idx[(var, s, k)]['clear_min'] - idx[('T025', s, k)]['clear_min'] for s, k in keys_run
                               if idx[('T025', s, k)]['category'] in NARROW]),
        )
    json.dump(S, open(os.path.join(OUT, 'r4e_summary.json'), 'w'), indent=1, default=str)

    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(1, 3, figsize=(18, 4.8))
        cols = {'T025': 'tab:gray', 'T020': 'tab:blue', 'T015': 'tab:green', 'T010': 'tab:orange'}
        xs = np.arange(len(SCEN))
        for i, var in enumerate(RTG_VARS):
            ax[0].bar(xs + 0.2 * i, [S['by_scenario'][f'{s}|{var}']['correct'] for s in SCEN], 0.2, color=cols[var], label=var)
        ax[0].set_xticks(xs + 0.3)
        ax[0].set_xticklabels([s.split('_', 1)[1] for s in SCEN], rotation=25, fontsize=8)
        ax[0].set_title('SUCCEEDED correcto (de 64)')
        ax[0].legend(fontsize=8)
        for var in RTG_VARS:
            rr = [r for r in rows if r['variant'] == var and r['reached_xy']]
            ax[1].hist([r['final_xy_err'] for r in rr], bins=50, range=(0, 0.35), histtype='step', lw=1.5, color=cols[var], label=var)
            ax[2].hist([r['final_yaw_err'] for r in rr], bins=50, range=(0, 0.6), histtype='step', lw=1.5, color=cols[var], label=var)
        for a_ in ax[1:]:
            a_.axvline(0.25, color='k', ls=':')
            a_.legend(fontsize=8)
        ax[1].set_xlabel('error xy final VERDADERO (m), corridas que llegan en xy')
        ax[2].set_xlabel('error yaw final VERDADERO (rad), corridas que llegan en xy (recortado 0.6)')
        fig.tight_layout()
        fig.savefig(os.path.join(OUT, 'r4e_overview.png'), dpi=110)
    except Exception as ex:  # noqa
        print('figura no generada:', ex)

    for var in SRC:
        v = S['variants'][var]
        print(f"{var}: correct {v['correct']} goaltol {v['goal_tol']} false {v['false_success']} contacts {v['contacts']} "
              f"bloq {v['bloqueos']} xy_p90 {v['xy_true_reached']['p90']:.3f} yaw_p90 {v['yaw_true_reached']['p90']:.3f} "
              f"clear_min {v['clear_min']:.3f}")
    for var in NEW_VARS:
        p = S['paired'][var]
        print(var, p['labels'], '46rec', p['r4d_broken46_recovered'], '122rebroken', p['r4d_fixed122_rebroken'],
              'contact_new', p['contact_new'], 'block_new', len(p['block_new']), 'block_removed', len(p['block_removed']))


if __name__ == '__main__':
    main()
