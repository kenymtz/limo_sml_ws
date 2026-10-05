#!/usr/bin/python3
"""NAV-OFFLINE-R4-G0 — fixed_plan vs replan_1hz (NavFn real a 1 Hz) vs replan_same_1hz (ablación: mismo path) en 25 casos.

Por caso (scenario + path_id, misma semilla):
  - fixed_plan del binario nuevo vs R4-F 0.10 (dwb_r4f/sim_<S>_010.jsonl): log de control idéntico bit a bit (sin ms)
  - outcome/clase (reglas R4-C..R4-F), métricas (evaluate_dwb_r4b.run_metrics), primer ciclo en que el comando difiere
  - setPlan: número, tiempos, fallos de NavFn, desvío geométrico del path nuevo respecto del original
  - Oscillation: ilegales por Oscillation en el ciclo anterior a cada setPlan vs el ciclo del setPlan (reset observable)
Salidas: dwb_r4g0/r4g0_cases.csv, r4g0_summary.json
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
from evaluate_dwb_r4e import near_goal  # noqa: E402
from evaluate_dwb_r4f import osc_split, subcls  # noqa: E402

OUT = os.path.join(LAB, 'dwb_r4g0')
SCEN = ['S1_nominal', 'S2_ruido_xy', 'S4_latencia_0p6', 'S5_combinado_medido']
MODES = ['fixed_plan', 'replan_1hz', 'replan_same_1hz']
WALL = {'wall_s', 'dwb_ms_mean', 'dwb_ms_max', 'updatemap_ms_mean', 'updatemap_ms_max'}


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def load(fn):
    L = [json.loads(z) for z in open(fn)]
    return L[0]['meta'], {x['run']: x for x in L[1:]}


def load_eval(fn):
    return {e['run']: e for e in (json.loads(z) for z in open(fn))} if os.path.exists(fn) else {}


def path_dev(newp, ref):
    """máxima distancia de los puntos del path nuevo al polilínea original"""
    ref = np.asarray(ref)[:, :2]
    best = 0.0
    for p in newp:
        d = np.hypot(ref[:, 0] - p[0], ref[:, 1] - p[1]).min()
        best = max(best, d)
    return best


def main():
    pts, _, _ = load_map(os.path.join(LAB, 'synthetic_arena/synthetic_arena.yaml'))
    clr = Clearance(pts)
    paths = {r['test_id']: (np.array(r['path']), r['category'])
             for r in json.load(open(os.path.join(LAB, 'planner_runs/paths_R3.json')))['results']}
    rows, fixed_vs_r4f = [], {}
    raw = {}
    for s in SCEN:
        _, R4F = load(os.path.join(LAB, 'dwb_r4f', f'sim_{s}_010.jsonl'))
        for mode in MODES:
            fn = os.path.join(OUT, f'sim_{s}_{mode}.jsonl')
            meta, X = load(fn)
            EV = load_eval(fn + '.eval.jsonl')
            for k, x in X.items():
                raw[(mode, s, k)] = (x, EV.get(k))
                m = run_metrics(x, meta, paths[k][0], clr)
                cls, trans, flips = classify(x, m)
                m.update(extra(x))
                m.update(near_goal(x, m, 0.15))
                m.update(osc_split(x))
                m.update(mode=mode, scenario=s, path_id=k, category=paths[k][1], cls=cls)
                m['subcls'] = subcls(m)
                m['n_setplan'] = x.get('n_setplan', 1)
                m['n_plan_fail'] = x.get('n_plan_fail', 0)
                sp = x.get('setplan', [])
                m['setplan_times'] = [e[0] for e in sp]
                newpaths_dev = []
                if mode == 'replan_1hz':
                    # desvío del primer punto del plan respecto de la pose estimada y largo
                    m['setplan_first_pose_dist_max'] = max([e[6] for e in sp if e[6] is not None] or [0])
                m['w_flips_total'] = sign_changes(np.array(x['ctrl'], float)[:, 10])
                if mode == 'fixed_plan':
                    a = {kk: v for kk, v in R4F[k].items() if kk not in WALL}
                    b = {kk: v for kk, v in x.items() if kk not in WALL}
                    a['ctrl'] = [r[:12] for r in a['ctrl']]
                    b['ctrl'] = [r[:12] for r in b['ctrl']]
                    fixed_vs_r4f[f'{s}:{k}'] = a == b
                rows.append(m)
    idx = {(r['mode'], r['scenario'], r['path_id']): r for r in rows}
    cases = sorted({(s, k) for (_, s, k) in idx})
    # comparación por caso
    out_cases = []
    for s, k in cases:
        f, rp, rs = (idx[(md, s, k)] for md in MODES)
        cf = np.array(raw[('fixed_plan', s, k)][0]['ctrl'], float)[:, :12]
        rec = dict(scenario=s, path_id=k, category=f['category'],
                   fixed=f['subcls'], replan=rp['subcls'], same=rs['subcls'],
                   fixed_status=f['status'], replan_status=rp['status'], same_status=rs['status'])
        for md, r in (('replan', rp), ('same', rs)):
            c = np.array(raw[({'replan': 'replan_1hz', 'same': 'replan_same_1hz'}[md], s, k)][0]['ctrl'], float)[:, :12]
            m_ = min(len(c), len(cf))
            diff = np.where(np.any(c[:m_, 9:11] != cf[:m_, 9:11], axis=1))[0]
            rec[f'{md}_identical'] = bool(len(c) == len(cf) and np.array_equal(c, cf) and r['status'] == f['status'])
            rec[f'{md}_first_cmd_diff_t'] = float(cf[diff[0], 0]) if len(diff) else None
            rec[f'{md}_same_class'] = r['subcls'] == f['subcls']
            rec[f'{md}_n_setplan'] = r['n_setplan']
            rec[f'{md}_n_plan_fail'] = r['n_plan_fail']
            for q in ('t_goal', 't_end', 'final_xy_err', 'final_yaw_err', 'clear_min', 'w_flips_total', 'flips_post_rot', 'contact'):
                rec[f'{md}_{q}'] = r[q]
        for q in ('t_goal', 't_end', 'final_xy_err', 'final_yaw_err', 'clear_min', 'w_flips_total', 'flips_post_rot', 'contact'):
            rec[f'fixed_{q}'] = f[q]
        # geometría de los paths de replan_1hz: desvío máximo respecto del path original
        x_rp = raw[('replan_1hz', s, k)][0]
        rec['replan_plan0_dev_vs_R3'] = path_dev(x_rp['plan0'], paths[k][0]) if x_rp.get('plan0') else None
        # Oscillation: reset observable en cada setPlan (ilegales por Oscillation antes vs en el ciclo del setPlan)
        for md, mode in (('replan', 'replan_1hz'), ('same', 'replan_same_1hz')):
            x, ev = raw[(mode, s, k)]
            if not ev:
                continue
            ts = [round(c[0], 6) for c in ev['cycles']]
            osc = [c[6].get('Oscillation', 0) for c in ev['cycles']]
            before_after = []
            for e in x.get('setplan', [])[1:]:
                if not e[1]:
                    continue
                t_sp = round(e[0], 6)
                if t_sp in ts:
                    i = ts.index(t_sp)
                    if i > 0:
                        before_after.append((osc[i - 1], osc[i]))
            n_active = sum(1 for b, a in before_after if b > 0)
            n_reset = sum(1 for b, a in before_after if b > 0 and a == 0)
            rec[f'{md}_osc_active_before_setplan'] = n_active
            rec[f'{md}_osc_cleared_at_setplan'] = n_reset
            rec[f'{md}_osc_illegal_cycles'] = sum(1 for o in osc if o > 0)
        _, evf = raw[('fixed_plan', s, k)]
        if evf:
            rec['fixed_osc_illegal_cycles'] = sum(1 for c in evf['cycles'] if c[6].get('Oscillation', 0) > 0)
        out_cases.append(rec)
    os.makedirs(OUT, exist_ok=True)
    keys = list(out_cases[0])
    for r in out_cases:
        for kk in r:
            if kk not in keys:
                keys.append(kk)
    with open(os.path.join(OUT, 'r4g0_cases.csv'), 'w', newline='') as fh:
        w = csv.DictWriter(fh, fieldnames=keys)
        w.writeheader()
        w.writerows(out_cases)

    def agg(md):
        mode = {'fixed': 'fixed_plan', 'replan': 'replan_1hz', 'same': 'replan_same_1hz'}[md]
        rr = [idx[(mode, s, k)] for s, k in cases]
        return dict(correct=sum(r['cls'] == 'OK' for r in rr),
                    abort_progress=sum(r['status'] == 'ABORT_PROGRESS' for r in rr),
                    abort_progress_near_goal=sum(r['status'] == 'ABORT_PROGRESS' and r['reached_xy'] for r in rr),
                    goal_tolerance=sum(r['cls'] == 'GOAL_TOLERANCE' for r in rr),
                    oscilacion=sum(r['cls'] == 'OSCILACION' for r in rr), timeout=sum(r['cls'] == 'TIMEOUT' for r in rr),
                    sin_trayect=sum(r['cls'] == 'OTRO_SIN_TRAYECT' for r in rr),
                    false_success=sum(r['cls'] == 'OTRO_EXITO_APARENTE' for r in rr),
                    contacts=sum(bool(r['contact']) for r in rr), clear_min=min(r['clear_min'] for r in rr),
                    classes=dict(collections.Counter(r['subcls'] for r in rr)),
                    w_flips_total_sum=int(sum(r['w_flips_total'] for r in rr)),
                    n_setplan_sum=int(sum(r['n_setplan'] for r in rr)), n_plan_fail_sum=int(sum(r['n_plan_fail'] for r in rr)))
    S = dict(n_cases=len(cases), fixed_identical_to_R4F=fixed_vs_r4f,
             fixed_identical_to_R4F_all=all(fixed_vs_r4f.values()),
             agg={md: agg(md) for md in ('fixed', 'replan', 'same')},
             replan_identical=sum(r['replan_identical'] for r in out_cases),
             replan_same_class=sum(r['replan_same_class'] for r in out_cases),
             same_identical=sum(r['same_identical'] for r in out_cases),
             same_same_class=sum(r['same_same_class'] for r in out_cases),
             changed_outcome_replan=[f"{r['scenario']}:{r['path_id']} {r['fixed']}->{r['replan']}" for r in out_cases if not r['replan_same_class']],
             changed_outcome_same=[f"{r['scenario']}:{r['path_id']} {r['fixed']}->{r['same']}" for r in out_cases if not r['same_same_class']])
    json.dump(S, open(os.path.join(OUT, 'r4g0_summary.json'), 'w'), indent=1, default=str)
    print(json.dumps({k: v for k, v in S.items() if k != 'fixed_identical_to_R4F'}, indent=1, default=str))


if __name__ == '__main__':
    main()
