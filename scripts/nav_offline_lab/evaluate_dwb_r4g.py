#!/usr/bin/python3
"""NAV-OFFLINE-R4-G — progress checker: R4-F yaw 0.10 (existente) vs PR005 / PR003 (required_movement_radius) y
PT35 / PT45 (movement_time_allowance). Mismas 384 corridas; criterio de "correcto" fijo (pose REAL ≤ 0.25 m / 0.25 rad).

Además:
  - prueba de prefijo: el log de control (sin ms) de cada variante debe coincidir bit a bit con R4-F hasta el último ciclo
    común (el progress checker solo decide cuándo abortar). Valida también el binario nuevo (f45f04b8…) con debug OFF.
  - ABORT_PROGRESS separado: cerca del goal (llegó a la ventana xy) vs lejos (clase OSCILACION).
  - "abort ocultado": corridas que en R4-F abortaban y en la variante terminan en TIMEOUT o en éxito tardío.
"""
import collections
import csv
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from evaluate_dwb_r4a2 import Clearance, load_map, LAB  # noqa: E402
from evaluate_dwb_r4b import run_metrics  # noqa: E402
from evaluate_dwb_r4d import classify, extra  # noqa: E402
from evaluate_dwb_r4e import near_goal, pct, summ, SCEN, NARROW  # noqa: E402
from evaluate_dwb_r4f import osc_split, subcls, BLOCK  # noqa: E402

OUT = os.path.join(LAB, 'dwb_r4g')
SRC = {'BASE': ('dwb_r4f', '010'), 'PR005': ('dwb_r4g', 'PR005'), 'PR003': ('dwb_r4g', 'PR003'),
       'PT35': ('dwb_r4g', 'PT35'), 'PT45': ('dwb_r4g', 'PT45')}
FP_TOL = 0.15


def main():
    pts, _, _ = load_map(os.path.join(LAB, 'synthetic_arena/synthetic_arena.yaml'))
    clr = Clearance(pts)
    paths = {r['test_id']: (np.array(r['path']), r['category'])
             for r in json.load(open(os.path.join(LAB, 'planner_runs/paths_R3.json')))['results']}
    seeds = {(s, ln.split()[1]): ln.split()[8] for s in SCEN
             for ln in open(os.path.join(LAB, 'dwb_r4b', f'runs_{s}.txt')) if ln.startswith('RUN')}
    partial = bool(os.environ.get('R4G_PARTIAL'))
    rows, ctrl = [], {}
    for var, (sub, suf) in SRC.items():
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
            for x in L[1:]:
                m = run_metrics(x, meta, paths[x['run']][0], clr)
                cls, trans, flips = classify(x, m)
                m.update(extra(x))
                m.update(near_goal(x, m, FP_TOL))
                m.update(osc_split(x))
                m.update(variant=var, progress_radius=meta['progress_radius'], progress_time=meta['progress_time'],
                         scenario=s, path_id=x['run'], seed=seeds[(s, x['run'])], category=paths[x['run']][1], cls=cls,
                         end_trans_25s=trans, end_flips_25s=flips)
                m['subcls'] = subcls(m)
                m['abort_progress_near_goal'] = m['status'] == 'ABORT_PROGRESS' and bool(m['reached_xy'])
                m['abort_progress_far'] = m['status'] == 'ABORT_PROGRESS' and not m['reached_xy']
                rows.append(m)
                ctrl[(var, s, x['run'])] = np.array(x['ctrl'], dtype=float)[:, :12]
    have = [v for v in SRC if sum(r['variant'] == v for r in rows) == 384]
    rows = [r for r in rows if r['variant'] in have]
    print('variantes completas:', have)
    r4f = {(r['scenario'], r['path_id']): r['cls'] for r in csv.DictReader(open(os.path.join(LAB, 'dwb_r4f/r4f_runs.csv')))
           if r['variant'] == 'Y010'}
    mism = [(r['scenario'], r['path_id']) for r in rows if r['variant'] == 'BASE' and r4f[(r['scenario'], r['path_id'])] != r['cls']]
    assert not mism, f'BASE != R4-F 0.10: {mism[:5]}'
    os.makedirs(OUT, exist_ok=True)
    first = ['variant', 'progress_radius', 'progress_time', 'scenario', 'path_id', 'seed', 'category', 'cls', 'subcls', 'status']
    keys = first + [k for k in rows[0] if k not in first + ['run']]
    with open(os.path.join(OUT, 'r4g_runs.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=keys, extrasaction='ignore')
        w.writeheader()
        w.writerows(rows)

    def summ_g(rr):
        s = summ(rr)
        sc = collections.Counter(r['subcls'] for r in rr)
        succ = [r for r in rr if r['status'] == 'SUCCEEDED']
        s.update(subcls=dict(sorted(sc.items())), status=dict(collections.Counter(r['status'] for r in rr)),
                 abort_progress=sum(r['status'] == 'ABORT_PROGRESS' for r in rr),
                 abort_progress_near_goal=sum(r['abort_progress_near_goal'] for r in rr),
                 abort_progress_far=sum(r['abort_progress_far'] for r in rr), timeout=s['cls'].get('TIMEOUT', 0),
                 false_yaw=sc.get('FALSE_YAW', 0) + sc.get('FALSE_XY_YAW', 0),
                 t_end_all=pct([r['t_end'] for r in rr]), t_end_fail=pct([r['t_end'] for r in rr if r['cls'] != 'OK']),
                 t_succ_minus_xy=pct([r['t_goal'] - r['t_xy'] for r in succ if r['t_xy'] is not None]),
                 t_near_stopped=pct([r['t_near_stopped'] for r in rr if r['reached_xy']]),
                 t_near_rotating_fail=pct([r['t_near_rotating'] for r in rr if r['reached_xy'] and r['cls'] != 'OK']),
                 sum_time_all=float(sum(r['t_end'] for r in rr)))
        return s

    S = dict(variants={}, by_scenario={}, narrow={}, paired={}, prefix={})
    for v in have:
        rr = [r for r in rows if r['variant'] == v]
        S['variants'][v] = summ_g(rr)
        for s in SCEN:
            S['by_scenario'][f'{s}|{v}'] = summ_g([r for r in rr if r['scenario'] == s])
        for c in NARROW:
            S['narrow'][f'{c}|{v}'] = summ_g([r for r in rr if r['category'] == c])
    idx = {(r['variant'], r['scenario'], r['path_id']): r for r in rows}
    keys_run = sorted((s, k) for (v, s, k) in idx if v == 'BASE')
    ok = lambda v, s, k: idx[(v, s, k)]['cls'] == 'OK'  # noqa: E731
    for v in [v for v in have if v != 'BASE']:
        # prefijo idéntico hasta el último ciclo común (excluido)
        npre, bad = 0, []
        for s, k in keys_run:
            a, b = ctrl[('BASE', s, k)], ctrl[(v, s, k)]
            m = min(len(a), len(b))
            same = np.array_equal(a[:m - 1], b[:m - 1])
            npre += same
            if not same:
                bad.append(f'{s}:{k}')
        S['prefix'][v] = dict(n=len(keys_run), identical_prefix=npre, mismatches=bad, PASS=not bad)
        lab = collections.Counter()
        for s, k in keys_run:
            a, b = ok('BASE', s, k), ok(v, s, k)
            lab['UNCHANGED_OK' if a and b else 'BROKEN' if a else 'FIXED' if b else 'UNCHANGED_FAIL'] += 1
        base_ap = [(s, k) for s, k in keys_run if idx[('BASE', s, k)]['status'] == 'ABORT_PROGRESS']
        trans = collections.Counter(f"{idx[('BASE', s, k)]['subcls']}->{idx[(v, s, k)]['subcls']}" for s, k in keys_run
                                    if idx[('BASE', s, k)]['subcls'] != idx[(v, s, k)]['subcls'])
        both = [(s, k) for s, k in keys_run if ok('BASE', s, k) and ok(v, s, k)]
        S['paired'][v] = dict(
            labels=dict(lab), changed_transitions=dict(sorted(trans.items(), key=lambda z: -z[1])),
            base_abort_progress=len(base_ap),
            base_abort_progress_dest={f'{s}:{k}': f"{idx[('BASE', s, k)]['subcls']}->{idx[(v, s, k)]['subcls']} "
                                      f"(t_end {idx[('BASE', s, k)]['t_end']:.1f}->{idx[(v, s, k)]['t_end']:.1f})"
                                      for s, k in base_ap if idx[(v, s, k)]['status'] != 'ABORT_PROGRESS'
                                      or abs(idx[(v, s, k)]['t_end'] - idx[('BASE', s, k)]['t_end']) > 1e-6},
            near_goal_aborts_base=[f'{s}:{k}' for s, k in keys_run if idx[('BASE', s, k)]['abort_progress_near_goal']],
            near_goal_aborts_dest={f'{s}:{k}': f"{idx[(v, s, k)]['subcls']} t_end {idx[(v, s, k)]['t_end']:.1f}"
                                   for s, k in keys_run if idx[('BASE', s, k)]['abort_progress_near_goal']},
            hidden_abort_to_timeout=sorted(f'{s}:{k}' for s, k in base_ap if idx[(v, s, k)]['status'] == 'TIMEOUT'),
            late_success_from_abort=sorted(f'{s}:{k}' for s, k in base_ap if idx[(v, s, k)]['status'] == 'SUCCEEDED'),
            broken_list=sorted(f'{s}:{k}:{idx[(v, s, k)]["subcls"]}' for s, k in keys_run if ok('BASE', s, k) and not ok(v, s, k)),
            fixed_list=sorted(f'{s}:{k}:{idx[("BASE", s, k)]["subcls"]}' for s, k in keys_run if not ok('BASE', s, k) and ok(v, s, k)),
            contact_new=sorted(f'{s}:{k}' for s, k in keys_run if idx[(v, s, k)]['contact'] and not idx[('BASE', s, k)]['contact']),
            block_new=sorted(f'{s}:{k}:{idx[(v, s, k)]["cls"]}' for s, k in keys_run
                             if idx[(v, s, k)]['cls'] in BLOCK and idx[('BASE', s, k)]['cls'] not in BLOCK),
            dclear=pct([idx[(v, s, k)]['clear_min'] - idx[('BASE', s, k)]['clear_min'] for s, k in keys_run]),
            dt_end_all=pct([idx[(v, s, k)]['t_end'] - idx[('BASE', s, k)]['t_end'] for s, k in keys_run]),
            dt_goal_both_ok=pct([idx[(v, s, k)]['t_goal'] - idx[('BASE', s, k)]['t_goal'] for s, k in both]),
        )
    json.dump(S, open(os.path.join(OUT, 'r4g_summary.json'), 'w'), indent=1, default=str)
    for v in have:
        x = S['variants'][v]
        print(f"{v}: correct {x['correct']} AP {x['abort_progress']} (near {x['abort_progress_near_goal']} far {x['abort_progress_far']}) "
              f"timeout {x['timeout']} contacts {x['contacts']} bloq {x['bloqueos']} false {x['false_success']} "
              f"t_goal_p50 {x['t_goal']['p50']:.1f} clear_min {x['clear_min']:.3f} sum_t {x['sum_time_all']:.0f}")
    for v, p in S['paired'].items():
        print(v, 'prefix', S['prefix'][v]['PASS'], p['labels'], '| near-goal', p['near_goal_aborts_dest'], '| ->TIMEOUT',
              len(p['hidden_abort_to_timeout']), '| late succ', p['late_success_from_abort'], '| contact_new', p['contact_new'],
              '| block_new', p['block_new'])


if __name__ == '__main__':
    main()
