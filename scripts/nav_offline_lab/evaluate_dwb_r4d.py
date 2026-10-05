#!/usr/bin/python3
"""NAV-OFFLINE-R4-D — BASELINE (ObstacleFootprint, R4-B variante B) vs EXPERIMENTAL (+ RotateToGoal).

Mismas métricas (evaluate_dwb_r4b.run_metrics) y mismas reglas de clasificación que R4-C; se verifica que la
clasificación de la baseline coincide EXACTAMENTE con la de R4-C para la variante B.
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

SCEN = ['S1_nominal', 'S2_ruido_xy', 'S3_ruido_yaw', 'S4_latencia_0p6', 'S5_combinado_medido', 'S6_combinado_lat0p6']
OUT = os.path.join(LAB, 'dwb_r4d')
TOL = 0.25
SRC = {'BASE': ('dwb_r4b', 'B'), 'RTG': ('dwb_r4d', 'R')}
CLS = ['OK', 'GOAL_TOLERANCE', 'OSCILACION', 'TIMEOUT', 'TRACKING', 'OTRO_SIN_TRAYECT', 'OTRO_EXITO_CONTACTO',
       'OTRO_EXITO_APARENTE', 'OTRO']


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def classify(x, m):
    """reglas idénticas a analyze_r4c_failures.py"""
    c = np.array(x['ctrl'], dtype=float)
    last = c[c[:, 0] >= c[-1, 0] - 25.0]
    trans = float(np.hypot(last[-1, 1] - last[0, 1], last[-1, 2] - last[0, 2]))
    s = [np.sign(w) for w in last[:, 10] if abs(w) > 0.05]
    flips = sum(1 for a, b in zip(s, s[1:]) if a != b)
    st = x['status']
    if st == 'SUCCEEDED':
        if m['contact']:
            return 'OTRO_EXITO_CONTACTO', trans, flips
        return ('OK' if m['final_xy_err'] <= TOL and m['final_yaw_err'] <= TOL else 'OTRO_EXITO_APARENTE'), trans, flips
    if m['reached_xy']:
        return 'GOAL_TOLERANCE', trans, flips
    if m['lat_max'] > TOL:
        return 'TRACKING', trans, flips
    if st == 'ABORT_PROGRESS' and trans < 0.1 and flips >= 4:
        return 'OSCILACION', trans, flips
    if st == 'TIMEOUT':
        return 'TIMEOUT', trans, flips
    if st == 'ABORT_CONTROLLER':
        return 'OTRO_SIN_TRAYECT', trans, flips
    return 'OTRO', trans, flips


def extra(x):
    """comandos finales y dinámica en el disparo del goal checker"""
    c = np.array(x['ctrl'], dtype=float)
    g = x['goal']
    nz = [r for r in c if abs(r[9]) > 1e-6 or abs(r[10]) > 1e-6]
    ln = nz[-1] if nz else np.zeros(13)
    trig = c[-1]
    d = dict(last_nz_v=float(ln[9]), last_nz_w=float(ln[10]),
             last_nz_rot_in_place=bool(abs(ln[9]) < 0.03 and abs(ln[10]) >= 0.1),
             w_at_end=float(abs(trig[8])), v_at_end=float(abs(trig[7])))
    if x['status'] == 'SUCCEEDED':
        yt = abs(wrap(trig[3] - g[2]))
        yf = abs(wrap(x['final_true'][2] - g[2]))
        d.update(yaw_at_trigger=float(yt), yaw_overshoot=float(yf - yt),
                 xy_at_trigger=float(math.hypot(trig[1] - g[0], trig[2] - g[1])))
    return d


def stats(v):
    v = [a for a in v if a is not None and not (isinstance(a, float) and math.isnan(a))]
    if not v:
        return None
    return dict(n=len(v), med=float(np.median(v)), p90=float(np.percentile(v, 90)), max=float(np.max(v)),
                mean=float(np.mean(v)))


def main():
    pts, _, _ = load_map(os.path.join(LAB, 'synthetic_arena/synthetic_arena.yaml'))
    clr = Clearance(pts)
    paths = {r['test_id']: (np.array(r['path']), r['category'])
             for r in json.load(open(os.path.join(LAB, 'planner_runs/paths_R3.json')))['results']}
    rows = []
    for var, (sub, suf) in SRC.items():
        for s in SCEN:
            fn = os.path.join(LAB, sub, f'sim_{s}_{suf}.jsonl')
            if not os.path.exists(fn):
                continue
            L = [json.loads(z) for z in open(fn)]
            meta = L[0]['meta']
            for x in L[1:]:
                m = run_metrics(x, meta, paths[x['run']][0], clr)
                cls, trans, flips = classify(x, m)
                m.update(extra(x))
                m.update(variant=var, scenario=s, category=paths[x['run']][1], cls=cls, end_trans_25s=trans,
                         end_flips_25s=flips, path_id=x['run'])
                rows.append(m)
    # verificación: baseline == clasificación R4-C variante B
    r4c = {(r['scenario'], r['path_id']): r['cls'] for r in csv.DictReader(open(os.path.join(LAB, 'dwb_r4c/r4c_all_runs.csv')))
           if r['variant'] == 'B'}
    mism = [(r['scenario'], r['path_id']) for r in rows if r['variant'] == 'BASE' and r4c.get((r['scenario'], r['path_id'])) != r['cls']]
    assert not mism, f'baseline no coincide con R4-C: {mism[:5]}'
    os.makedirs(OUT, exist_ok=True)
    keys = ['variant', 'scenario', 'path_id', 'category', 'cls'] + [k for k in rows[0] if k not in
                                                                    ('variant', 'scenario', 'path_id', 'category', 'cls', 'run')]
    with open(os.path.join(OUT, 'r4d_runs.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=keys, extrasaction='ignore')
        w.writeheader()
        w.writerows(rows)

    def summ(rr):
        cl = collections.Counter(r['cls'] for r in rr)
        succ = [r for r in rr if r['status'] == 'SUCCEEDED']
        return dict(
            n=len(rr), cls={c: cl.get(c, 0) for c in CLS}, correct=cl.get('OK', 0), succeeded=len(succ),
            contacts=sum(r['contact'] for r in rr), clear_min=min(r['clear_min'] for r in rr),
            bloqueos=cl.get('OSCILACION', 0) + cl.get('TIMEOUT', 0) + cl.get('OTRO_SIN_TRAYECT', 0),
            final_xy_all=stats([r['final_xy_err'] for r in rr]), final_yaw_all=stats([r['final_yaw_err'] for r in rr]),
            final_xy_ok=stats([r['final_xy_err'] for r in rr if r['cls'] == 'OK']),
            final_yaw_ok=stats([r['final_yaw_err'] for r in rr if r['cls'] == 'OK']),
            final_xy_reached=stats([r['final_xy_err'] for r in rr if r['reached_xy']]),
            final_yaw_reached=stats([r['final_yaw_err'] for r in rr if r['reached_xy']]),
            t_goal=stats([r['t_goal'] for r in succ]), t_xy=stats([r['t_xy'] for r in rr if r['reached_xy']]),
            t_goal_minus_xy=stats([r['t_goal'] - r['t_xy'] for r in succ if r['t_xy'] is not None]),
            w_flips_per_min=stats([r['w_sign_changes_per_min'] for r in rr]),
            w_flips_after_xy=stats([r['w_sign_changes_after_xy'] for r in rr if r['reached_xy']]),
            end_flips_25s_fail=stats([r['end_flips_25s'] for r in rr if r['cls'] != 'OK']),
            frac_rotate=stats([r['frac_rotate_in_place'] for r in rr]),
            last_nz_rot_in_place=sum(r['last_nz_rot_in_place'] for r in rr),
            w_at_trigger=stats([r['w_at_end'] for r in succ]), v_at_trigger=stats([r['v_at_end'] for r in succ]),
            yaw_overshoot=stats([r.get('yaw_overshoot') for r in succ]),
            yaw_at_trigger=stats([r.get('yaw_at_trigger') for r in succ]),
            xy_at_trigger=stats([r.get('xy_at_trigger') for r in succ]),
            lat_mean=stats([r['lat_mean'] for r in rr]), dwb_ms=stats([r['dwb_ms_mean'] for r in rr]),
            dwb_ms_max=max(r['dwb_ms_max'] for r in rr), n_fail_dwb=sum(r['n_fail'] for r in rr),
        )
    S = dict(variants={}, by_scenario={}, by_category={})
    for var in SRC:
        rr = [r for r in rows if r['variant'] == var]
        if not rr:
            continue
        S['variants'][var] = summ(rr)
        for s in SCEN:
            q = [r for r in rr if r['scenario'] == s]
            if q:
                S['by_scenario'][f'{s}|{var}'] = summ(q)
        for c in sorted({r['category'] for r in rr}):
            S['by_category'][f'{c}|{var}'] = summ([r for r in rr if r['category'] == c])
    # pareado
    idx = {(r['variant'], r['scenario'], r['path_id']): r for r in rows}
    pairs = [(s, k) for (v, s, k) in idx if v == 'BASE' and ('RTG', s, k) in idx]
    trans = collections.Counter((idx[('BASE', s, k)]['cls'], idx[('RTG', s, k)]['cls']) for s, k in pairs)
    S['paired'] = dict(
        n=len(pairs), transitions={f'{a}->{b}': v for (a, b), v in sorted(trans.items(), key=lambda z: -z[1])},
        fixed=sum(v for (a, b), v in trans.items() if a != 'OK' and b == 'OK'),
        broken=sum(v for (a, b), v in trans.items() if a == 'OK' and b != 'OK'),
        broken_list=sorted(f'{s}:{k}:{idx[("RTG", s, k)]["cls"]}' for s, k in pairs
                           if idx[('BASE', s, k)]['cls'] == 'OK' and idx[('RTG', s, k)]['cls'] != 'OK'),
        contact_new=sorted(f'{s}:{k}' for s, k in pairs if idx[('RTG', s, k)]['contact'] and not idx[('BASE', s, k)]['contact']),
        contact_removed=sorted(f'{s}:{k}' for s, k in pairs if idx[('BASE', s, k)]['contact'] and not idx[('RTG', s, k)]['contact']),
        dt_goal_both_ok=stats([idx[('RTG', s, k)]['t_goal'] - idx[('BASE', s, k)]['t_goal'] for s, k in pairs
                               if idx[('BASE', s, k)]['cls'] == 'OK' and idx[('RTG', s, k)]['cls'] == 'OK']),
        dxy_both_ok=stats([idx[('RTG', s, k)]['final_xy_err'] - idx[('BASE', s, k)]['final_xy_err'] for s, k in pairs
                           if idx[('BASE', s, k)]['cls'] == 'OK' and idx[('RTG', s, k)]['cls'] == 'OK']),
    )
    S['baseline_matches_r4c'] = True
    json.dump(S, open(os.path.join(OUT, 'r4d_summary.json'), 'w'), indent=1, default=str)
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(1, 3, figsize=(18, 4.8))
        xs = np.arange(len(SCEN))
        for i, (var, col) in enumerate((('BASE', 'tab:gray'), ('RTG', 'tab:green'))):
            ax[0].bar(xs + 0.4 * i, [S['by_scenario'].get(f'{s}|{var}', {}).get('correct', 0) for s in SCEN], 0.4, color=col,
                      label=var)
        ax[0].set_xticks(xs + 0.2)
        ax[0].set_xticklabels([s.split('_', 1)[1] for s in SCEN], rotation=25, fontsize=8)
        ax[0].set_title('SUCCEEDED correcto (de 64)')
        ax[0].legend()
        for var, col in (('BASE', 'tab:gray'), ('RTG', 'tab:green')):
            rr = [r for r in rows if r['variant'] == var and r['reached_xy']]
            ax[1].hist([r['final_yaw_err'] for r in rr], bins=60, range=(0, math.pi), alpha=0.5, color=col, label=var)
            ax[2].hist([r['final_xy_err'] for r in rr], bins=60, range=(0, 0.4), alpha=0.5, color=col, label=var)
        ax[1].axvline(TOL, color='k', ls=':')
        ax[2].axvline(TOL, color='k', ls=':')
        ax[1].set_xlabel('error yaw final verdadero (rad), corridas que llegan en xy')
        ax[2].set_xlabel('error xy final verdadero (m), corridas que llegan en xy')
        ax[1].legend()
        ax[2].legend()
        fig.tight_layout()
        fig.savefig(os.path.join(OUT, 'r4d_overview.png'), dpi=110)
    except Exception as ex:  # noqa
        print('figura no generada:', ex)
    for var, v in S['variants'].items():
        print(var, 'correct', v['correct'], 'cls', v['cls'], 'contacts', v['contacts'], 'bloqueos', v['bloqueos'])
        print('   yaw_reached', v['final_yaw_reached'], '\n   xy_reached', v['final_xy_reached'], '\n   t_goal', v['t_goal'],
              '\n   flips/min', v['w_flips_per_min'], 'after_xy', v['w_flips_after_xy'], '\n   overshoot', v['yaw_overshoot'],
              'w_trig', v['w_at_trigger'])
    print('paired', {k: (v if not isinstance(v, list) else len(v)) for k, v in S['paired'].items()})


if __name__ == '__main__':
    main()
