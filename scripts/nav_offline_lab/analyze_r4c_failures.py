#!/usr/bin/python3
"""NAV-OFFLINE-R4-C — análisis de terminaciones de las 768 corridas de R4-B (SOLO datos existentes, sin simular).

Clasificación (una clase por corrida, en orden de prioridad):
  OK                  SUCCEEDED, sin contacto, error VERDADERO final <= 0.25 m y <= 0.25 rad
  OTRO_EXITO_CONTACTO SUCCEEDED con contacto real del footprint (clearance < 0)
  OTRO_EXITO_APARENTE SUCCEEDED (goal checker sobre pose ESTIMADA) pero error verdadero fuera de tolerancia
  GOAL_TOLERANCE      llegó en xy (etapa 1 del goal checker, stateful) y no terminó (yaw)
  TRACKING            no llegó en xy y error lateral máx > 0.25 m antes de fallar
  OSCILACION          no llegó, ABORT_PROGRESS con el robot quieto (< 0.1 m en 25 s) y >= 4 inversiones de ω
  TIMEOUT             no llegó, t_max
  OTRO_SIN_TRAYECT    no llegó, ABORT_CONTROLLER (DWB sin trayectorias legales > failure_tolerance)
  OTRO                resto
Contrafactual de tolerancias (exacto: parar antes no altera la trayectoria previa): goal checker stateful re-evaluado
sobre la pose estimada registrada con (xy_tol, yaw_tol) mayores; cuenta como resuelta si dispara antes del fin, sin
contacto hasta ese instante, y con la pose VERDADERA dentro de las nuevas tolerancias.
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

D = os.path.join(LAB, 'dwb_r4b')
OUT = os.path.join(LAB, 'dwb_r4c')
SCEN = ['S1_nominal', 'S2_ruido_xy', 'S3_ruido_yaw', 'S4_latencia_0p6', 'S5_combinado_medido', 'S6_combinado_lat0p6']
XY_TOLS = [0.25, 0.30, 0.40, 0.50]
YAW_TOLS = [0.25, 0.35, 0.50, 0.785, 1.571, math.pi + 0.01]
TOL = 0.25


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def load_runs(s, v):
    L = [json.loads(x) for x in open(os.path.join(D, f'sim_{s}_{v}.jsonl'))]
    return L[0]['meta'], {r['run']: r for r in L[1:]}


def stuck_features(c, window=25.0):
    last = c[c[:, 0] >= c[-1, 0] - window]
    trans = float(np.hypot(last[-1, 1] - last[0, 1], last[-1, 2] - last[0, 2]))
    s = [np.sign(w) for w in last[:, 10] if abs(w) > 0.05]
    flips = sum(1 for a, b in zip(s, s[1:]) if a != b)
    wamp = float(np.abs(last[:, 10]).max()) if len(last) else 0.0
    return trans, flips, wamp


def counterfactual(c, hi_clear_t, goal, xy_tol, yaw_tol, contact_t):
    """primer instante en que el goal checker (stateful, pose estimada) dispararía; None si nunca."""
    check_xy = True
    for row in c:
        t, x, y, th, ex, ey, eth = row[:7]
        if check_xy and math.hypot(ex - goal[0], ey - goal[1]) <= xy_tol:
            check_xy = False
        if not check_xy and abs(wrap(eth - goal[2])) < yaw_tol:
            ok_contact = contact_t is None or contact_t > t
            true_ok = math.hypot(x - goal[0], y - goal[1]) <= xy_tol and abs(wrap(th - goal[2])) < yaw_tol
            return t, ok_contact and true_ok, math.hypot(x - goal[0], y - goal[1]), abs(wrap(th - goal[2]))
    return None, False, None, None


def main():
    os.makedirs(OUT, exist_ok=True)
    base = {(r['scenario'], r['variant'], r['run']): r for r in csv.DictReader(open(os.path.join(D, 'r4b_runs.csv')))}
    rows = []
    for s in SCEN:
        for v in ('A', 'B'):
            meta, runs = load_runs(s, v)
            for k, x in runs.items():
                b = base[(s, v, k)]
                c = np.array(x['ctrl'], dtype=float)
                g = x['goal']
                reached = b['reached_xy'] == 'True'
                contact = b['contact'] == 'True'
                fxy, fyaw = float(b['final_xy_err']), float(b['final_yaw_err'])
                lat_max = float(b['lat_max'])
                trans, flips, wamp = stuck_features(c)
                st = x['status']
                if st == 'SUCCEEDED':
                    cls = 'OTRO_EXITO_CONTACTO' if contact else ('OK' if fxy <= TOL and fyaw <= TOL else 'OTRO_EXITO_APARENTE')
                elif reached:
                    cls = 'GOAL_TOLERANCE'
                elif lat_max > TOL:
                    cls = 'TRACKING'
                elif st == 'ABORT_PROGRESS' and trans < 0.1 and flips >= 4:
                    cls = 'OSCILACION'
                elif st == 'TIMEOUT':
                    cls = 'TIMEOUT'
                elif st == 'ABORT_CONTROLLER':
                    cls = 'OTRO_SIN_TRAYECT'
                else:
                    cls = 'OTRO'
                nz = [r_ for r_ in c if abs(r_[9]) > 1e-6 or abs(r_[10]) > 1e-6]
                last_cmd = (float(c[-1, 9]), float(c[-1, 10]))
                last_nz = (float(nz[-1][9]), float(nz[-1][10])) if nz else (0.0, 0.0)
                d_true_end = math.hypot(c[-1, 1] - g[0], c[-1, 2] - g[1])
                phase = ('en goal (xy)' if reached else f'a {d_true_end:.2f} m del goal') + \
                        (', girando en el lugar' if trans < 0.1 and flips >= 4 else (', quieto' if trans < 0.1 else ', desplazándose'))
                ct = float(b['first_contact_t']) if b['first_contact_t'] not in ('', 'None') else None
                true_ok_trig = (st == 'SUCCEEDED' and math.hypot(c[-1, 1] - g[0], c[-1, 2] - g[1]) <= TOL and
                                abs(wrap(c[-1, 3] - g[2])) <= TOL)
                cf = {}
                if cls != 'OK':
                    for xt in XY_TOLS:
                        for yt in YAW_TOLS:
                            t_, ok, _, _ = counterfactual(c, None, g, xt, yt, ct)
                            cf[f'{xt}|{yt:.3f}'] = bool(ok and t_ is not None and t_ <= x['t_end'])
                rows.append(dict(
                    path_id=k, variant=v, variant_name='BaseObstacle' if v == 'A' else 'ObstacleFootprint', scenario=s,
                    category=b['category'], cls=cls, status=st, reason=x['reason'][:60], reached_xy=reached,
                    t_end=float(x['t_end']), t_xy=float(b['t_xy']) if b['t_xy'] not in ('', 'None') else None,
                    lat_max=lat_max, lat_mean=float(b['lat_mean']), final_xy_err=fxy, final_yaw_err=fyaw,
                    clear_min=float(b['clear_min']), contact=contact, first_contact_t=ct,
                    last_cmd_v=last_cmd[0], last_cmd_w=last_cmd[1], last_nz_cmd_v=last_nz[0], last_nz_cmd_w=last_nz[1],
                    last_state=f'{st} ({x["reason"][:40] or "—"}); {phase}',
                    end_trans_25s=trans, end_w_flips_25s=flips, end_w_amp=wamp, n_fail_dwb=int(x['n_fail']),
                    path_len=float(x['path_len']), true_ok_at_trigger=true_ok_trig, cf=cf))
    # atribución (reglas explícitas; ver docstring del reporte)
    idx = {(r['scenario'], r['variant'], r['path_id']): r for r in rows}
    for r in rows:
        r['subcls'] = ''
        if r['cls'] == 'OK':
            r['attribution'] = '—'
            continue
        other = idx[(r['scenario'], 'B' if r['variant'] == 'A' else 'A', r['path_id'])]
        tol_fix = any(ok for kk, ok in r['cf'].items()
                      if float(kk.split('|')[0]) <= 0.40 and float(kk.split('|')[1]) <= 0.785 and kk != '0.25|0.250')
        if r['cls'] == 'OTRO_EXITO_APARENTE':
            r['subcls'] = 'SOBREPASO_AL_FRENAR' if r['true_ok_at_trigger'] else 'ERROR_LOCALIZACION'
            r['attribution'] = ('DINAMICA DE PARADA: dentro de tolerancia al detectar, fuera al detenerse (latencia + ω alta)'
                                if r['true_ok_at_trigger'] else 'LOCALIZACION: goal checker sobre pose estimada')
        elif r['cls'] == 'GOAL_TOLERANCE':
            r['attribution'] = ('PARAMETRO: tolerancias de goal (verificado por contrafactual xy<=0.40 / yaw<=0.785)' if tol_fix else
                                'PARAMETRO (inferido): sin critic de orientación final (RotateToGoal comentado); '
                                'no se resuelve con yaw <= 0.785')
        elif other['cls'] == 'OK':
            r['attribution'] = 'PARAMETRO: critic de obstáculos (verificado: la variante pareada termina OK)'
        else:
            r['attribution'] = 'DWB: falla en ambas variantes (no atribuible al critic de obstáculos ni a tolerancias)'
    # salidas
    keys = [k for k in rows[0] if k != 'cf']
    with open(os.path.join(OUT, 'r4c_all_runs.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=keys + ['subcls', 'attribution'])
        w.writeheader()
        for r in rows:
            w.writerow({k: r.get(k) for k in keys + ['subcls', 'attribution']})
    fails = [r for r in rows if r['cls'] != 'OK']
    fkeys = ['path_id', 'variant_name', 'scenario', 'category', 'cls', 'subcls', 'attribution', 'lat_max', 'final_xy_err',
             'final_yaw_err', 't_end', 'clear_min', 'last_cmd_v', 'last_cmd_w', 'last_nz_cmd_v', 'last_nz_cmd_w', 'last_state']
    with open(os.path.join(OUT, 'r4c_failures.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=fkeys)
        w.writeheader()
        for r in sorted(fails, key=lambda r: (r['cls'], r['path_id'], r['variant_name'], r['scenario'])):
            w.writerow({k: r[k] for k in fkeys})
    # contrafactual agregado
    cf_tab = {}
    for xt in XY_TOLS:
        for yt in YAW_TOLS:
            kk = f'{xt}|{yt:.3f}'
            res = collections.Counter(r['cls'] for r in fails if r['cf'].get(kk))
            cf_tab[kk] = dict(total=sum(res.values()), by_cls=dict(res))
    # casos difíciles por path (12 corridas: 6 escenarios x 2 variantes)
    per = collections.defaultdict(list)
    for r in rows:
        per[r['path_id']].append(r)
    hard = []
    for k, rr in per.items():
        nf = sum(r['cls'] != 'OK' for r in rr)
        nfs = sum(r['cls'] not in ('OK', 'OTRO_EXITO_APARENTE') for r in rr)
        hard.append(dict(path_id=k, category=rr[0]['category'], path_len=rr[0]['path_len'], n_runs=len(rr), n_fail=nf,
                         n_fail_sin_aparente=nfs, n_contact=sum(r['contact'] for r in rr),
                         clear_min=min(r['clear_min'] for r in rr), lat_max=max(r['lat_max'] for r in rr),
                         min_final_xy=min(r['final_xy_err'] for r in rr),
                         classes=dict(collections.Counter(r['cls'] for r in rr))))
    hard.sort(key=lambda h: (-h['n_fail_sin_aparente'], -h['n_contact'], -h['min_final_xy'], h['clear_min']))
    S = dict(
        n_runs=len(rows), n_ok=sum(r['cls'] == 'OK' for r in rows), n_fail=len(fails),
        by_cls=dict(collections.Counter(r['cls'] for r in rows)),
        by_cls_variant={v: dict(collections.Counter(r['cls'] for r in rows if r['variant'] == v)) for v in 'AB'},
        by_cls_scenario={s: dict(collections.Counter(r['cls'] for r in rows if r['scenario'] == s)) for s in SCEN},
        by_cls_category={c: dict(collections.Counter(r['cls'] for r in rows if r['category'] == c))
                         for c in sorted({r['category'] for r in rows})},
        attribution=dict(collections.Counter(r['attribution'] for r in fails)),
        subcls=dict(collections.Counter(r['subcls'] for r in fails if r['subcls'])),
        attribution_by_variant={v: dict(collections.Counter(r['attribution'] for r in fails if r['variant'] == v)) for v in 'AB'},
        attribution_by_cls={c: dict(collections.Counter(r['attribution'] for r in fails if r['cls'] == c))
                            for c in sorted({r['cls'] for r in fails})},
        counterfactual=cf_tab, hard=hard[:10],
        cls_stats={c: dict(n=len(rr), lat_max_max=max(r['lat_max'] for r in rr),
                           final_xy_med=float(np.median([r['final_xy_err'] for r in rr])),
                           final_yaw_med=float(np.median([r['final_yaw_err'] for r in rr])),
                           t_end_med=float(np.median([r['t_end'] for r in rr])),
                           clear_min=min(r['clear_min'] for r in rr),
                           end_flips_med=float(np.median([r['end_w_flips_25s'] for r in rr])),
                           end_w_amp_med=float(np.median([r['end_w_amp'] for r in rr])))
                   for c in sorted({r['cls'] for r in rows}) for rr in [[r for r in rows if r['cls'] == c]]},
        aparente_excess=dict(
            xy_gt_tol=sum(1 for r in rows if r['cls'] == 'OTRO_EXITO_APARENTE' and r['final_xy_err'] > TOL),
            yaw_gt_tol=sum(1 for r in rows if r['cls'] == 'OTRO_EXITO_APARENTE' and r['final_yaw_err'] > TOL),
            xy_gt_tol_plus5cm=sum(1 for r in rows if r['cls'] == 'OTRO_EXITO_APARENTE' and r['final_xy_err'] > TOL + 0.05),
            yaw_gt_tol_plus01=sum(1 for r in rows if r['cls'] == 'OTRO_EXITO_APARENTE' and r['final_yaw_err'] > TOL + 0.1)),
        goal_tol_detail=dict(
            n=sum(r['cls'] == 'GOAL_TOLERANCE' for r in rows),
            final_xy_med=float(np.median([r['final_xy_err'] for r in rows if r['cls'] == 'GOAL_TOLERANCE'])),
            final_yaw_med=float(np.median([r['final_yaw_err'] for r in rows if r['cls'] == 'GOAL_TOLERANCE'])),
            final_yaw_p90=float(np.percentile([r['final_yaw_err'] for r in rows if r['cls'] == 'GOAL_TOLERANCE'], 90)),
            t_after_xy_med=float(np.median([r['t_end'] - r['t_xy'] for r in rows if r['cls'] == 'GOAL_TOLERANCE' and r['t_xy']])),
        ),
    )
    json.dump(S, open(os.path.join(OUT, 'r4c_summary.json'), 'w'), indent=1, default=str)
    print(json.dumps({k: S[k] for k in ('n_runs', 'n_ok', 'n_fail', 'by_cls', 'by_cls_variant', 'attribution',
                                        'aparente_excess', 'goal_tol_detail')}, indent=1, default=str))
    for kk in (f'{0.25}|{0.350:.3f}', f'{0.25}|{0.500:.3f}', f'{0.25}|{0.785:.3f}', f'{0.25}|{math.pi + 0.01:.3f}',
               f'{0.4}|{0.785:.3f}', f'{0.5}|{math.pi + 0.01:.3f}'):
        print('CF', kk, cf_tab[kk])
    for h in hard[:10]:
        print('HARD', h)


if __name__ == '__main__':
    main()
