#!/usr/bin/python3
"""NAV-OFFLINE-R4-B — análisis de las corridas en lazo cerrado (dwb_closed_loop_sim).

Verdad de terreno OFFLINE: geometría del PGM + footprint real sin padding (Clearance de evaluate_dwb_r4a2).
Métricas por corrida: estado, llegada xy (goal checker etapa 1, pose estimada) y verdadera, error final, error lateral
al path, clearance mínima y contactos (pose verdadera a 50 Hz), oscilaciones del comando, uso de comandos.
Chequeos del modelo (MODELO PASS/FAIL): saturación, límites de aceleración, latencia, ganancia, cinemática, determinismo,
ruido pareado A/B.
"""
import argparse
import collections
import csv
import glob
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from evaluate_dwb_r4a2 import Clearance, load_map, LAB, PADDING  # noqa: E402

D = os.path.join(LAB, 'dwb_r4b')
SCEN = ['S1_nominal', 'S2_ruido_xy', 'S3_ruido_yaw', 'S4_latencia_0p6', 'S5_combinado_medido', 'S6_combinado_lat0p6']


def load(fn):
    L = [json.loads(x) for x in open(fn)]
    return L[0]['meta'], {r['run']: r for r in L[1:]}


def seg_dist(P, Q):
    """distancia de cada punto P (n,2) a la polilínea Q (m,2)."""
    A, B = Q[:-1], Q[1:]
    AB = B - A
    L2 = np.maximum((AB ** 2).sum(1), 1e-12)
    t = np.clip(((P[:, None, :] - A[None]) * AB[None]).sum(2) / L2[None], 0, 1)
    C = A[None] + t[..., None] * AB[None]
    return np.sqrt(((P[:, None, :] - C) ** 2).sum(2)).min(1)


def sign_changes(w, thr=0.05):
    s = [np.sign(x) for x in w if abs(x) > thr]
    return sum(1 for a, b in zip(s, s[1:]) if a != b)


def run_metrics(r, meta, path, clr):
    c = np.array(r['ctrl'], dtype=float)   # t x y th ex ey eth v w cv cw ok ms
    hi = np.array(r['hi'], dtype=float) if r['hi'] else c[:, :4]
    g = np.array(r['goal'])
    t = c[:, 0]
    d_est = np.hypot(c[:, 4] - g[0], c[:, 5] - g[1])
    d_true = np.hypot(c[:, 1] - g[0], c[:, 2] - g[1])
    i_xy = np.argmax(d_est <= meta['goal_xy']) if (d_est <= meta['goal_xy']).any() else None
    t_xy = float(t[i_xy]) if i_xy is not None else None
    track = slice(0, i_xy if i_xy is not None else len(c))
    lat = seg_dist(c[track, 1:3], path[:, :2]) if len(c[track]) else np.array([0.0])
    # clearance a 50 Hz (pose verdadera)
    cl = np.array([clr.pose(x, y, th)[0] for _, x, y, th in hi])
    dt_hi = float(np.median(np.diff(hi[:, 0]))) if len(hi) > 1 else 0.02
    cv, cw, ok = c[:, 9], c[:, 10], c[:, 11]
    moving = (np.abs(cv) > 1e-6) | (np.abs(cw) > 1e-6)
    fx = np.array(r['final_true'])
    return dict(
        run=r['run'], status=r['status'], reason=r['reason'], t_goal=r['t_goal'], t_end=r['t_end'],
        path_len=r['path_len'], t_xy=t_xy, reached_xy=t_xy is not None,
        t_after_xy=(r['t_end'] - t_xy) if t_xy is not None else None,
        final_xy_err=float(math.hypot(fx[0] - g[0], fx[1] - g[1])),
        final_yaw_err=float(abs(math.atan2(math.sin(fx[2] - g[2]), math.cos(fx[2] - g[2])))),
        min_dist_true_to_goal=float(d_true.min()),
        lat_mean=float(lat.mean()), lat_p95=float(np.percentile(lat, 95)), lat_max=float(lat.max()),
        clear_min=float(cl.min()), contact=bool((cl < 0).any()), contact_time=float((cl < 0).sum() * dt_hi),
        margin_viol=bool((cl < PADDING).any()), margin_time=float((cl < PADDING).sum() * dt_hi),
        first_contact_t=float(hi[np.argmax(cl < 0), 0]) if (cl < 0).any() else None,
        n_fail=r['n_fail'], n_ctrl=r['n_ctrl'],
        w_sign_changes=sign_changes(cw), w_sign_changes_per_min=sign_changes(cw) / max(t[-1], 1e-6) * 60,
        w_sign_changes_after_xy=sign_changes(cw[i_xy:]) if i_xy is not None else 0,
        cmd_v_mean=float(np.abs(cv[moving]).mean()) if moving.any() else 0.0,
        frac_rotate_in_place=float(((np.abs(cv) < 0.03) & (np.abs(cw) >= 0.1)).mean()),
        frac_zero_cmd=float((~moving).mean()), frac_fail=float((ok == 0).mean()),
        frac_sat_v=float((np.abs(cv) >= meta['max_v'] - 1e-3).mean()),
        frac_sat_w=float((np.abs(cw) >= meta['max_w'] - 1e-3).mean()),
        dwb_ms_mean=r['dwb_ms_mean'], dwb_ms_max=r['dwb_ms_max'],
    )


def model_checks(runs_by, metas):
    """Invariantes del modelo de base sobre TODOS los logs de control."""
    res = collections.OrderedDict()
    worst = dict(v=0, w=0, dv=0, dw=0, step=0)
    for key, runs in runs_by.items():
        m = metas[key]
        for r in runs.values():
            c = np.array(r['ctrl'], dtype=float)
            worst['v'] = max(worst['v'], np.abs(c[:, 7]).max() - m['max_v'])
            worst['w'] = max(worst['w'], np.abs(c[:, 8]).max() - m['max_w'])
            if len(c) > 1:
                dt = np.diff(c[:, 0])
                worst['dv'] = max(worst['dv'], (np.abs(np.diff(c[:, 7])) / dt).max() - max(m['acc_v'], abs(m['dec_v'])))
                worst['dw'] = max(worst['dw'], (np.abs(np.diff(c[:, 8])) / dt).max() - max(m['acc_w'], abs(m['dec_w'])))
            hi = np.array(r['hi'], dtype=float)
            if len(hi) > 1:
                step = np.hypot(np.diff(hi[:, 1]), np.diff(hi[:, 2])) / np.diff(hi[:, 0])
                worst['step'] = max(worst['step'], step.max() - m['max_v'])
    res['saturacion_v'] = dict(exceso_max=worst['v'], PASS=worst['v'] <= 1e-6)
    res['saturacion_w'] = dict(exceso_max=worst['w'], PASS=worst['w'] <= 1e-6)
    res['acel_v'] = dict(exceso_max_m_s2=worst['dv'], PASS=worst['dv'] <= 1e-3)
    res['acel_w'] = dict(exceso_max_rad_s2=worst['dw'], PASS=worst['dw'] <= 1e-3)
    res['cinematica_vel_pose'] = dict(exceso_max_m_s=worst['step'], PASS=worst['step'] <= 1e-3)
    # latencia: primer movimiento real vs primer comando no nulo
    for key in [k for k in runs_by if k[0] in ('S1_nominal', 'S4_latencia_0p6')]:
        lags = []
        for r in runs_by[key].values():
            c = np.array(r['ctrl'], dtype=float)
            nz = np.nonzero((np.abs(c[:, 9]) > 1e-6) | (np.abs(c[:, 10]) > 1e-6))[0]
            mv = np.nonzero((np.abs(c[:, 7]) > 1e-6) | (np.abs(c[:, 8]) > 1e-6))[0]
            if len(nz) and len(mv):
                lags.append(c[mv[0], 0] - c[nz[0], 0])
        lat = metas[key]['latency']
        # con muestreo a 10 Hz el primer movimiento se ve en la muestra siguiente a t_cmd+latencia
        okl = all(lat - 1e-6 <= x <= lat + 0.1 + 1e-6 for x in lags)
        res[f'latencia_{key[0]}_{key[1]}'] = dict(config=lat, observada_min=min(lags), observada_max=max(lags), PASS=okl)
    # ganancia en régimen (S5): v/cmd con cmd constante >= 1 s y aceleración terminada
    for key in [k for k in runs_by if k[0] == 'S5_combinado_medido']:
        rv, rw = [], []
        for r in runs_by[key].values():
            c = np.array(r['ctrl'], dtype=float)
            for i in range(10, len(c)):
                win = c[i - 10:i + 1]
                if np.ptp(win[:, 9]) < 1e-9 and abs(win[-1, 9]) > 0.05 and np.ptp(win[-3:, 7]) < 1e-6:
                    rv.append(c[i, 7] / c[i, 9])
                if np.ptp(win[:, 10]) < 1e-9 and abs(win[-1, 10]) > 0.1 and np.ptp(win[-3:, 8]) < 1e-6:
                    rw.append(c[i, 8] / c[i, 10])
        m = metas[key]
        res[f'ganancia_{key[1]}'] = dict(k_v=m['k_v'], v_ratio_med=float(np.median(rv)) if rv else None, n_v=len(rv),
                                         k_w=m['k_w'], w_ratio_med=float(np.median(rw)) if rw else None, n_w=len(rw),
                                         PASS=bool(rv) and abs(np.median(rv) - m['k_v']) < 0.01 and
                                         (not rw or abs(np.median(rw) - m['k_w']) < 0.01))
    # ruido pareado: A y B arrancan con la misma pose estimada en cada escenario
    for s in SCEN:
        if (s, 'A') in runs_by and (s, 'B') in runs_by:
            A, B = runs_by[(s, 'A')], runs_by[(s, 'B')]
            diff = max(abs(A[k]['ctrl'][0][4 + j] - B[k]['ctrl'][0][4 + j]) for k in A if k in B for j in range(3))
            res[f'ruido_pareado_{s}'] = dict(max_diff_pose_est_t0=diff, PASS=diff < 1e-9)
    # determinismo: mismas corridas en distinto orden (det1: T001,T011,T021,T039,T047; det2: T021,T047,T039)
    f1, f2 = os.path.join(D, 'sim_det1.jsonl'), os.path.join(D, 'sim_det2.jsonl')
    if os.path.exists(f1) and os.path.exists(f2):
        _, R1 = load(f1)
        _, R2 = load(f2)
        same = {k: [x[:-1] for x in R1[k]['ctrl']] == [x[:-1] for x in R2[k]['ctrl']] for k in R2 if k in R1}
        res['determinismo_orden_independiente'] = dict(comparados=len(same), distintos=[k for k, v in same.items() if not v],
                                                       PASS=all(same.values()))
    # contenido del costmap al inicio de cada corrida
    sl = [r.get('start_lethal_cells', 0) for runs in runs_by.values() for r in runs.values()]
    res['costmap_con_obstaculos'] = dict(corridas=len(sl), min_celdas_lethal=min(sl), mediana=float(np.median(sl)),
                                         PASS=float(np.median(sl)) > 0)
    # S1 A: repetición completa de S1_nominal_A para piso de ruido (si existe)
    res['PASS'] = all(v['PASS'] for v in res.values() if isinstance(v, dict))
    return res


def agg(rows):
    def s(k, f=np.mean):
        v = [r[k] for r in rows if r[k] is not None]
        return float(f(v)) if v else None
    n = len(rows)
    return dict(
        n=n, succeeded=sum(r['status'] == 'SUCCEEDED' for r in rows),
        abort_progress=sum(r['status'] == 'ABORT_PROGRESS' for r in rows),
        abort_controller=sum(r['status'] == 'ABORT_CONTROLLER' for r in rows),
        timeout=sum(r['status'] == 'TIMEOUT' for r in rows),
        reached_xy=sum(r['reached_xy'] for r in rows),
        contacts=sum(r['contact'] for r in rows), contact_time_total=sum(r['contact_time'] for r in rows),
        margin_viol=sum(r['margin_viol'] for r in rows),
        clear_min=s('clear_min', np.min), clear_p5=s('clear_min', lambda v: np.percentile(v, 5)),
        lat_mean=s('lat_mean'), lat_p95_mean=s('lat_p95'), lat_max=s('lat_max', np.max),
        final_xy_med=s('final_xy_err', np.median), final_xy_max=s('final_xy_err', np.max),
        final_yaw_med=s('final_yaw_err', np.median),
        t_xy_med=s('t_xy', np.median), t_goal_med=s('t_goal', np.median), t_after_xy_med=s('t_after_xy', np.median),
        w_flips_per_min=s('w_sign_changes_per_min'), w_flips_after_xy_mean=s('w_sign_changes_after_xy'),
        frac_rotate=s('frac_rotate_in_place'), frac_zero=s('frac_zero_cmd'), frac_fail=s('frac_fail'),
        frac_sat_v=s('frac_sat_v'), frac_sat_w=s('frac_sat_w'), cmd_v_mean=s('cmd_v_mean'),
        n_fail_total=sum(r['n_fail'] for r in rows), dwb_ms_mean=s('dwb_ms_mean'), dwb_ms_max=s('dwb_ms_max', np.max),
        contact_runs=sorted(r['run'] for r in rows if r['contact']),
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--map', default=os.path.join(LAB, 'synthetic_arena/synthetic_arena.yaml'))
    a = ap.parse_args()
    pts, _, _ = load_map(a.map)
    clr = Clearance(pts)
    paths = {r['test_id']: (np.array(r['path']), r['category'])
             for r in json.load(open(os.path.join(LAB, 'planner_runs/paths_R3.json')))['results']}
    runs_by, metas, rows = {}, {}, []
    for s in SCEN:
        for v in ('A', 'B'):
            fn = os.path.join(D, f'sim_{s}_{v}.jsonl')
            if not os.path.exists(fn):
                continue
            meta, runs = load(fn)
            runs_by[(s, v)], metas[(s, v)] = runs, meta
            for k, r in runs.items():
                m = run_metrics(r, meta, paths[k][0], clr)
                m.update(scenario=s, variant=v, category=paths[k][1])
                rows.append(m)
    keys = ['scenario', 'variant', 'category'] + [k for k in rows[0] if k not in ('scenario', 'variant', 'category')]
    with open(os.path.join(D, 'r4b_runs.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)
    S = dict(model_checks=model_checks(runs_by, metas), metas={f'{k[0]}|{k[1]}': v for k, v in metas.items()})
    S['by_scenario'] = {}
    for s in SCEN:
        for v in ('A', 'B'):
            rr = [r for r in rows if r['scenario'] == s and r['variant'] == v]
            if rr:
                S['by_scenario'][f'{s}|{v}'] = agg(rr)
    S['by_category'] = {}
    for v in ('A', 'B'):
        for c in sorted({r['category'] for r in rows}):
            rr = [r for r in rows if r['variant'] == v and r['category'] == c]
            S['by_category'][f'{c}|{v}'] = agg(rr)
    S['totals'] = {v: agg([r for r in rows if r['variant'] == v]) for v in ('A', 'B')}
    # pareado: corridas con contacto en A y no en B, y viceversa
    idx = {(r['scenario'], r['variant'], r['run']): r for r in rows}
    pa = [(s, k) for (s, v, k) in idx if v == 'A' and (s, 'B', k) in idx]
    S['paired'] = dict(
        n=len(pa),
        contact_A_only=sorted(f'{s}:{k}' for s, k in pa if idx[(s, 'A', k)]['contact'] and not idx[(s, 'B', k)]['contact']),
        contact_B_only=sorted(f'{s}:{k}' for s, k in pa if idx[(s, 'B', k)]['contact'] and not idx[(s, 'A', k)]['contact']),
        contact_both=sorted(f'{s}:{k}' for s, k in pa if idx[(s, 'B', k)]['contact'] and idx[(s, 'A', k)]['contact']),
        status_diff=sum(1 for s, k in pa if idx[(s, 'A', k)]['status'] != idx[(s, 'B', k)]['status']))
    json.dump(S, open(os.path.join(D, 'r4b_summary.json'), 'w'), indent=1, default=str)

    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(1, 3, figsize=(18, 4.8))
        xs = np.arange(len(SCEN))
        for i, (v, col) in enumerate((('A', 'tab:red'), ('B', 'tab:blue'))):
            ax[0].bar(xs + i * 0.4, [S['by_scenario'].get(f'{s}|{v}', {}).get('contacts', 0) for s in SCEN], 0.4,
                      color=col, label=f'{v} ({"BaseObstacle" if v == "A" else "ObstacleFootprint"})')
            ax[1].bar(xs + i * 0.4, [S['by_scenario'].get(f'{s}|{v}', {}).get('succeeded', 0) for s in SCEN], 0.4,
                      color=col, alpha=0.8)
            ax[1].bar(xs + i * 0.4, [S['by_scenario'].get(f'{s}|{v}', {}).get('reached_xy', 0) for s in SCEN], 0.4,
                      color=col, alpha=0.25)
        for x_ in ax[:2]:
            x_.set_xticks(xs + 0.2)
            x_.set_xticklabels([s.split('_', 1)[1] for s in SCEN], rotation=25, fontsize=8)
        ax[0].set_title('corridas con contacto real (de 64)')
        ax[0].legend(fontsize=8)
        ax[1].set_title('SUCCEEDED (oscuro) y llegada xy (claro), de 64')
        for v, col in (('A', 'tab:red'), ('B', 'tab:blue')):
            cl = [r['clear_min'] for r in rows if r['variant'] == v]
            ax[2].hist(np.clip(cl, -0.05, 0.4), bins=90, alpha=0.5, color=col, label=v)
        ax[2].axvline(0, color='k')
        ax[2].axvline(PADDING, color='k', ls=':')
        ax[2].set_yscale('log')
        ax[2].set_xlabel('clearance mínima del footprint real por corrida (m)')
        ax[2].legend()
        fig.tight_layout()
        fig.savefig(os.path.join(D, 'r4b_overview.png'), dpi=110)
    except Exception as ex:  # noqa
        print('figura no generada:', ex)
    print(json.dumps(S['model_checks'], indent=1, default=str))
    for k, v in S['by_scenario'].items():
        print(k, {x: v[x] for x in ('succeeded', 'reached_xy', 'abort_progress', 'abort_controller', 'timeout',
                                    'contacts', 'margin_viol', 'clear_min', 'lat_mean', 'final_xy_med', 't_xy_med')})
    print('paired', {k: (v if isinstance(v, int) else len(v)) for k, v in S['paired'].items()})


if __name__ == '__main__':
    main()
