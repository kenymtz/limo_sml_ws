#!/usr/bin/python3
"""NAV-OFFLINE-R4-G Fase 1 — traza ciclo a ciclo de los casos reproducidos (debug_eval_runs).

Entrada: dwb_r4g/sim_repro_<S>_<ID>_dbg.jsonl (+ .eval.jsonl). Salida: dwb_r4g/trace_<S>_<ID>.csv y trace_summary.json.
Columnas: t, pose real (x,y,th), pose estimada, distancia xy real/estimada al goal, error de yaw real/estimado (con signo),
v/ω reales, comando (cv, cw), clearance real del footprint (sin padding), base del progress checker (bx, by, bt),
tiempo sin progreso (t − bt), distancia a la base, n trayectorias legales, ilegales por critic, término de cada critic de la
trayectoria elegida.
"""
import csv
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from evaluate_dwb_r4a2 import Clearance, load_map, LAB  # noqa: E402

CASES = [('S1_nominal', 'T041'), ('S2_ruido_xy', 'T045'), ('S5_combinado_medido', 'T041')]


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def main():
    pts, _, _ = load_map(os.path.join(LAB, 'synthetic_arena/synthetic_arena.yaml'))
    clr = Clearance(pts)
    summ = {}
    for S, ID in CASES:
        base = os.path.join(LAB, 'dwb_r4g', f'sim_repro_{S}_{ID}_dbg.jsonl')
        X = json.loads(list(open(base))[1])
        E = json.loads(open(base + '.eval.jsonl').readline())
        C = X['ctrl']
        g = X['goal']
        names = E['critics']
        assert len(C) == len(E['cycles']), (len(C), len(E['cycles']))
        rows = []
        for r, c in zip(C, E['cycles']):
            t, x, y, th, ex, ey, eth, v, w, cv, cw = r[:11]
            _, bx, by, bt, bi, n, ill, sc, lg, exc = c
            best = next((l for l in lg if abs(l[0] - cv) < 1e-9 and abs(l[1] - cw) < 1e-9), None) if lg else None
            row = dict(t=t, x=x, y=y, th=th, x_est=ex, y_est=ey, th_est=eth,
                       d_xy_true=math.hypot(x - g[0], y - g[1]), d_xy_est=math.hypot(ex - g[0], ey - g[1]),
                       yaw_err_true=wrap(th - g[2]), yaw_err_est=wrap(eth - g[2]), v=v, w=w, cmd_v=cv, cmd_w=cw,
                       clearance_true=clr.pose(x, y, th)[0], pc_base_x=bx, pc_base_y=by, pc_base_t=bt,
                       t_without_progress=t - bt, d_from_pc_base=math.hypot(x - bx, y - by),
                       n_twists=n, n_legal=n - sum(ill.values()) if n else 0,
                       illegal=';'.join(f'{k}:{v_}' for k, v_ in sorted(ill.items())), exception=exc)
            for j, nm in enumerate(names):
                row[f'chosen_{nm}'] = best[3 + j] if best else None
            rows.append(row)
        fn = os.path.join(LAB, 'dwb_r4g', f'trace_{S}_{ID}.csv')
        with open(fn, 'w', newline='') as f:
            w_ = csv.DictWriter(f, fieldnames=list(rows[0]))
            w_.writeheader()
            w_.writerows(rows)
        # resumen: fase final de giro sin progreso
        last_bt = rows[-1]['pc_base_t']
        stuck = [r for r in rows if r['t'] >= last_bt]
        flips = sum(1 for a, b in zip(stuck, stuck[1:]) if a['cmd_w'] * b['cmd_w'] < 0)
        summ[f'{S}:{ID}'] = dict(
            status=X['status'], t_end=X['t_end'], last_progress_t=last_bt, t_without_progress_at_abort=rows[-1]['t_without_progress'],
            translation_since_last_progress=max(r['d_from_pc_base'] for r in stuck),
            yaw_err_est_range_stuck=[min(r['yaw_err_est'] for r in stuck if r['t'] > last_bt + 4),
                                     max(r['yaw_err_est'] for r in stuck if r['t'] > last_bt + 4)],
            min_abs_yaw_err_true_stuck=min(abs(r['yaw_err_true']) for r in stuck if r['t'] > last_bt + 4),
            d_xy_true_stuck=[min(r['d_xy_true'] for r in stuck), max(r['d_xy_true'] for r in stuck)],
            clearance_true_min_stuck=min(r['clearance_true'] for r in stuck),
            cmd_w_values_stuck=sorted({round(r['cmd_w'], 3) for r in stuck}),
            cmd_w_sign_flips_stuck=flips,
            illegal_critics_stuck=sorted({k.split(':')[0] for r in stuck for k in r['illegal'].split(';') if k}),
            cycles_with_ObstacleFootprint_illegal=sum('ObstacleFootprint' in r['illegal'] for r in stuck),
            cycles_with_Oscillation_illegal=sum('Oscillation' in r['illegal'] for r in stuck),
            n_cycles_stuck=len(stuck))
        print(S, ID, json.dumps(summ[f'{S}:{ID}'], default=lambda z: round(z, 4) if isinstance(z, float) else z))
    json.dump(summ, open(os.path.join(LAB, 'dwb_r4g', 'trace_summary.json'), 'w'), indent=1)


if __name__ == '__main__':
    main()
