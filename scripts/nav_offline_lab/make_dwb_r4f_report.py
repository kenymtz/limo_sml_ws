#!/usr/bin/python3
"""NAV-OFFLINE-R4-F — tablas MEDIDO desde dwb_r4f/r4f_summary.json -> dwb_r4f/r4f_tables.md"""
import json
import os

LAB = os.path.expanduser('~/limo_sml_ws/results/navigation_offline_lab')
S = json.load(open(os.path.join(LAB, 'dwb_r4f/r4f_summary.json')))
V = ['Y025', 'Y020', 'Y015', 'Y010']
NAME = {'Y025': 'yaw 0.25 (R4-E 0.15)', 'Y020': 'yaw 0.20', 'Y015': 'yaw 0.15', 'Y010': 'yaw 0.10'}
SCEN = ['S1_nominal', 'S2_ruido_xy', 'S3_ruido_yaw', 'S4_latencia_0p6', 'S5_combinado_medido', 'S6_combinado_lat0p6']


def f(x, d=3):
    return '—' if x is None else f'{x:.{d}f}'


def trip(st, d=3):
    return '—' if not st else f'{st["p50"]:.{d}f} / {st["p90"]:.{d}f} / {st["p95"]:.{d}f}'


def hdr(first):
    return [f'| {first} | ' + ' | '.join(NAME[v] for v in V) + ' |', '|---' * (len(V) + 1) + '|']


o = ['### Tabla principal (384 corridas por variante)\n',
     '| YawTol | Correct | FalseSuccess | FalseYaw | GoalTol | Timeout | Contacts | Blocks | Yaw p90 (rad) | Time p50 / p90 (s) |',
     '|---|---|---|---|---|---|---|---|---|---|']
for v in V:
    s = S['variants'][v]
    o.append(f'| {NAME[v]} | {s["correct"]} | {s["false_success"]} | {s["false_yaw"]} | {s["goal_tol"]} | {s["timeout"]} | '
             f'{s["contacts"]} | {s["bloqueos"]} | {f(s["yaw_true_reached"]["p90"])} | {s["t_goal"]["p50"]:.1f} / {s["t_goal"]["p90"]:.1f} |')
o.append('\nYaw p90 = error final REAL de las corridas que llegan en xy. Time = tiempo hasta SUCCEEDED. FalseYaw = SUCCEEDED con yaw '
         'REAL > 0.25 rad. Timeout = clase TIMEOUT (no llega a la ventana xy). GoalTol = llega en xy sin SUCCEEDED; su causa en todas '
         'las variantes es ABORT_PROGRESS (ver abajo). Criterio de "correcto" fijo: pose REAL ≤ 0.25 m / 0.25 rad.\n')

o += ['### Clases extendidas'] + [''] + hdr('clase')
keys = sorted({k for v in V for k in S['variants'][v]['subcls']})
for k in keys:
    o.append(f'| {k} | ' + ' | '.join(str(S['variants'][v]['subcls'].get(k, 0)) for v in V) + ' |')

o += ['', '### Error final: p50 / p90 / p95 (corridas que llegan en xy, n = 338 en todas)', ''] + hdr('métrica')
for k, lab in (('xy_true_reached', 'XY REAL (m)'), ('xy_est_reached', 'XY ESTIMADO (m)'),
               ('yaw_true_reached', 'yaw REAL (rad)'), ('yaw_est_reached', 'yaw ESTIMADO (rad)'),
               ('yaw_true_ok', 'yaw REAL, solo correctas (rad)'), ('xy_true_ok', 'XY REAL, solo correctas (m)')):
    o.append(f'| {lab} | ' + ' | '.join(trip(S['variants'][v][k]) for v in V) + ' |')

o += ['', '### Tiempo, giro cerca del goal, oscilaciones y comandos finales (p50 / p90 / p95)', ''] + hdr('métrica')
for k, lab, d in (('t_goal', 'tiempo hasta SUCCEEDED (s)', 1), ('t_goal_minus_xy', 'SUCCEEDED − entrada en ventana xy (s)', 1),
                  ('t_near_rotating', 'tiempo girando en el lugar dentro de 0.25 m (s)', 1),
                  ('t_near_goal', 'tiempo total dentro de 0.25 m (s)', 1),
                  ('w_flips_total', 'cambios de signo de angular.z, corrida completa', 0),
                  ('flips_pre_rot', 'cambios de signo antes de empezar a girar', 0),
                  ('flips_post_rot', 'cambios de signo después de empezar a girar', 0),
                  ('yaw_overshoot', 'sobrepaso de yaw al frenar (rad)', 3), ('w_at_trigger', '|ω| real al disparar el checker (rad/s)', 2),
                  ('last_nz_w', '|ω| del último comando no nulo, SUCCEEDED (rad/s)', 3),
                  ('last_nz_v', '|v| del último comando no nulo, SUCCEEDED (m/s)', 3)):
    o.append(f'| {lab} | ' + ' | '.join(trip(S['variants'][v][k], d) for v in V) + ' |')
o.append('| cambios de signo después de empezar a girar, media | ' + ' | '.join(f(S['variants'][v]['flips_post_rot']['mean'], 2) for v in V) + ' |')
o.append('| corridas cuyo último comando no nulo es giro en el lugar | ' + ' | '.join(str(S['variants'][v]['last_nz_rot_in_place']) for v in V) + ' |')
o.append('| clearance mínima global (m) | ' + ' | '.join(f(S['variants'][v]['clear_min']) for v in V) + ' |')

o += ['', '### Por escenario: correctos / FalseYaw / GoalTol / bloqueos / contactos', ''] + hdr('escenario')
for s in SCEN:
    o.append(f'| {s} | ' + ' | '.join('{correct} / {false_yaw} / {goal_tol} / {bloqueos} / {contacts}'.format(**S['by_scenario'][f'{s}|{v}'])
                                       for v in V) + ' |')
o += ['', '### Corredores estrechos: correctos / FalseYaw / GoalTol / bloqueos / contactos / clearance mín (m)', ''] + hdr('corredor')
for c in ('corridor_N1', 'corridor_N2'):
    o.append(f'| {c} | ' + ' | '.join('{correct} / {false_yaw} / {goal_tol} / {bloqueos} / {contacts} / {clear_min:.3f}'.format(
        **S['narrow'][f'{c}|{v}']) for v in V) + ' |')

o += ['', '### Comparación pareada contra yaw 0.25 (clave scenario + path_id + seed)', '',
      '| variante | FIXED | BROKEN | UNCHANGED_OK | UNCHANGED_FAIL | 25 FalseYaw → OK | → fallo | siguen FalseYaw | correctos rotos (destino) | contactos nuevos | bloqueos nuevos | Δ yaw REAL p50, pares OK (rad) | Δ t SUCCEEDED p50 / p90, pares OK (s) |',
      '|---|---|---|---|---|---|---|---|---|---|---|---|---|']
for v in V[1:]:
    p = S['paired'][v]
    L = p['labels']
    o.append(f'| {NAME[v]} | {L.get("FIXED", 0)} | {L.get("BROKEN", 0)} | {L.get("UNCHANGED_OK", 0)} | {L.get("UNCHANGED_FAIL", 0)} | '
             f'{p["base_false_yaw_to_ok"]} | {p["base_false_yaw_to_fail"]} | {p["base_false_yaw_still_false"]} | '
             f'{p["base_ok_broken"]} ({", ".join(f"{k} {n}" for k, n in p["base_ok_broken_dest"].items()) or "—"}) | '
             f'{len(p["contact_new"])} | {len(p["block_new"])} | {f(p["dyaw_both_ok"]["p50"])} | '
             f'{p["dt_goal_both_ok"]["p50"]:.2f} / {p["dt_goal_both_ok"]["p90"]:.2f} |')
o.append('')
for v in V[1:]:
    p = S['paired'][v]
    o.append(f'<details><summary>{NAME[v]}: transiciones y listas</summary>\n')
    o.append('| transición (yaw 0.25 → ' + NAME[v] + ') | corridas |\n|---|---|')
    for k, n in p['transitions'].items():
        o.append(f'| {k} | {n} |')
    o.append(f'\nBROKEN: {", ".join(p["broken_list"]) or "—"}\n')
    o.append(f'FIXED: {", ".join(p["fixed_list"]) or "—"}\n')
    o.append('</details>\n')
open(os.path.join(LAB, 'dwb_r4f/r4f_tables.md'), 'w').write('\n'.join(o) + '\n')
print('\n'.join(o))
