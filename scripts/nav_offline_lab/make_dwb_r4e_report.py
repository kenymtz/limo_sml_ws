#!/usr/bin/python3
"""NAV-OFFLINE-R4-E — tablas MEDIDO desde dwb_r4e/r4e_summary.json -> dwb_r4e/r4e_tables.md"""
import json
import os

LAB = os.path.expanduser('~/limo_sml_ws/results/navigation_offline_lab')
S = json.load(open(os.path.join(LAB, 'dwb_r4e/r4e_summary.json')))
V = ['T025', 'T020', 'T015', 'T010']
NAME = {'NORTG': 'sin RTG (R4-B)', 'T025': '0.25 (R4-D)', 'T020': '0.20', 'T015': '0.15', 'T010': '0.10'}
SCEN = ['S1_nominal', 'S2_ruido_xy', 'S3_ruido_yaw', 'S4_latencia_0p6', 'S5_combinado_medido', 'S6_combinado_lat0p6']


def f(x, d=3):
    return '—' if x is None else f'{x:.{d}f}'


def q(st, k='p50', d=3):
    return '—' if not st else f(st[k], d)


def trip(st, d=3):
    return '—' if not st else f'{st["p50"]:.{d}f} / {st["p90"]:.{d}f} / {st["p95"]:.{d}f}'


o = []
o.append('### Tabla principal (384 corridas por variante)\n')
o.append('| Tolerance | Correct | GoalTol | False success | Contacts | Blocks | XY p90 | Yaw p90 |')
o.append('|---|---|---|---|---|---|---|---|')
for v in ['NORTG'] + V:
    s = S['variants'][v]
    o.append(f'| {NAME[v]} | {s["correct"]} | {s["goal_tol"]} | {s["false_success"]} | {s["contacts"]} | {s["bloqueos"]} | '
             f'{q(s["xy_true_reached"], "p90")} | {q(s["yaw_true_reached"], "p90")} |')
o.append('\nXY / Yaw p90 = error final VERDADERO (m / rad) de las corridas que llegan en xy (ventana del checker, pose estimada). '
         '"False success" = SUCCEEDED según pose estimada con pose verdadera fuera de 0.25 m / 0.25 rad (clase OTRO_EXITO_APARENTE). '
         '"Blocks" = OSCILACIÓN + TIMEOUT + sin trayectorias legales.\n')

o.append('### Clases\n')
o.append('| clase | ' + ' | '.join(NAME[v] for v in V) + ' |')
o.append('|---' * (len(V) + 1) + '|')
for c in S['variants']['T025']['cls']:
    if any(S['variants'][v]['cls'][c] for v in V):
        o.append(f'| {c} | ' + ' | '.join(str(S['variants'][v]['cls'][c]) for v in V) + ' |')

o.append('\n### Precisión final: p50 / p90 / p95 (corridas que llegan en xy)\n')
o.append('| métrica | ' + ' | '.join(NAME[v] for v in V) + ' |')
o.append('|---' * (len(V) + 1) + '|')
for k, lab, d in (('xy_true_reached', 'error XY final REAL (m)', 3), ('xy_est_reached', 'error XY final ESTIMADO (m)', 3),
                  ('yaw_true_reached', 'error yaw final REAL (rad)', 3), ('yaw_est_reached', 'error yaw final ESTIMADO (rad)', 3),
                  ('xy_true_ok', 'error XY REAL, solo correctas (m)', 3), ('yaw_true_ok', 'error yaw REAL, solo correctas (rad)', 3)):
    o.append(f'| {lab} | ' + ' | '.join(trip(S['variants'][v][k], d) for v in V) + ' |')
o.append('\nn de corridas que llegan en xy: ' + ', '.join(f'{NAME[v]} {S["variants"][v]["xy_true_reached"]["n"]}' for v in V) + '.\n')

o.append('### Tiempo, comportamiento cerca del goal, oscilaciones y comandos finales\n')
o.append('| métrica | ' + ' | '.join(NAME[v] for v in V) + ' |')
o.append('|---' * (len(V) + 1) + '|')
rows = (('t_goal', 'tiempo hasta SUCCEEDED p50 / p90 / p95 (s)', 'trip', 1),
        ('t_goal_minus_xy', 'SUCCEEDED − entrada en ventana xy p50 / p90 / p95 (s)', 'trip', 1),
        ('t_near_goal', 'tiempo dentro de 0.25 m del goal (est.) p50 / p90 / p95 (s)', 'trip', 1),
        ('t_near_rotating', '  … girando en el lugar p50 / p90 / p95 (s)', 'trip', 1),
        ('t_near_stopped', '  … detenido (v y ω ≈ 0) p50 / p90 / p95 (s)', 'trip', 1),
        ('t_near_translating', '  … trasladando p50 / p90 / p95 (s)', 'trip', 1),
        ('rot_start_xy_true', 'distancia REAL al goal al empezar a girar p50 / p90 / p95 (m)', 'trip', 3),
        ('yaw_overshoot', 'sobrepaso de yaw al frenar p50 / p90 / p95 (rad)', 'trip', 3),
        ('w_at_trigger', '|ω| al disparar el checker p50 / p90 / p95 (rad/s)', 'trip', 2),
        ('w_flips_after_xy', 'inversiones de ω tras llegar en xy p50 / p90 / p95', 'trip', 1),
        ('w_flips_per_min', 'inversiones de ω por minuto p50 / p90 / p95', 'trip', 1))
for k, lab, kind, d in rows:
    o.append(f'| {lab} | ' + ' | '.join(trip(S['variants'][v][k], d) for v in V) + ' |')
o.append('| inversiones de ω tras llegar en xy, media | ' + ' | '.join(f(S['variants'][v]['w_flips_after_xy']['mean'], 2) for v in V) + ' |')
o.append('| corridas cuyo último comando no nulo es giro en el lugar | ' + ' | '.join(str(S['variants'][v]['last_nz_rot_in_place']) for v in V) + ' |')
o.append('| ciclos DWB sin trayectoria legal | ' + ' | '.join(str(S['variants'][v]['n_fail_dwb']) for v in V) + ' |')
o.append('| tiempo DWB por ciclo p50 (ms) | ' + ' | '.join(q(S['variants'][v]['dwb_ms'], 'p50', 2) for v in V) + ' |')

o.append('\n### Por escenario: correctos / GoalTol / falsos éxitos / bloqueos / contactos\n')
o.append('| escenario | ' + ' | '.join(NAME[v] for v in V) + ' |')
o.append('|---' * (len(V) + 1) + '|')
for s in SCEN:
    o.append(f'| {s} | ' + ' | '.join('{correct} / {goal_tol} / {false_success} / {bloqueos} / {contacts}'.format(**S['by_scenario'][f'{s}|{v}'])
                                       for v in V) + ' |')

o.append('\n### Corredores estrechos N1 / N2 (48 corridas cada uno = 8 paths × 6 escenarios)\n')
o.append('| corredor | variante | correctos | GoalTol | falsos éxitos | bloqueos | contactos | tiempo en contacto (s) | corridas < padding | clearance mín (m) | clearance p05 (m) |')
o.append('|---|---|---|---|---|---|---|---|---|---|---|')
for c in ('corridor_N1', 'corridor_N2'):
    for v in V:
        s = S['narrow'][f'{c}|{v}']
        o.append(f'| {c} | {NAME[v]} | {s["correct"]} | {s["goal_tol"]} | {s["false_success"]} | {s["bloqueos"]} | {s["contacts"]} | '
                 f'{s["contact_time"]:.2f} | {s["margin_viol"]} | {s["clear_min"]:.3f} | {s["clear_min_p05"]:.3f} |')

o.append('\n### Por categoría: correctos (GoalTol / falsos éxitos)\n')
cats = sorted({k.split('|')[0] for k in S['by_category']})
o.append('| categoría | ' + ' | '.join(NAME[v] for v in V) + ' |')
o.append('|---' * (len(V) + 1) + '|')
for c in cats:
    o.append(f'| {c} | ' + ' | '.join('{correct} ({goal_tol} / {false_success})'.format(**S['by_category'][f'{c}|{v}']) for v in V) + ' |')

o.append('\n### Comparación pareada contra R4-D 0.25 (clave scenario + path_id; semilla fija por path y escenario)\n')
o.append('| variante | FIXED | BROKEN | UNCHANGED_OK | UNCHANGED_FAIL | 46 rotas por R4-D recuperadas | 122 arregladas por R4-D que se rompen | contactos nuevos | bloqueos nuevos / eliminados |')
o.append('|---|---|---|---|---|---|---|---|---|')
for v in V[1:]:
    p = S['paired'][v]
    L = p['labels']
    o.append(f'| {NAME[v]} | {L.get("FIXED", 0)} | {L.get("BROKEN", 0)} | {L.get("UNCHANGED_OK", 0)} | {L.get("UNCHANGED_FAIL", 0)} | '
             f'{p["r4d_broken46_recovered"]}/46 | {p["r4d_fixed122_rebroken"]}/122 | {len(p["contact_new"])} | '
             f'{len(p["block_new"])} / {len(p["block_removed"])} |')
o.append('')
for v in V[1:]:
    p = S['paired'][v]
    o.append(f'**{NAME[v]}**: de las 46 rotas por R4-D siguen fallando {46 - p["r4d_broken46_recovered"]} '
             f'({", ".join(f"{k} {n}" for k, n in sorted(p["r4d_broken46_still_fail"].items()))}). '
             f'Pares OK en ambas: Δ error xy REAL p50 {q(p["dxy_true_both_ok"])} m, Δ yaw REAL p50 {q(p["dyaw_true_both_ok"])} rad, '
             f'Δ t SUCCEEDED p50 {q(p["dt_goal_both_ok"], "p50", 1)} s. Δ clearance mínima por corrida en N1/N2 (variante − 0.25) '
             f'p50 {q(p["dclear_narrow"])} m, mín {q(p["dclear_narrow"], "min")} m.')
o.append('')
for v in V[1:]:
    p = S['paired'][v]
    o.append(f'<details><summary>{NAME[v]}: transiciones y listas</summary>\n')
    o.append('| transición (0.25 → ' + NAME[v] + ') | corridas |\n|---|---|')
    for k, n in p['transitions'].items():
        o.append(f'| {k} | {n} |')
    o.append(f'\nBROKEN: {", ".join(p["broken_list"]) or "—"}\n')
    o.append(f'122 re-rotas: {", ".join(p["r4d_fixed122_rebroken_list"]) or "—"}\n')
    o.append(f'Contactos nuevos: {", ".join(p["contact_new"]) or "—"}; eliminados: {", ".join(p["contact_removed"]) or "—"}\n')
    o.append(f'Bloqueos nuevos: {", ".join(p["block_new"]) or "—"}; eliminados: {", ".join(p["block_removed"]) or "—"}\n')
    o.append('</details>\n')
open(os.path.join(LAB, 'dwb_r4e/r4e_tables.md'), 'w').write('\n'.join(o) + '\n')
print('\n'.join(o))
