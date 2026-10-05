#!/usr/bin/python3
"""NAV-OFFLINE-R4-D — genera results/navigation_offline_lab/DWB_R4D_ROTATE_TO_GOAL_REPORT.md desde dwb_r4d/r4d_summary.json."""
import json
import os

LAB = os.path.expanduser('~/limo_sml_ws/results/navigation_offline_lab')
D = os.path.join(LAB, 'dwb_r4d')
SCEN = ['S1_nominal', 'S2_ruido_xy', 'S3_ruido_yaw', 'S4_latencia_0p6', 'S5_combinado_medido', 'S6_combinado_lat0p6']
CLS = ['OK', 'GOAL_TOLERANCE', 'OSCILACION', 'TIMEOUT', 'TRACKING', 'OTRO_SIN_TRAYECT', 'OTRO_EXITO_CONTACTO',
       'OTRO_EXITO_APARENTE', 'OTRO']


def f(x, n=3):
    return '—' if x is None else f'{x:.{n}f}'


def st(d, k='med', n=3):
    return '—' if not d else f(d[k], n)


def main():
    S = json.load(open(os.path.join(D, 'r4d_summary.json')))
    man = json.load(open(os.path.expanduser('~/limo_sml_ws/config/offline_lab/dwb_offline_R4D_manifest.json')))
    extra = open(os.path.join(D, 'findings.md')).read() if os.path.exists(os.path.join(D, 'findings.md')) else ''
    rec = open(os.path.join(D, 'recommendation.md')).read() if os.path.exists(os.path.join(D, 'recommendation.md')) else ''
    B, R = S['variants']['BASE'], S['variants']['RTG']
    P = S['paired']
    L = []
    A = L.append
    A('# NAV-OFFLINE-R4-D — RotateToGoal en lazo cerrado (DWB real + simulador R4-B)\n')
    A('Fecha: 2026-09-29. 100 % offline (ROS_DOMAIN_ID=99, ROS_LOCALHOST_ONLY=1). Sin hardware, sin limo_base, sin LiDAR, sin `/cmd_vel` '
      '(simulador sin publishers Twist; auditoría del grafo en cada corrida). `navigation2_real.yaml` sin cambios '
      f'(sha256 `{man["real_sha256"][:16]}…`), `~/limoatwork` sin cambios.\n')
    A('## Resumen\n')
    A('```')
    A('R4-D')
    A(f'Sin RotateToGoal:  correct successes {B["correct"]}/{B["n"]}')
    A(f'Con RotateToGoal:  correct successes {R["correct"]}/{R["n"]}')
    A(f'Cambio yaw:        error yaw final (corridas que llegan en xy) mediana {st(B["final_yaw_reached"])} → {st(R["final_yaw_reached"])} rad, '
      f'p90 {st(B["final_yaw_reached"], "p90")} → {st(R["final_yaw_reached"], "p90")} rad')
    A(f'Cambio oscilaciones: inversiones de ω tras llegar en xy (media) {st(B["w_flips_after_xy"], "mean", 1)} → {st(R["w_flips_after_xy"], "mean", 1)}; '
      f'por minuto (mediana) {st(B["w_flips_per_min"], "med", 1)} → {st(R["w_flips_per_min"], "med", 1)}')
    A('```\n')
    A('## Método\n')
    A('- **Config experimental** `config/offline_lab/navigation2_R4D.yaml`: copia completa de `navigation2_real.yaml`. Cambios verificados por diff '
      f'de dicts: {", ".join(x.split(".")[-1] for x in man["diff_vs_real"])}; critics = `{man["critics"]}`. Es decir, footprint B (ObstacleFootprint, R4-A2) '
      '+ `RotateToGoal` en la posición de la línea comentada original, con sus parámetros ya presentes en la config real (scale 32, slowing_factor 5, '
      'lookahead_time −1; usa `xy_goal_tolerance` 0.25 y `trans_stopped_velocity` 0.03 de FollowPath).')
    A('- **Simulador**: `config/offline_lab/dwb_offline_R4D_fp_rtg.yaml` = config R4-B variante B + RotateToGoal. Assert: su `controller_server` es '
      'idéntico al de `navigation2_R4D.yaml`.')
    A('- **BASELINE** = variante B de R4-B (ObstacleFootprint, sin RotateToGoal), **reutilizada**. Antes se verificó que el binario actual la reproduce '
      'idéntica paso a paso (16/16 corridas en S1 y S5, `dwb_r4d/sim_chk_base_*.jsonl`). **EXPERIMENTAL** = misma config + RotateToGoal, mismos 64 paths, '
      'mismos 6 escenarios, mismos archivos de corridas y mismas semillas de ruido (comparación pareada).')
    A('- **Clasificación**: reglas idénticas a R4-C (assert: la baseline reproduce exactamente las clases de R4-C para B). "Correcto" = SUCCEEDED, '
      'sin contacto, pose **verdadera** final ≤ 0.25 m y ≤ 0.25 rad. "Bloqueos" = OSCILACIÓN + TIMEOUT + sin trayectorias legales.\n')
    A('## CONFIRMADO\n')
    A('- `dwb_critics::RotateToGoalCritic` se carga con la config experimental (`Using critic "RotateToGoal"`), junto con los otros 6 critics.')
    A('- Baseline reproducible: 16/16 corridas idénticas a R4-B con el binario actual; clasificación de la baseline = R4-C (assert).')
    if extra:
        A(extra)
    A('')
    A('## MEDIDO\n')
    A('### Clases (384 corridas por variante)\n')
    A('| clase | sin RotateToGoal | con RotateToGoal |')
    A('|---|---|---|')
    for c in CLS:
        if B['cls'].get(c) or R['cls'].get(c):
            A(f"| {c} | {B['cls'].get(c, 0)} | {R['cls'].get(c, 0)} |")
    A(f"| **contactos** | {B['contacts']} | {R['contacts']} |")
    A(f"| **bloqueos** | {B['bloqueos']} | {R['bloqueos']} |")
    A('')
    A('### Por escenario: correctos / GOAL_TOLERANCE / éxitos aparentes / bloqueos / contactos\n')
    A('| escenario | sin RTG | con RTG |')
    A('|---|---|---|')
    for s in SCEN:
        b, r = S['by_scenario'].get(f'{s}|BASE'), S['by_scenario'].get(f'{s}|RTG')
        fmt = lambda x: (f"{x['correct']} / {x['cls']['GOAL_TOLERANCE']} / {x['cls']['OTRO_EXITO_APARENTE']} / {x['bloqueos']} / {x['contacts']}"  # noqa: E731
                         if x else '—')
        A(f'| {s} | {fmt(b)} | {fmt(r)} |')
    A('')
    A('### Precisión final, tiempo, oscilaciones y comandos\n')
    A('| métrica | sin RotateToGoal | con RotateToGoal |')
    A('|---|---|---|')
    rowsm = [
        ('error xy final, corridas que llegan en xy: mediana / p90 (m)', 'final_xy_reached', 'med', 'p90', 3),
        ('error yaw final, corridas que llegan en xy: mediana / p90 (rad)', 'final_yaw_reached', 'med', 'p90', 3),
        ('error xy final, correctas: mediana / p90 (m)', 'final_xy_ok', 'med', 'p90', 3),
        ('error yaw final, correctas: mediana / p90 (rad)', 'final_yaw_ok', 'med', 'p90', 3),
        ('tiempo hasta SUCCEEDED: mediana / p90 (s)', 't_goal', 'med', 'p90', 1),
        ('tiempo hasta llegada xy: mediana / p90 (s)', 't_xy', 'med', 'p90', 1),
        ('tiempo de SUCCEEDED − llegada xy: mediana / p90 (s)', 't_goal_minus_xy', 'med', 'p90', 1),
        ('inversiones de ω por minuto: mediana / p90', 'w_flips_per_min', 'med', 'p90', 1),
        ('inversiones de ω tras llegar en xy: media / p90', 'w_flips_after_xy', 'mean', 'p90', 1),
        ('|ω| real al disparar el goal checker: mediana / máx (rad/s)', 'w_at_trigger', 'med', 'max', 2),
        ('sobrepaso de yaw al frenar: mediana / máx (rad)', 'yaw_overshoot', 'med', 'max', 3),
        ('error lateral medio por corrida: mediana (m)', 'lat_mean', 'med', 'p90', 3),
        ('tiempo DWB por ciclo, media por corrida: mediana / máx (ms)', 'dwb_ms', 'med', 'max', 2),
    ]
    for lab, k, a1, a2, n in rowsm:
        A(f'| {lab} | {st(B[k], a1, n)} / {st(B[k], a2, n)} | {st(R[k], a1, n)} / {st(R[k], a2, n)} |')
    A(f"| clearance mínima (m) | {f(B['clear_min'])} | {f(R['clear_min'])} |")
    A(f"| corridas cuyo último comando no nulo es giro en el lugar | {B['last_nz_rot_in_place']} | {R['last_nz_rot_in_place']} |")
    A(f"| fallas de DWB (ciclos sin trayectoria legal) | {B['n_fail_dwb']} | {R['n_fail_dwb']} |")
    A(f"| tiempo DWB máximo por ciclo (ms) | {f(B['dwb_ms_max'], 1)} | {f(R['dwb_ms_max'], 1)} |")
    A('')
    A('### Comparación pareada (mismo path, escenario y semilla)\n')
    A(f"- Pares: {P['n']}. Arreglados (no OK → OK): **{P['fixed']}**. Rotos (OK → no OK): **{P['broken']}**"
      + (f" ({', '.join(P['broken_list'])})" if P['broken_list'] else '') + '.')
    A(f"- Contactos nuevos con RotateToGoal: {len(P['contact_new'])} ({', '.join(P['contact_new']) or '—'}); "
      f"contactos que desaparecen: {len(P['contact_removed'])} ({', '.join(P['contact_removed']) or '—'}).")
    if P.get('dt_goal_both_ok'):
        A(f"- Pares OK en ambas: Δt de SUCCEEDED (RTG − base) mediana {f(P['dt_goal_both_ok']['med'], 1)} s; Δ error xy final mediana "
          f"{f(P['dxy_both_ok']['med'])} m (p90 {f(P['dxy_both_ok']['p90'])}).")
    A('\n| transición (sin RTG → con RTG) | corridas |')
    A('|---|---|')
    for k, v in P['transitions'].items():
        A(f'| {k} | {v} |')
    A('')
    A('### Por categoría de path: correctos sin RTG → con RTG\n')
    A('| categoría | correctos | GOAL_TOLERANCE | bloqueos | contactos |')
    A('|---|---|---|---|---|')
    for c in sorted({k.split('|')[0] for k in S['by_category']}):
        b, r = S['by_category'][f'{c}|BASE'], S['by_category'].get(f'{c}|RTG')
        if r:
            A(f"| {c} | {b['correct']} → {r['correct']} | {b['cls']['GOAL_TOLERANCE']} → {r['cls']['GOAL_TOLERANCE']} | {b['bloqueos']} → {r['bloqueos']} | "
              f"{b['contacts']} → {r['contacts']} |")
    A('\nFigura: `dwb_r4d/r4d_overview.png`. Datos por corrida: `dwb_r4d/r4d_runs.csv`.\n')
    A('## LIMITACIONES\n')
    A('- Mismas del simulador R4-B: modelo cinemático mínimo, odometría perfecta, percepción perfecta (mapa estático), sin BT (plan fijo, sin '
      'recuperaciones), lógica de controller_server replicada, una semilla por caso, ruido de localización sintético, mapa sintético.')
    A('- Solo se probó RotateToGoal con sus parámetros existentes (scale 32, slowing_factor 5, lookahead −1) y con las tolerancias actuales '
      '(xy 0.25, yaw 0.25). No se exploraron otros valores.')
    A('- Solo con ObstacleFootprint (footprint B). No se midió BaseObstacle + RotateToGoal.')
    A('- La precisión final depende de `xy_goal_tolerance`, que aquí vale lo mismo para RotateToGoal y para el goal checker (0.25 m).\n')
    A('## Archivos\n')
    A('- Configs: `config/offline_lab/navigation2_R4D.yaml` (candidata completa, NO producción), `dwb_offline_R4D_fp_rtg.yaml` (simulador), '
      '`dwb_offline_R4D_manifest.json`')
    A('- Scripts: `scripts/nav_offline_lab/{make_dwb_r4d_config.py, run_r4d_all.sh, evaluate_dwb_r4d.py, make_dwb_r4d_report.py}`')
    A('- Resultados: `results/navigation_offline_lab/dwb_r4d/` (sim_*_R.jsonl, sim_chk_*.jsonl, r4d_runs.csv, r4d_summary.json, r4d_overview.png, auditorías)\n')
    A('## Recomendación\n')
    A(rec)
    open(os.path.join(LAB, 'DWB_R4D_ROTATE_TO_GOAL_REPORT.md'), 'w').write('\n'.join(L) + '\n')
    print('ok')


if __name__ == '__main__':
    main()
