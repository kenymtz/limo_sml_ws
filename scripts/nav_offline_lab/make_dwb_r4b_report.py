#!/usr/bin/python3
"""NAV-OFFLINE-R4-B — genera results/navigation_offline_lab/DWB_R4B_CLOSED_LOOP_REPORT.md desde r4b_summary.json."""
import json
import os
import sys

LAB = os.path.expanduser('~/limo_sml_ws/results/navigation_offline_lab')
D = os.path.join(LAB, 'dwb_r4b')
SCEN = ['S1_nominal', 'S2_ruido_xy', 'S3_ruido_yaw', 'S4_latencia_0p6', 'S5_combinado_medido', 'S6_combinado_lat0p6']
SDESC = {
    'S1_nominal': 'sin ruido, sin latencia, tracking ideal (k=1)',
    'S2_ruido_xy': 'ruido de localización XY σ=0.03 m por eje (Gauss-Markov τ=1 s)',
    'S3_ruido_yaw': 'ruido de localización yaw σ=0.05 rad (2.9°), τ=1 s',
    'S4_latencia_0p6': 'latencia pura de base 0.6 s (valor pedido; ESTRÉS, ver CONFIRMADO)',
    'S5_combinado_medido': 'XY 0.03 + yaw 0.05 + latencia 0.05 s (medida) + tracking k_v 0.95 / k_w 0.92',
    'S6_combinado_lat0p6': 'S5 con latencia 0.6 s',
}


def f(x, n=3):
    return '—' if x is None else (f'{x:.{n}f}' if isinstance(x, float) else str(x))


def main():
    S = json.load(open(os.path.join(D, 'r4b_summary.json')))
    NEXT = open(os.path.join(D, 'next_step.md')).read() if os.path.exists(os.path.join(D, 'next_step.md')) else ''
    EXTRA = open(os.path.join(D, 'findings_extra.md')).read() if os.path.exists(os.path.join(D, 'findings_extra.md')) else ''
    mc = S['model_checks']
    B = S['by_scenario']
    T = S['totals']
    L = []
    A = L.append
    A('# NAV-OFFLINE-R4-B — Lazo cerrado: simulador cinemático LIMO + DWB real (BaseObstacle vs ObstacleFootprint)\n')
    A('Fecha: 2026-09-29. 100 % offline (ROS_DOMAIN_ID=99, ROS_LOCALHOST_ONLY=1). Sin hardware, sin limo_base, sin LiDAR, sin teleop, '
      'sin `/cmd_vel` (el simulador no crea publishers Twist; auditoría de grafo PASS en cada corrida). `navigation2_real.yaml` y `~/limoatwork` sin cambios.\n')
    A('## Resumen\n')
    A('```')
    A('R4-B')
    A(f'MODELO:                     {"PASS" if mc["PASS"] else "FAIL"}')
    A(f'ESCENARIOS:                 {len(SCEN)}  (× 64 paths × 2 variantes = {T["A"]["n"] + T["B"]["n"]} corridas)')
    A(f'CONTACTOS BaseObstacle:     {T["A"]["contacts"]} corridas con contacto real (de {T["A"]["n"]})')
    A(f'CONTACTOS ObstacleFootprint:{T["B"]["contacts"]:>3} corridas con contacto real (de {T["B"]["n"]})')
    A('```\n')
    A('## Método\n')
    A('- **Simulador** `src/limo_nav_offline_lab/src/dwb_closed_loop_sim.cpp`: la clase real `dwb_core::DWBLocalPlanner` y el `Costmap2DROS` real '
      '(mismos parámetros que controller_server, configs `config/offline_lab/dwb_offline_R4B_{base,fp}.yaml` derivadas de A2) en un lazo de tiempo simulado:')
    A('  - control 10 Hz (`controller_frequency`): `computeVelocityCommands(pose, odom_twist)` → lógica replicada de controller_server '
      '(`failure_tolerance` 0.3 s con comando cero, `SimpleProgressChecker` 0.1 m / 25 s, `SimpleGoalChecker` xy 0.25 / yaw 0.25 stateful) → cola de latencia;')
    A('  - base FOUR_DIFF (vx, yaw_rate) a 100 Hz: ganancia de tracking k_v/k_w, saturación |vx| ≤ 0.25 y |ω| ≤ 1.0, rampas acc 1.0 / 1.5 y decel 1.0 / 1.5, '
      'timeout de comando 0.6 s; integración exacta de arco;')
    A('  - localización: odometría perfecta (odom = pose verdadera); el error de localización entra como `map→odom` ruidoso (Gauss-Markov). '
      'El plan (frame `map`) se ve desplazado; las paredes del local costmap (`static_layer`, frame `map_true` colgado de odom) se ven en su posición '
      'relativa real, como un LiDAR (proxy de percepción perfecta);')
    A('  - costmap: loop propio desactivado y `updateMap()` síncrono en cada paso; chequeo de cordura al arrancar (120 celdas LETHAL junto a la pared oeste).')
    A('- **Paths**: los mismos 64 paths NavFn de R3/R4-A/A2 sobre `synthetic_arena`, desde el start del benchmark (v=0), plan fijo (sin replanificación del BT).')
    A('- **Pareado**: la semilla del ruido depende solo de (path, escenario): A y B reciben la misma secuencia.')
    A('- **Verdad de terreno offline**: geometría del PGM y footprint real sin padding (clearance con signo a 50 Hz sobre la pose verdadera). '
      'Contacto = clearance < 0.\n')
    A('| escenario | definición |')
    A('|---|---|')
    for s in SCEN:
        A(f'| {s} | {SDESC[s]} |')
    A('')

    A('## CONFIRMADO\n')
    A('- **Modelo** (chequeos automáticos sobre todos los logs):\n')
    A('| chequeo | resultado | detalle |')
    A('|---|---|---|')
    for k, v in mc.items():
        if isinstance(v, dict):
            det = ', '.join(f'{a}={f(b) if isinstance(b, float) else b}' for a, b in v.items() if a != 'PASS')
            A(f'| {k} | {"PASS" if v["PASS"] else "FAIL"} | {det} |')
    A('')
    A('- **Latencia de la base, dato real**: `BASELINE_REAL_MAP_R1.md` (bag 2026-09-18) mide latencia de arranque ~47–50 ms y un **timeout** de '
      'comando de ~0.5–0.6 s (la base se detiene si no recibe comando), no una latencia de 0.6 s. S4/S6 usan 0.6 s de latencia pura porque se pidió '
      'explícitamente: son escenarios de estrés, no el comportamiento medido. S5 usa lo medido (0.05 s + timeout 0.6 s).')
    A('- **`RotateToGoal` está desactivado en la config real** (comentado en `navigation2_real.yaml`) y el goal checker exige yaw ≤ 0.25 rad. '
      'Ningún critic activo orienta el robot al yaw del goal: GoalAlign/PathAlign alinean con la dirección del path. Consecuencia medida abajo.')
    A('- El orden de padres en TF importa: tf2 admite un solo padre por frame. La primera versión colgaba `odom` de `map` y de `map_true` y dejaba el '
      'local costmap vacío (detectado por el warning de StaticLayer y corregido; esos resultados se descartaron). El chequeo de cordura evita repetirlo.\n')
    if EXTRA:
        A(EXTRA + '\n')

    A('## MEDIDO\n')
    A('### Resultado por escenario\n')
    A('| escenario | var. | SUCCEEDED | llegada xy | ABORT progreso | ABORT controller | TIMEOUT | **contactos** | clearance < 0.02 | clearance mín (m) | '
      't llegada xy med (s) | t tras llegada xy med (s) |')
    A('|---|---|---|---|---|---|---|---|---|---|---|---|')
    for s in SCEN:
        for v in ('A', 'B'):
            b = B.get(f'{s}|{v}')
            if b:
                A(f"| {s} | {v} | {b['succeeded']} | {b['reached_xy']} | {b['abort_progress']} | {b['abort_controller']} | {b['timeout']} | "
                  f"**{b['contacts']}** | {b['margin_viol']} | {f(b['clear_min'])} | {f(b['t_xy_med'], 1)} | {f(b['t_after_xy_med'], 1)} |")
    A('\nA = BaseObstacle (DWB real), B = ObstacleFootprint. "llegada xy" = la pose estimada entra en 0.25 m del goal (primera etapa del goal checker). '
      '"t tras llegada xy" = tiempo que el robot sigue activo después de llegar en xy.\n')
    A('### Seguimiento y error final\n')
    A('| escenario | var. | error lateral medio (m) | p95 medio (m) | máx (m) | error final xy med / máx (m) | error final yaw med (rad) |')
    A('|---|---|---|---|---|---|---|')
    for s in SCEN:
        for v in ('A', 'B'):
            b = B.get(f'{s}|{v}')
            if b:
                A(f"| {s} | {v} | {f(b['lat_mean'])} | {f(b['lat_p95_mean'])} | {f(b['lat_max'])} | {f(b['final_xy_med'])} / "
                  f"{f(b['final_xy_max'])} | {f(b['final_yaw_med'])} |")
    A('\nError lateral = distancia de la pose verdadera al path NavFn hasta la llegada xy.\n')
    A('### Oscilaciones y comandos\n')
    A('| escenario | var. | cambios de signo de ω / min | cambios de signo tras llegada xy (media) | % giro en el lugar | % comando cero | '
      '% fallas DWB | % vx saturada | % ω saturada | vx media (m/s) | DWB ms media / máx |')
    A('|---|---|---|---|---|---|---|---|---|---|---|')
    for s in SCEN:
        for v in ('A', 'B'):
            b = B.get(f'{s}|{v}')
            if b:
                A(f"| {s} | {v} | {f(b['w_flips_per_min'], 1)} | {f(b['w_flips_after_xy_mean'], 1)} | {100 * b['frac_rotate']:.0f} | "
                  f"{100 * b['frac_zero']:.1f} | {100 * b['frac_fail']:.1f} | {100 * b['frac_sat_v']:.0f} | {100 * b['frac_sat_w']:.0f} | "
                  f"{f(b['cmd_v_mean'])} | {f(b['dwb_ms_mean'], 1)} / {f(b['dwb_ms_max'], 1)} |")
    A('\nCambio de signo de ω = inversión del giro comandado (|ω| > 0.05) respecto del anterior no nulo.\n')
    n1p = os.path.join(D, 'r4b_n1_entry.json')
    if os.path.exists(n1p):
        N1 = json.load(open(n1p))
        A('### corridor_N1: entrada al pasillo y contactos\n')
        A('| escenario | A entra / 8 | A contactos | B entra / 8 | B contactos |')
        A('|---|---|---|---|---|')
        te = {'A': [0, 0], 'B': [0, 0]}
        for sc in SCEN:
            a, b = N1[f'{sc}|A'], N1[f'{sc}|B']
            A(f"| {sc} | {a['entered']} | {a['contacts']} | {b['entered']} | {b['contacts']} |")
            for v, x in (('A', a), ('B', b)):
                te[v][0] += x['entered']
                te[v][1] += x['contacts']
        A(f"| **total** | **{te['A'][0]}/48** | **{te['A'][1]}** | **{te['B'][0]}/48** | **{te['B'][1]}** |")
        A('\nTodos los contactos de ambas variantes ocurren en corridor_N1. A entra poco porque se traba en la entrada (mínimo local, ver CONFIRMADO), '
          'pero **cuando entra, choca en 5 de 8 corridas** (penetración hasta 0.057 m, hasta 2.7 s en contacto) y termina SUCCEEDED sin enterarse. '
          'B entra en 37/48 y choca 2 veces, solo con latencia 0.6 s (0.008–0.009 m, < 0.65 s). En ambos casos DWB-B se queda sin trayectorias legales '
          'y el controller aborta (el robot se detiene).\n')
    A('### Fallas que no son contacto\n')
    for v in ('A', 'B'):
        t = T[v]
        A(f"- {v}: llegan en xy {t['reached_xy']}/{t['n']}, SUCCEEDED {t['succeeded']}/{t['n']}: **{t['reached_xy'] - t['succeeded']} corridas llegan en xy y "
          f"no cumplen el yaw** (giran de un lado a otro hasta el progress checker). Tiempo mediano de llegada xy {f(t['t_xy_med'], 1)} s.")
    ao = {v: S['by_category'][f'around_obstacle|{v}'] for v in ('A', 'B')}
    A(f"- around_obstacle: SUCCEEDED A {ao['A']['succeeded']}/{ao['A']['n']}, B {ao['B']['succeeded']}/{ao['B']['n']}. Mínimo local con el goal detrás "
      "del obstáculo: el robot gira en el lugar desde el arranque sin avanzar, en ambas variantes. No depende del critic de obstáculos.")
    A('- Las fallas de yaw casi desaparecen con ruido XY (S2, S5, S6) pero no con ruido de yaw (S3). Hipótesis, no verificada: el ruido XY mueve el plan y la '
      'meta estimada y agita el rumbo hasta cruzar la ventana de ±0.25 rad por azar. No es una mejora real del controlador.\n')
    A('### Contactos: detalle pareado\n')
    p = S['paired']
    A(f'- Pares (escenario, path) comparados: {p["n"]}. Estado final distinto entre A y B: {p["status_diff"]}.')
    A(f'- Contacto solo en A: {len(p["contact_A_only"])} ({", ".join(p["contact_A_only"]) or "—"})')
    A(f'- Contacto solo en B: {len(p["contact_B_only"])} ({", ".join(p["contact_B_only"]) or "—"})')
    A(f'- Contacto en ambas: {len(p["contact_both"])} ({", ".join(p["contact_both"]) or "—"})\n')
    A('### Por categoría de path (todos los escenarios)\n')
    A('| categoría | var. | corridas | SUCCEEDED | llegada xy | contactos | clearance mín (m) | error lateral medio (m) |')
    A('|---|---|---|---|---|---|---|---|')
    for k, b in sorted(S['by_category'].items()):
        c, v = k.split('|')
        A(f"| {c} | {v} | {b['n']} | {b['succeeded']} | {b['reached_xy']} | {b['contacts']} | {f(b['clear_min'])} | {f(b['lat_mean'])} |")
    A('\nFigura: `dwb_r4b/r4b_overview.png`. Datos por corrida: `dwb_r4b/r4b_runs.csv`.\n')

    A('## LIMITACIONES\n')
    A('- **Modelo cinemático mínimo**: sin deslizamiento, sin dinámica de motores más allá de las rampas, sin cuantización ni ruido de odometría. '
      'Odometría perfecta: todo el error de pose es de localización (map→odom).')
    A('- **Base identificada con pocos datos** (BASELINE_REAL_MAP_R1: solo v=0.239 y ω=±0.478). Ganancias y latencia de S5 son valores medidos gruesos; '
      'las rampas asumen que la base respeta acc 1.0 / 1.5 (coherente con el ratio 0.28 en los primeros 0.15 s, no verificado en detalle).')
    A('- **Ruido de localización sintético** Gauss-Markov con σ elegidos (0.03 m, 0.05 rad, τ = 1 s). No es el error real de AMCL en la arena. '
      'Una sola semilla por (path, escenario).')
    A('- **Percepción perfecta**: el local costmap usa el mapa estático sin ruido de LiDAR, oclusiones ni rango. La config real solo tiene `obstacle_layer` '
      '(sin LiDAR, sin protección; R4-A).')
    A('- **Sin BT**: plan fijo (sin replanificación a 1 Hz), sin recuperaciones (spin / backup / clear costmap) tras un ABORT. '
      'El resultado ABORT es el del controller_server, no el de la navegación completa.')
    A('- **Lógica de controller_server replicada**, no ejecutada: goal/progress checker y `failure_tolerance` reimplementados en tiempo simulado según '
      'la semántica de Humble 1.1.20. El costmap se actualiza en cada ciclo de control (real: 5 Hz asíncrono).')
    A('- Mapa sintético único; contacto evaluado con celdas de 0.03 m (±~0.0075 m).\n')
    A('## Archivos\n')
    A('- Simulador: `src/limo_nav_offline_lab/src/dwb_closed_loop_sim.cpp`; configs `config/offline_lab/dwb_offline_R4B_{base,fp}.yaml` (+ manifest), '
      'launch `dwb_offline.launch.py mode:=closedloop`')
    A('- Scripts: `scripts/nav_offline_lab/{make_dwb_r4b_config.py, make_r4b_runs.py, run_r4b_all.sh, evaluate_dwb_r4b.py, make_dwb_r4b_report.py}`')
    A('- Resultados: `results/navigation_offline_lab/dwb_r4b/` (sim_*.jsonl crudos con logs de control y pose a 50 Hz, r4b_runs.csv, r4b_summary.json, '
      'auditorías, r4b_overview.png)\n')
    A('## Siguiente paso recomendado\n')
    A(NEXT)
    open(os.path.join(LAB, 'DWB_R4B_CLOSED_LOOP_REPORT.md'), 'w').write('\n'.join(L) + '\n')
    print('ok')


if __name__ == '__main__':
    sys.exit(main())
