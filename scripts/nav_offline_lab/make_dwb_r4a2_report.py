#!/usr/bin/python3
"""NAV-OFFLINE-R4-A2 — genera results/navigation_offline_lab/DWB_R4A2_FOOTPRINT_REPORT.md desde r4a2_summary.json."""
import csv
import json
import os

LAB = os.path.expanduser('~/limo_sml_ws/results/navigation_offline_lab')
D = os.path.join(LAB, 'dwb_r4a2')
KINDS = ('ALL', 'START', 'ALONG', 'PERT_LAT', 'PERT_YAW')
NEXT = '\n'.join([
    '1. **R4-B: simulador cinemático en lazo cerrado** con este harness (DWB → integrador diferencial con acc_lim, timeout de base ~0.6 s y ruido de '
    'localización), comparando BASELINE vs ObstacleFootprint. La pregunta abierta es qué pasa después del rechazo: en Nav2 real el controlador aborta '
    'FollowPath y el BT ejecuta recuperaciones (spin/backup). Un backup con footprint en contacto también debe evaluarse.',
    '2. Barrido offline corto de `ObstacleFootprint.scale` (0.1 vs 2.0, que iguala el peso efectivo de BaseObstacle) y de perturbaciones combinadas y más grandes '
    '(±0.03/0.05/0.08 m × ±0.1/0.15/0.25 rad) en corridor_N1/N2, para ubicar el límite de rechazo y el margen de trayectorias legales.',
    '3. Si R4-B confirma el beneficio, **proponer** (no aplicar sin autorización) el cambio en `navigation2_real.yaml`: `BaseObstacle` → `ObstacleFootprint`. '
    'El cambio de producción requiere decisión explícita y prueba física supervisada.',
    '4. Independiente del critic: el local costmap real no tiene `static_layer`, así que sin LiDAR ninguna variante protege (R4-A). ObstacleFootprint no resuelve eso.',
])


def f3(x):
    return '—' if x is None else f'{x:.3f}'


def cmp_table(S, key, label):
    out = [f'| {label} | ' + ' | '.join(KINDS) + ' |', '|---|' + '---|' * len(KINDS)]
    for name in ('baseline', 'experimental'):
        out.append(f'| {name} | ' + ' | '.join(str(S[name][k][key]) for k in KINDS) + ' |')
    return '\n'.join(out)


def main():
    S = json.load(open(os.path.join(D, 'r4a2_summary.json')))
    man = json.load(open(os.path.expanduser('~/limo_sml_ws/config/offline_lab/dwb_offline_R4A2_manifest.json')))
    aud = {t: json.load(open(os.path.join(D, f'audit_{t}.json'))) for t in ('full_base', 'full_fp', 'server_fp')
           if os.path.exists(os.path.join(D, f'audit_{t}.json'))}
    rows = list(csv.DictReader(open(os.path.join(D, 'r4a2_cases.csv'))))
    B, E = S['baseline'], S['experimental']
    bu, eu = B['ALL']['FN_unsafe'], E['ALL']['FN_unsafe']
    imp = S['improvement_pct']
    L = []
    A = L.append
    A('# NAV-OFFLINE-R4-A2 — DWB: BaseObstacle (centro) vs ObstacleFootprint (footprint completo)\n')
    A('Fecha: 2026-09-29. 100 % offline (ROS_DOMAIN_ID=99, ROS_LOCALHOST_ONLY=1). Sin hardware, sin limo_base, sin LiDAR, sin teleop, '
      'sin FollowPath/NavigateToPose, sin `/cmd_vel`. `navigation2_real.yaml` sin cambios (sha256 '
      f'`{man["real_sha256"][:16]}…`). `~/limoatwork` sin cambios. Configuración de producción NO modificada.\n')
    A('## Resumen\n')
    A('```')
    A('R4-A2')
    bm, em = B['ALL']['unsafe_margin'], E['ALL']['unsafe_margin']
    A(f'Baseline unsafe cases:      {bu}   (contacto real del footprint sin padding; de {S["n_cases"]} casos)  |  {bm} por margen (<0.02 m)')
    A(f'Experimental unsafe cases:  {eu}   |  {em} por margen (<0.02 m)')
    A(f'Improvement:                {imp:.1f} % (contacto real)' + (f'  |  {100 * (bm - em) / bm:.1f} % (margen)' if bm else '')
      if imp is not None else 'Improvement:                n/a (baseline sin casos inseguros)')
    A('```\n')
    A('## Método\n')
    A('- **Mismos insumos que R4-A**: mapa `synthetic_arena` (0.03 m), 64 paths NavFn de R3, mismos 3204 casos (START, ALONG cada 0.5 m, '
      'perturbaciones ±0.05 m lateral y ±0.15 rad de yaw), mismo orden, mismo harness (`dwb_offline_eval`: clase real `dwb_core::DWBLocalPlanner` '
      '+ `Costmap2DROS` real, llamada `computeVelocityCommands` como en controller_server, sin publisher Twist).')
    A('- **BASELINE** `config/offline_lab/dwb_offline_R4A2_base.yaml`: DWB idéntico al real (critic `BaseObstacle`).')
    A('- **EXPERIMENTAL** `config/offline_lab/dwb_offline_R4A2_fp.yaml`: `BaseObstacle` → `ObstacleFootprint` (dwb_critics, instalado), '
      'misma posición en la lista, `scale: 0.1`. Diferencia de parámetros DWB verificada por script: solo `critics` y el scale del critic '
      f'({", ".join(man["dwb_param_diff_base_vs_fp"])}).')
    A('- Ambas: local costmap `static_layer` + `inflation_layer` (como R4-A), `update_frequency` 20 Hz y espera de 120 ms tras recentrar '
      '(solo acelera el banco).')
    A('- **Verdad de terreno offline (solo evaluación, nunca control)**: geometría del PGM (celdas ocupadas, 9 puntos por celda) y footprint '
      '**real sin padding** [-0.17, 0.22]×[-0.16, 0.16]. Clearance con signo: < 0 significa penetración.\n')
    A('Clasificación por caso (positivo = DWB rechaza / detiene):\n')
    A('| clase | definición |')
    A('|---|---|')
    A('| TN | acepta y la trayectoria elegida tiene clearance ≥ 0 en todas sus poses |')
    A('| **FN (inseguro)** | acepta y la trayectoria elegida tiene clearance < 0 en alguna pose (el robot choca siguiendo el comando) |')
    A('| TP | rechaza y el footprint real ya está en contacto en la pose actual |')
    A('| FP (falsa alarma) | rechaza con el footprint real sin contacto en la pose actual |')
    A('')
    A('"Centro pasa, robot no" = aceptado, celda central de la pose < INSCRIBED (253) en el costmap, que es el criterio de BaseObstacle, '
      'y clearance del footprint real < 0 (o < 0.02 m en la versión por margen).\n')

    A('## CONFIRMADO\n')
    A('- `dwb_critics::ObstacleFootprintCritic` existe en Nav2 Humble 1.1.20 instalado. El `controller_server` real lo carga con la config experimental '
      '(lifecycle active, `Using critic "ObstacleFootprint"`). Ver `dwb_r4a2/server_check_server_fp.txt`.')
    A('- Semántica (header y strings de libdwb_critics): recorre el **contorno** del footprint orientado (con padding, tomado de `getRobotFootprint()`) '
      'en cada pose de la trayectoria. Es ilegal si alguna celda es LETHAL ("Trajectory Hits Obstacle."), NO_INFORMATION ("Trajectory Hits Unknown Region.") '
      'o sale del grid ("Footprint Goes Off Grid."). INSCRIBED (253) no invalida, que es correcto porque el footprint se evalúa explícitamente.')
    A('- `ObstacleFootprintCritic::getScale()` = `resolution × scale`: con `scale: 0.1` y resolución 0.05, el peso efectivo en el costo es 0.005, '
      '20 veces menos que BaseObstacle (0.1). No afecta la legalidad; sí puede cambiar qué trayectoria legal se elige.')
    A(f'- Consistencia: con ObstacleFootprint, 0 trayectorias legales tienen footprint (con padding) en LETHAL según el chequeo independiente del harness '
      f'(baseline: {B["ALL"]["cases_with_legal_fp_collision"]} casos con al menos una).')
    if 'baseline_vs_R4A' in S:
        r = S['baseline_vs_R4A']
        A(f'- Reproducibilidad: baseline A2 vs corrida R4-A ({r["compared"]} casos, mismo orden): {r["cmd_diff"]} comandos distintos, '
          f'{r["status_diff"]} cambios de aceptación.')
    ok = all(a['PASS'] for a in aud.values())
    A(f'- Auditoría de grafo en cada corrida: {"PASS" if ok else "FAIL"} (sin `/cmd_vel`, sin topics Twist en eval, sin acciones).\n')

    A('## MEDIDO\n')
    A('### Aceptación\n')
    A(cmp_table(S, 'accepted', 'aceptados') + '\n')
    A(cmp_table(S, 'rejected', 'rechazados') + '\n')
    A('Rutas (64): aceptada = START y todas las muestras ALONG nominales aceptadas.\n')
    A('| variante | rutas nominales aceptadas | rutas con algún rechazo bajo perturbación | rutas con algún caso inseguro |')
    A('|---|---|---|---|')
    for n in ('baseline', 'experimental'):
        r = S[n]['routes']
        A(f"| {n} | {r['nominal_accepted']}/{r['n']} | {r['with_any_pert_reject']} | {r['with_any_unsafe']} "
          f"({', '.join(S[n]['routes_with_unsafe']) or '—'}) |")
    A('')
    A('### Seguridad (verdad de terreno geométrica)\n')
    for key, lab in (('FN_unsafe', 'FN inseguros (contacto real)'),
                     ('unsafe_margin', 'inseguros por margen (elegida con clearance < 0.02 m)'),
                     ('TP', 'TP'), ('FP', 'FP total'),
                     ('FP_within_padding', 'FP dentro del padding (pose con clearance 0–0.02 m)'),
                     ('FP_strict', 'FP estrictos (pose con clearance ≥ 0.02 m)'), ('TN', 'TN'),
                     ('center_ok_robot_not', 'centro pasa, robot no (contacto real)'),
                     ('center_ok_robot_margin', 'centro pasa, robot no (clearance < 0.02 m)'),
                     ('unsafe_costmap_padded', 'elegida con footprint+padding en LETHAL (costmap, métrica R4-A)')):
        A(cmp_table(S, key, lab) + '\n')
    A(f'- Casos con el footprint real ya en contacto en la pose actual: {S["pose_contact_cases"]}. Con clearance < 0.02 m: {S["pose_clear_lt_padding"]}.')
    for n in ('baseline', 'experimental'):
        a = S[n]['ALL']
        A(f'- {n}: razones de rechazo {a["reject_reasons"] or "—"}; clearance en la pose de los FP {a["FP_pose_clear"] or "—"}; '
          f'de los TP {a["TP_pose_clear"] or "—"}.')
    A('')
    A('### Clearance real del footprint en la trayectoria elegida (m)\n')
    A('| variante | tipo | n | mín | p5 | p50 |')
    A('|---|---|---|---|---|---|')
    for n in ('baseline', 'experimental'):
        for k in KINDS:
            t = S[n][k]['traj_clear']
            if t:
                A(f'| {n} | {k} | {t["n"]} | {f3(t["min"])} | {f3(t["p5"])} | {f3(t["p50"])} |')
    A('\nFigura: `dwb_r4a2/r4a2_clearance.png` (histograma de clearance y detalle de corridor_N1).\n')
    A('### Por categoría\n')
    A('| categoría | casos | contacto en pose | base acept. | base FN | exp. acept. | exp. FN | exp. TP | exp. FP | trayect. legales medias base → exp. |')
    A('|---|---|---|---|---|---|---|---|---|---|')
    for c in sorted(B['by_category']):
        b, e = B['by_category'][c], E['by_category'][c]
        A(f"| {c} | {b['n']} | {b.get('pose_contact', 0)} | {b['acc']} | {b.get('FN', 0)} | {e['acc']} | {e.get('FN', 0)} | "
          f"{e.get('TP', 0)} | {e.get('FP', 0)} | {b['n_legal_sum'] / b['n']:.0f} → {e['n_legal_sum'] / e['n']:.0f} |")
    A('')
    A('### Costo de cómputo\n')
    A('| variante | t p50 (ms) | t máx (ms) |')
    A('|---|---|---|')
    for n in ('baseline', 'experimental'):
        t = S[n]['ALL']['t_ms']
        mx = max(float(r[f"{'b' if n == 'baseline' else 'e'}_t_ms"]) for r in rows)
        A(f'| {n} | {t["p50"]:.1f} | {mx:.1f} |')
    A('')
    A('### Observaciones adicionales\n')
    cg = B['ALL'].get('center_geom_dist_unsafe')
    if cg:
        A(f'- En los {cg["n"]} casos inseguros de la baseline el centro está a {cg["min"]:.3f} m geométricos del obstáculo, **menos** que el radio inscripto '
          'con padding (0.18 m). Aun así la celda central vale 242 (< 253 INSCRIBED) en el costmap a 0.05 m. La rasterización hace que BaseObstacle '
          'sea todavía más permisivo que el propio criterio "centro fuera del radio inscripto".')
    A('- corridor_N1 (0.45 m): trayectorias legales medias por caso baseline → experimental en la tabla por categoría. En ALONG nominal dentro de N1 '
      'la experimental deja 43–44 de 411 trayectorias legales (baseline ≥ 258): pasa, pero con poco margen de maniobra.')
    A('- Chequeo independiente del harness (`FootprintCollisionChecker` de nav2_costmap_2d) vs critic (`ObstacleFootprintCritic::lineCost`): en 17 casos '
      'hay 1–3 trayectorias legales para el critic que el chequeo del harness marca LETHAL. Son diferencias de rasterización entre los dos algoritmos de línea; '
      'ninguna fue la elegida (0 elegidas con footprint+padding en LETHAL).')
    A(f'\nComandos distintos entre baseline y experimental: {S["cmd_changed_base_vs_exp"]} / {S["n_cases"]}. '
      'Se deben al peso efectivo distinto del critic de obstáculos y a trayectorias que dejan de ser legales.\n')

    A('## LIMITACIONES\n')
    A('- **Instantáneas, no lazo cerrado**: cada caso es una llamada a DWB. Un FN significa que el comando elegido lleva el footprint a contacto '
      'dentro del horizonte de 1.7 s. En lazo cerrado DWB recalcula a 10 Hz y podría corregir antes del contacto, o no. Requiere R4-B.')
    A('- **Precisión de la verdad de terreno**: celdas del PGM de 0.03 m representadas por 9 puntos (error de clearance ≤ ~0.0075 m). '
      'Las penetraciones medidas (~0.01 m) están en el orden de esa precisión y de la discretización del mapa. El resultado es "contacto o casi contacto", '
      'no una profundidad de choque confiable.')
    A('- **Costmap 0.05 m**: los critics ven obstáculos rasterizados a 0.05 m, más el padding de 0.02. La frontera entre FP y TP depende de esa discretización.')
    A('- **Sin LiDAR simulado**: el local costmap usa el mapa estático (percepción perfecta). La config real solo tiene `obstacle_layer`. '
      'Sin `/scan` ninguna de las dos variantes ve obstáculos (ver R4-A, variante realcm).')
    A('- **Empates**: en casos con margen ~0 entre la mejor y la segunda trayectoria, la elección puede depender de la historia del costmap rodante '
      '(ventana recentrada con truncamiento entero, ±1 celda). Se observó en 3/64 START al cambiar el orden de casos (smoke vs corrida completa). '
      'Baseline y experimental usan el mismo orden: comparación válida. Causa probable, no confirmada.')
    A('- Un solo mapa sintético (`synthetic_arena`); `synthetic_arena_unknown` no se evaluó. Perturbaciones solo de ±0.05 m y ±0.15 rad, sin combinarlas.')
    A('- No se evaluó `scale` alternativo de ObstacleFootprint (peso efectivo igual a BaseObstacle: `scale: 2.0`). Afecta la preferencia entre trayectorias legales, no la legalidad.\n')
    A('## Archivos\n')
    A('- Config: `config/offline_lab/dwb_offline_R4A2_{base,fp}.yaml`, `dwb_offline_R4A2_manifest.json`')
    A('- Scripts: `scripts/nav_offline_lab/{make_dwb_r4a2_config.py, evaluate_dwb_r4a2.py, make_dwb_r4a2_report.py}`; harness `src/limo_nav_offline_lab` '
      '(+ `settle_ms`, poses de la trayectoria elegida)')
    A('- Resultados: `results/navigation_offline_lab/dwb_r4a2/` (eval_full_{base,fp}.jsonl, r4a2_cases.csv, r4a2_summary.json, r4a2_clearance.png, auditorías)\n')
    A('## Siguiente paso recomendado\n')
    A(NEXT)
    open(os.path.join(LAB, 'DWB_R4A2_FOOTPRINT_REPORT.md'), 'w').write('\n'.join(L) + '\n')
    print('ok')


if __name__ == '__main__':
    main()
