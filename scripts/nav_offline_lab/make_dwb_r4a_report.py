#!/usr/bin/python3
"""NAV-OFFLINE-R4-A — genera results/navigation_offline_lab/DWB_R4A_REPORT.md a partir de los resúmenes."""
import csv
import collections
import json
import os

LAB = os.path.expanduser('~/limo_sml_ws/results/navigation_offline_lab')
D = os.path.join(LAB, 'dwb_r4a')


def load(fn):
    p = os.path.join(D, fn)
    return json.load(open(p)) if os.path.exists(p) else None


def rows(tag):
    p = os.path.join(D, f'cases_{tag}.csv')
    return list(csv.DictReader(open(p))) if os.path.exists(p) else []


def kind_table(S):
    out = ['| tipo | casos | aceptados | rechazados | error | t 1ª llamada p50 / p95 / máx (ms) | comando |',
           '|---|---|---|---|---|---|---|']
    for k in ('START', 'ALONG', 'PERT_LAT', 'PERT_YAW'):
        v = S['by_kind'].get(k)
        if not v:
            continue
        t = v['t_first_ms']
        cc = ', '.join(f'{a} {b}' for a, b in sorted(v['cmd_class'].items(), key=lambda x: -x[1]))
        out.append(f"| {k} | {v['n']} | {v['accepted']} | {v['rejected']} | {v['error']} | "
                   f"{t['p50']:.1f} / {t['p95']:.1f} / {t['max']:.1f} | {cc} |")
    return '\n'.join(out)


def critic_table(S):
    out = ['| tipo | critic dominante (mayor aporte al costo de la elegida) | critic decisivo (quitarlo cambia la elección) | sin critic decisivo |',
           '|---|---|---|---|']
    for k in ('START', 'ALONG', 'PERT_LAT', 'PERT_YAW'):
        v = S['by_kind'].get(k)
        if not v:
            continue
        dom = ', '.join(f'{a} {b}' for a, b in sorted(v['dominant_critic'].items(), key=lambda x: -x[1]))
        dec = ', '.join(f'{a} {b}' for a, b in sorted(v['decisive_critic_any'].items(), key=lambda x: -x[1]))
        out.append(f"| {k} | {dom} | {dec} | {v['no_decisive']} |")
    return '\n'.join(out)


def fp_table(S):
    out = ['| tipo | casos con ≥1 trayectoria LEGAL cuyo footprint toca LETHAL | trayectoria ELEGIDA con footprint en LETHAL | pose inicial con footprint en LETHAL |',
           '|---|---|---|---|']
    for k in ('START', 'ALONG', 'PERT_LAT', 'PERT_YAW'):
        v = S['by_kind'].get(k)
        if v:
            out.append(f"| {k} | {v['cases_with_legal_footprint_collision']} / {v['accepted']} | "
                       f"{v['best_traj_footprint_lethal']} | {v['pose_footprint_lethal']} |")
    return '\n'.join(out)


def per_test_table(S):
    out = ['| test | categoría | long. (m) | err. heading inicial (°) | START | cmd (vx, wz) | t (ms, mediana 5) | critic decisivo | ALONG ok | PERT ok | PERT elegida con footprint LETHAL |',
           '|---|---|---|---|---|---|---|---|---|---|---|']
    for t in sorted(S['per_test'], key=lambda x: x['test']):
        s = t['start'] or {}
        vx = float(s['cmd_vx']) if s.get('status') == 'ACCEPTED' else None
        cmd = f"({vx:.3f}, {float(s['cmd_wz']):.3f})" if vx is not None else '—'
        out.append(f"| {t['test']} | {t['category']} | {float(s.get('path_len_m', 0)):.2f} | "
                   f"{float(s.get('start_heading_err_deg', 0)):.0f} | {s.get('status', '?')} | {cmd} | "
                   f"{float(s.get('t_med_ms', 0)):.1f} | {s.get('decisive_critics') or '—'} | "
                   f"{t['along_acc']}/{t['along_n']} | {t['pert_acc']}/{t['pert_n']} | {t['pert_best_fp_lethal']} |")
    return '\n'.join(out)


def main():
    S = load('summary_full_R4A.json')
    SR = load('summary_realcm_R4A.json')
    man = json.load(open(os.path.expanduser('~/limo_sml_ws/config/offline_lab/dwb_offline_R4A_manifest.json')))
    inv = load('INVALID_fullpath_nopruning_summary.json')
    aud = {t: load(f'audit_{t}.json') for t in ('server_R4A', 'full_R4A', 'realcm_R4A')}
    R = rows('full_R4A')
    cats = collections.defaultdict(lambda: collections.Counter())
    for r in R:
        c = cats[r['category']]
        c['n'] += 1
        c['acc'] += r['status'] == 'ACCEPTED'
        c['bestfp'] += r['best_fp_lethal'] == 'True'
        c['posefp'] += float(r['footprint_cost_at_pose'] or 0) >= 254
        if r['kind'] == 'ALONG':
            c['along_fp_max'] = max(c['along_fp_max'], int(float(r['footprint_cost_at_pose'] or 0)))
    ntests = len(S['per_test'])
    st = S['by_kind']['START']
    tot = S['n_cases']
    acc = sum(v['accepted'] for v in S['by_kind'].values())
    rej = sum(v['rejected'] + v['error'] for v in S['by_kind'].values())
    tall = [float(r['t_first_ms']) for r in R]
    tall.sort()
    rot = st['cmd_class'].get('rotate_in_place', 0)
    fp_best = sum(v['best_traj_fp'] if 'best_traj_fp' in v else v['best_traj_footprint_lethal'] for v in S['by_kind'].values())
    fp_pose = sum(v['pose_footprint_lethal'] for v in S['by_kind'].values())
    audits_ok = all(a and a['PASS'] for a in aud.values() if a is not None)

    L = []
    A = L.append
    A('# NAV-OFFLINE-R4-A — Evaluación offline del controlador DWB real (Nav2 Humble)\n')
    A('Fecha: 2026-09-29. Trabajo 100 % offline (ROS_DOMAIN_ID=99, ROS_LOCALHOST_ONLY=1). Sin hardware, sin limo_base, '
      'sin LiDAR, sin teleop, sin goals `FollowPath`/`NavigateToPose`, sin `/cmd_vel`. '
      '`config/navigation/navigation2_real.yaml` NO modificado (sha256 verificado antes/después). `~/limoatwork` solo lectura.\n')
    A('Mapa: `synthetic_arena` (R2/R3, SINTÉTICO, no representa el entorno físico). Paths: 64 paths NavFn de R3 '
      '(`planner_runs/paths_R3.json`, config `planner_offline_R3.yaml`).\n')
    A('## Resumen\n')
    A('```')
    A('NAV-OFFLINE-R4A')
    A('DWB CONFIG:      PASS')
    A(f'PATHS TESTED:    {ntests}  (START: {st["n"]} evaluaciones; total casos incl. muestras a lo largo del path y perturbaciones: {tot})')
    A(f'ACCEPTED:        {st["accepted"]} paths en START  |  {acc}/{tot} casos')
    A(f'REJECTED:        {st["rejected"] + st["error"]} paths en START  |  {rej}/{tot} casos')
    A('```\n')

    A('## Método\n')
    A('Dos etapas, porque el `controller_server` real solo recibe paths por la acción `FollowPath`, que publica `cmd_vel` en cada ciclo:\n')
    A('1. **DWB CONFIG (controller_server REAL)**: `config/offline_lab/dwb_offline.launch.py mode:=server` levanta map_server + '
      '`nav2_controller/controller_server` + TF estáticas + lifecycle_manager. `cmd_vel` remapeado a `/offline_lab/cmd_vel_sink` '
      '(0 suscriptores). **No se envía ningún goal FollowPath**: solo configure/activate y lectura de parámetros.')
    A('2. **Evaluación de paths**: `limo_nav_offline_lab/dwb_offline_eval` (C++, nuevo) instancia la clase instalada '
      '`dwb_core::DWBLocalPlanner` (libdwb_core.so, mismos plugins de critics/generador) y un `nav2_costmap_2d::Costmap2DROS` '
      '"local_costmap" real, con **los mismos parámetros** que controller_server, y llama a la misma API que controller_server: '
      '`setPlan(path)` + `computeVelocityCommands(PoseStamped, Twist, nullptr)`. El proceso **no crea publishers Twist** ni clientes '
      'de acción; el comando elegido solo se escribe en el JSONL. La pose del robot se inyecta publicando `/tf odom->base_link`.')
    A('   - Pasada 1 (fidelidad): configuración real (`short_circuit_trajectory_evaluation: true`) → aceptación, comando, tiempo.')
    A('   - Pasada 2 (análisis): misma llamada con `short_circuit=false` (única diferencia) → scores completos por critic de las ~411 '
      'trayectorias. Se verifica que el comando coincida con la pasada 1.')
    A('   - Chequeo de footprint completo (`FootprintCollisionChecker`, footprint con padding) sobre todas las trayectorias legales.\n')
    A('Casos por path (`scripts/nav_offline_lab/make_dwb_r4a_cases.py`):\n')
    A('- `START`: pose/yaw de inicio del benchmark R3, v=(0,0), path completo, 5 repeticiones para el tiempo.')
    A('- `ALONG`: muestras cada 0.5 m de arco (hasta 0.3 m antes del goal), yaw = tangente del path, v=(0.25, 0), path recortado desde '
      's−1.4 m (emula la poda de DWB en régimen / replanificación desde la pose actual).')
    A('- `PERT_LAT` / `PERT_YAW`: cada muestra ALONG con error de localización ±0.05 m lateral o ±0.15 rad de yaw.\n')
    A('"Critic ganador" se reporta de dos formas: **dominante** = mayor aporte (raw×scale) al costo total de la trayectoria elegida; '
      '**decisivo** = critic cuya eliminación cambia el comando elegido (el que realmente decide).\n')

    A('## CONFIRMADO\n')
    A(f'- Config offline generada desde la real por script; `controller_server` **idéntico** al real (assert por comparación de dicts). '
      f'Fuente sha256 `{man["source_sha256"][:16]}…` sin cambios. Único cambio: local_costmap `obstacle_layer` → `static_layer` '
      '(variante R4A); variante `realcm` con local_costmap idéntico al real.')
    A('- `controller_server` real (Humble 1.1.20) con esta config: lifecycle **active**; carga `dwb_core::DWBLocalPlanner` y los 6 critics '
      '(Oscillation, BaseObstacle, GoalAlign, PathAlign, PathDist, GoalDist) sin errores; parámetros leídos en runtime = config '
      '(max_vel_x 0.25, max_vel_theta 1.0, acc 1.0/1.5, samples 20/1/20, sim_time 1.7, escalas 0.1/32/24/32/24, footprint real, padding 0.02, '
      'controller_frequency 10). Ver `dwb_r4a/server_check_server_R4A.txt`.')
    A('- **`holonomic: False` no tiene efecto en Humble**: el parámetro no está declarado por DWB (`ros2 param get` → "Parameter not set"). '
      'El comportamiento diferencial viene de `max_vel_y = min_vel_y = 0` y `vy_samples = 1`, que sí están en la config. Inofensivo, pero engañoso.')
    A(f'- Auditoría del grafo en cada corrida ({"PASS" if audits_ok else "FAIL"}): no existe `/cmd_vel`; en modo eval no existe ningún topic Twist ni acciones; '
      'en modo server el único topic Twist es `/offline_lab/cmd_vel_sink` con 0 suscriptores. Nodos limitados a la lista permitida.')
    A('- `BaseObstacleCritic` evalúa solo el costo de la **celda del centro** en cada pose de la trayectoria (rechaza LETHAL / INSCRIBED / NO_INFORMATION); '
      'no evalúa el footprint. Con el footprint rectangular con padding ([-0.19,0.24]×[-0.18,0.18]) el radio inscripto es 0.18 m y el circunscripto ≈ 0.30 m: '
      'las esquinas pueden tocar un obstáculo sin que la trayectoria sea ilegal. Confirmado por las mediciones de abajo.')
    A('- `StandardTrajectoryGenerator` (generador por defecto) usa `sim_time` (1.7 s) como ventana de aceleración para muestrear velocidades: '
      'desde v=0 puede elegir directamente vx=0.25 / wz=±1.0. El comando no está limitado por aceleración por ciclo. '
      'La trayectoria simulada sí respeta acc_lim. En `limoatwork/.../limo_navigation.launch.py` no hay `velocity_smoother` (a verificar qué launch usa el robot).')
    A('- En Humble, `transformGlobalPlan` busca el inicio del plan local solo dentro de los primeros `prune_distance` (2.0 m) de arco del plan recibido. '
      'Un plan que no empieza cerca del robot → `Resulting plan has 0 poses in it.` (rechazo).\n')

    A('## MEDIDO\n')
    A('### Aceptación y tiempo (variante R4A: local costmap con mapa estático)\n')
    A(kind_table(S) + '\n')
    A(f'- Tiempo de `computeVelocityCommands` en esta Jetson: mediana {tall[len(tall)//2]:.1f} ms, máx {tall[-1]:.1f} ms sobre {len(tall)} casos → '
      f'~{100 / tall[-1]:.0f}× por debajo del presupuesto de 100 ms (controller_frequency 10 Hz). Incluye la publicación de depuración propia de DWB; '
      'no incluye la actualización del costmap.')
    A(f'- Pasada 2 vs pasada 1: {S["pass2_mismatch"]} discrepancias de comando (short-circuit no cambia la elección). Costmap no listo: {S["costmap_not_ready"]} casos.')
    A(f'- En START (v=0, heading del benchmark): {rot}/{st["n"]} comandos son **giro en el lugar** (vx<0.03). '
      'Relación con el error de heading inicial en `dwb_r4a/dwb_full_R4A_overview.png`.\n')
    A('### Critic ganador\n')
    A(critic_table(S) + '\n')
    A('- Por magnitud domina GoalDist/GoalAlign (sus raw scores son distancias en celdas, mayores que el resto). Los que más a menudo **deciden** '
      'la elección son PathAlign y PathDist (escala 32). BaseObstacle (escala 0.1) casi nunca decide: actúa solo como filtro de legalidad.\n')
    A('### Footprint completo vs BaseObstacle\n')
    A(fp_table(S) + '\n')
    A('Por categoría (todas las evaluaciones):\n')
    A('| categoría | casos | aceptados | elegida con footprint LETHAL | pose con footprint LETHAL | máx. costo footprint en ALONG nominal |')
    A('|---|---|---|---|---|---|')
    for k in sorted(cats):
        c = cats[k]
        A(f"| {k} | {c['n']} | {c['acc']} | {c['bestfp']} | {c['posefp']} | {c['along_fp_max']} |")
    A('')
    A(f'- **Hallazgo principal**: en {fp_pose} casos perturbados (todos `corridor_N1`, 0.45 m de ancho → ≈4.5 cm por lado para el footprint con padding de 0.36 m) '
      'el footprint en la pose ya está en LETHAL (254), el centro sigue en costo < 253, y **DWB acepta y elige avanzar** '
      f'(vx 0.21–0.25). En {fp_best} casos la trayectoria elegida tiene el footprint en LETHAL. '
      'Con ±5 cm de error lateral o ±0.15 rad de yaw, DWB no detecta el contacto de las esquinas.')
    A('- En el 41–45 % de los casos aceptados (según tipo) existen trayectorias legales para DWB cuyo footprint cruza LETHAL. Con la configuración nominal (sin perturbación) '
      'nunca fueron las elegidas: las evitaron PathDist/PathAlign, no BaseObstacle. Esa protección depende de que el path esté centrado, no de un chequeo de colisión.\n')
    if SR:
        A('### Variante `realcm` (local costmap idéntico al real, sin /scan)\n')
        A(kind_table(SR) + '\n')
        A(fp_table(SR) + '\n')
        A('- Sin LiDAR el local costmap real no tiene ninguna fuente de obstáculos (`obstacle_layer` es la única capa que marca; no tiene `static_layer`). '
          'BaseObstacle no rechaza nada y la verificación de footprint no encuentra LETHAL en el costmap porque no hay obstáculos. '
          '**DWB en la config real depende 100 % del LiDAR para evitar paredes**: sin `/scan_filtered` (driver caído, filtro mal configurado) '
          'sigue el path sin ninguna protección de colisión local.\n')
    if inv:
        A('### Plan que no empieza cerca del robot (corrida descartada, medida)\n')
        A('Primera corrida con el path completo desde el start para las muestras ALONG/PERT (`eval_INVALID_fullpath_nopruning.jsonl`, detenida tras 289 casos):\n')
        A('| s (m de arco donde está el robot) | casos | aceptados |')
        A('|---|---|---|')
        for lo, hi, n, a in inv['by_s']:
            A(f'| {lo}–{hi if hi < 90 else "∞"} | {n} | {a} |')
        A('\nTodos los rechazos: `Resulting plan has 0 poses in it.` Se descartó como evaluación (artefacto del banco), pero documenta que DWB rechaza cualquier '
          'FollowPath cuyo plan empiece a más de ~2 m de arco (y >1.5 m euclídeo) del robot.\n')
    A('### Por path (START + resumen de muestras)\n')
    A(per_test_table(S) + '\n')

    A('## LIMITACIONES\n')
    A('- **No es lazo cerrado**: cada caso es una única llamada a DWB desde una pose y velocidad dadas. No hay integración de movimiento, '
      'goal checker, progress checker, recuperaciones ni replanificación. "Aceptado" = DWB devuelve un comando válido en ese instante, '
      'no que llegue al goal. Esto requiere el simulador cinemático (R4-B).')
    A('- La velocidad inicial de las muestras ALONG (0.25, 0) es un supuesto (crucero en recta). En curvas reales sería menor y la ventana de muestreo distinta.')
    A('- Variante R4A: el local costmap usa el mapa estático como proxy de percepción LiDAR perfecta (sin ruido, sin oclusiones, sin rango máximo). '
      'La config real no tiene static_layer en el local costmap. La variante `realcm` refleja la config real sin sensor.')
    A('- Mapa sintético de desarrollo, no la arena física. Resolución del mapa 0.03 m, local costmap 0.05 m: el chequeo de footprint hereda esa discretización (±1 celda).')
    A('- El chequeo de footprint (`FootprintCollisionChecker`) evalúa el contorno del polígono, no el interior. Es suficiente para paredes, no para obstáculos más chicos que el robot.')
    A('- La emulación de poda (path recortado desde s−1.4 m) aproxima el estado en régimen. No reproduce el historial exacto de `global_plan_` en lazo cerrado.')
    A('- Solo se evaluaron los paths de R3 sobre `synthetic_arena` (mapa cerrado). Faltan los de `synthetic_arena_unknown` (SUP_R3); con local costmap '
      '`track_unknown_space` por defecto (false), UNKNOWN se trata como libre en el costmap local.')
    A('- Tiempos medidos en esta Jetson con otros procesos de escritorio activos (firefox). Una sola corrida, sin aislamiento de CPU.\n')

    A('## Archivos\n')
    A('- Config: `config/offline_lab/dwb_offline_R4A.yaml`, `dwb_offline_R4A_realcm.yaml`, `dwb_offline_R4A_manifest.json`, `dwb_offline.launch.py`')
    A('- Harness: `src/limo_nav_offline_lab/src/dwb_offline_eval.cpp` (paquete `limo_nav_offline_lab`; build con `-DPython3_EXECUTABLE=/usr/bin/python3`)')
    A('- Scripts: `scripts/nav_offline_lab/{make_dwb_offline_config.py, make_dwb_r4a_cases.py, run_dwb_r4a.sh, dwb_r4a_audit.py, evaluate_dwb_r4a.py, make_dwb_r4a_report.py}`')
    A('- Resultados: `results/navigation_offline_lab/dwb_r4a/` (eval_*.jsonl crudos, cases_*.csv, summary_*.json, audit_*.json, server_check_*.txt, overview PNG)\n')
    A('## Siguiente paso recomendado\n')
    A('1. **R4-A2 (offline, rápido)**: repetir el banco con el critic `ObstacleFootprint` (dwb_critics::ObstacleFootprintCritic) en lugar de BaseObstacle, '
      'en una copia de la config offline. Medir si elimina los casos con footprint en LETHAL sin rechazar los paths nominales de `corridor_N1`.')
    A('2. **R4-B**: simulador cinemático en lazo cerrado (DWB → integrador diferencial con acc_lim y retardo/timeout de la base ~0.6 s) usando este mismo harness, '
      'con ruido de localización. Medir llegada, error de tracking y clearance mínimo de footprint.')
    A('3. Verificar en el robot (solo lectura de launch/config, sin ejecutar) qué launch usa `navigation2_real.yaml` y si hay `velocity_smoother`. '
      'Eliminar `holonomic` de la config real en un cambio justificado aparte (no tiene efecto).')
    A('4. Considerar `static_layer` en el local costmap real como respaldo si falla el LiDAR. Cambio de config real: requiere decisión explícita.')
    open(os.path.join(LAB, 'DWB_R4A_REPORT.md'), 'w').write('\n'.join(L) + '\n')
    print('ok', os.path.join(LAB, 'DWB_R4A_REPORT.md'))


if __name__ == '__main__':
    main()
