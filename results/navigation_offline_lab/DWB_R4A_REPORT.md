# NAV-OFFLINE-R4-A — Evaluación offline del controlador DWB real (Nav2 Humble)

Fecha: 2026-09-29. Trabajo 100 % offline (ROS_DOMAIN_ID=99, ROS_LOCALHOST_ONLY=1). Sin hardware, sin limo_base, sin LiDAR, sin teleop, sin goals `FollowPath`/`NavigateToPose`, sin `/cmd_vel`. `config/navigation/navigation2_real.yaml` NO modificado (sha256 verificado antes/después). `~/limoatwork` solo lectura.

Mapa: `synthetic_arena` (R2/R3, SINTÉTICO, no representa el entorno físico). Paths: 64 paths NavFn de R3 (`planner_runs/paths_R3.json`, config `planner_offline_R3.yaml`).

## Resumen

```
NAV-OFFLINE-R4A
DWB CONFIG:      PASS
PATHS TESTED:    64  (START: 64 evaluaciones; total casos incl. muestras a lo largo del path y perturbaciones: 3204)
ACCEPTED:        64 paths en START  |  3204/3204 casos
REJECTED:        0 paths en START  |  0/3204 casos
```

## Método

Dos etapas, porque el `controller_server` real solo recibe paths por la acción `FollowPath`, que publica `cmd_vel` en cada ciclo:

1. **DWB CONFIG (controller_server REAL)**: `config/offline_lab/dwb_offline.launch.py mode:=server` levanta map_server + `nav2_controller/controller_server` + TF estáticas + lifecycle_manager. `cmd_vel` remapeado a `/offline_lab/cmd_vel_sink` (0 suscriptores). **No se envía ningún goal FollowPath**: solo configure/activate y lectura de parámetros.
2. **Evaluación de paths**: `limo_nav_offline_lab/dwb_offline_eval` (C++, nuevo) instancia la clase instalada `dwb_core::DWBLocalPlanner` (libdwb_core.so, mismos plugins de critics/generador) y un `nav2_costmap_2d::Costmap2DROS` "local_costmap" real, con **los mismos parámetros** que controller_server, y llama a la misma API que controller_server: `setPlan(path)` + `computeVelocityCommands(PoseStamped, Twist, nullptr)`. El proceso **no crea publishers Twist** ni clientes de acción; el comando elegido solo se escribe en el JSONL. La pose del robot se inyecta publicando `/tf odom->base_link`.
   - Pasada 1 (fidelidad): configuración real (`short_circuit_trajectory_evaluation: true`) → aceptación, comando, tiempo.
   - Pasada 2 (análisis): misma llamada con `short_circuit=false` (única diferencia) → scores completos por critic de las ~411 trayectorias. Se verifica que el comando coincida con la pasada 1.
   - Chequeo de footprint completo (`FootprintCollisionChecker`, footprint con padding) sobre todas las trayectorias legales.

Casos por path (`scripts/nav_offline_lab/make_dwb_r4a_cases.py`):

- `START`: pose/yaw de inicio del benchmark R3, v=(0,0), path completo, 5 repeticiones para el tiempo.
- `ALONG`: muestras cada 0.5 m de arco (hasta 0.3 m antes del goal), yaw = tangente del path, v=(0.25, 0), path recortado desde s−1.4 m (emula la poda de DWB en régimen / replanificación desde la pose actual).
- `PERT_LAT` / `PERT_YAW`: cada muestra ALONG con error de localización ±0.05 m lateral o ±0.15 rad de yaw.

"Critic ganador" se reporta de dos formas: **dominante** = mayor aporte (raw×scale) al costo total de la trayectoria elegida; **decisivo** = critic cuya eliminación cambia el comando elegido (el que realmente decide).

## CONFIRMADO

- Config offline generada desde la real por script; `controller_server` **idéntico** al real (assert por comparación de dicts). Fuente sha256 `dfb36e108d6872a5…` sin cambios. Único cambio: local_costmap `obstacle_layer` → `static_layer` (variante R4A); variante `realcm` con local_costmap idéntico al real.
- `controller_server` real (Humble 1.1.20) con esta config: lifecycle **active**; carga `dwb_core::DWBLocalPlanner` y los 6 critics (Oscillation, BaseObstacle, GoalAlign, PathAlign, PathDist, GoalDist) sin errores; parámetros leídos en runtime = config (max_vel_x 0.25, max_vel_theta 1.0, acc 1.0/1.5, samples 20/1/20, sim_time 1.7, escalas 0.1/32/24/32/24, footprint real, padding 0.02, controller_frequency 10). Ver `dwb_r4a/server_check_server_R4A.txt`.
- **`holonomic: False` no tiene efecto en Humble**: el parámetro no está declarado por DWB (`ros2 param get` → "Parameter not set"). El comportamiento diferencial viene de `max_vel_y = min_vel_y = 0` y `vy_samples = 1`, que sí están en la config. Inofensivo, pero engañoso.
- Auditoría del grafo en cada corrida (PASS): no existe `/cmd_vel`; en modo eval no existe ningún topic Twist ni acciones; en modo server el único topic Twist es `/offline_lab/cmd_vel_sink` con 0 suscriptores. Nodos limitados a la lista permitida.
- `BaseObstacleCritic` evalúa solo el costo de la **celda del centro** en cada pose de la trayectoria (rechaza LETHAL / INSCRIBED / NO_INFORMATION); no evalúa el footprint. Con el footprint rectangular con padding ([-0.19,0.24]×[-0.18,0.18]) el radio inscripto es 0.18 m y el circunscripto ≈ 0.30 m: las esquinas pueden tocar un obstáculo sin que la trayectoria sea ilegal. Confirmado por las mediciones de abajo.
- `StandardTrajectoryGenerator` (generador por defecto) usa `sim_time` (1.7 s) como ventana de aceleración para muestrear velocidades: desde v=0 puede elegir directamente vx=0.25 / wz=±1.0. El comando no está limitado por aceleración por ciclo. La trayectoria simulada sí respeta acc_lim. En `limoatwork/.../limo_navigation.launch.py` no hay `velocity_smoother` (a verificar qué launch usa el robot).
- En Humble, `transformGlobalPlan` busca el inicio del plan local solo dentro de los primeros `prune_distance` (2.0 m) de arco del plan recibido. Un plan que no empieza cerca del robot → `Resulting plan has 0 poses in it.` (rechazo).

## MEDIDO

### Aceptación y tiempo (variante R4A: local costmap con mapa estático)

| tipo | casos | aceptados | rechazados | error | t 1ª llamada p50 / p95 / máx (ms) | comando |
|---|---|---|---|---|---|---|
| START | 64 | 64 | 0 | 0 | 9.5 / 10.6 / 11.7 | rotate_in_place 36, forward_turn 22, forward 6 |
| ALONG | 628 | 628 | 0 | 0 | 10.0 / 10.7 / 12.7 | forward 507, forward_turn 116, rotate_in_place 5 |
| PERT_LAT | 1256 | 1256 | 0 | 0 | 9.9 / 10.6 / 16.0 | forward_turn 782, forward 469, rotate_in_place 5 |
| PERT_YAW | 1256 | 1256 | 0 | 0 | 9.9 / 10.6 / 12.7 | forward_turn 1106, forward 140, rotate_in_place 10 |

- Tiempo de `computeVelocityCommands` en esta Jetson: mediana 9.9 ms, máx 16.0 ms sobre 3204 casos → ~6× por debajo del presupuesto de 100 ms (controller_frequency 10 Hz). Incluye la publicación de depuración propia de DWB; no incluye la actualización del costmap.
- Pasada 2 vs pasada 1: 0 discrepancias de comando (short-circuit no cambia la elección). Costmap no listo: 0 casos.
- En START (v=0, heading del benchmark): 36/64 comandos son **giro en el lugar** (vx<0.03). Relación con el error de heading inicial en `dwb_r4a/dwb_full_R4A_overview.png`.

### Critic ganador

| tipo | critic dominante (mayor aporte al costo de la elegida) | critic decisivo (quitarlo cambia la elección) | sin critic decisivo |
|---|---|---|---|
| START | GoalDist 37, GoalAlign 27 | PathAlign 35, GoalAlign 25, PathDist 22, GoalDist 11, BaseObstacle 6 | 15 |
| ALONG | GoalDist 441, GoalAlign 153, BaseObstacle 24, PathAlign 10 | PathAlign 252, PathDist 187, GoalAlign 97, GoalDist 76, BaseObstacle 3 | 283 |
| PERT_LAT | GoalDist 842, GoalAlign 345, BaseObstacle 48, PathAlign 21 | PathAlign 622, PathDist 397, GoalAlign 292, GoalDist 204, BaseObstacle 11 | 431 |
| PERT_YAW | GoalDist 873, GoalAlign 317, BaseObstacle 48, PathAlign 18 | PathAlign 589, PathDist 366, GoalAlign 226, GoalDist 175, BaseObstacle 18 | 500 |

- Por magnitud domina GoalDist/GoalAlign (sus raw scores son distancias en celdas, mayores que el resto). Los que más a menudo **deciden** la elección son PathAlign y PathDist (escala 32). BaseObstacle (escala 0.1) casi nunca decide: actúa solo como filtro de legalidad.

### Footprint completo vs BaseObstacle

| tipo | casos con ≥1 trayectoria LEGAL cuyo footprint toca LETHAL | trayectoria ELEGIDA con footprint en LETHAL | pose inicial con footprint en LETHAL |
|---|---|---|---|
| START | 29 / 64 | 0 | 0 |
| ALONG | 271 / 628 | 0 | 0 |
| PERT_LAT | 518 / 1256 | 24 | 24 |
| PERT_YAW | 513 / 1256 | 24 | 24 |

Por categoría (todas las evaluaciones):

| categoría | casos | aceptados | elegida con footprint LETHAL | pose con footprint LETHAL | máx. costo footprint en ALONG nominal |
|---|---|---|---|---|---|
| around_obstacle | 300 | 300 | 0 | 0 | 253 |
| corridor_N1 | 128 | 128 | 48 | 48 | 253 |
| corridor_N2 | 1048 | 1048 | 0 | 0 | 242 |
| corridor_WIDE | 128 | 128 | 0 | 0 | 219 |
| long | 905 | 905 | 0 | 0 | 242 |
| random | 650 | 650 | 0 | 0 | 242 |
| short | 45 | 45 | 0 | 0 | 219 |

- **Hallazgo principal**: en 48 casos perturbados (todos `corridor_N1`, 0.45 m de ancho → ≈4.5 cm por lado para el footprint con padding de 0.36 m) el footprint en la pose ya está en LETHAL (254), el centro sigue en costo < 253, y **DWB acepta y elige avanzar** (vx 0.21–0.25). En 48 casos la trayectoria elegida tiene el footprint en LETHAL. Con ±5 cm de error lateral o ±0.15 rad de yaw, DWB no detecta el contacto de las esquinas.
- En el 41–45 % de los casos aceptados (según tipo) existen trayectorias legales para DWB cuyo footprint cruza LETHAL. Con la configuración nominal (sin perturbación) nunca fueron las elegidas: las evitaron PathDist/PathAlign, no BaseObstacle. Esa protección depende de que el path esté centrado, no de un chequeo de colisión.

### Variante `realcm` (local costmap idéntico al real, sin /scan)

| tipo | casos | aceptados | rechazados | error | t 1ª llamada p50 / p95 / máx (ms) | comando |
|---|---|---|---|---|---|---|
| START | 64 | 64 | 0 | 0 | 9.5 / 10.0 / 10.8 | rotate_in_place 36, forward_turn 22, forward 6 |
| ALONG | 628 | 628 | 0 | 0 | 10.0 / 10.5 / 10.9 | forward 506, forward_turn 117, rotate_in_place 5 |

| tipo | casos con ≥1 trayectoria LEGAL cuyo footprint toca LETHAL | trayectoria ELEGIDA con footprint en LETHAL | pose inicial con footprint en LETHAL |
|---|---|---|---|
| START | 0 / 64 | 0 | 0 |
| ALONG | 0 / 628 | 0 | 0 |

- Sin LiDAR el local costmap real no tiene ninguna fuente de obstáculos (`obstacle_layer` es la única capa que marca; no tiene `static_layer`). BaseObstacle no rechaza nada y la verificación de footprint no encuentra LETHAL en el costmap porque no hay obstáculos. **DWB en la config real depende 100 % del LiDAR para evitar paredes**: sin `/scan_filtered` (driver caído, filtro mal configurado) sigue el path sin ninguna protección de colisión local.

### Plan que no empieza cerca del robot (corrida descartada, medida)

Primera corrida con el path completo desde el start para las muestras ALONG/PERT (`eval_INVALID_fullpath_nopruning.jsonl`, detenida tras 289 casos):

| s (m de arco donde está el robot) | casos | aceptados |
|---|---|---|
| 0–1.5 | 65 | 65 |
| 1.5–2.0 | 15 | 15 |
| 2.0–2.5 | 15 | 15 |
| 2.5–3.0 | 11 | 11 |
| 3.0–∞ | 170 | 19 |

Todos los rechazos: `Resulting plan has 0 poses in it.` Se descartó como evaluación (artefacto del banco), pero documenta que DWB rechaza cualquier FollowPath cuyo plan empiece a más de ~2 m de arco (y >1.5 m euclídeo) del robot.

### Por path (START + resumen de muestras)

| test | categoría | long. (m) | err. heading inicial (°) | START | cmd (vx, wz) | t (ms, mediana 5) | critic decisivo | ALONG ok | PERT ok | PERT elegida con footprint LETHAL |
|---|---|---|---|---|---|---|---|---|---|---|
| T001 | short | 0.66 | -119 | ACCEPTED | (0.000, 0.895) | 8.5 | PathAlign;PathDist | 0/0 | 0/0 | 0 |
| T002 | short | 1.49 | -157 | ACCEPTED | (0.000, -0.263) | 7.2 | PathAlign | 2/2 | 8/8 | 0 |
| T003 | short | 1.10 | -27 | ACCEPTED | (0.237, 0.684) | 7.8 | PathAlign;PathDist | 1/1 | 4/4 | 0 |
| T004 | short | 1.06 | -116 | ACCEPTED | (0.013, 0.895) | 9.0 | GoalAlign;GoalDist | 1/1 | 4/4 | 0 |
| T005 | short | 0.71 | -116 | ACCEPTED | (0.000, 0.789) | 4.3 | GoalAlign | 0/0 | 0/0 | 0 |
| T006 | short | 1.07 | 27 | ACCEPTED | (0.250, -0.789) | 8.9 | PathAlign | 1/1 | 4/4 | 0 |
| T007 | short | 0.64 | 35 | ACCEPTED | (0.171, -1.000) | 8.3 | GoalAlign;PathAlign;PathDist;GoalDist | 0/0 | 0/0 | 0 |
| T008 | short | 0.73 | -34 | ACCEPTED | (0.250, 0.684) | 8.5 | GoalAlign;PathAlign;PathDist | 0/0 | 0/0 | 0 |
| T009 | short | 1.18 | -65 | ACCEPTED | (0.053, 1.000) | 8.9 | GoalAlign;PathAlign;PathDist | 1/1 | 4/4 | 0 |
| T010 | short | 1.03 | 77 | ACCEPTED | (0.053, -1.000) | 8.6 | GoalAlign;PathAlign;PathDist | 1/1 | 4/4 | 0 |
| T011 | long | 13.67 | 104 | ACCEPTED | (0.000, -1.000) | 9.3 | GoalAlign;PathAlign;PathDist | 26/26 | 104/104 | 0 |
| T012 | long | 9.51 | 30 | ACCEPTED | (0.224, -0.579) | 9.2 | GoalAlign;PathAlign | 18/18 | 72/72 | 0 |
| T013 | long | 7.49 | -38 | ACCEPTED | (0.224, 0.474) | 9.2 | GoalAlign;PathAlign;PathDist;GoalDist | 14/14 | 56/56 | 0 |
| T014 | long | 10.58 | 124 | ACCEPTED | (0.000, -1.000) | 9.0 | — | 20/20 | 80/80 | 0 |
| T015 | long | 8.72 | 86 | ACCEPTED | (0.000, -1.000) | 5.5 | PathAlign | 16/16 | 64/64 | 0 |
| T016 | long | 8.43 | 120 | ACCEPTED | (0.000, -1.000) | 9.3 | PathAlign;PathDist | 16/16 | 64/64 | 0 |
| T017 | long | 9.07 | -156 | ACCEPTED | (0.000, 0.789) | 9.1 | GoalAlign;PathAlign;PathDist | 17/17 | 68/68 | 0 |
| T018 | long | 10.03 | -65 | ACCEPTED | (0.250, 1.000) | 9.5 | GoalDist | 19/19 | 76/76 | 0 |
| T019 | long | 7.40 | 68 | ACCEPTED | (0.000, -0.895) | 9.5 | GoalAlign;PathAlign;PathDist | 14/14 | 56/56 | 0 |
| T020 | long | 10.05 | -157 | ACCEPTED | (0.000, 0.579) | 9.2 | GoalAlign;PathAlign | 19/19 | 76/76 | 0 |
| T021 | around_obstacle | 3.58 | 114 | ACCEPTED | (0.000, -1.000) | 9.5 | PathDist | 6/6 | 24/24 | 0 |
| T022 | around_obstacle | 3.31 | 143 | ACCEPTED | (0.000, -1.000) | 9.6 | GoalAlign;PathAlign;PathDist | 6/6 | 24/24 | 0 |
| T023 | around_obstacle | 2.24 | -157 | ACCEPTED | (0.000, 0.474) | 9.2 | GoalAlign;PathAlign;PathDist | 3/3 | 12/12 | 0 |
| T024 | around_obstacle | 2.28 | -33 | ACCEPTED | (0.118, 0.895) | 9.3 | GoalAlign;PathAlign;GoalDist | 3/3 | 12/12 | 0 |
| T025 | around_obstacle | 3.67 | -136 | ACCEPTED | (0.000, 0.789) | 9.0 | GoalAlign;PathAlign;PathDist | 6/6 | 24/24 | 0 |
| T026 | around_obstacle | 3.67 | 131 | ACCEPTED | (0.000, -1.000) | 9.5 | GoalAlign;PathAlign;PathDist | 6/6 | 24/24 | 0 |
| T027 | around_obstacle | 3.34 | 22 | ACCEPTED | (0.000, -0.263) | 10.3 | PathAlign | 6/6 | 24/24 | 0 |
| T028 | around_obstacle | 2.47 | -92 | ACCEPTED | (0.000, 0.158) | 11.5 | GoalAlign;PathAlign;PathDist | 4/4 | 16/16 | 0 |
| T029 | around_obstacle | 4.36 | -6 | ACCEPTED | (0.013, -1.000) | 10.6 | GoalAlign;PathDist | 8/8 | 32/32 | 0 |
| T030 | around_obstacle | 5.67 | 75 | ACCEPTED | (0.237, -1.000) | 9.0 | GoalAlign;PathAlign;PathDist;GoalDist | 10/10 | 40/40 | 0 |
| T031 | corridor_WIDE | 2.22 | 0 | ACCEPTED | (0.250, 0.000) | 9.0 | — | 3/3 | 12/12 | 0 |
| T032 | corridor_WIDE | 2.27 | -10 | ACCEPTED | (0.250, 0.263) | 9.2 | GoalAlign;PathAlign;GoalDist | 3/3 | 12/12 | 0 |
| T033 | corridor_WIDE | 2.26 | -1 | ACCEPTED | (0.250, 0.158) | 8.8 | — | 3/3 | 12/12 | 0 |
| T034 | corridor_WIDE | 2.22 | -90 | ACCEPTED | (0.000, 0.895) | 9.2 | — | 3/3 | 12/12 | 0 |
| T035 | corridor_WIDE | 2.24 | -44 | ACCEPTED | (0.250, 0.895) | 8.8 | GoalDist | 3/3 | 12/12 | 0 |
| T036 | corridor_WIDE | 2.23 | 0 | ACCEPTED | (0.224, 0.053) | 9.1 | — | 3/3 | 12/12 | 0 |
| T037 | corridor_WIDE | 2.22 | 135 | ACCEPTED | (0.000, -1.000) | 9.4 | — | 3/3 | 12/12 | 0 |
| T038 | corridor_WIDE | 2.22 | -180 | ACCEPTED | (0.000, -1.000) | 8.9 | — | 3/3 | 12/12 | 0 |
| T039 | corridor_N1 | 2.23 | 15 | ACCEPTED | (0.066, -0.053) | 9.6 | BaseObstacle | 3/3 | 12/12 | 6 |
| T040 | corridor_N1 | 2.23 | -16 | ACCEPTED | (0.079, -0.053) | 9.7 | BaseObstacle | 3/3 | 12/12 | 6 |
| T041 | corridor_N1 | 2.23 | 15 | ACCEPTED | (0.066, -0.053) | 9.6 | BaseObstacle | 3/3 | 12/12 | 6 |
| T042 | corridor_N1 | 2.23 | -106 | ACCEPTED | (0.000, 1.000) | 8.9 | — | 3/3 | 12/12 | 6 |
| T043 | corridor_N1 | 2.23 | -30 | ACCEPTED | (0.039, 0.684) | 9.6 | BaseObstacle | 3/3 | 12/12 | 6 |
| T044 | corridor_N1 | 2.23 | -16 | ACCEPTED | (0.079, -0.053) | 9.7 | BaseObstacle | 3/3 | 12/12 | 6 |
| T045 | corridor_N1 | 2.23 | 150 | ACCEPTED | (0.000, 0.368) | 9.4 | PathAlign | 3/3 | 12/12 | 6 |
| T046 | corridor_N1 | 2.23 | 164 | ACCEPTED | (0.000, -1.000) | 8.9 | — | 3/3 | 12/12 | 6 |
| T047 | corridor_N2 | 13.69 | 101 | ACCEPTED | (0.000, -1.000) | 10.4 | PathAlign | 26/26 | 104/104 | 0 |
| T048 | corridor_N2 | 13.65 | -94 | ACCEPTED | (0.000, 1.000) | 10.5 | PathAlign;PathDist | 26/26 | 104/104 | 0 |
| T049 | corridor_N2 | 13.69 | 101 | ACCEPTED | (0.000, -1.000) | 10.3 | PathAlign | 26/26 | 104/104 | 0 |
| T050 | corridor_N2 | 13.65 | 176 | ACCEPTED | (0.000, 1.000) | 9.0 | GoalAlign | 26/26 | 104/104 | 0 |
| T051 | corridor_N2 | 13.69 | 56 | ACCEPTED | (0.092, -0.895) | 9.9 | BaseObstacle;PathAlign | 26/26 | 104/104 | 0 |
| T052 | corridor_N2 | 13.65 | -94 | ACCEPTED | (0.000, 1.000) | 10.6 | PathAlign;PathDist | 26/26 | 104/104 | 0 |
| T053 | corridor_N2 | 13.69 | -124 | ACCEPTED | (0.000, 0.789) | 9.4 | PathAlign | 26/26 | 104/104 | 0 |
| T054 | corridor_N2 | 13.65 | 86 | ACCEPTED | (0.000, -1.000) | 8.9 | — | 26/26 | 104/104 | 0 |
| T055 | random | 1.26 | 173 | ACCEPTED | (0.000, -1.000) | 8.9 | GoalAlign | 1/1 | 4/4 | 0 |
| T056 | random | 12.59 | 7 | ACCEPTED | (0.237, -0.158) | 9.2 | PathAlign;GoalDist | 24/24 | 96/96 | 0 |
| T057 | random | 7.45 | -77 | ACCEPTED | (0.250, 0.895) | 9.4 | — | 14/14 | 56/56 | 0 |
| T058 | random | 10.86 | -34 | ACCEPTED | (0.237, 1.000) | 9.5 | — | 21/21 | 84/84 | 0 |
| T059 | random | 7.47 | 14 | ACCEPTED | (0.250, -0.579) | 9.5 | — | 14/14 | 56/56 | 0 |
| T060 | random | 6.05 | -116 | ACCEPTED | (0.000, 0.895) | 9.2 | GoalAlign;PathAlign;PathDist | 11/11 | 44/44 | 0 |
| T061 | random | 3.66 | -116 | ACCEPTED | (0.000, 1.000) | 9.3 | — | 6/6 | 24/24 | 0 |
| T062 | random | 4.69 | -149 | ACCEPTED | (0.000, 0.368) | 9.0 | PathAlign;GoalDist | 8/8 | 32/32 | 0 |
| T063 | random | 5.56 | -66 | ACCEPTED | (0.250, 1.000) | 9.1 | — | 10/10 | 40/40 | 0 |
| T064 | random | 9.97 | 52 | ACCEPTED | (0.237, -1.000) | 9.3 | GoalAlign;GoalDist | 19/19 | 76/76 | 0 |

## LIMITACIONES

- **No es lazo cerrado**: cada caso es una única llamada a DWB desde una pose y velocidad dadas. No hay integración de movimiento, goal checker, progress checker, recuperaciones ni replanificación. "Aceptado" = DWB devuelve un comando válido en ese instante, no que llegue al goal. Esto requiere el simulador cinemático (R4-B).
- La velocidad inicial de las muestras ALONG (0.25, 0) es un supuesto (crucero en recta). En curvas reales sería menor y la ventana de muestreo distinta.
- Variante R4A: el local costmap usa el mapa estático como proxy de percepción LiDAR perfecta (sin ruido, sin oclusiones, sin rango máximo). La config real no tiene static_layer en el local costmap. La variante `realcm` refleja la config real sin sensor.
- Mapa sintético de desarrollo, no la arena física. Resolución del mapa 0.03 m, local costmap 0.05 m: el chequeo de footprint hereda esa discretización (±1 celda).
- El chequeo de footprint (`FootprintCollisionChecker`) evalúa el contorno del polígono, no el interior. Es suficiente para paredes, no para obstáculos más chicos que el robot.
- La emulación de poda (path recortado desde s−1.4 m) aproxima el estado en régimen. No reproduce el historial exacto de `global_plan_` en lazo cerrado.
- Solo se evaluaron los paths de R3 sobre `synthetic_arena` (mapa cerrado). Faltan los de `synthetic_arena_unknown` (SUP_R3); con local costmap `track_unknown_space` por defecto (false), UNKNOWN se trata como libre en el costmap local.
- Tiempos medidos en esta Jetson con otros procesos de escritorio activos (firefox). Una sola corrida, sin aislamiento de CPU.

## Archivos

- Config: `config/offline_lab/dwb_offline_R4A.yaml`, `dwb_offline_R4A_realcm.yaml`, `dwb_offline_R4A_manifest.json`, `dwb_offline.launch.py`
- Harness: `src/limo_nav_offline_lab/src/dwb_offline_eval.cpp` (paquete `limo_nav_offline_lab`; build con `-DPython3_EXECUTABLE=/usr/bin/python3`)
- Scripts: `scripts/nav_offline_lab/{make_dwb_offline_config.py, make_dwb_r4a_cases.py, run_dwb_r4a.sh, dwb_r4a_audit.py, evaluate_dwb_r4a.py, make_dwb_r4a_report.py}`
- Resultados: `results/navigation_offline_lab/dwb_r4a/` (eval_*.jsonl crudos, cases_*.csv, summary_*.json, audit_*.json, server_check_*.txt, overview PNG)

## Siguiente paso recomendado

1. **R4-A2 (offline, rápido)**: repetir el banco con el critic `ObstacleFootprint` (dwb_critics::ObstacleFootprintCritic) en lugar de BaseObstacle, en una copia de la config offline. Medir si elimina los casos con footprint en LETHAL sin rechazar los paths nominales de `corridor_N1`.
2. **R4-B**: simulador cinemático en lazo cerrado (DWB → integrador diferencial con acc_lim y retardo/timeout de la base ~0.6 s) usando este mismo harness, con ruido de localización. Medir llegada, error de tracking y clearance mínimo de footprint.
3. Verificar en el robot (solo lectura de launch/config, sin ejecutar) qué launch usa `navigation2_real.yaml` y si hay `velocity_smoother`. Eliminar `holonomic` de la config real en un cambio justificado aparte (no tiene efecto).
4. Considerar `static_layer` en el local costmap real como respaldo si falla el LiDAR. Cambio de config real: requiere decisión explícita.
