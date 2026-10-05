# NAV-OFFLINE-R4-B — Lazo cerrado: simulador cinemático LIMO + DWB real (BaseObstacle vs ObstacleFootprint)

Fecha: 2026-09-29. 100 % offline (ROS_DOMAIN_ID=99, ROS_LOCALHOST_ONLY=1). Sin hardware, sin limo_base, sin LiDAR, sin teleop, sin `/cmd_vel` (el simulador no crea publishers Twist; auditoría de grafo PASS en cada corrida). `navigation2_real.yaml` y `~/limoatwork` sin cambios.

## Resumen

```
R4-B
MODELO:                     PASS
ESCENARIOS:                 6  (× 64 paths × 2 variantes = 768 corridas)
CONTACTOS BaseObstacle:     5 corridas con contacto real (de 384)
CONTACTOS ObstacleFootprint:  2 corridas con contacto real (de 384)
```

## Método

- **Simulador** `src/limo_nav_offline_lab/src/dwb_closed_loop_sim.cpp`: la clase real `dwb_core::DWBLocalPlanner` y el `Costmap2DROS` real (mismos parámetros que controller_server, configs `config/offline_lab/dwb_offline_R4B_{base,fp}.yaml` derivadas de A2) en un lazo de tiempo simulado:
  - control 10 Hz (`controller_frequency`): `computeVelocityCommands(pose, odom_twist)` → lógica replicada de controller_server (`failure_tolerance` 0.3 s con comando cero, `SimpleProgressChecker` 0.1 m / 25 s, `SimpleGoalChecker` xy 0.25 / yaw 0.25 stateful) → cola de latencia;
  - base FOUR_DIFF (vx, yaw_rate) a 100 Hz: ganancia de tracking k_v/k_w, saturación |vx| ≤ 0.25 y |ω| ≤ 1.0, rampas acc 1.0 / 1.5 y decel 1.0 / 1.5, timeout de comando 0.6 s; integración exacta de arco;
  - localización: odometría perfecta (odom = pose verdadera); el error de localización entra como `map→odom` ruidoso (Gauss-Markov). El plan (frame `map`) se ve desplazado; las paredes del local costmap (`static_layer`, frame `map_true` colgado de odom) se ven en su posición relativa real, como un LiDAR (proxy de percepción perfecta);
  - costmap: loop propio desactivado y `updateMap()` síncrono en cada paso; chequeo de cordura al arrancar (120 celdas LETHAL junto a la pared oeste).
- **Paths**: los mismos 64 paths NavFn de R3/R4-A/A2 sobre `synthetic_arena`, desde el start del benchmark (v=0), plan fijo (sin replanificación del BT).
- **Pareado**: la semilla del ruido depende solo de (path, escenario): A y B reciben la misma secuencia.
- **Verdad de terreno offline**: geometría del PGM y footprint real sin padding (clearance con signo a 50 Hz sobre la pose verdadera). Contacto = clearance < 0.

| escenario | definición |
|---|---|
| S1_nominal | sin ruido, sin latencia, tracking ideal (k=1) |
| S2_ruido_xy | ruido de localización XY σ=0.03 m por eje (Gauss-Markov τ=1 s) |
| S3_ruido_yaw | ruido de localización yaw σ=0.05 rad (2.9°), τ=1 s |
| S4_latencia_0p6 | latencia pura de base 0.6 s (valor pedido; ESTRÉS, ver CONFIRMADO) |
| S5_combinado_medido | XY 0.03 + yaw 0.05 + latencia 0.05 s (medida) + tracking k_v 0.95 / k_w 0.92 |
| S6_combinado_lat0p6 | S5 con latencia 0.6 s |

## CONFIRMADO

- **Modelo** (chequeos automáticos sobre todos los logs):

| chequeo | resultado | detalle |
|---|---|---|
| saturacion_v | PASS | exceso_max=0 |
| saturacion_w | PASS | exceso_max=0 |
| acel_v | PASS | exceso_max_m_s2=0.000 |
| acel_w | PASS | exceso_max_rad_s2=0.000 |
| cinematica_vel_pose | PASS | exceso_max_m_s=0.001 |
| latencia_S1_nominal_A | PASS | config=0, observada_min=0.100, observada_max=0.100 |
| latencia_S1_nominal_B | PASS | config=0, observada_min=0.100, observada_max=0.100 |
| latencia_S4_latencia_0p6_A | PASS | config=0.600, observada_min=0.700, observada_max=0.700 |
| latencia_S4_latencia_0p6_B | PASS | config=0.600, observada_min=0.700, observada_max=0.700 |
| ganancia_A | PASS | k_v=0.950, v_ratio_med=0.950, n_v=147, k_w=0.920, w_ratio_med=0.920, n_w=715 |
| ganancia_B | PASS | k_v=0.950, v_ratio_med=0.950, n_v=236, k_w=0.920, w_ratio_med=0.920, n_w=373 |
| ruido_pareado_S1_nominal | PASS | max_diff_pose_est_t0=0.000 |
| ruido_pareado_S2_ruido_xy | PASS | max_diff_pose_est_t0=0.000 |
| ruido_pareado_S3_ruido_yaw | PASS | max_diff_pose_est_t0=0.000 |
| ruido_pareado_S4_latencia_0p6 | PASS | max_diff_pose_est_t0=0.000 |
| ruido_pareado_S5_combinado_medido | PASS | max_diff_pose_est_t0=0.000 |
| ruido_pareado_S6_combinado_lat0p6 | PASS | max_diff_pose_est_t0=0.000 |
| determinismo_orden_independiente | PASS | comparados=3, distintos=[] |
| costmap_con_obstaculos | PASS | corridas=768, min_celdas_lethal=0, mediana=193.000 |

- **Latencia de la base, dato real**: `BASELINE_REAL_MAP_R1.md` (bag 2026-09-18) mide latencia de arranque ~47–50 ms y un **timeout** de comando de ~0.5–0.6 s (la base se detiene si no recibe comando), no una latencia de 0.6 s. S4/S6 usan 0.6 s de latencia pura porque se pidió explícitamente: son escenarios de estrés, no el comportamiento medido. S5 usa lo medido (0.05 s + timeout 0.6 s).
- **`RotateToGoal` está desactivado en la config real** (comentado en `navigation2_real.yaml`) y el goal checker exige yaw ≤ 0.25 rad. Ningún critic activo orienta el robot al yaw del goal: GoalAlign/PathAlign alinean con la dirección del path. Consecuencia medida abajo.
- El orden de padres en TF importa: tf2 admite un solo padre por frame. La primera versión colgaba `odom` de `map` y de `map_true` y dejaba el local costmap vacío (detectado por el warning de StaticLayer y corregido; esos resultados se descartaron). El chequeo de cordura evita repetirlo.

- **Mínimo local de BaseObstacle en pasillos (verificado con el desglose de critics, `eval_n1stuck_{base,fp}.jsonl`)**: `BaseObstacleCritic` sin `sum_scores` puntúa el costo de la **última** pose de la trayectoria. En la entrada de corridor_N1 la línea central vale 219 (< 253, legal; volcado `start_ray_costs` en `sim_n1diag_*.jsonl`), así que avanzar recto cuesta 219 × 0.1 = 21.9. GoalDist (24/m) + GoalAlign solo ahorran ~10 por avanzar 0.43 m. Total avanzar = 45.9 contra 31–34 de un giro casi en el lugar. Con la config real el robot **no entra** a zonas infladas de costo alto y gira de un lado a otro hasta el progress checker. Con ObstacleFootprint (peso efectivo resolución × scale = 0.005) la misma penalización vale 1.3 y el robot entra. La hipótesis inicial ("el centro cae en INSCRIBED por rasterización") quedó **refutada** por el volcado de costos.
- **Determinismo**: con estado idéntico las corridas son idénticas. La dependencia de historia entre corridas venía de la **deriva de punto flotante del origen del costmap rodante** (se acumula en cada recentrado), que voltea casi-empates de DWB. Se corrigió reinicializando el origen exacto (`LayeredCostmap::resizeMap`) al inicio de cada corrida; chequeo `determinismo_orden_independiente` PASS. Efecto colateral: cambiar la fase de la grilla (medio píxel) cambió el resultado de T047 (SUCCEEDED → ABORT_PROGRESS). El desenlace en el goal es sensible a la discretización.


## MEDIDO

### Resultado por escenario

| escenario | var. | SUCCEEDED | llegada xy | ABORT progreso | ABORT controller | TIMEOUT | **contactos** | clearance < 0.02 | clearance mín (m) | t llegada xy med (s) | t tras llegada xy med (s) |
|---|---|---|---|---|---|---|---|---|---|---|---|
| S1_nominal | A | 14 | 50 | 49 | 0 | 1 | **0** | 5 | 0.017 | 22.5 | 27.4 |
| S1_nominal | B | 15 | 56 | 46 | 2 | 1 | **0** | 6 | 0.010 | 15.3 | 27.4 |
| S2_ruido_xy | A | 49 | 51 | 14 | 0 | 1 | **1** | 5 | -0.057 | 22.5 | 9.7 |
| S2_ruido_xy | B | 54 | 55 | 2 | 5 | 3 | **0** | 6 | 0.011 | 15.7 | 10.6 |
| S3_ruido_yaw | A | 18 | 50 | 44 | 0 | 2 | **0** | 4 | 0.012 | 22.4 | 27.0 |
| S3_ruido_yaw | B | 26 | 55 | 33 | 4 | 1 | **0** | 5 | 0.010 | 15.7 | 26.6 |
| S4_latencia_0p6 | A | 22 | 50 | 41 | 0 | 1 | **0** | 0 | 0.040 | 23.1 | 26.6 |
| S4_latencia_0p6 | B | 28 | 56 | 31 | 4 | 1 | **2** | 4 | -0.009 | 16.1 | 26.5 |
| S5_combinado_medido | A | 52 | 55 | 10 | 0 | 2 | **3** | 5 | -0.054 | 21.2 | 12.2 |
| S5_combinado_medido | B | 59 | 59 | 3 | 0 | 2 | **0** | 7 | 0.012 | 16.0 | 9.4 |
| S6_combinado_lat0p6 | A | 52 | 52 | 11 | 0 | 1 | **1** | 3 | -0.005 | 23.0 | 7.9 |
| S6_combinado_lat0p6 | B | 57 | 57 | 4 | 2 | 1 | **0** | 2 | 0.018 | 17.5 | 8.3 |

A = BaseObstacle (DWB real), B = ObstacleFootprint. "llegada xy" = la pose estimada entra en 0.25 m del goal (primera etapa del goal checker). "t tras llegada xy" = tiempo que el robot sigue activo después de llegar en xy.

### Seguimiento y error final

| escenario | var. | error lateral medio (m) | p95 medio (m) | máx (m) | error final xy med / máx (m) | error final yaw med (rad) |
|---|---|---|---|---|---|---|
| S1_nominal | A | 0.044 | 0.117 | 0.309 | 0.017 / 2.385 | 1.378 |
| S1_nominal | B | 0.047 | 0.123 | 0.337 | 0.014 / 2.380 | 1.065 |
| S2_ruido_xy | A | 0.047 | 0.121 | 0.325 | 0.051 / 2.334 | 0.157 |
| S2_ruido_xy | B | 0.051 | 0.130 | 0.355 | 0.047 / 2.329 | 0.094 |
| S3_ruido_yaw | A | 0.045 | 0.119 | 0.319 | 0.016 / 2.385 | 1.033 |
| S3_ruido_yaw | B | 0.048 | 0.126 | 0.335 | 0.008 / 2.380 | 0.851 |
| S4_latencia_0p6 | A | 0.021 | 0.057 | 0.230 | 0.024 / 2.385 | 1.216 |
| S4_latencia_0p6 | B | 0.025 | 0.065 | 0.230 | 0.016 / 2.384 | 0.803 |
| S5_combinado_medido | A | 0.046 | 0.124 | 0.311 | 0.050 / 2.317 | 0.125 |
| S5_combinado_medido | B | 0.050 | 0.133 | 0.374 | 0.045 / 2.337 | 0.103 |
| S6_combinado_lat0p6 | A | 0.029 | 0.076 | 0.242 | 0.045 / 2.380 | 0.455 |
| S6_combinado_lat0p6 | B | 0.033 | 0.086 | 0.242 | 0.045 / 2.356 | 0.422 |

Error lateral = distancia de la pose verdadera al path NavFn hasta la llegada xy.

### Oscilaciones y comandos

| escenario | var. | cambios de signo de ω / min | cambios de signo tras llegada xy (media) | % giro en el lugar | % comando cero | % fallas DWB | % vx saturada | % ω saturada | vx media (m/s) | DWB ms media / máx |
|---|---|---|---|---|---|---|---|---|---|---|
| S1_nominal | A | 30.9 | 12.1 | 54 | 0.2 | 0.0 | 21 | 3 | 0.100 | 4.8 / 10.6 |
| S1_nominal | B | 37.8 | 12.5 | 47 | 0.5 | 0.3 | 24 | 4 | 0.114 | 11.9 / 26.1 |
| S2_ruido_xy | A | 42.7 | 6.1 | 38 | 0.1 | 0.0 | 26 | 12 | 0.129 | 4.8 / 10.8 |
| S2_ruido_xy | B | 45.8 | 5.9 | 29 | 0.5 | 0.5 | 30 | 12 | 0.145 | 12.0 / 27.9 |
| S3_ruido_yaw | A | 38.5 | 10.5 | 53 | 0.2 | 0.0 | 22 | 5 | 0.103 | 4.8 / 10.4 |
| S3_ruido_yaw | B | 39.6 | 9.8 | 42 | 0.5 | 0.4 | 26 | 6 | 0.124 | 12.0 / 26.1 |
| S4_latencia_0p6 | A | 22.5 | 4.8 | 50 | 0.2 | 0.0 | 23 | 6 | 0.112 | 4.8 / 10.1 |
| S4_latencia_0p6 | B | 23.2 | 5.1 | 42 | 0.5 | 0.4 | 27 | 8 | 0.130 | 12.0 / 27.1 |
| S5_combinado_medido | A | 45.1 | 7.3 | 37 | 0.0 | 0.0 | 27 | 13 | 0.131 | 4.8 / 10.1 |
| S5_combinado_medido | B | 44.3 | 5.8 | 27 | 0.0 | 0.0 | 32 | 13 | 0.152 | 12.0 / 25.6 |
| S6_combinado_lat0p6 | A | 38.1 | 4.5 | 36 | 0.1 | 0.0 | 29 | 13 | 0.138 | 4.8 / 10.4 |
| S6_combinado_lat0p6 | B | 38.3 | 4.8 | 27 | 0.2 | 0.2 | 33 | 15 | 0.155 | 12.1 / 27.0 |

Cambio de signo de ω = inversión del giro comandado (|ω| > 0.05) respecto del anterior no nulo.

### corridor_N1: entrada al pasillo y contactos

| escenario | A entra / 8 | A contactos | B entra / 8 | B contactos |
|---|---|---|---|---|
| S1_nominal | 0 | 0 | 6 | 0 |
| S2_ruido_xy | 1 | 1 | 5 | 0 |
| S3_ruido_yaw | 0 | 0 | 5 | 0 |
| S4_latencia_0p6 | 0 | 0 | 6 | 2 |
| S5_combinado_medido | 5 | 3 | 8 | 0 |
| S6_combinado_lat0p6 | 2 | 1 | 7 | 0 |
| **total** | **8/48** | **5** | **37/48** | **2** |

Todos los contactos de ambas variantes ocurren en corridor_N1. A entra poco porque se traba en la entrada (mínimo local, ver CONFIRMADO), pero **cuando entra, choca en 5 de 8 corridas** (penetración hasta 0.057 m, hasta 2.7 s en contacto) y termina SUCCEEDED sin enterarse. B entra en 37/48 y choca 2 veces, solo con latencia 0.6 s (0.008–0.009 m, < 0.65 s). En ambos casos DWB-B se queda sin trayectorias legales y el controller aborta (el robot se detiene).

### Fallas que no son contacto

- A: llegan en xy 308/384, SUCCEEDED 207/384: **101 corridas llegan en xy y no cumplen el yaw** (giran de un lado a otro hasta el progress checker). Tiempo mediano de llegada xy 22.5 s.
- B: llegan en xy 338/384, SUCCEEDED 239/384: **99 corridas llegan en xy y no cumplen el yaw** (giran de un lado a otro hasta el progress checker). Tiempo mediano de llegada xy 16.0 s.
- around_obstacle: SUCCEEDED A 17/60, B 17/60. Mínimo local con el goal detrás del obstáculo: el robot gira en el lugar desde el arranque sin avanzar, en ambas variantes. No depende del critic de obstáculos.
- Las fallas de yaw casi desaparecen con ruido XY (S2, S5, S6) pero no con ruido de yaw (S3). Hipótesis, no verificada: el ruido XY mueve el plan y la meta estimada y agita el rumbo hasta cruzar la ventana de ±0.25 rad por azar. No es una mejora real del controlador.

### Contactos: detalle pareado

- Pares (escenario, path) comparados: 384. Estado final distinto entre A y B: 70.
- Contacto solo en A: 5 (S2_ruido_xy:T043, S5_combinado_medido:T042, S5_combinado_medido:T043, S5_combinado_medido:T046, S6_combinado_lat0p6:T043)
- Contacto solo en B: 2 (S4_latencia_0p6:T042, S4_latencia_0p6:T045)
- Contacto en ambas: 0 (—)

### Por categoría de path (todos los escenarios)

| categoría | var. | corridas | SUCCEEDED | llegada xy | contactos | clearance mín (m) | error lateral medio (m) |
|---|---|---|---|---|---|---|---|
| around_obstacle | A | 60 | 17 | 24 | 0 | 0.015 | 0.040 |
| around_obstacle | B | 60 | 17 | 25 | 0 | 0.010 | 0.058 |
| corridor_N1 | A | 48 | 8 | 8 | 5 | -0.057 | 0.020 |
| corridor_N1 | B | 48 | 30 | 37 | 2 | -0.009 | 0.025 |
| corridor_N2 | A | 48 | 30 | 48 | 0 | 0.010 | 0.026 |
| corridor_N2 | B | 48 | 32 | 48 | 0 | 0.025 | 0.026 |
| corridor_WIDE | A | 48 | 34 | 48 | 0 | 0.154 | 0.030 |
| corridor_WIDE | B | 48 | 35 | 48 | 0 | 0.143 | 0.030 |
| long | A | 60 | 46 | 60 | 0 | 0.059 | 0.030 |
| long | B | 60 | 46 | 60 | 0 | 0.031 | 0.030 |
| random | A | 60 | 38 | 60 | 0 | 0.067 | 0.038 |
| random | B | 60 | 41 | 60 | 0 | 0.014 | 0.038 |
| short | A | 60 | 34 | 60 | 0 | 0.018 | 0.078 |
| short | B | 60 | 38 | 60 | 0 | 0.024 | 0.080 |

Figura: `dwb_r4b/r4b_overview.png`. Datos por corrida: `dwb_r4b/r4b_runs.csv`.

## LIMITACIONES

- **Modelo cinemático mínimo**: sin deslizamiento, sin dinámica de motores más allá de las rampas, sin cuantización ni ruido de odometría. Odometría perfecta: todo el error de pose es de localización (map→odom).
- **Base identificada con pocos datos** (BASELINE_REAL_MAP_R1: solo v=0.239 y ω=±0.478). Ganancias y latencia de S5 son valores medidos gruesos; las rampas asumen que la base respeta acc 1.0 / 1.5 (coherente con el ratio 0.28 en los primeros 0.15 s, no verificado en detalle).
- **Ruido de localización sintético** Gauss-Markov con σ elegidos (0.03 m, 0.05 rad, τ = 1 s). No es el error real de AMCL en la arena. Una sola semilla por (path, escenario).
- **Percepción perfecta**: el local costmap usa el mapa estático sin ruido de LiDAR, oclusiones ni rango. La config real solo tiene `obstacle_layer` (sin LiDAR, sin protección; R4-A).
- **Sin BT**: plan fijo (sin replanificación a 1 Hz), sin recuperaciones (spin / backup / clear costmap) tras un ABORT. El resultado ABORT es el del controller_server, no el de la navegación completa.
- **Lógica de controller_server replicada**, no ejecutada: goal/progress checker y `failure_tolerance` reimplementados en tiempo simulado según la semántica de Humble 1.1.20. El costmap se actualiza en cada ciclo de control (real: 5 Hz asíncrono).
- Mapa sintético único; contacto evaluado con celdas de 0.03 m (±~0.0075 m).

## Archivos

- Simulador: `src/limo_nav_offline_lab/src/dwb_closed_loop_sim.cpp`; configs `config/offline_lab/dwb_offline_R4B_{base,fp}.yaml` (+ manifest), launch `dwb_offline.launch.py mode:=closedloop`
- Scripts: `scripts/nav_offline_lab/{make_dwb_r4b_config.py, make_r4b_runs.py, run_r4b_all.sh, evaluate_dwb_r4b.py, make_dwb_r4b_report.py}`
- Resultados: `results/navigation_offline_lab/dwb_r4b/` (sim_*.jsonl crudos con logs de control y pose a 50 Hz, r4b_runs.csv, r4b_summary.json, auditorías, r4b_overview.png)

## Siguiente paso recomendado

1. **R4-C (offline): ajuste de critics de DWB en copia experimental, con este mismo simulador y los mismos 6 escenarios**:
   - `RotateToGoal` activo (está comentado en la config real): ataca el ~26–30 % de corridas que llegan en xy y abortan por yaw.
   - ObstacleFootprint con `scale` que compense el factor de resolución (p. ej. 2.0 → peso efectivo 0.1) y/o BaseObstacle con `sum_scores`: buscar la escala que no bloquee N1 (mínimo local de A) sin volver a los contactos.
   - Métrica de aceptación: 0 contactos en S1–S5, 0 contactos con penetración > 1 cm en S6, llegada ≥ 60/64 en S1.
2. Paralelo, barato: repetir B en S4/S6 con varias semillas para estimar la tasa de contacto con latencia alta. Hoy es 2/48 con una semilla.
3. **No** llevar ObstacleFootprint solo a `navigation2_real.yaml` todavía. Reduce contactos en lazo cerrado (5 → 0 fuera del estrés de latencia) y desbloquea N1, pero no resuelve la llegada en yaw ni el mínimo local de `around_obstacle`, que son las fallas dominantes. Cualquier cambio de producción requiere tu confirmación y una prueba física supervisada.
4. Medir en el robot (solo lectura de bags, sin mover) la latencia real cmd → movimiento a varias velocidades, y el error de AMCL en la arena, para reemplazar los σ supuestos.

