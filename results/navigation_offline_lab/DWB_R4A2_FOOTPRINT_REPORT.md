# NAV-OFFLINE-R4-A2 — DWB: BaseObstacle (centro) vs ObstacleFootprint (footprint completo)

Fecha: 2026-09-29. 100 % offline (ROS_DOMAIN_ID=99, ROS_LOCALHOST_ONLY=1). Sin hardware, sin limo_base, sin LiDAR, sin teleop, sin FollowPath/NavigateToPose, sin `/cmd_vel`. `navigation2_real.yaml` sin cambios (sha256 `dfb36e108d6872a5…`). `~/limoatwork` sin cambios. Configuración de producción NO modificada.

## Resumen

```
R4-A2
Baseline unsafe cases:      24   (contacto real del footprint sin padding; de 3204 casos)  |  48 por margen (<0.02 m)
Experimental unsafe cases:  0   |  0 por margen (<0.02 m)
Improvement:                100.0 % (contacto real)  |  100.0 % (margen)
```

## Método

- **Mismos insumos que R4-A**: mapa `synthetic_arena` (0.03 m), 64 paths NavFn de R3, mismos 3204 casos (START, ALONG cada 0.5 m, perturbaciones ±0.05 m lateral y ±0.15 rad de yaw), mismo orden, mismo harness (`dwb_offline_eval`: clase real `dwb_core::DWBLocalPlanner` + `Costmap2DROS` real, llamada `computeVelocityCommands` como en controller_server, sin publisher Twist).
- **BASELINE** `config/offline_lab/dwb_offline_R4A2_base.yaml`: DWB idéntico al real (critic `BaseObstacle`).
- **EXPERIMENTAL** `config/offline_lab/dwb_offline_R4A2_fp.yaml`: `BaseObstacle` → `ObstacleFootprint` (dwb_critics, instalado), misma posición en la lista, `scale: 0.1`. Diferencia de parámetros DWB verificada por script: solo `critics` y el scale del critic (BaseObstacle.scale, ObstacleFootprint.scale, critics).
- Ambas: local costmap `static_layer` + `inflation_layer` (como R4-A), `update_frequency` 20 Hz y espera de 120 ms tras recentrar (solo acelera el banco).
- **Verdad de terreno offline (solo evaluación, nunca control)**: geometría del PGM (celdas ocupadas, 9 puntos por celda) y footprint **real sin padding** [-0.17, 0.22]×[-0.16, 0.16]. Clearance con signo: < 0 significa penetración.

Clasificación por caso (positivo = DWB rechaza / detiene):

| clase | definición |
|---|---|
| TN | acepta y la trayectoria elegida tiene clearance ≥ 0 en todas sus poses |
| **FN (inseguro)** | acepta y la trayectoria elegida tiene clearance < 0 en alguna pose (el robot choca siguiendo el comando) |
| TP | rechaza y el footprint real ya está en contacto en la pose actual |
| FP (falsa alarma) | rechaza con el footprint real sin contacto en la pose actual |

"Centro pasa, robot no" = aceptado, celda central de la pose < INSCRIBED (253) en el costmap, que es el criterio de BaseObstacle, y clearance del footprint real < 0 (o < 0.02 m en la versión por margen).

## CONFIRMADO

- `dwb_critics::ObstacleFootprintCritic` existe en Nav2 Humble 1.1.20 instalado. El `controller_server` real lo carga con la config experimental (lifecycle active, `Using critic "ObstacleFootprint"`). Ver `dwb_r4a2/server_check_server_fp.txt`.
- Semántica (header y strings de libdwb_critics): recorre el **contorno** del footprint orientado (con padding, tomado de `getRobotFootprint()`) en cada pose de la trayectoria. Es ilegal si alguna celda es LETHAL ("Trajectory Hits Obstacle."), NO_INFORMATION ("Trajectory Hits Unknown Region.") o sale del grid ("Footprint Goes Off Grid."). INSCRIBED (253) no invalida, que es correcto porque el footprint se evalúa explícitamente.
- `ObstacleFootprintCritic::getScale()` = `resolution × scale`: con `scale: 0.1` y resolución 0.05, el peso efectivo en el costo es 0.005, 20 veces menos que BaseObstacle (0.1). No afecta la legalidad; sí puede cambiar qué trayectoria legal se elige.
- Consistencia: con ObstacleFootprint, 0 trayectorias legales tienen footprint (con padding) en LETHAL según el chequeo independiente del harness (baseline: 1331 casos con al menos una).
- Reproducibilidad: baseline A2 vs corrida R4-A (3204 casos, mismo orden): 0 comandos distintos, 0 cambios de aceptación.
- Auditoría de grafo en cada corrida: PASS (sin `/cmd_vel`, sin topics Twist en eval, sin acciones).

## MEDIDO

### Aceptación

| aceptados | ALL | START | ALONG | PERT_LAT | PERT_YAW |
|---|---|---|---|---|---|
| baseline | 3204 | 64 | 628 | 1256 | 1256 |
| experimental | 3156 | 64 | 628 | 1232 | 1232 |

| rechazados | ALL | START | ALONG | PERT_LAT | PERT_YAW |
|---|---|---|---|---|---|
| baseline | 0 | 0 | 0 | 0 | 0 |
| experimental | 48 | 0 | 0 | 24 | 24 |

Rutas (64): aceptada = START y todas las muestras ALONG nominales aceptadas.

| variante | rutas nominales aceptadas | rutas con algún rechazo bajo perturbación | rutas con algún caso inseguro |
|---|---|---|---|
| baseline | 64/64 | 0 | 8 (T039, T040, T041, T042, T043, T044, T045, T046) |
| experimental | 64/64 | 8 | 0 (—) |

### Seguridad (verdad de terreno geométrica)

| FN inseguros (contacto real) | ALL | START | ALONG | PERT_LAT | PERT_YAW |
|---|---|---|---|---|---|
| baseline | 24 | 0 | 0 | 24 | 0 |
| experimental | 0 | 0 | 0 | 0 | 0 |

| inseguros por margen (elegida con clearance < 0.02 m) | ALL | START | ALONG | PERT_LAT | PERT_YAW |
|---|---|---|---|---|---|
| baseline | 48 | 0 | 0 | 24 | 24 |
| experimental | 0 | 0 | 0 | 0 | 0 |

| TP | ALL | START | ALONG | PERT_LAT | PERT_YAW |
|---|---|---|---|---|---|
| baseline | 0 | 0 | 0 | 0 | 0 |
| experimental | 24 | 0 | 0 | 24 | 0 |

| FP total | ALL | START | ALONG | PERT_LAT | PERT_YAW |
|---|---|---|---|---|---|
| baseline | 0 | 0 | 0 | 0 | 0 |
| experimental | 24 | 0 | 0 | 0 | 24 |

| FP dentro del padding (pose con clearance 0–0.02 m) | ALL | START | ALONG | PERT_LAT | PERT_YAW |
|---|---|---|---|---|---|
| baseline | 0 | 0 | 0 | 0 | 0 |
| experimental | 24 | 0 | 0 | 0 | 24 |

| FP estrictos (pose con clearance ≥ 0.02 m) | ALL | START | ALONG | PERT_LAT | PERT_YAW |
|---|---|---|---|---|---|
| baseline | 0 | 0 | 0 | 0 | 0 |
| experimental | 0 | 0 | 0 | 0 | 0 |

| TN | ALL | START | ALONG | PERT_LAT | PERT_YAW |
|---|---|---|---|---|---|
| baseline | 3180 | 64 | 628 | 1232 | 1256 |
| experimental | 3156 | 64 | 628 | 1232 | 1232 |

| centro pasa, robot no (contacto real) | ALL | START | ALONG | PERT_LAT | PERT_YAW |
|---|---|---|---|---|---|
| baseline | 24 | 0 | 0 | 24 | 0 |
| experimental | 0 | 0 | 0 | 0 | 0 |

| centro pasa, robot no (clearance < 0.02 m) | ALL | START | ALONG | PERT_LAT | PERT_YAW |
|---|---|---|---|---|---|
| baseline | 48 | 0 | 0 | 24 | 24 |
| experimental | 0 | 0 | 0 | 0 | 0 |

| elegida con footprint+padding en LETHAL (costmap, métrica R4-A) | ALL | START | ALONG | PERT_LAT | PERT_YAW |
|---|---|---|---|---|---|
| baseline | 48 | 0 | 0 | 24 | 24 |
| experimental | 0 | 0 | 0 | 0 | 0 |

- Casos con el footprint real ya en contacto en la pose actual: 24. Con clearance < 0.02 m: 48.
- baseline: razones de rechazo —; clearance en la pose de los FP —; de los TP —.
- experimental: razones de rechazo {'No valid trajectories out of 411!': 48}; clearance en la pose de los FP {'n': 24, 'min': 0.018930542448241904, 'p5': 0.018930542448241904, 'p50': 0.018969775613630876}; de los TP {'n': 24, 'min': -0.0008227054013021373, 'p5': -0.0008227054013021373, 'p50': -1.0597641897466814e-05}.

### Clearance real del footprint en la trayectoria elegida (m)

| variante | tipo | n | mín | p5 | p50 |
|---|---|---|---|---|---|
| baseline | ALL | 3204 | -0.009 | 0.133 | 0.251 |
| baseline | START | 64 | 0.047 | 0.061 | 0.233 |
| baseline | ALONG | 628 | 0.043 | 0.159 | 0.252 |
| baseline | PERT_LAT | 1256 | -0.009 | 0.136 | 0.262 |
| baseline | PERT_YAW | 1256 | 0.003 | 0.153 | 0.240 |
| experimental | ALL | 3156 | 0.026 | 0.153 | 0.254 |
| experimental | START | 64 | 0.031 | 0.043 | 0.233 |
| experimental | ALONG | 628 | 0.043 | 0.152 | 0.252 |
| experimental | PERT_LAT | 1232 | 0.026 | 0.150 | 0.267 |
| experimental | PERT_YAW | 1232 | 0.026 | 0.168 | 0.244 |

Figura: `dwb_r4a2/r4a2_clearance.png` (histograma de clearance y detalle de corridor_N1).

### Por categoría

| categoría | casos | contacto en pose | base acept. | base FN | exp. acept. | exp. FN | exp. TP | exp. FP | trayect. legales medias base → exp. |
|---|---|---|---|---|---|---|---|---|---|
| around_obstacle | 300 | 0 | 300 | 0 | 300 | 0 | 0 | 0 | 410 → 384 |
| corridor_N1 | 128 | 24 | 128 | 24 | 80 | 0 | 24 | 24 | 256 → 40 |
| corridor_N2 | 1048 | 0 | 1048 | 0 | 1048 | 0 | 0 | 0 | 410 → 398 |
| corridor_WIDE | 128 | 0 | 128 | 0 | 128 | 0 | 0 | 0 | 411 → 390 |
| long | 905 | 0 | 905 | 0 | 905 | 0 | 0 | 0 | 411 → 401 |
| random | 650 | 0 | 650 | 0 | 650 | 0 | 0 | 0 | 411 → 402 |
| short | 45 | 0 | 45 | 0 | 45 | 0 | 0 | 0 | 411 → 410 |

### Costo de cómputo

| variante | t p50 (ms) | t máx (ms) |
|---|---|---|
| baseline | 9.9 | 13.3 |
| experimental | 26.2 | 41.6 |

### Observaciones adicionales

- En los 24 casos inseguros de la baseline el centro está a 0.160 m geométricos del obstáculo, **menos** que el radio inscripto con padding (0.18 m). Aun así la celda central vale 242 (< 253 INSCRIBED) en el costmap a 0.05 m. La rasterización hace que BaseObstacle sea todavía más permisivo que el propio criterio "centro fuera del radio inscripto".
- corridor_N1 (0.45 m): trayectorias legales medias por caso baseline → experimental en la tabla por categoría. En ALONG nominal dentro de N1 la experimental deja 43–44 de 411 trayectorias legales (baseline ≥ 258): pasa, pero con poco margen de maniobra.
- Chequeo independiente del harness (`FootprintCollisionChecker` de nav2_costmap_2d) vs critic (`ObstacleFootprintCritic::lineCost`): en 17 casos hay 1–3 trayectorias legales para el critic que el chequeo del harness marca LETHAL. Son diferencias de rasterización entre los dos algoritmos de línea; ninguna fue la elegida (0 elegidas con footprint+padding en LETHAL).

Comandos distintos entre baseline y experimental: 311 / 3204. Se deben al peso efectivo distinto del critic de obstáculos y a trayectorias que dejan de ser legales.

## LIMITACIONES

- **Instantáneas, no lazo cerrado**: cada caso es una llamada a DWB. Un FN significa que el comando elegido lleva el footprint a contacto dentro del horizonte de 1.7 s. En lazo cerrado DWB recalcula a 10 Hz y podría corregir antes del contacto, o no. Requiere R4-B.
- **Precisión de la verdad de terreno**: celdas del PGM de 0.03 m representadas por 9 puntos (error de clearance ≤ ~0.0075 m). Las penetraciones medidas (~0.01 m) están en el orden de esa precisión y de la discretización del mapa. El resultado es "contacto o casi contacto", no una profundidad de choque confiable.
- **Costmap 0.05 m**: los critics ven obstáculos rasterizados a 0.05 m, más el padding de 0.02. La frontera entre FP y TP depende de esa discretización.
- **Sin LiDAR simulado**: el local costmap usa el mapa estático (percepción perfecta). La config real solo tiene `obstacle_layer`. Sin `/scan` ninguna de las dos variantes ve obstáculos (ver R4-A, variante realcm).
- **Empates**: en casos con margen ~0 entre la mejor y la segunda trayectoria, la elección puede depender de la historia del costmap rodante (ventana recentrada con truncamiento entero, ±1 celda). Se observó en 3/64 START al cambiar el orden de casos (smoke vs corrida completa). Baseline y experimental usan el mismo orden: comparación válida. Causa probable, no confirmada.
- Un solo mapa sintético (`synthetic_arena`); `synthetic_arena_unknown` no se evaluó. Perturbaciones solo de ±0.05 m y ±0.15 rad, sin combinarlas.
- No se evaluó `scale` alternativo de ObstacleFootprint (peso efectivo igual a BaseObstacle: `scale: 2.0`). Afecta la preferencia entre trayectorias legales, no la legalidad.

## Archivos

- Config: `config/offline_lab/dwb_offline_R4A2_{base,fp}.yaml`, `dwb_offline_R4A2_manifest.json`
- Scripts: `scripts/nav_offline_lab/{make_dwb_r4a2_config.py, evaluate_dwb_r4a2.py, make_dwb_r4a2_report.py}`; harness `src/limo_nav_offline_lab` (+ `settle_ms`, poses de la trayectoria elegida)
- Resultados: `results/navigation_offline_lab/dwb_r4a2/` (eval_full_{base,fp}.jsonl, r4a2_cases.csv, r4a2_summary.json, r4a2_clearance.png, auditorías)

## Siguiente paso recomendado

1. **R4-B: simulador cinemático en lazo cerrado** con este harness (DWB → integrador diferencial con acc_lim, timeout de base ~0.6 s y ruido de localización), comparando BASELINE vs ObstacleFootprint. La pregunta abierta es qué pasa después del rechazo: en Nav2 real el controlador aborta FollowPath y el BT ejecuta recuperaciones (spin/backup). Un backup con footprint en contacto también debe evaluarse.
2. Barrido offline corto de `ObstacleFootprint.scale` (0.1 vs 2.0, que iguala el peso efectivo de BaseObstacle) y de perturbaciones combinadas y más grandes (±0.03/0.05/0.08 m × ±0.1/0.15/0.25 rad) en corridor_N1/N2, para ubicar el límite de rechazo y el margen de trayectorias legales.
3. Si R4-B confirma el beneficio, **proponer** (no aplicar sin autorización) el cambio en `navigation2_real.yaml`: `BaseObstacle` → `ObstacleFootprint`. El cambio de producción requiere decisión explícita y prueba física supervisada.
4. Independiente del critic: el local costmap real no tiene `static_layer`, así que sin LiDAR ninguna variante protege (R4-A). ObstacleFootprint no resuelve eso.
