# Auditoría completa de Cartographer — LIMO RoboCup SML

Fecha: 2026-09-17
Método: solo lectura y análisis estático. No se ejecutó ROS, no se inició Cartographer, no se modificó ningún archivo. Consulta de `~/limoatwork` y del paquete Cartographer del sistema estrictamente en modo lectura (referencia documental).

Fuentes:
- `config/cartographer/limo_lds_2d_no_correlative.lua` (proyecto)
- `config/cartographer/laser_filters_range_only.yaml` (proyecto)
- `results/baseline/slam_ab_r1/RESULTS.txt`, `results/baseline/stage3_cartographer_static/*`
- Referencia (solo lectura): `limoatwork/src/limo_ros2/limo_bringup/config_files/limo_lds_2d.lua`, `.../launch/cartographer.launch.py`, `.../config_files/laser_filters.yaml`
- Defaults del paquete: `/opt/ros/humble/share/cartographer/configuration_files/trajectory_builder_2d.lua`

---

## 1. Archivos Cartographer existentes

### En el proyecto (`~/limo_sml_ws/config/cartographer/`)
| Archivo | Fecha | Rol |
|---|---|---|
| `limo_lds_2d_no_correlative.lua` | 2026-09-15 17:32 | Config Cartographer, variante **correlative OFF** |
| `laser_filters_range_only.yaml` | 2026-09-15 19:44 | Filtro LaserScan, **solo range** (sin speckle) |

### Referencia de fábrica (`~/limoatwork/...`, INMUTABLE, solo lectura)
| Archivo | Rol / diferencia con el proyecto |
|---|---|
| `config_files/limo_lds_2d.lua` | Config base del fabricante, **correlative ON**. Idéntica al `.lua` del proyecto salvo `use_online_correlative_scan_matching` (true vs false). |
| `config_files/laser_filters.yaml` | Filtro del fabricante: **2 etapas** (range gate + speckle). |
| `launch/cartographer.launch.py` | Launch que arranca el `cartographer_node` + el filtro. |
| `config_files/occupancy_grid.launch.py` (incluido) | Publica el OccupancyGrid a partir de los submapas. |

### Defaults del paquete (`/opt/ros/humble/.../trajectory_builder_2d.lua`)
Aportan todos los valores que **ninguno** de los `.lua` sobreescribe (voxel filters, pesos Ceres, solver options, pose extrapolator, etc.).

## 2. Configuración activa — con una advertencia crítica de cableado

**No hay evidencia dentro de `~/limo_sml_ws` de que `limo_lds_2d_no_correlative.lua` sea efectivamente la config que Cartographer carga.** Razones, todas verificables:

- El único launch de Cartographer localizado (`limoatwork/.../cartographer.launch.py`) tiene:
  - `configuration_basename` por defecto = **`limo_lds_2d.lua`** (la variante correlative ON del fabricante), línea 35.
  - `cartographer_config_dir` por defecto = `limo_bringup/config_files/` (dentro de `limoatwork`), líneas 32-33.
- Para que Cartographer use el `.lua` del proyecto habría que **sobreescribir ambos argumentos** (`configuration_basename:=limo_lds_2d_no_correlative.lua` y `cartographer_config_dir:=~/limo_sml_ws/config/cartographer`) al lanzar. **Ningún launch del workspace del proyecto (`limo_sml_ws`) hace esto** — el único launch propio es `lidar_real.launch.py`, que solo arranca el YDLiDAR.
- Conclusión: el `.lua` del proyecto es, con la evidencia disponible, un **archivo de configuración candidato/experimental**, no una config demostrablemente conectada a un launch. Cómo se activó durante los tests A/B/C de `slam_ab_r1` (probablemente por override manual en línea de comandos) **no está documentado en el repo**.

**Evidencia cruzada con el baseline**: `stage3_cartographer_static/nodes.txt` lista el nodo `cartographer_scan_filter_chain` — que es exactamente el `name` del nodo de filtro definido en `cartographer.launch.py` (línea 98). Esto indica que la sesión que generó el mapa de 63.75 m² (`stage3`, 15:26) se corrió con **el launch del fabricante**, cuyo `laser_filters.yaml` por defecto es el de **2 filtros (range + speckle)** — no con el `range_only` del proyecto (que además es posterior: 19:44). Es decir, el mapa baseline de mayor escala se generó, muy probablemente, con la configuración del fabricante (correlative ON + filtro range+speckle), no con la config actual del proyecto.

## 3. Parámetros relevantes

Tabla comparativa: valor por defecto del paquete vs. override del `.lua` del proyecto. "heredado" = el proyecto no lo toca.

| Parámetro | Default paquete | Proyecto (`no_correlative.lua`) | Estado |
|---|---|---|---|
| `use_online_correlative_scan_matching` | false | **false (explícito)** | Ver §4 — coincide con SLAM-B descartado |
| `real_time_correlative_scan_matcher` (etapa previa al Ceres) | linear_search=0.1 m, angular=20°, weights 1e-1 | **heredado — pero INACTIVO** (porque el flag anterior es false) | La "etapa previa" no opera en esta config |
| `ceres_scan_matcher.occupied_space_weight` | 1.0 | heredado (1.0) | Sin ajuste |
| `ceres_scan_matcher.translation_weight` | 10.0 | heredado (10.0) | Sin ajuste — ver §5 |
| `ceres_scan_matcher.rotation_weight` | 40.0 | heredado (40.0) | Sin ajuste |
| `ceres_solver_options.max_num_iterations` | 20 | heredado (20) | Sin ajuste |
| `ceres_solver_options.num_threads` | 1 | heredado (1) | Sin ajuste |
| `voxel_filter_size` | 0.025 m | heredado (0.025) | Sin ajuste |
| `adaptive_voxel_filter` | max_length=0.5, min_num_points=200, max_range=50 | heredado | Sin ajuste; `max_range=50` no se recorta pese a `TRAJECTORY_BUILDER_2D.max_range=8` |
| `loop_closure_adaptive_voxel_filter` | max_length=0.9, min_num_points=100, max_range=50 | heredado | Sin ajuste |
| `motion_filter.max_angle_radians` | rad(1.0) | **rad(0.1)** (override) | Mucho más sensible que el default |
| `motion_filter.max_distance_meters` | 0.2 | heredado (0.2) | Sin ajuste |
| `motion_filter.max_time_seconds` | 5.0 | heredado (5.0) | Sin ajuste |
| `submaps.num_range_data` | 90 | **120** (override) | Submapas más grandes |
| `submaps.grid_options_2d.resolution` | 0.05 | **0.03** (override) | Coincide con `map.pgm` candidato y con occupancy_grid del launch |
| `range_data_inserter.hit_probability` | 0.55 | **0.57** (override) | Contraste hit/miss ligeramente mayor |
| `range_data_inserter.miss_probability` | 0.49 | **0.495** (override) | — |
| `pose_extrapolator.use_imu_based` | false | heredado (false) | Extrapolación de pose = modelo velocidad constante + odometría/IMU-gravedad |
| `min_range` / `max_range` | 0.0 / 30.0 | **0.1 / 8.0** (override) | `min_range` 0.1 vs 0.12 de AMCL/filtro |
| `missing_data_ray_length` | 5.0 | **0.1** (override) | Muy corto — apenas limpia espacio libre en rayos sin retorno |
| `POSE_GRAPH.constraint_builder.min_score` | (default paquete) | **0.65** | Umbral de loop-closure |
| `POSE_GRAPH.optimize_every_n_nodes` | (default paquete) | **comentado** | El comentario sugería 30, pero queda en el default del include |

### Filtros LaserScan (productor de `/scan_filtered`)
| | Proyecto (`laser_filters_range_only.yaml`) | Fabricante (`laser_filters.yaml`) |
|---|---|---|
| filter1 | `LaserScanRangeFilter` [0.12, 8.0] → NaN | `LaserScanRangeFilter` [0.12, 8.0] → NaN |
| filter2 | **(ausente)** | `LaserScanSpeckleFilter` (max_range=8.0, max_range_difference=0.15, window=2) |

El proyecto **elimina el filtro de speckle**. Esto es directamente relevante a los tests "RAW vs filtrado" de `slam_ab_r1` (ver §4).

## 4. Comparación contra `slam_ab_r1/RESULTS.txt` y baseline

1. **La config del proyecto es la variante que el propio test descartó.** `slam_ab_r1` concluye textualmente "No adoptar configuración B" (correlative OFF). El único `.lua` del proyecto tiene correlative OFF. Reconfirmado respecto a `AUDITORIA_CERES_SCAN_MATCHER.md`, ahora con el dato adicional de que el launch por defecto ni siquiera apunta a este archivo.

2. **Sesgo lateral persiste con y sin correlative** (MAP−ODOM lateral +1.4 a +2.8 cm en A, B y C). Como la etapa `real_time_correlative_scan_matcher` está inactiva en la config del proyecto, y el sesgo aparece igualmente con correlative ON (fabricante), la causa del sesgo **no puede atribuirse a esa etapa** con la evidencia disponible. Los pesos de Ceres nunca se tocaron (§5).

3. **"RAW vs filtrado" (tests C):** el proyecto creó `laser_filters_range_only.yaml` (solo range, sin speckle) el 2026-09-15 19:44, minutos después de los RESULTS de `slam_ab_r1` (19:42). Es plausible que "RAW"/"scan sin filtro completo" en el test se refiera precisamente a quitar el speckle filter (pasar de 2 filtros a 1). **No está confirmado** si "RAW" significó `/scan` crudo directo o este `range_only`. Queda como pregunta abierta.

4. **El mapa baseline de 63.75 m² (`stage3`) se generó con el launch del fabricante** (nodo `cartographer_scan_filter_chain` presente), es decir muy probablemente con correlative ON + filtro range+speckle — **no** con la config actual del proyecto. Esto refuerza que el mapa de mayor escala no corresponde a la configuración `no_correlative` presente hoy.

## 5. Clasificación de parámetros

### Confirmados coherentes con evidencia
- `use_imu_data = true`, frames (`tracking_frame=base_link`, `map_frame=map`, `odom_frame=odom`) — coherentes con la cadena TF del proyecto y con la cinemática diferencial confirmada en `base_motion_r1`.
- `submaps.resolution = 0.03` — coherente con el occupancy_grid del launch (0.03) y con el mapa candidato `map.pgm`.
- `min_range/max_range = 0.1/8.0` — coherente en el máximo (8.0) con AMCL y con el filtro range.

### Sospechosos (a investigar, sin tocar)
- **`ceres_scan_matcher.translation_weight = 10` frente a `occupied_space_weight = 1`**: el matcher pondera 10× la cercanía a la pose extrapolada (odometría/velocidad constante) sobre el ajuste al scan, en traslación. Si la odometría arrastra una componente lateral, un `translation_weight` alto tendería a preservarla. Es una **observación**, no una causa probada del sesgo lateral; requiere prueba dedicada.
- **`motion_filter.max_angle_radians = rad(0.1)`**: muy agresivo (10× más sensible que el default). Genera muchos nodos; impacto en CPU/consistencia no medido.
- **`missing_data_ray_length = 0.1`**: muy corto (default 5.0). Con el filtro que convierte fuera-de-rango en NaN, los rayos sin retorno apenas limpian espacio libre — puede afectar cómo se cierran zonas abiertas del mapa. No medido.
- **`min_range` 0.1 (Cartographer) vs 0.12 (filtro/AMCL)**: pequeña incoherencia numérica, origen no documentado.
- **Discrepancia de config activa**: el proyecto mantiene una variante (`no_correlative`) que el launch por defecto no usa y que el test descartó — sospechoso a nivel de proceso, no de un parámetro individual.

### Que NO deben modificarse todavía
- Pesos de `ceres_scan_matcher` (`occupied_space_weight`, `translation_weight`, `rotation_weight`) — `slam_ab_r1` pide repeticiones/auditoría antes de tocarlos.
- `use_online_correlative_scan_matching` — no cambiar unilateralmente; requiere decisión documentada sobre cuál config es la oficial.
- `motion_filter.max_angle_radians`, `missing_data_ray_length`, `submaps.num_range_data`, `hit/miss_probability` — overrides no validados con prueba; no tocar sin datos.
- `submaps.grid_options_2d.resolution` — acoplado a la resolución del mapa y a la decisión pendiente sobre mapas v1/run3.

## 6. Preguntas abiertas para el equipo

1. ¿Con qué launch y con qué argumentos se ejecutó Cartographer en los tests `slam_ab_r1` y en la sesión `stage3`? ¿Se usó el launch del fabricante con override, o hay un launch propio no localizado?
2. ¿Por qué el único `.lua` del proyecto es la variante correlative OFF (SLAM-B), que el propio test recomendó no adoptar? ¿Fue un descuido al no revertir?
3. ¿"RAW scan" en los tests C de `slam_ab_r1` significó `/scan` crudo directo, o el filtro `range_only` (sin speckle)? Impacta la interpretación del hallazgo RAW vs filtrado.
4. ¿Con qué configuración exacta (correlative ON/OFF, filtro 1 o 2 etapas, resolución) se generó el mapa que se pretende usar como oficial para AMCL/Nav2?
5. ¿Se ha intentado alguna vez ajustar los pesos de Ceres fuera del repo? La evidencia dice que no, pero conviene confirmarlo.
6. ¿Los overrides `motion_filter=rad(0.1)`, `num_range_data=120`, `missing_data_ray_length=0.1` fueron elecciones deliberadas y probadas, o heredadas de una plantilla?

---

*Informe generado por análisis estático de archivos existentes. No se propusieron cambios de parámetros. No se ejecutó ROS, launch ni Cartographer.*
