# Auditoría de configuración AMCL — LIMO RoboCup SML

Fecha: 2026-09-17
Fuente: `config/navigation/navigation2_real.yaml` (bloque `amcl`), comparado contra `results/baseline/` y `CLAUDE.md`.
Método: solo lectura y análisis estático. No se ejecutó ningún launch, nodo ROS ni comando de movimiento.

---

## 1. Parámetros AMCL activos (extraídos de `navigation2_real.yaml`)

```yaml
use_sim_time: False
robot_model_type: "nav2_amcl::DifferentialMotionModel"
base_frame_id: "base_link"
global_frame_id: "map"
odom_frame_id: "odom"
scan_topic: /scan_filtered
tf_broadcast: true
transform_tolerance: 1.0

laser_model_type: "likelihood_field"
laser_max_range: 8.0
laser_min_range: 0.12
laser_likelihood_max_dist: 2.0
max_beams: 60
z_hit: 0.5
z_max: 0.05
z_rand: 0.5
z_short: 0.05
sigma_hit: 0.2
lambda_short: 0.1
do_beamskip: false
beam_skip_distance: 0.5
beam_skip_error_threshold: 0.9
beam_skip_threshold: 0.3

alpha1..alpha5: 0.2 (los 5 iguales)

max_particles: 2000
min_particles: 500
pf_err: 0.05
pf_z: 0.99
resample_interval: 1
recovery_alpha_fast: 0.0
recovery_alpha_slow: 0.0

update_min_d: 0.25
update_min_a: 0.2
save_pose_rate: 0.5
```

---

## 2. `frame_id` y transformaciones esperadas

- `global_frame_id: "map"`, `odom_frame_id: "odom"`, `base_frame_id: "base_link"` — cadena estándar de Nav2 (`map → odom → base_link`), consistente con lo documentado en CLAUDE.md ("AMCL para localización", "TF").
- `tf_broadcast: true` — AMCL publica la transformada `map → odom`. Coincide con lo indicado en CLAUDE.md ("amcl: activo, tf_broadcast = true").
- **Comparación con evidencia real disponible**: la única medición de TF documentada en `results/baseline/` es `stage3_cartographer_static/map_to_base.txt`, que mide `map → base_link` **generado por Cartographer** (nodo `/cartographer_node`, ver `stage3_cartographer_static/nodes.txt`), no por AMCL. En esa etapa **no hay un nodo `amcl` en la lista de nodos activos** (`nodes.txt` solo lista `cartographer_node`, `cartographer_scan_filter_chain`, `occupancy_grid_node`, `imu_link_broadcaster`, `static_tf_pub_laser`, `ydlidar_ros2_driver_node`, `limo/limo_base`).
- **No se encontró en `results/baseline/` ningún archivo que capture AMCL corriendo en vivo** (sin `ros2 node info /amcl`, sin `ros2 topic echo /amcl_pose`, sin `ros2 param dump` de amcl). El estado "amcl: activo, tf_broadcast=true, robot_model_type=DifferentialMotionModel" que reporta CLAUDE.md **no tiene un artefacto de prueba equivalente a `base_motion_r1/RESULTS.txt` o `slam_ab_r1/RESULTS.txt`** que lo respalde con datos crudos dentro de este workspace.

## 3. `update_min_d` (0.25 m)

- AMCL solo actualiza el filtro de partículas cuando el robot se desplaza ≥0.25 m desde la última actualización.
- No hay una prueba en `results/baseline/` que mida específicamente el comportamiento de actualización de AMCL con este umbral (las pruebas de `base_motion_r1` validan odometría/IMU, no el ciclo de actualización de AMCL).
- Es un valor típico/por defecto de Nav2 (idéntico al usado en la plantilla TurtleBot3), no hay evidencia de que haya sido ajustado específicamente para el LIMO.

## 4. `update_min_a` (0.2 rad ≈ 11.46°)

- Mismo caso que `update_min_d`: sin prueba específica documentada en `results/baseline/`, valor consistente con plantilla estándar de Nav2.
- Relación con lo medido en `base_motion_r1`: los tests MOTION-06/07 (giros de ±90°) superan ampliamente este umbral, por lo que un giro completo dispararía múltiples actualizaciones de AMCL — pero esto es inferencia, no una medición directa del comportamiento de AMCL.

## 5. `laser_model_type` (`likelihood_field`)

- Modelo estándar y recomendado por Nav2 para LiDAR 2D — coherente con el uso de YDLiDAR confirmado en `results/baseline/serial_devices.txt` y `stage2_base_lidar`/`stage2_calibrated_lidar`.
- Parámetros asociados (`z_hit`, `z_max`, `z_rand`, `z_short`, `sigma_hit`, `lambda_short`) no tienen contraparte de validación empírica en `results/baseline/` — no hay una prueba que compare localización real vs. reportada por AMCL para calibrar estos pesos.
- `laser_max_range: 8.0` m — no hay en `results/baseline/` una medición del rango máximo real del YDLiDAR para confirmar que 8 m es apropiado (los archivos `scan_info.txt`/`scan_header.txt` solo confirman tipo de mensaje y tasa, no rango).

## 6. Particle filter (`max_particles`, `min_particles`, `pf_err`, `pf_z`, `resample_interval`)

- `min_particles: 500`, `max_particles: 2000`, `pf_err: 0.05`, `pf_z: 0.99` — valores estándar de Nav2/TurtleBot3, sin evidencia de ajuste específico para el LIMO o para el tamaño de los mapas del proyecto (mapas evaluados en `results/footprint_analysis/` son pequeños, 10-14 m² de área libre; 2000 partículas es generoso para ese tamaño, pero no hay una prueba de rendimiento/CPU documentada).
- `recovery_alpha_fast: 0.0`, `recovery_alpha_slow: 0.0` — **recuperación por inyección aleatoria de partículas deshabilitada**. Si AMCL se desubica (p. ej. por secuestro del robot o error grande), no se recupera automáticamente generando partículas aleatorias; depende entera y únicamente de la reinicialización de pose. No hay prueba en `results/baseline/` que valide el comportamiento de AMCL ante pérdida de localización.

## 7. Relación con mapa conocido (known map navigation)

- El flujo documentado en CLAUDE.md es: mapa conocido → planificar ruta → LiDAR detecta cambios → si hay mismatch, detener. AMCL es el componente que ubica al robot dentro de ese mapa conocido.
- **No se confirmó en esta auditoría, ni en las anteriores, cuál es el mapa real que AMCL usa en producción** (`map_server.yaml_filename` está vacío en el yaml, se sobreescribe por launch — coherente con Nav2 estándar, pero significa que el mapa real depende de qué se le pase en tiempo de lanzamiento, no está fijado en este archivo).
- Los candidatos a mapa "v1"/"run3" identificados en auditorías previas (`map.pgm` res. 0.03, `map_20260718_185606.pgm` res. 0.05) siguen sin confirmación del equipo.
- Dato adicional relevante encontrado en esta auditoría: `results/baseline/stage3_cartographer_static/map_metadata.txt` reporta un mapa generado en vivo por Cartographer de **220×322 px a 0.03 m/px ≈ 6.6 × 9.66 m (~63.7 m²)** — sustancialmente más grande que cualquiera de los mapas estáticos guardados en `limoatwork/.../limo_bringup/maps/` (máximo 26.5 m²). Esto sugiere que **ninguno de los mapas guardados en disco corresponde necesariamente al área completa mapeada en esa sesión de Cartographer**, y que podría existir un mapa "run3 full arena" no localizado aún, o que ese mapa fue una sesión de prueba no guardada.

---

## Parámetros confirmados (con evidencia directa en `results/baseline/` o coherentes con hallazgos ya validados)

- `robot_model_type: DifferentialMotionModel` — coherente con cinemática real confirmada en `base_motion_r1` (MODE_FOUR_DIFF).
- `base_frame_id`, `odom_frame_id`, `global_frame_id` — cadena de frames estándar, sin evidencia de error.
- `scan_topic: /scan_filtered` — el topic existe y publica a ~10 Hz según `stage3_cartographer_static/scan_filtered_hz.txt` (aunque esa medición corresponde a la etapa de Cartographer, no a una sesión con AMCL activo).
- `laser_model_type: likelihood_field` — apropiado para LiDAR 2D confirmado en el proyecto.

## Posibles riesgos

1. **AMCL nunca fue validado con una prueba documentada equivalente a `base_motion_r1` o `slam_ab_r1`.** El estado "activo" en CLAUDE.md es una afirmación sin artefacto de respaldo en `results/baseline/` dentro de este workspace — no se puede confirmar el error real de localización de AMCL (solo se conoce el error del TF de Cartographer, que es un sistema distinto).
2. **Recuperación por partículas aleatorias deshabilitada** (`recovery_alpha_fast/slow: 0.0`) — sin prueba de qué pasa si AMCL pierde la localización durante una misión real; en una competencia esto podría dejar al robot sin capacidad de auto-recuperación ante un error grande.
3. **Parámetros del modelo de sensor (`z_hit`, `sigma_hit`, etc.) sin calibrar empíricamente** para el YDLiDAR real del proyecto — son valores de plantilla, no ajustados con datos propios.
4. **Dependencia del mapa correcto sin confirmar** (ver sección 7) — cualquier validación de AMCL que se haga en el futuro depende de resolver primero qué mapa es el de producción.
5. **Posible mapa de mayor tamaño no localizado** (~63.7 m² según metadata de Cartographer en stage3) — si ese es el mapa real de la arena y no se está usando, AMCL podría estar operando (o probándose a futuro) contra un mapa incorrecto o incompleto.

## Inconsistencias detectadas

- CLAUDE.md declara AMCL "activo" y con parámetros confirmados, pero **no existe en `results/baseline/` ningún archivo de captura de AMCL en ejecución** (nodo, topic `/amcl_pose`, o dump de parámetros) — a diferencia de BASE-MOTION-R1 y SLAM-AB-R1, que sí tienen `RESULTS.txt` con datos crudos.
- El tamaño del mapa generado en la sesión de Cartographer de `stage3_cartographer_static` (~63.7 m²) es muy superior a cualquier mapa estático guardado encontrado hasta ahora (máx. 26.5 m²) — no está claro si corresponde al mismo mapa candidato "v1"/"run3" evaluado en auditorías previas o a una sesión distinta no persistida como archivo.

## Qué NO modificar todavía

- **No cambiar `update_min_d` / `update_min_a`** sin antes generar una prueba equivalente a `base_motion_r1` que mida el comportamiento real de actualización de AMCL.
- **No habilitar `recovery_alpha_fast`/`recovery_alpha_slow`** (ni dejarlos en 0) sin decidir explícitamente, con el equipo, la estrategia de recuperación ante pérdida de localización — es un cambio de comportamiento de seguridad, no cosmético.
- **No recalibrar los parámetros del modelo de sensor** (`z_hit`, `z_max`, `z_rand`, `z_short`, `sigma_hit`, `lambda_short`) sin datos de campo que comparen pose de AMCL vs. posición real conocida.
- **No asumir que el mapa usado en producción está resuelto** — cualquier cambio en AMCL debe esperar a la confirmación del mapa real (v1/run3) ya señalada como pendiente en `ESTADO_PROYECTO.md`.
- **No ejecutar AMCL en vivo para "solo probar"** sin que sea una prueba planificada y documentada (siguiendo el mismo formato que `base_motion_r1/RESULTS.txt`), dado que actualmente no existe ese respaldo.

---

*Informe generado a partir de `config/navigation/navigation2_real.yaml`, `results/baseline/` y `CLAUDE.md`. No se inventaron resultados; donde no existe evidencia directa, se indicó explícitamente. No se ejecutó ningún comando ROS, launch, ni de movimiento para su elaboración.*
