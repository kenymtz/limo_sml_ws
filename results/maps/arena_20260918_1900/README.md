# Sesión de mapeo — arena_20260918_1900

**ESTADO: PRE-MAPPING READY. El robot NO se ha movido. Sin autorización de movimiento.**

- Fecha/hora: 2026-09-18 ~19:00 (local)
- `ROS_DOMAIN_ID`: 17 (aislado del AiNex)
- Robot: LIMO, `motion_mode=0=MODE_FOUR_DIFF`, error_code 0
- Batería inicial: **11.8 V** (cargada)
- TF `base_link→laser_frame = [0.180, 0.000, -0.090]`

## SLAM
- Cartographer, `limo_lds_2d.lua` (correlative ON), filtro 2 etapas, resolución 0.03 m.

## Artefactos (todo aquí, NUNCA en ~/limoatwork)
- `bag/` — rosbag (grabando; topics: /scan, /scan_filtered, /odom, /imu, /tf, /tf_static, /limo_status; /cmd_vel se añade al iniciar teleop)
- `map.pgm` + `map.yaml` — a guardar al finalizar el mapeo
- `map.pbstream` — a guardar al finalizar (write_state de Cartographer)

## RESULTADO — mapeo completado (2026-09-18 ~19:58)

Recorrido conducido por teleop de teclado (humano), en dominio 17, robot en MODE_FOUR_DIFF. `/cmd_vel` exclusivo del teleop durante todo el recorrido. Sin abort. Batería final ~12.1 V.

**Artefactos guardados (todos en este directorio):**
| Archivo | Detalle |
|---|---|
| `map.pgm` | 382×336 px @ 0.03 m/px (≈11.5 × 10.1 m), 128 KB |
| `map.yaml` | resolution 0.03, origin [-3.32, -4.88, 0], trinary |
| `map.pbstream` | 19.8 MB (finish_trajectory 0 + write_state, code 0) |
| `bag/bag_0.db3` + `bag/metadata.yaml` | 574 MB, cierre limpio, `ros2 bag info` válido |

**Rosbag — 8 topics, ~57 min, 1.06M mensajes:**
`/scan` 34233, `/scan_filtered` 34233, `/odom` 170274, `/imu` 340549, `/tf` 477281, `/tf_static` 2, `/cmd_vel` 125, `/limo_status` 6811.

Config SLAM usada: Cartographer `limo_lds_2d.lua` (correlative ON), filtro 2 etapas, resolución 0.03 m.

**NOTA:** No se emiten conclusiones de calidad del mapa en esta etapa (evaluación offline pendiente).

**Comandos usados para guardar (referencia):**
```
ros2 run nav2_map_server map_saver_cli -f <este_dir>/map
ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}"
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '<este_dir>/map.pbstream', include_unfinished_submaps: true}"
```
