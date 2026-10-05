# LIVE_NAV_R1 — Auditoría de launches (FASE 1, parte read-only) — LIMO RoboCup SML

Fecha: 2026-09-18
Modo: lectura de archivos launch. No se ejecutó ningún launch para elaborar esta auditoría. Autorización del usuario: levantar componentes SIN movimiento tras auditar y proponer.

## Veredicto por launch

| Launch | Qué arranca | ¿Puede mover el robot? | Veredicto |
|---|---|---|---|
| `limo_sml_tools/lidar_real.launch.py` (proyecto) | YDLiDAR (`ydlidar_ros2_driver`) + TF estática `base_link→laser_frame (0.180,0,−0.090)` | **No** (sensor puro) | **SAFE TO START** |
| `limo_base/limo_base.launch.py` (proyecto = fabricante, idénticos) | Driver `limo_base` (ns `limo`): `/odom`, TF `odom→base_link`, `/limo_status`, sub `/cmd_vel`; TF `base_link→imu_link` | **No autónomamente** (suscribe `/cmd_vel`, nunca lo publica). Energiza la ruta de comando. | **SAFE TO START** (con cautela: arma `/cmd_vel`) |
| `limo_bringup/limo_start.launch.py` (fabricante) | base + **`urg_node2` (Hokuyo)** + TF lidar `(0.12,0,0)` + filtro scan + rviz(off) | No autónomamente | **NO USAR para este robot** (usa LiDAR/TF equivocados; ver §Hallazgos) |
| `limo_bringup/limo_localization.launch.py` | `amcl` + `map_server` + `lifecycle_manager` (autostart) | **No** | **SAFE TO START** (requiere override de `params_file` y `map`; ver §Hallazgos) |
| `limo_bringup/limo_navigation.launch.py` | `controller_server`, `planner_server`, `recoveries_server`, `bt_navigator`, `waypoint_follower` + lifecycle (autostart) | No sin goal, **pero arma `/cmd_vel`** (el `controller_server` lo publica al ejecutar path) y monta recoveries | **START DIFERIDO** — solo si se necesita para FASE 5/6, con cautela; no requerido para FASES 1–4 |
| `limo_bringup/navigation2.launch.py` | nav2_bringup completo + RViz + filtro scan | Igual que arriba | **START DIFERIDO** |
| `limo_bringup/limo_start_navigation.launch.py` | navigation + localization + lifecycle (stack completo) | Igual que arriba | **START DIFERIDO** |
| `limo_bringup/cartographer.launch.py` | Cartographer SLAM (publica `map→odom`) | No, pero **conflictúa con AMCL** (ambos publican `map→odom`) | **NO USAR** en navegación con mapa conocido |
| `limo_bringup/limon_teletop_keyboard.launch.py` | Teleop de teclado → **publica `/cmd_vel`** | **SÍ (movimiento directo)** | **DO NOT START** |
| `mycobot_ros2/*`, `myarm_*`, `*_moveit2` | Brazo / MoveIt / controladores de articulaciones | **SÍ (actuadores)** | **DO NOT START** |
| `limo_description/gazebo_models_diff.launch.py` | Gazebo (simulación) | Spawnea sim, contamina dominio | **DO NOT START** |

## Plan de bringup sin movimiento (FASES 1–4)

1. `lidar_real.launch.py` (YDLiDAR + TF calibrada) — sensor puro.
2. `limo_base.launch.py` — driver base (odom/imu/status + TF `odom→base_link`).
3. Tras cada arranque: verificar `ros2 topic info /cmd_vel -v` → **Publisher count debe seguir en 0**.
4. Para FASE 3 (AMCL estático): `limo_localization.launch.py` con override `params_file:=~/limo_sml_ws/config/navigation/navigation2_real.yaml` y `map:=<mapa confirmado>`.

## Hallazgos de la auditoría (documentados, sin corregir nada)

1. **LiDAR: dos drivers posibles.** El fabricante (`limo_start.launch.py`) usa `urg_node2` (Hokuyo) con TF `(0.12,0,0)`; el proyecto (`lidar_real.launch.py`) usa **YDLiDAR** con TF calibrada **`(0.180,0,−0.090)`**. Para validar el proyecto se usa el segundo. Verificar en vivo que el YDLiDAR es el hardware real (aparición de `/scan`).
2. **TF lidar divergente:** fabricante `(0.12,0,0)` vs proyecto/baseline `(0.180,0,−0.090)`. Solo se informa; no se corrige.
3. **`limo_localization.launch.py` usa parámetros del FABRICANTE por defecto** (`limo_bringup/param/navigation2.yaml`, `maps/limo.yaml`), **no** el `navigation2_real.yaml` del proyecto (que tiene `DifferentialMotionModel`, footprint corregido, etc.). Si se lanza sin override, AMCL correría con la config del fabricante — esto es exactamente lo que FASE 5 debe verificar (config disco vs live).
4. **`autostart=true`** en localization y navigation → los nodos se activan solos (map_server/amcl, y en navigation también controller/planner/bt).
5. **`limo_base` corre en namespace `limo`** (`pub_odom_tf=True`). Verificar en vivo los nombres efectivos de topics (`/odom` vs `/limo/odom`) y de TF.

---

*Auditoría de launches por lectura. Aún no se ejecutó ningún launch. DO NOT START marca lo prohibido; START DIFERIDO marca lo que arma `/cmd_vel` y se pospone.*
