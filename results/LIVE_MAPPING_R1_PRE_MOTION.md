# LIVE_MAPPING_R1 — Precheck pre-movimiento (Cartographer listo) — LIMO RoboCup SML

Fecha: 2026-09-18 18:18 (local)
Dominio: `ROS_DOMAIN_ID=17` (aislado del AiNex). **Robot físicamente inmóvil durante todo el procedimiento. Ningún comando de movimiento enviado. `/cmd_vel` sin publisher.**

## VEREDICTO: ✅ **PRE-MAPPING STATIC: PASS**

Sistema listo para comenzar un nuevo mapa con Cartographer + grabación completa, **detenido en el último punto seguro previo a mover el robot**.

## 1. Procesos / nodos ROS activos

| Rol | Nodo | PID |
|---|---|---|
| Driver base | `/limo/limo_base` | 12492 |
| TF `base_link→imu_link` | `/imu_link_broadcaster` | (base launch) |
| LiDAR | `/ydlidar_ros2_driver_node` | 12368 |
| TF `base_link→laser_frame` | `/static_tf_pub_laser` | (lidar launch) |
| SLAM | `/cartographer_node` | 16080 |
| OccupancyGrid | `/occupancy_grid_node` | 16082 |
| Filtro scan (Cartographer) | `/cartographer_scan_filter_chain` | 16078 |
| RViz | `rviz2` | 16086 |
| Grabación | `ros2 bag record` | 15961/15985 |

**Detenidos (para no conflictuar con Cartographer):** map_server, amcl, lifecycle managers, controller/planner/behavior/bt_navigator, y el filtro standalone previo. **Confirmado: 0 nodos Nav2/AMCL activos.**

## 2. Seguridad `/cmd_vel`

- **Publisher count = 0** (verificado antes de bajar Nav2, tras bajar Nav2, tras arrancar Cartographer, y al final). 
- Subscription count = 1 (solo el driver base).
- **Nadie comanda movimiento. El robot no se ha movido.**

## 3. Frecuencias (robot quieto)

| Topic | Frecuencia |
|---|---|
| `/scan` | ~10.06 Hz |
| `/scan_filtered` | ~10.04 Hz (Cartographer) |
| `/odom` | ~49.9 Hz |
| `/imu` | ~99.7 Hz |

## 4. TF

| Transform | Valor / fuente | Estado |
|---|---|---|
| `map → odom` | **solo Cartographer** (AMCL detenido, sin publisher residual) | ✅ |
| `odom → base_link` | driver `limo_base` | ✅ |
| `base_link → laser_frame` | `[0.180, 0.000, -0.090]` | ✅ |
| `base_link → imu_link` | `[0, 0, 0]` | ✅ |

**Estabilidad estática `map→odom` (16 s, 1414 muestras, robot quieto):**
- drift X range = **3.28 mm**, drift Y range = **3.26 mm**, drift yaw range = **0.059°** → estable (nivel de ruido).
- (El offset absoluto yaw ≈ −17.5° es solo el origen arbitrario del frame `map` de Cartographer; irrelevante para el precheck.)

## 5. Cartographer — config realmente cargada (verificada en log + cmdline)

- `-configuration_directory /home/limopro/limoatwork/install/limo_bringup/share/limo_bringup/config_files`
- `-configuration_basename` = **`limo_lds_2d.lua`** ✅ (fabricante, **online correlative scan matching ON**, filtro 2 etapas range+speckle, resolución occupancy grid 0.03 m).
- **NO** se usó `limo_lds_2d_no_correlative.lua`.

## 6. Estado del robot

- `motion_mode = 0 = MODE_FOUR_DIFF` ✅
- `error_code = 0`, `vehicle_state = 0` ✅
- **Batería = 10.9 V** ⚠️ (baja; **cargar antes del mapeo real**; abort si <10.5 V).

## 7. Ubicaciones exactas

- **Rosbag (grabando):** `/home/limopro/limo_sml_ws/results/maps/arena_20260918_1816/bag/` (`bag_0.db3`, ~43 MB y creciendo).
  - Topics capturados: `/scan`, `/scan_filtered`, `/odom`, `/imu`, `/tf`, `/tf_static`, `/limo_status` (7/8). `/cmd_vel` se añadirá automáticamente cuando aparezca un publisher (al iniciar teleop).
- **Mapa a guardar al finalizar (NUNCA en ~/limoatwork):**
  - `/home/limopro/limo_sml_ws/results/maps/arena_20260918_1816/map.pgm`
  - `/home/limopro/limo_sml_ws/results/maps/arena_20260918_1816/map.yaml`
  - `/home/limopro/limo_sml_ws/results/maps/arena_20260918_1816/map.pbstream`
- **README de sesión:** `/home/limopro/limo_sml_ws/results/maps/arena_20260918_1816/README.md`

## 8. PASS/FAIL del precheck

| Check | Resultado |
|---|---|
| Base + LiDAR activos, sin reiniciar | ✅ PASS |
| Nav2/AMCL/map_server detenidos, sin residuos | ✅ PASS |
| `map→odom` solo de Cartographer | ✅ PASS |
| 1 cartographer_node / 1 occupancy_grid / 1 scan_filter | ✅ PASS |
| `/scan` y `/scan_filtered` ~10 Hz | ✅ PASS |
| `/odom` ~50 Hz, `/imu` ~100 Hz | ✅ PASS |
| TF calibrada laser correcta | ✅ PASS |
| `map→odom` estable en estático | ✅ PASS |
| Config `limo_lds_2d.lua` (correlative ON) cargada | ✅ PASS |
| Rosbag grabando en directorio de sesión | ✅ PASS |
| `/cmd_vel` Publisher = 0 | ✅ PASS |
| Robot inmóvil | ✅ PASS |
| Batería | ⚠️ 10.9 V (baja, no bloqueante para estático; cargar antes de mover) |

---

*Precheck pre-movimiento. El robot NO se ha movido. No se envió `/cmd_vel`. `~/limoatwork` solo lectura. Todo lo generado vive en `results/maps/arena_20260918_1816/`.*
