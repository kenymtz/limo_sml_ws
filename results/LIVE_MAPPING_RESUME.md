# LIVE_MAPPING_RESUME — Estado PRE-MAPPING restaurado — LIMO RoboCup SML

Fecha: 2026-09-18 ~19:00 (local)
Dominio: `ROS_DOMAIN_ID=17` (aislado del AiNex). **Robot físicamente inmóvil. Ningún comando de movimiento enviado. `/cmd_vel` sin publisher. Sin autorización de movimiento aún.**

Contexto: la sesión anterior fue apagada **intencionalmente** por el usuario (no por fallo ni batería). Se restauró el estado PRE-MAPPING para el primer mapeo físico del entorno actual (el mapa que existía en disco NO corresponde a este entorno; por eso se remapea con SLAM). **No se usa AMCL / map_server / Nav2 en este proceso.**

## 1. Estado actual

| Ítem | Valor |
|---|---|
| `ROS_DOMAIN_ID` | 17 |
| Contaminación AiNex (dominio 5) | 0 (aislado) |
| **`/cmd_vel` Publisher count** | **0** (Sub 1 = driver base) |
| Batería | **11.8 V** (cargada) |
| `error_code` / `vehicle_state` | 0 / 0 |
| `motion_mode` | 0 = MODE_FOUR_DIFF |
| Robot | quieto, `/odom` en origen (0,0) |

## 2. Nodos activos

| Rol | Nodo |
|---|---|
| Driver base | `/limo/limo_base` |
| TF `base_link→imu_link` | `/imu_link_broadcaster` |
| LiDAR | `/ydlidar_ros2_driver_node` |
| TF `base_link→laser_frame` | `/static_tf_pub_laser` |
| SLAM | `/cartographer_node` |
| OccupancyGrid | `/occupancy_grid_node` |
| Filtro scan | `/cartographer_scan_filter_chain` |
| RViz | `rviz2` |
| Grabación | `ros2 bag record` (PID 9907/9930) |

**NO activos (correcto):** amcl, map_server, controller_server, planner_server, bt_navigator, lifecycle managers. Confirmado 0 nodos Nav2/AMCL.

## 3. Configuración usada

- **Cartographer:** `configuration_basename = limo_lds_2d.lua` (fabricante) → **online correlative scan matching ON**; filtro **2 etapas** (range + speckle); resolución occupancy grid **0.03 m**. **NO** se usó `no_correlative`. Verificado en el log (`Found '.../limo_lds_2d.lua'`).
- **Filtro:** `/scan → /scan_filtered` (cartographer_scan_filter_chain).
- **Topics consumidos por Cartographer:** `/scan_filtered`, `/odom`, `/imu`, `/tf`.
- **Topics publicados:** `/map`, `/submap_list`, TF `map→odom`.
- **Frecuencias:** `/scan` ~10.1 Hz, `/scan_filtered` ~9.98 Hz, `/odom` ~50 Hz, `/imu` ~100 Hz.
- **TF:** `map→odom` (solo Cartographer), `odom→base_link` (base), `base_link→laser_frame = [0.180, 0.000, -0.090]`, `base_link→imu_link = [0,0,0]`.

## 4. Grabación (rosbag)

- **Ubicación:** `/home/limopro/limo_sml_ws/results/maps/arena_20260918_1900/bag/` (`bag_0.db3`, creciendo). **NO en /tmp.**
- **Topics suscritos (7):** `/scan`, `/scan_filtered`, `/odom`, `/imu`, `/tf`, `/tf_static`, `/limo_status`.
- `/cmd_vel`: aún sin publisher (robot quieto); el recorder lo añadirá automáticamente al iniciar teleop.
- **Mapa a guardar al finalizar** (mismo directorio, NUNCA en `~/limoatwork`): `map.pgm`, `map.yaml`, `map.pbstream`.

## 5. Riesgos pendientes

- **Movimiento no autorizado aún.** No se ha conducido el robot; no existe mapa válido todavía.
- **Batería 11.8 V:** buena para empezar; vigilar durante el recorrido (abort si <10.5 V).
- **AiNex (dominio 5):** mitigado por aislamiento en dominio 17; si el rosbag/robag se corta sin cierre limpio, el `.db3` queda sin `metadata.yaml` (recuperable pero no ideal) → cerrar el bag limpiamente al terminar.
- **Guardado del mapa:** al finalizar hay que guardar `.pgm/.yaml/.pbstream` y cerrar el bag limpiamente para no repetir la pérdida tipo `stage3`.
- **Config `no_correlative` vs vendor:** se usa la vendor (correlative ON); decisión de config de SLAM sigue abierta con el equipo (no bloquea este mapeo).

## 6. Siguiente acción

**Esperar la autorización explícita del usuario: `"AUTORIZO MOVIMIENTO DE MAPEO"`.**
Solo entonces se inicia el recorrido físico (teleop humano, recorrido mínimo en bucle cerrado, velocidad ≤0.15 m/s lineal / ≤0.4 rad/s angular, según `PLAN_MAPEO_ENTORNO_ACTUAL.md`), vigilando las condiciones de abort. Antes de habilitar teleop: reverificar exclusividad de `/cmd_vel` (exactamente 1 publisher = el teleop).

*(No se emiten conclusiones de calidad del mapa: el mapeo aún no ha comenzado.)*

---

*Estado PRE-MAPPING restaurado. El robot NO se ha movido. `/cmd_vel` Publisher = 0. `~/limoatwork` solo lectura. Artefactos en `results/maps/arena_20260918_1900/`.*
