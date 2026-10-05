# Sesión de mapeo — arena_20260918_1816

**ESTADO: PRE-MOVIMIENTO. El robot NO se ha movido. Sin autorización de movimiento aún.**

## Datos de sesión
- Fecha/hora inicio: 2026-09-18 18:16 (local)
- `ROS_DOMAIN_ID`: **17** (aislado del AiNex del dominio 5)
- `RMW_IMPLEMENTATION`: rmw_cyclonedds_cpp
- Robot: AgileX LIMO, **motion_mode=0 = MODE_FOUR_DIFF** (diferencial 4 ruedas, leído del hardware)
- error_code: 0, vehicle_state: 0, control_mode: 1
- **Batería inicial: 10.9 V** ⚠️ (baja; cargar antes del mapeo real — abort si <10.5 V)

## TF usada
- `base_link → laser_frame = [0.180, 0.000, -0.090]` (calibrada, verificada en vivo)
- `base_link → imu_link = [0, 0, 0]`
- `odom → base_link`: del driver `limo_base`

## Cartographer previsto
- Launch: `ros2 launch limo_bringup cartographer.launch.py use_sim_time:=false open_rviz:=true`
- Config `.lua`: **`limo_lds_2d.lua`** (fabricante) → **online correlative scan matching ON**
- **NO** se usa `limo_lds_2d_no_correlative.lua`
- Resolución occupancy grid: 0.03 m

## Filtro previsto
- `cartographer_scan_filter_chain` (incluido por el launch de Cartographer): **filtro del fabricante de 2 etapas** (range gate + speckle), `/scan → /scan_filtered`.

## Artefactos de esta sesión (todo aquí, NUNCA en ~/limoatwork)
- `bag/` — rosbag (grabación desde antes de mover)
- `map.pgm` + `map.yaml` — a guardar al finalizar
- `map.pbstream` — a guardar al finalizar (write_state de Cartographer)

## Advertencia
Este directorio se prepara con el robot **físicamente inmóvil**. Ningún comando de movimiento fue enviado. El mapeo real requiere autorización explícita nueva.
