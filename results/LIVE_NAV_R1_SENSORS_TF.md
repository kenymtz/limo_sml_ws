# LIVE_NAV_R1 — Sensores y TF (FASE 1) — LIMO RoboCup SML

Fecha: 2026-09-18
Modo: se lanzaron **solo** componentes sin capacidad de movimiento autónomo: YDLiDAR (`lidar_real.launch.py`) y driver base (`limo_base.launch.py`). **Aislados en `ROS_DOMAIN_ID=17`** (dedicado, para separar del sistema AiNex que contamina el dominio 5). No se publicó `/cmd_vel`. No se movió el robot. `~/limoatwork` intacto.

## Estado FASE 1: ✅ **PASS**

Precondición de seguridad verificada: **`/cmd_vel` → Publisher count 0**, Subscription 1 (solo el driver base). Nadie comanda movimiento. Robot quieto durante toda la fase.

## 1. BASE (driver `limo_base`)

| Señal | Valor | Estado |
|---|---|---|
| `/odom` frecuencia | ~49.9 Hz (min 0.016 s, max 0.025 s, std 1.4 ms) | ✅ estable (~control_rate 50) |
| `/odom` frames | `odom → base_link` | ✅ |
| `/odom` estabilidad estática | pose x=0, y=0 en 3 muestras separadas; sin deriva con robot quieto | ✅ |
| `/imu` frecuencia | ~99.9 Hz (std 1.6 ms) | ✅ estable (~100 Hz) |
| `/limo_status` `error_code` | **0** | ✅ sin errores |
| `/limo_status` `vehicle_state` | 0 | ✅ normal |
| `/limo_status` `control_mode` | 1 | acepta comandos ROS (modo command) |
| `/limo_status` `battery_voltage` | **11.2 V** | ⚠️ algo bajo (nominal ~11–12.6 V); vigilar para pruebas largas |
| `/limo_status` `motion_mode` | **0 = MODE_FOUR_DIFF** (por `limo_protocol.h:98`) | ✅ **diferencial 4 ruedas, como se esperaba** (valor leído del hardware real) |

## 2. LiDAR (YDLiDAR, `/scan`)

| Señal | Valor | Estado |
|---|---|---|
| Publisher | 1 (`/ydlidar_ros2_driver_node`), Sub 0 | ✅ es el nuestro |
| Frecuencia | ~10.2 Hz (std ~1.2 ms) | ✅ estable |
| `frame_id` | `laser_frame` | ✅ |
| FOV | angle_min/max ∓1.9199 rad (±110°, ~220°) | ✅ |
| Haces / scan | 250 (angle_increment 0.015362 rad) | ✅ |
| **Retornos válidos** | **217 / 250 = 86.8%** (inf=0, nan=0, cero=33) | ✅ sano (33 sin retorno/max-range) |
| `range_min / max` | 0.03 / 64.0 m (máx nominal driver) | ✅ |
| `scan_time` | ~0.106 s | ✅ |

## 3. TF

| Transform | Valor medido | Esperado | Estado |
|---|---|---|---|
| `odom → base_link` | identidad [0,0,0] (robot en origen) | dinámica del driver | ✅ (1 publisher, sin duplicados) |
| `base_link → laser_frame` | **[0.180, 0.000, −0.090]**, sin rotación | (0.180, 0, −0.090) | ✅ **coincide exactamente** |
| `base_link → imu_link` | [0.000, 0.000, 0.000], sin rotación | (0,0,0) | ✅ |
| `/tf` publishers | 1 (driver base) | 1 | ✅ sin duplicados |
| `/tf_static` publishers | 2 (imu_link + laser_frame) | 2 | ✅ sin duplicados |

## 4. Notas / hallazgos

- **Aislamiento de dominio necesario:** el sistema AiNex (robot humanoide de fútbol) reaparece en `ROS_DOMAIN_ID=5` y llegó a publicar `/cmd_vel` activamente. Toda esta fase se ejecutó en `ROS_DOMAIN_ID=17` para evitar que el LIMO ejecute comandos ajenos. **Recordatorio operativo:** para cualquier sesión futura del LIMO, aislar el dominio o apagar el AiNex de forma persistente.
- **Batería 11.2 V**: usable pero no llena; conviene cargar antes de pruebas de movimiento.
- **LiDAR: driver del proyecto = YDLiDAR** con TF calibrada, confirmado en vivo (no el `urg_node2` del fabricante).
- **`/imu`** (no `/imu/data`): el driver publica en `/imu`. Verificar que Cartographer/AMCL, si se usan, consuman el topic correcto.

---

*FASE 1 completada en dominio aislado. Solo sensores + driver base (sin movimiento). `/cmd_vel` Publisher=0 durante toda la fase. No se movió el robot. `~/limoatwork` solo lectura.*
