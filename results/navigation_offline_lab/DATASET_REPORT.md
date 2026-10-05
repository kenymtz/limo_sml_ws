# DATASET_REPORT — arena_20260918_1900 (NAVIGATION_OFFLINE_LAB_R1)

Fecha análisis: 2026-09-29. Modo: **100 % offline, solo lectura**. Sin hardware, sin stack ROS del robot.

Dataset: `~/limo_sml_ws/results/maps/arena_20260918_1900/`
Script: `scripts/nav_offline_lab/dataset_audit.py` → `dataset_audit.json`
Integridad originales: `checksums_before.sha256` (sha256 de map.yaml/pgm/pbstream/metadata + md5 del .db3), re-verificado al final: **sin cambios**. El bag se abrió con sqlite `mode=ro&immutable=1` (no crea -journal/-wal).

Leyenda: **[C]** CONFIRMADO (medido) · **[E]** EVIDENCIA (indicio medido, interpretación) · **[H]** HIPÓTESIS

---

## Veredicto

| Artefacto | Estado | Motivo |
|---|---|---|
| map.yaml / map.pgm | **PASS** (integridad) / **FAIL** (carga en Nav2) | Archivos coherentes, pero `free_thresh: 0.25` hace que Nav2 cargue el desconocido como libre (ver §1.3) |
| map.pbstream | **PASS** (integridad básica) | Estructura de chunks válida hasta EOF; contenido no deserializado |
| rosbag | **PASS** | Cierre limpio, 8 topics, tasas estables, sin huecos en sensores |

---

## 1. Mapa

### 1.1 Parámetros [C]
| Campo | Valor |
|---|---|
| map.yaml | existe (121 B) |
| map.pgm | existe (128 367 B), P5, maxval 255 |
| mode | trinary |
| resolution | 0.03 m/px |
| width × height | 382 × 336 px |
| dimensiones físicas | 11.46 × 10.08 m |
| origin | [-3.32, -4.88, 0] |
| extensión | x ∈ [-3.32, 8.14], y ∈ [-4.88, 5.20] m |
| occupied_thresh | 0.65 |
| free_thresh | 0.25 |
| negate | 0 |

### 1.2 Coherencia yaml ↔ pgm [C]
- `image: map.pgm` existe; payload = 128 352 B = 382 × 336 exacto.
- Valores de píxel presentes: solo {0: 1 908, 205: 91 768, 254: 34 676}, que corresponden a la salida trinary de `map_saver`.

### 1.3 PROBLEMA CRÍTICO: el desconocido se carga como libre [C]
- La fórmula de nav2_map_server es occ = (255 − px)/255. Para px 205 da occ = 0.196, que es **< free_thresh 0.25**, así que la celda se marca **LIBRE**.
- **Verificado empíricamente** con el `nav2_map_server` real en modo aislado (`scripts/nav_offline_lab/verify_map_server_load.sh`, `ROS_LOCALHOST_ONLY=1`, `ROS_DOMAIN_ID=99`; solo publica un OccupancyGrid):
  `{"w":382,"h":336,"res":0.03,"values":{"0":126444,"100":1908}}`. El resultado tiene **0 celdas -1**: las 91 768 celdas desconocidas (71.5 %) llegan como libres.
- Impacto [E]: con este yaml, AMCL (el likelihood field y la generación de partículas en espacio "libre"), el global costmap y el planner tratarían como transitable todo lo no observado.
- Corrección candidata [H, no aplicada]: usar una **copia derivada** del yaml con `free_thresh` ≤ 0.196 (por ejemplo 0.196, el default de ROS1). Así el 205 da 0.196, que no es < 0.196, y queda desconocido. Hay que verificarlo con el mismo script. El original no se modifica.
- Queda por revisar [H]: otros mapas del proyecto (v1, run3) guardados con `map_saver_cli` podrían tener el mismo problema.

## 2. map.pbstream [C]
| Campo | Valor |
|---|---|
| tamaño | 19 791 278 B |
| fecha (mtime) | 2026-09-18 19:55:27 -0300 |
| magic Cartographer | `0x7b1d1f7b5bf501db` OK |
| chunks (uint64 len + gzip) | 172 658, todos con cabecera gzip `1f8b` |
| cadena de chunks | termina **exactamente** en EOF (sin truncado) |

No verificado: la deserialización protobuf, la cantidad de submapas/nodos y la trayectoria interna. Requiere las herramientas de Cartographer, que no se ejecutaron (fuera de alcance R1).

## 3. Rosbag [C]
- `bag/bag_0.db3`: 574 074 880 B, sqlite3, formato rosbag2 v5.
- Duración: **3 408.9 s (56.8 min)**. Inicio 2026-09-18 ~19:01. Mensajes: 1 063 508.

| Topic | Tipo | Msgs | Hz media | dt máx | Notas |
|---|---|---|---|---|---|
| /scan | LaserScan | 34 233 | 10.04 | 0.147 s | frame `laser_frame`, 250 rayos, ±110° (±1.9199 rad), inc 0.01536, range [0.03, 64] m, ~68 % de rayos válidos |
| /scan_filtered | LaserScan | 34 233 | 10.04 | 0.146 s | misma geometría, ~56 % válidos (el filtro descarta ~12 pp más) |
| /odom | Odometry | 170 274 | 49.95 | 0.089 s | `odom→base_link`, retardo recepción−stamp mediana 0.3 ms |
| /imu | Imu | 340 549 | 99.90 | 0.072 s | `imu_link`, \|a\| mediana 9.98 m/s² |
| /tf | TFMessage | 477 281 | 140 | 0.060 s | `map→odom` 307 006 (~90 Hz, Cartographer) y `odom→base_link` 170 275 |
| /tf_static | TFMessage | 2 | — | — | `base_link→imu_link` [0,0,0]; `base_link→laser_frame` [0.18, 0, −0.09] |
| /cmd_vel | Twist | 125 | ráfagas | 631.8 s | solo en t ∈ [2110, 2897] s; v ∈ {0, 0.239}, w ∈ {0, ±0.478}, vy = 0 |
| /limo_status | LimoStatus | 6 811 | 2.0 | 0.55 s | vehicle_state 0, control_mode 1, motion_mode 0, error_code 0; batería 11.8–12.5 V (11.9 → 12.0) |

- Sin huecos > 0.5 s en /scan, /odom, /imu ni /tf [C]. Los 5 728 "gaps > 0.5 s" de /limo_status son su período nominal de 0.5 s, no un fallo.
- El TF de LiDAR grabado coincide con README (`[0.180, 0, −0.090]`) [C].
- El bag **no se reprodujo**. Solo se leyó.

## 4. Hallazgo sobre el contenido del bag (detalle en BASELINE)
- [C] El robot **estuvo casi siempre quieto**:
  - 34.6 s en movimiento dentro de 56.8 min.
  - 3.05 m recorridos según odometría.
  - Movimiento sostenido solo en t ≈ 2745–2900 s.
- [C] Lo confirma una señal independiente de la odometría: la mediana de Δscan a 1 s es 3 mm con odom quieta y 141 mm con odom en movimiento, y no hay intervalos ≥ 2 s con el scan cambiando y la odom quieta.
- Consecuencia [E]: el mapa se construyó casi desde un único punto de vista. Ver `MAP_ANALYSIS_R1.md`.
