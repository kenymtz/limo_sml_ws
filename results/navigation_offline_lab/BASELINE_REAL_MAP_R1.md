# BASELINE_REAL_MAP_R1 — NAVIGATION_OFFLINE_LAB_R1

Fecha: 2026-09-29. Dataset: `results/maps/arena_20260918_1900/` (Cartographer + LIMO real, 2026-09-18).
Modo: offline, solo lectura. Sin hardware y sin movimiento. No se tocó Nav2, el footprint, el planner, el controller ni AMCL.

Leyenda: **[C]** CONFIRMADO (medido en esta sesión) · **[E]** EVIDENCIA · **[H]** HIPÓTESIS

Documentos: `DATASET_REPORT.md`, `MAP_ANALYSIS_R1.md`.
Scripts: `~/limo_sml_ws/scripts/nav_offline_lab/`
- `common.py`
- `dataset_audit.py`
- `extract_trajectory.py`
- `diag_motion_consistency.py`
- `diag_odom_tracking.py`
- `map_analysis.py`
- `verify_map_server_load.sh`

---

## 1. Trayectoria real (FASE 3)

Archivos:
- `real_trajectory.csv`: 170 274 filas, todas las muestras de /odom a 50 Hz. Columnas: `t_rel_s, stamp_s, odom_x/y/yaw, map_x/y/yaw, v_x, w_z, map_odom_age_s`.
- `real_trajectory.png`
- `cmd_vs_odom.png`
- `motion_consistency.png`
- `trajectory_metrics.json`, `odom_tracking.json`, `motion_consistency.json`

Pose en map = T(map→odom de /tf, Cartographer) ∘ /odom, usando el map→odom más reciente con stamp ≤ stamp de odom.

### 1.1 Métricas [C]
| Métrica | Valor |
|---|---|
| Duración del bag | 3 408.9 s (56.8 min) |
| Ventana activa (primer → último movimiento) | t = 1 413.7 → 2 898.2 s (1 484.6 s) |
| Tiempo realmente en movimiento (\|v\| > 0.01 o \|w\| > 0.02) | **34.6 s** |
| Distancia recorrida (odom) | **3.05 m** (decimada a 1 s: 2.98 m; twist integrado: 3.05 m) |
| Velocidad media en la ventana activa | 0.002 m/s |
| Velocidad media mientras se mueve | 0.088 m/s |
| Velocidad máxima (twist) | 0.267 m/s (por diferenciación 0.2 s: 0.248 m/s) |
| Velocidad angular máxima | 0.553 rad/s |
| Yaw acumulado \|Δyaw\| (pose odom) | 10.02 rad (1.59 vueltas); con twist integrado: 7.80 rad |
| Yaw neto | 5.55 rad |
| Tramos de movimiento | 56 (pulsaciones de teleop de ~0.5–0.7 s) |
| Detenciones ≥ 1 s dentro de la ventana activa | 32 (mín 1.2 s, mediana 3.7 s, máx 1 310 s) |
| BBox de la trayectoria en map | x ∈ [−0.04, 1.02], y ∈ [−0.57, 0.38] m |

- Movimiento sostenido solo en t ≈ 2 745–2 900 s.
- Evento aislado en t ≈ 1 413.7–1 416 s: unos 2 s de movimiento, un pico w de −0.25 rad/s, un salto de map→odom de ~0.08 m y un cambio de escena en el LiDAR.

### 1.2 ¿Se movió realmente tan poco? [C]
Contraste con una señal independiente de la odometría, la mediana del cambio de /scan a 1 s:
- con odom quieta: 3 mm (p95 5 mm);
- con odom en movimiento: 141 mm;
- no hay ningún intervalo ≥ 2 s con el scan cambiando > 5 cm y la odom quieta.

→ **Confirmado: el robot recorrió ~3 m en total.** El mapa de 11.5 × 10 m viene del alcance del LiDAR, no de un barrido del área.

### 1.3 /cmd_vel vs /odom [C salvo indicación]
- /cmd_vel tiene 125 mensajes, solo en t ∈ [2 110, 2 897] s.
  - Valores discretos: v ∈ {0, 0.239}, w ∈ {0, ±0.478}.
  - Tiempo entre mensajes: mediana 37 ms (auto-repetición de tecla), p90 4.8 s.
- **La base no mantiene el último comando.** Con el comando de edad 0.6–2 s, el ratio odom/cmd es 0, contra 1.0 con edad 0.15–0.6 s.
  - [E] Hay un timeout de comando en la base de ~0.5–0.6 s: cada pulsación produce un tramo de ~0.6 s.
  - [H] El valor exacto sale de `limo_base`; no se leyó.
- Seguimiento en régimen (edad de comando 0.15–0.6 s):
  - v_odom/v_cmd = **1.02 / 0.99**;
  - w_odom/w_cmd = **0.92 / 0.99** (medianas por ventana).
- Transitorio (edad < 0.15 s): v al 28 % del comando.
  - Latencia del primer comando no nulo al primer movimiento: 47 ms.
  - [E] Hay rampa de aceleración.
- [E] Hubo movimiento sin /cmd_vel grabado: 2.1 s en t ≈ 1 413 s, antes del primer cmd.
  - El README indica que /cmd_vel "se añade al iniciar teleop", así que la ausencia de mensajes no prueba ausencia de comando.
  - [H] Pudo ser una manipulación manual o un comando no grabado.
- **No hay comparación cmd vs odom posible fuera de t ∈ [2 110, 2 897] s.**

### 1.4 Odometría vs SLAM: discrepancia [C medida · causa H]
- map→odom (la corrección de Cartographer) crece de 0.06 m a **0.56 m** durante el tramo en movimiento (t ≈ 2 780–2 850 s). Rango total: 0.59 m y 0.36 rad.
- Distancia en map, decimada a 1 s: **9.49 m**, contra **2.98 m** de la odometría.
  - Sin decimar da 45.5 m, dominado por jitter: 267 saltos de map→odom > 1 cm, 16 > 5 cm, el máximo de 0.22 m.
  - **La distancia en frame map no es una métrica válida de recorrido.**
- Yaw final: odom −0.731 rad, map −0.745 rad (consistentes). La posición final difiere unos 0.56 m sobre 3 m recorridos (~18 %).
- Yaw de la pose odom (10.0 rad acumulado) frente al twist integrado (7.8 rad): la pose publicada no es la integración simple del twist publicado.
  - [H] `limo_base` podría fusionar el IMU o usar otro modelo cinemático (MODE_FOUR_DIFF, skid-steer).
- [H] No se puede decidir offline cuál es más fiel, odometría o SLAM, sin ground truth. Candidatos:
  - (a) deslizamiento skid-steer en giros en el sitio;
  - (b) scan matching inestable con poca excitación: pulsaciones cortas y escena casi estática;
  - (c) ambos.

---

## 2. ¿Qué tenemos CONFIRMADO?
1. El dataset está íntegro y es legible offline: bag cerrado limpio, pbstream estructuralmente completo, yaml ↔ pgm coherentes, originales sin modificar (checksums).
2. Los sensores son estables durante 57 min:
   - /scan 10.04 Hz, /odom 49.95 Hz, /imu 99.9 Hz, map→odom ~95 Hz;
   - sin huecos > 0.15 s;
   - error_code 0 y batería 11.8–12.5 V.
3. El TF base_link→laser_frame es [0.18, 0, −0.09] y base_link→imu_link es identidad.
4. El LiDAR tiene 250 rayos en ±110°, con ~68 % de rayos válidos en /scan y ~56 % en /scan_filtered.
5. La base sigue a /cmd_vel con un ratio ≈ 0.92–1.02 en régimen. Tiene timeout de comando (~0.6 s, [E]) y latencia de arranque ~50 ms.
6. **El robot recorrió solo ~3 m, en ~35 s de movimiento, dentro de un área de ~1 × 1 m.**
7. **El mapa es radial desde un solo punto, no un mapa de arena**: el 85 % de la frontera del espacio libre es desconocido y no hay perímetro cerrado.
8. **Con el `map.yaml` actual, Nav2 carga el 71.5 % desconocido como libre.** Verificado con nav2_map_server real en aislamiento.
9. La odometría y Cartographer discrepan ~0.56 m en posición tras 3 m de recorrido.

## 3. ¿Qué se puede evaluar SIN robot?
- **Pipeline de mapas**: carga correcta en Nav2 (umbrales), costmaps estáticos, inflado con el footprint real y búsqueda de rutas sobre mapas. Válido como test de herramientas, no de la arena.
- **Replay offline de Cartographer** con el bag, con distintas configuraciones y comparando map→odom y la calidad del mapa. El bag trae /scan, /odom, /imu y /tf completos. (Excluido de R1 por instrucción.)
- **Replay offline de AMCL** con el bag sobre un mapa. Técnicamente posible, pero sin valor sobre este mapa, que no cubre el área (excluido de R1).
- **Característica de la base**: seguimiento cmd→odom, rampa, timeout y latencia. Solo para v ∈ {0.239} y w ∈ {±0.478}, un dataset pobre para identificación.
- **Auditar otros mapas** (v1, run3) con los mismos scripts, en especial el problema de `free_thresh`.
- **Propiedades del LiDAR**: tasa de rayos válidos, efecto del filtro y ruido estático (hay ~45 min de datos con el robot quieto, ideales para caracterizar ruido).

## 4. ¿Qué requiere obligatoriamente el robot físico?
- **Un mapeo real de la arena** con un barrido completo del área y lazos cerrados. Es imprescindible y el dataset actual no lo cubre.
- Ground truth de posición (marcas medidas en el piso) para decidir entre odometría y SLAM (§1.4).
- La identificación de la base a otras velocidades, curvas combinadas v+w y el timeout exacto.
- Cualquier validación de navegación, localización en marcha o docking.

Todo lo anterior requiere autorización explícita de movimiento.

## 5. Próximos pasos de mayor valor (ordenados)
1. **Corregir la carga del mapa [bloqueante, offline, bajo riesgo]**:
   - generar una copia derivada del yaml con `free_thresh: 0.196`, sin tocar el original;
   - verificar con `verify_map_server_load.sh` que 205 → −1;
   - auditar los yaml de v1 y run3 y de los launch/params que los usan.
2. **Planificar un nuevo mapeo de arena** (requiere robot y autorización): recorrido que cubra todo el perímetro y cierre el lazo, a velocidad constante y baja (sin pulsaciones cortas), grabando /cmd_vel desde el inicio y con marcas de ground truth en el piso.
3. **Replay offline de Cartographer sobre este bag** (R2) para aislar si la discrepancia odom/SLAM viene del scan matching. Barato y sin robot.
4. Caracterizar el ruido estático del LiDAR con los ~45 min en reposo (offline).
5. Solo después: AMCL replay y rutas con footprint SE2 sobre un mapa válido de arena.

## 6. Problemas encontrados (resumen)
| # | Problema | Nivel | Severidad |
|---|---|---|---|
| P1 | `free_thresh 0.25` hace que Nav2 cargue el desconocido (205) como libre | C | **Alta** |
| P2 | El mapa no es de arena: radial, 1 punto de vista, sin perímetro cerrado | C/E | **Alta** |
| P3 | El robot solo recorrió ~3 m y el dataset no ejercita el SLAM | C | Alta |
| P4 | Odometría vs SLAM difieren 0.56 m / 3 m; map→odom con jitter de hasta 0.22 m | C (causa H) | Media |
| P5 | /cmd_vel solo grabado desde t = 2 110 s; hubo movimiento en t ≈ 1 413 s sin comando grabado | C/H | Baja |
| P6 | El README dice "mapeo completado" sin mencionar que el recorrido fue mínimo | E | Media (documental) |
| P7 | Trayectoria SLAM a 9 cm de celdas ocupadas (0.9 % de muestras < r_ins) | C (causa H) | Baja |
