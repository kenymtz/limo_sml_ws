# PLAN DE MAPEO DEL ENTORNO ACTUAL — LIMO RoboCup SML

Fecha: 2026-09-18
**Estado: PROPUESTA. NO EJECUTADO.** Este documento prepara el mapeo del entorno físico actual (el mapa cargado hoy, `map.pgm`, NO corresponde a este entorno). El mapeo **requiere movimiento físico** del robot y por tanto **autorización explícita nueva** antes de iniciarse.

Contexto de seguridad heredado: trabajar SIEMPRE en `ROS_DOMAIN_ID=17` (aislado del AiNex, que contamina el dominio 5 y llega a publicar `/cmd_vel`). `~/limoatwork` estrictamente solo lectura — nada se guarda ahí.

---

## 1. Qué SLAM usar

**Recomendado: Cartographer** (`cartographer_ros`), por dos razones: (a) es el que generó todo el baseline histórico (incl. `stage3`), lo que preserva comparabilidad; (b) puede exportar **`.pbstream`** (estado serializado del pose-graph), clave para no volver a perder el mapa.

**Config `.lua`: usar la del fabricante `limo_lds_2d.lua` (correlative ON + filtro 2 etapas)** — es el default del launch, baseline-consistente. **NO** usar `limo_lds_2d_no_correlative.lua` del proyecto (el propio test `slam_ab_r1` recomendó no adoptarla). *Esta elección conviene confirmarla con el equipo (decisión abierta, ver `AUDITORIA_CARTOGRAPHER_COMPLETA.md`).* No se modifica ningún `.lua`.

**Alternativa:** `slam_toolbox` (`limo_slam/launch/slam_toolbox.launch.py`) — guarda pose-graph serializado y mapa; más simple de guardar, pero rompe comparabilidad con el baseline Cartographer.

## 2. Launch exacto necesario

Con **base + LiDAR ya levantados** (`limo_base.launch.py` + `lidar_real.launch.py`, ya auditados SAFE), y **localización/Nav2 DETENIDOS** (ver §3):

```
ros2 launch limo_bringup cartographer.launch.py
# defaults: configuration_basename=limo_lds_2d.lua, use_scan_filter=true, resolution=0.03, open_rviz=true
```

Nota: este launch **incluye su propio filtro** (`cartographer_scan_filter_chain`, /scan→/scan_filtered). No correr otro filtro en paralelo.

## 3. Auditoría del launch (obligatoria, hecha por lectura)

`cartographer.launch.py` arranca: `cartographer_scan_filter_chain` (filtro 2 etapas), `cartographer_node` (SLAM), `occupancy_grid.launch.py` (publica `/map`), `rviz2` (open_rviz=true).

- **¿Publica `/cmd_vel`? NO.** **¿Envía goals/acciones de movimiento? NO.** **¿Activa controladores/actuadores? NO.** SLAM es **pasivo**: consume `/scan_filtered` + `/odom` + `/imu` y publica `/map` + TF `map→odom`.
- **Conflicto crítico:** publica **`map→odom`**, igual que AMCL. **Antes de mapear hay que DETENER la localización/Nav2** (amcl, map_server, y el stack de inspección) para que no haya dos publishers de `map→odom`.
- **El movimiento no lo genera este launch:** para explorar y construir el mapa, el robot debe ser **conducido manualmente** (teleop humano) → esa es la parte que requiere **autorización de movimiento** y precauciones (§6–§9).
- Veredicto: **SAFE TO START (no genera movimiento autónomo)**, condicionado a detener antes la localización.

## 4. Topics a grabar en rosbag (esta vez SÍ, para no repetir la pérdida)

Grabar **desde antes de empezar a mover** y durante todo el recorrido:
```
ros2 bag record -o ~/limo_sml_ws/results/maps/arena_YYYYMMDD_HHMM/bag \
  /scan /odom /imu /tf /tf_static
```
(Estos 5 son suficientes para reconstruir el mapa offline con Cartographer si algo falla. Opcional añadir `/scan_filtered`.)

## 5. Guardado OBLIGATORIO al finalizar (antes de bajar nada)

Guardar en `~/limo_sml_ws/results/maps/arena_YYYYMMDD_HHMM/` (**NUNCA en `~/limoatwork`**):

1. **`.pgm` + `.yaml`** (OccupancyGrid actual):
   ```
   ros2 run nav2_map_server map_saver_cli -f ~/limo_sml_ws/results/maps/arena_YYYYMMDD_HHMM/map
   ```
2. **`.pbstream`** (estado Cartographer; permite retomar/reprocesar):
   ```
   ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}"
   ros2 service call /write_state cartographer_ros_msgs/srv/WriteState \
     "{filename: '/home/limopro/limo_sml_ws/results/maps/arena_YYYYMMDD_HHMM/map.pbstream', include_unfinished_submaps: true}"
   ```
3. Verificar que los 3 archivos existen y pesan >0 **antes** de cerrar Cartographer.

## 6. Velocidad máxima propuesta

- Lineal: **≤ 0.15 m/s** (por debajo del `max_vel_x=0.25` de navegación; mapeo lento = mejor scan matching).
- Angular: **≤ 0.4 rad/s** (giros suaves; los giros rápidos degradan Cartographer).
- Batería hoy en **11.2 V** → cargar antes; evitar mapear con batería baja (afecta odometría).

## 7. Recorrido inicial más pequeño posible

Primer mapeo mínimo de validación: **un bucle cerrado corto** alrededor del punto de inicio (perímetro de ~2×2 m o una sola sala pequeña), **volviendo al punto de partida** para forzar un loop-closure. Objetivo: validar el pipeline de mapeo+guardado end-to-end con el menor recorrido posible, **antes** de mapear la arena completa.

## 8. Condiciones de ABORT

Abortar el mapeo (detener conducción, cerrar Cartographer sin sobrescribir un mapa bueno previo) si:
- El robot se acerca a <0.30 m de un obstáculo/persona.
- El mapa en RViz "salta"/se duplica (fallo de scan matching / loop-closure erróneo).
- `/scan` cae por debajo de ~8 Hz o se congela.
- Aparece cualquier publisher inesperado en `/cmd_vel` (ver §9).
- La TF `map→odom` diverge visiblemente del entorno real.
- Batería <10.5 V.
- Cualquier duda de seguridad física.

## 9. Verificación de exclusividad de `/cmd_vel` (antes y durante)

- **Antes de habilitar teleop:** `ros2 topic info /cmd_vel -v` → confirmar **exactamente 1 subscriber** (el driver `limo_base`) y **0 publishers**.
- **Al iniciar teleop:** debe aparecer **exactamente 1 publisher** (el nodo de teleop) y ningún otro. Si aparece >1 publisher → **ABORT** (posible contaminación / segundo comandante).
- Confirmar dominio limpio: `ros2 topic list | grep -c -E 'ainex|soccer|clock'` = 0.
- El teleop debe ser el **único** emisor; nada de Nav2/recoveries corriendo en paralelo.

## 10. Cómo evitar repetir la pérdida del mapa `stage3`

- Grabar rosbag (§4) **desde el inicio** → aunque el guardado falle, el mapa es reconstruible offline.
- Guardar en `~/limo_sml_ws/results/maps/` (persistente, versionable), **jamás en `/tmp`** ni en `~/limoatwork`.
- Guardar los **3 formatos** (§5) y verificar tamaño >0 antes de cerrar.
- Inmediatamente después, **commitear/respaldar** los archivos (git en `limo_sml_ws`, o copia externa) — `stage3` se perdió por no persistir/versionar.
- Anotar en un `README` de la carpeta: fecha, config `.lua` usada, recorrido, y hash del rosbag.

---

## Procedimiento de transición (localización → mapeo), cuando se autorice

1. Detener el stack de inspección Nav2, `amcl`, `map_server` (dejar corriendo base + LiDAR).
2. Confirmar `map→odom` sin publisher (AMCL detenido).
3. Iniciar rosbag (§4).
4. Lanzar `cartographer.launch.py` (§2), verificar `/map` y TF `map→odom` de Cartographer.
5. Verificar exclusividad `/cmd_vel` (§9).
6. **[REQUIERE AUTORIZACIÓN DE MOVIMIENTO]** Conducir el recorrido mínimo (§7) a velocidad ≤§6, vigilando §8.
7. Cerrar el bucle, detener conducción.
8. Guardar los 3 formatos (§5) y verificar.
9. Respaldar (§10).

---

**ESPERANDO AUTORIZACIÓN EXPLÍCITA PARA INICIAR MAPEO FÍSICO**
