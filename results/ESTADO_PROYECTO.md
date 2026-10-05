# Estado técnico consolidado — LIMO RoboCup SML

Fecha del resumen: 2026-09-17
Fuente: CLAUDE.md, `config/`, `src/`, `results/baseline/`, `results/footprint_analysis/` (workspace `~/limo_sml_ws`), e inspección de solo lectura de `~/limoatwork`.

---

## 1. Estado actual del sistema

- Plataforma: AgileX LIMO, ROS2 Humble.
- Objetivo: navegación autónoma para RoboCup Small Size / SML, basada en mapa conocido (known map navigation), con SLAM/AMCL y LiDAR como observables reales. Manipulación tratada como sistema separado (no evaluado en este resumen).
- Workspace de competición: `~/limo_sml_ws`. Solo contiene 2 paquetes fuente propios:
  - `limo_base` (C++): driver de bajo nivel (serial, protocolo motor).
  - `limo_sml_tools` (Python): herramientas propias (`mission_loader.py`, `robot_snapshot.py`).
- Los paquetes del stack de navegación (`limo_bringup`, `limo_description`, `limo_slam`, `limo_mission`, `limo_map_saver`, `limo_multirobot`, `limo_msgs`, `limo_mission_msgs`, `limo_apriltag_tracker`) **no están en `limo_sml_ws`**, viven en `~/limoatwork` (inmutable, solo lectura). La personalización de este proyecto ocurre vía archivos de configuración en `limo_sml_ws/config/` que se inyectan a esos nodos.
- Nav2 configurado y con evidencia de funcionamiento parcial: `amcl`, `map_server`, `controller_server`, `planner_server`, `bt_navigator` confirmados activos en pruebas previas (según CLAUDE.md). No hay en este workspace evidencia de una prueba end-to-end de navegación completa (planner + controller siguiendo una ruta real).

## 2. Componentes validados

- **Cinemática real del robot**: `MODE_FOUR_DIFF` (diferencial puro). `linear.y` no soportado. Confirmado en `results/baseline/base_motion_r1/RESULTS.txt`.
- **Driver de yaw corregido**: se eliminó un deadband de 0.1°/muestra y una inicialización defectuosa del driver original. Validado en pruebas angulares MOTION-06 (+90°) y MOTION-07 (−90°), error ≤0.1°. PASS.
- **Escala lineal del odom**: validada en MOTION-08/09/10 (objetivo 50 cm, error ~1 cm, sin sesgo lateral fijo). No se aplicó ni se recomienda factor de calibración lineal.
- **Modelo AMCL**: corregido de `OmniMotionModel` a `DifferentialMotionModel`, coherente con la cinemática real confirmada.
- **Parámetros DWB (controller_server)**: corregidos para reflejar robot no-holonómico (`holonomic: False`, `max/min_vel_y: 0.0`, `vy_samples: 1`) — antes configurados incorrectamente para mecanum.
- **TF `base_link → laser_frame`**: medido y estable: traslación (0.180, 0.000, −0.090) m, sin rotación. Confirmado en `stage2_calibrated_lidar`.
- **SLAM (Cartographer) — TF `map → base_link`**: estable, error de orden 1-3 cm en XY, <1° en yaw (`slam_ab_r1`).

## 3. Pruebas realizadas y resultados

| Prueba | Resultado | Notas |
|---|---|---|
| BASE-MOTION-R1 | **PASS** | Giros ±90°, desplazamientos lineales 50 cm, driver de yaw corregido validado en ambos sentidos. |
| SLAM A/B/C (`slam_ab_r1`) | **PASS** (parcial, hipótesis no confirmada) | Comparó online correlative scan matching ON/OFF y scan RAW vs filtrado. Ver hallazgos. |
| Análisis offline de footprint (`results/footprint_analysis/`) | Completado sobre 2 mapas | Ver sección 4. No involucró hardware ni ROS en ejecución. |
| Navegación Nav2 end-to-end | **No hay evidencia registrada en este workspace** | Falta un baseline documentado de planner+controller siguiendo una ruta real. |

## 4. Hallazgos importantes

1. **Discrepancia de footprint no justificada.** Existen dos valores de footprint en el historial de `config/navigation/`:
   - *Medido* (`base_motion_r1`, respaldado por prueba física): `[[-0.16,-0.10],[-0.16,0.10],[0.20,0.10],[0.20,-0.10]]`, padding 0.03 → 36×20 cm.
   - *Actual* (`navigation2_real.yaml` en producción, documentado en CLAUDE.md): `[[-0.17,-0.16],[-0.17,0.16],[0.22,0.16],[0.22,-0.16]]`, padding 0.02 → 39×32 cm.
   - El cambio al valor grande no tiene ninguna prueba física de respaldo en el repo, y el nombre del backup (`pre_footprint_real`) sugiere que se pensó como corrección hacia "lo real", pero contradice la única medición física documentada (4 horas antes, misma sesión del 2026-09-15). Viola la propia regla de CLAUDE.md: "Nunca modificar footprint sin justificar."

2. **Impacto cuantificado del footprint grande (análisis offline, 2 mapas de `limoatwork/.../limo_bringup/maps/`):**
   - `map.pgm` (res. 0.03 m, candidato a mapa "v1"/"run3"): pérdida de 8.2% del espacio libre transitable (0.82 m²).
   - `map_20260718_185606.pgm` (res. 0.05 m): pérdida de 8.9% del espacio libre transitable (1.23 m²).
   - En ambos mapas, la pérdida se manifiesta como un anillo de ~10 cm de margen alrededor de obstáculos y paredes, **no** como bloqueo total de corredores — ningún mapa de prueba disponible contiene un pasillo de ancho entre 0.26 m y 0.36 m que permita confirmar o descartar bloqueo real de corredores de competencia.
   - **No se localizó ningún archivo llamado literalmente "v1" o "run3"** en el sistema (fuera de `limoatwork`, `miniconda3`, `.vscode-server`, que son ruido). Los nombres reales existentes en `limoatwork/src/limo_ros2/limo_bringup/maps/` son: `map1.pgm`, `map_20260718_185606.pgm`, `map_20260720_160000.pgm`, `map_20260723_193027.pgm`, `map_20260723_195338.pgm`, `map.pgm` (este último con resolución 0.03 m, coincide con la nota de CLAUDE.md sobre mapas de evaluación, y es el modificado más recientemente).

3. **Sesgo lateral sistemático en SLAM (Cartographer).** En todos los tests de `slam_ab_r1`, `MAP - ODOM` en el eje lateral es consistentemente positivo (+1.4 a +2.8 cm), independientemente de si el online correlative scan matching está ON u OFF. Desactivarlo **no elimina** el sesgo y **empeora** la discrepancia longitudinal en algunos tests → no adoptar esa configuración. Usar scan RAW en vez de filtrado (`scan_filtered`) reduce la discrepancia promedio (0.895 cm vs 1.605 cm en avance) pero no la elimina, y solo se probó con 2 repeticiones (C1/C2).

4. **No existen bags útiles para validación con datos reales de sensores.** Los bags existentes en `~/bags/` (`volta_quadra*`) solo contienen `/cmd_vel` (teleop del 2026-08-20), sin `/scan`, `/map` ni `/tf` — no sirven para revalidar footprint, SLAM ni localización de forma offline.

## 5. Riesgos abiertos

- **Footprint de producción no validado físicamente** (hallazgo #1) — riesgo de planificación subóptima (rutas innecesariamente conservadoras) si el valor grande es un error; o riesgo de colisión si el valor medido (más pequeño) resulta ser el correcto y se revirtiera sin margen adicional.
- **Sesgo lateral de SLAM sin causa raíz confirmada** — si el mapa generado por Cartographer se usa como mapa de producción para AMCL, un sesgo sistemático de ~2 cm puede acumularse o interactuar mal con el error de footprint en pasillos con poco clearance.
- **Identidad de los mapas "v1 producción" y "run3 full arena" sin confirmar** — se trabajó con 2 mapas candidatos por heurística (fecha, resolución, tamaño), no confirmados por el equipo. Cualquier conclusión sobre corredores/clearance de la arena real queda condicionada a esta incertidumbre.
- **Dependencia de `~/limoatwork` para paquetes core de navegación** (bringup, description, slam) — cambios futuros en ese árbol (fuera del control de este workspace) podrían romper compatibilidad con los yaml de `limo_sml_ws/config` sin aviso.
- **Sin prueba end-to-end de Nav2 documentada** — no hay evidencia de que el stack completo (planner+controller+recoveries) haya sido validado siguiendo una ruta real en el robot físico.
- **`config/missions/test_arena.yaml` es un placeholder** — una sola estación (`WS01`) en el origen (0,0), no representa una misión real de competencia.

## 6. Decisiones que NO deben tomarse todavía

- **No modificar el footprint de `navigation2_real.yaml`** hasta confirmar con el equipo cuál de los dos valores (medido vs actual) es el correcto, idealmente con una re-medición física del robot.
- **No adoptar la configuración SLAM-B** (online correlative scan matching OFF) — ya evaluada y descartada explícitamente en `slam_ab_r1/RESULTS.txt`.
- **No cambiar `scan_topic` de `/scan_filtered` a scan RAW** en AMCL/costmaps basándose solo en los 2 tests C1/C2 — se requieren más repeticiones antes de decidir.
- **No asumir que los mapas `map.pgm` / `map_20260718_185606.pgm` son "v1" o "run3"** — son candidatos por heurística, no confirmados.
- **No dar por válido `test_arena.yaml`** como misión real de competencia.
- **No ejecutar navegación física ni pruebas de movimiento** sin autorización explícita, conforme a las reglas de seguridad del proyecto.

## 7. Próximas tareas (orden de prioridad)

1. **Confirmar con el equipo la identidad real de los mapas "v1"/"run3"** y, si es necesario, localizar o regenerar el mapa oficial de la arena completa — es el bloqueador principal para responder si el footprint actual afecta corredores reales de competencia.
2. **Resolver la discrepancia de footprint**: re-medir físicamente el robot (incluyendo protuberancias: soporte LiDAR, cables, antena) y decidir el valor definitivo, documentando la justificación en `navigation2_real.yaml` y CLAUDE.md.
3. **Auditar el Ceres scan matcher de Cartographer** (permanece activo incluso con correlative scan matching OFF) antes de tocar pesos de SLAM — pedido explícito en `slam_ab_r1/RESULTS.txt`.
4. **Repetir pruebas RAW vs scan filtrado** con más muestras antes de decidir cambio de `scan_topic`.
5. **Definir una misión real** en `config/missions/` con las estaciones reales de la arena de competencia (reemplazando el placeholder `test_arena.yaml`).
6. **Diseñar y documentar una prueba end-to-end de Nav2** (planner + controller + recoveries) sobre el mapa oficial confirmado, antes de considerar el sistema de navegación como validado.
7. **Grabar bags completos** (`/scan`, `/map`, `/tf`, `/amcl_pose`) en una próxima sesión con hardware, para poder repetir análisis offline de footprint/SLAM con datos reales en vez de mapas estáticos únicamente.

---

*Este documento fue generado a partir de información ya existente en el workspace (`CLAUDE.md`, `results/baseline/`, `results/footprint_analysis/`, `config/`) y de una inspección de solo lectura de `~/limoatwork`. No se inventaron resultados ni se ejecutó ningún comando de movimiento, launch u hardware para su elaboración.*
