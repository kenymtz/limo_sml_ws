# Preguntas pendientes para el equipo — LIMO RoboCup SML

Fecha: 2026-09-17
Propósito: consolidar en un solo lugar **todas las preguntas que requieren respuesta humana** y que hoy bloquean decisiones técnicas. Ninguna puede resolverse por análisis de archivos (ya agotado en las auditorías). Cada pregunta indica por qué bloquea y qué desbloquea.

Referencias: `ESTADO_PROYECTO.md`, `AUDITORIA_NAV2_CONFIG.md`, `AUDITORIA_AMCL.md`, `AUDITORIA_MAPAS.md`, `AUDITORIA_FULL_ARENA.md`, `AUDITORIA_CERES_SCAN_MATCHER.md`, `AUDITORIA_CARTOGRAPHER_COMPLETA.md`, `AUDITORIA_PIPELINE_LOCALIZACION.md`.

---

## A. Footprint  (bloquea: cualquier cambio en `navigation2_real.yaml`, análisis de clearance definitivo)

1. **¿Quién cambió el footprint** de `[[-0.16,-0.10],...]` (36×20 cm, padding 0.03) a `[[-0.17,-0.16],...]` (39×32 cm, padding 0.02) el 2026-09-15 ~21:03?
2. **¿Por qué se amplió a 39×32 cm?** No hay prueba física de respaldo en el repo, y contradice la medición de `base_motion_r1` (36×20 cm) de esa misma tarde.
3. **¿Existe una medición física nueva** del robot (incluyendo protuberancias: soporte LiDAR, cables, antena, bumper) que justifique el valor grande?
4. **¿Cuál debe considerarse el footprint oficial** para RoboCup SML?

Desbloquea: resolver la discrepancia de footprint (tarea prioritaria del cierre), y validar el análisis de clearance sobre el mapa correcto.

## B. Mapas  (bloquea: análisis de corredores de la arena real, config de `map_server`, misión real)

5. **¿Cuál archivo corresponde a "v1"?** No existe ningún archivo con ese nombre. Candidato débil: `map1.pgm`.
6. **¿Cuál corresponde a "run3"?** Ídem. Candidato por escala: el mapa en vivo de `stage3` (63.75 m²), pero no está persistido.
7. **¿Existe una copia del mapa "full arena"** (~63.75 m² o mayor) en algún dispositivo/backup no accesible desde este sistema? La búsqueda exhaustiva local no la encontró.
8. **¿Cuál `.pgm`+`.yaml` debe cargar `map_server` en producción?** Hoy `yaml_filename` está vacío en `navigation2_real.yaml`.
9. **¿El candidato `map.pgm` (0.03 m, 17.5 m²) es el mapa de evaluación**, o es un mapa parcial/de prueba?

Desbloquea: fijar el mapa oficial, responder si el footprint bloquea corredores reales, definir la misión.

## C. Cartographer / SLAM  (bloquea: decisión sobre config de SLAM, re-mapeo, ajuste de sesgo lateral)

10. **¿Por qué está activa (en el repo) la configuración `no_correlative`** (correlative OFF, SLAM-B), que el propio test `slam_ab_r1` recomendó **no adoptar**? ¿Fue un descuido al no revertir tras el test?
11. **¿Fue intencional** dejar correlative OFF, a pesar de que el launch del fabricante usa por defecto `limo_lds_2d.lua` (correlative ON)?
12. **¿Con qué launch y argumentos exactos se corrió Cartographer** en `slam_ab_r1` y en `stage3`? ¿Hubo override manual de `configuration_basename`/`cartographer_config_dir`, o un launch propio no localizado?
13. **¿Con qué configuración (correlative, filtro, resolución) se generó el mapa que se pretende usar como oficial?**
14. **¿"RAW scan" en los tests C** significó `/scan` crudo directo, o el filtro `range_only` (sin speckle)?
15. **¿El filtro de producción debe ser `range_only` (proyecto) o `range+speckle` (fabricante)?** El baseline se validó con el segundo.
16. **¿Se ajustaron alguna vez los pesos de Ceres** fuera del repo? La evidencia dice que no (siguen en fábrica: occupied=1, translation=10, rotation=40).

Desbloquea: elegir la config de SLAM de referencia, planificar el re-mapeo, abordar el sesgo lateral con base firme.

## D. Datos / rosbags  (bloquea: reconstrucción offline, evitar pérdida de mapas futuros)

17. **¿Existe un rosbag completo** (con `/scan`, `/tf`, `/tf_static`, `/odom`, `/imu`) de alguna sesión de Cartographer, en algún dispositivo/backup externo? Los únicos bags locales (`~/bags/volta_quadra*`) solo tienen `/cmd_vel`.
18. **¿Dónde está**, si existe?
19. **¿Se guardó el mapa** de la sesión `stage3` (63.75 m²) con `map_saver` antes de cerrar Cartographer, o se perdió al terminar el proceso? La búsqueda local no encontró ningún `.pgm`/`.pbstream` de esa fecha.

Desbloquea: determinar si el mapa "full arena" puede recuperarse offline (hoy: no, con datos locales) o si es obligatorio re-mapear físicamente.

## E. Sensores / hardware  (bloquea: activación de capas de costmap, validación AMCL)

20. **¿El robot tiene físicamente una cámara de profundidad** (RealSense R200 u otra)? El `global_costmap` referencia `/intel_realsense_r200_depth/points` en un `voxel_layer` inactivo; no aparece en `serial_devices.txt` ni en el resto del proyecto.
21. **¿Existe alguna prueba documentada de AMCL en vivo** (`/amcl_pose`, TF `map→odom`) fuera de `results/baseline/`? No se encontró ninguna; el estado "AMCL activo" de CLAUDE.md no tiene artefacto de respaldo.

Desbloquea: decidir si activar el `voxel_layer`, y cerrar el hueco de validación de AMCL.

---

## Prioridad sugerida de resolución

| Orden | Pregunta(s) | Motivo |
|---|---|---|
| 1 | B (5–9) | Identidad del mapa: bloquea footprint-real, misión, y config de map_server |
| 2 | D (17–19) | Determina si hay que remapear físicamente (única vía si no hay bag/mapa) |
| 3 | A (1–4) | Footprint: bloquea cambios en Nav2 |
| 4 | C (10–16) | Config de SLAM y sesgo lateral |
| 5 | E (20–21) | Sensores y validación AMCL |

---

*Documento de consolidación. No contiene decisiones ni cambios — solo preguntas que requieren input humano. Generado a partir de las auditorías existentes del proyecto.*
