# Matriz de evidencia consolidada — LIMO RoboCup SML

Fecha: 2026-09-17
Consolida hallazgos de todas las auditorías previas y de la trazabilidad de esta sesión. Estados permitidos: **CONFIRMADO** / **EVIDENCIA INDIRECTA** / **CONTRADICCIÓN** / **DESCONOCIDO**.

---

| Tema | Afirmación | Estado | Evidencia | Archivo/fuente | Confianza | Acción pendiente |
|---|---|---|---|---|---|---|
| Cinemática | Robot es diferencial puro (`MODE_FOUR_DIFF`), sin `linear.y` | CONFIRMADO | Prueba física documentada, PASS | `results/baseline/base_motion_r1/RESULTS.txt` | Alta | Ninguna |
| Cinemática | Driver de yaw corregido (deadband eliminado) | CONFIRMADO | Prueba física, PASS en ±90° | `base_motion_r1/RESULTS.txt` | Alta | Ninguna |
| AMCL | `robot_model_type` = DifferentialMotionModel | CONFIRMADO | Config estática, coherente con cinemática | `navigation2_real.yaml` | Alta | Ninguna |
| AMCL | AMCL validado en vivo con datos propios | DESCONOCIDO | No existe artefacto equivalente a `base_motion_r1`/`slam_ab_r1` para AMCL | `AUDITORIA_AMCL.md` | — | Diseñar y ejecutar prueba dedicada |
| Footprint | Footprint "medido" = 36×20 cm, padding 0.03 | CONFIRMADO | Prueba física documentada | `base_motion_r1/RESULTS.txt` | Alta | Ninguna |
| Footprint | Footprint "actual" = 39×32 cm, padding 0.02 | CONFIRMADO | Lectura directa del yaml activo | `navigation2_real.yaml` | Alta | Ninguna |
| Footprint | Cambio ocurrió 2026-09-15 21:02:23–21:03:03 | CONFIRMADO | Forense de timestamps (`stat`) | `TRAZABILIDAD_FOOTPRINT.md` | Alta | Ninguna |
| Footprint | Autor/herramienta del cambio | DESCONOCIDO | Sin rastro en bash_history ni en sesiones Claude Code previas (no existen) | `TRAZABILIDAD_FOOTPRINT.md` | — | Preguntar al equipo (P0) |
| Footprint | Cambio fue deliberado y justificado | DESCONOCIDO | Sin documentación escrita en ningún archivo del proyecto | `TRAZABILIDAD_FOOTPRINT.md` | — | Preguntar al equipo (P0) |
| Footprint | Padding del valor "actual" (0.02) coincide con `costmap_common.yaml` del fabricante | EVIDENCIA INDIRECTA | Comparación de valores, coincidencia parcial (solo padding, no polígono) | `TRAZABILIDAD_FOOTPRINT.md` §5 | Media | Ninguna (solo contexto) |
| Footprint | Impacto de clearance del footprint grande en mapas de prueba: pérdida de 8-9% del espacio libre | CONFIRMADO | Análisis offline (`distance_transform_edt`) sobre 2 mapas | `results/footprint_analysis/`, `AUDITORIA_FULL_ARENA.md` | Alta | Ninguna |
| Footprint | Footprint grande bloquea corredores reales de la arena de competencia | DESCONOCIDO | Ningún mapa de prueba disponible contiene pasillos de 0.26-0.36 m de ancho | `results/footprint_analysis/` | — | Repetir análisis con mapa oficial confirmado |
| Mapas | "v1" y "run3" son nombres de archivo literales en el sistema | CONTRADICCIÓN (con la premisa inicial) | Búsqueda exhaustiva de filesystem + shell history + git log: cero coincidencias en ningún artefacto | `AUDITORIA_MAPAS.md`, `TRAZABILIDAD_MAPAS_V1_RUN3.md` | Alta (de la ausencia) | Preguntar al equipo qué archivo corresponde a cada apodo (P0) |
| Mapas | `map.pgm` (0.03 m, 17.5 m²) es el mapa de producción/evaluación | EVIDENCIA INDIRECTA | Resolución coincide con nota de CLAUDE.md; es el más reciente; pero **no está commiteado en git** | `AUDITORIA_MAPAS.md`, `TRAZABILIDAD_MAPAS_V1_RUN3.md` | Media | Confirmar con equipo |
| Mapas | `map_20260718_185606` es el único mapa alguna vez versionado en git | CONFIRMADO | `git log`/`git show` en `limoatwork` (solo lectura); contenido idéntico al commit `1aa72b3` | `TRAZABILIDAD_MAPAS_V1_RUN3.md` §2 | Alta | Considerar como candidato adicional a "v1" |
| Mapas | Ningún mapa candidato (incluido el de mayor resolución) está protegido por control de versiones | CONFIRMADO | `git status --porcelain` en `limoatwork`: todos `??` (untracked) o modificados sin commit | `TRAZABILIDAD_MAPAS_V1_RUN3.md` §2 | Alta | **Riesgo operativo**: respaldar/commitear antes de cualquier limpieza de `limoatwork` |
| Mapas | Existe un mapa de ~63.75 m² (candidato "full arena") | CONFIRMADO (la metadata existe) | `stage3_cartographer_static/map_metadata.txt` | `AUDITORIA_FULL_ARENA.md` | Alta | — |
| Mapas | Ese mapa de 63.75 m² tiene un archivo `.pgm`/`.pbstream` persistido en algún lugar del sistema | CONTRADICCIÓN (con la expectativa de que debería existir) | Búsqueda exhaustiva por fecha/hora exacta en todo el filesystem: sin resultado | `AUDITORIA_FULL_ARENA.md`, sesión de búsqueda de rosbags | Alta (de la ausencia) | Confirmar con equipo si se guardó en backup externo (P0) |
| Mapas | Existe un rosbag que permita reconstruir offline el mapa de 63.75 m² | CONTRADICCIÓN | Los únicos bags (`~/bags/volta_quadra*`) solo contienen `/cmd_vel`, sin `/scan`/`/tf` | Auditoría de rosbags (sesión anterior) | Alta | Si no hay backup externo, remapear físicamente (requiere autorización) |
| SLAM | Config activa del proyecto (`no_correlative.lua`) es la variante que `slam_ab_r1` recomendó no adoptar | CONFIRMADO | Comparación textual directa con `RESULTS.txt` | `AUDITORIA_CERES_SCAN_MATCHER.md`, `AUDITORIA_CARTOGRAPHER_COMPLETA.md` | Alta | Decidir con el equipo si se reemplaza (P1) |
| SLAM | El `.lua` del proyecto está conectado a algún launch del workspace | CONTRADICCIÓN | El launch del fabricante usa por defecto `limo_lds_2d.lua` (ON); el `.lua` del proyecto solo se usa vía override manual explícito | `AUDITORIA_CARTOGRAPHER_COMPLETA.md`, `TRAZABILIDAD_SLAM_CONFIGS.md` | Alta | — |
| SLAM | Tests A/A2/B/B2 de `slam_ab_r1` usaron el filtro `range_only` del proyecto | CONTRADICCIÓN | El filtro está hardcodeado en `cartographer.launch.py` al del fabricante (2 etapas); no hay override posible sin flags no vistos | `TRAZABILIDAD_SLAM_CONFIGS.md` §1 | Alta | — |
| SLAM | Tests C1/C2 usaron scan verdaderamente sin filtrar | DESCONOCIDO | Ningún comando localizado permite ese escenario con el launch estándar | `TRAZABILIDAD_SLAM_CONFIGS.md` §3, `RELECTURA_SLAM_AB_R1.md` | — | Preguntar al equipo (P0) |
| SLAM | Pesos de `ceres_scan_matcher` nunca fueron ajustados | CONFIRMADO | Comparación del `.lua` del proyecto contra defaults del paquete Cartographer instalado | `AUDITORIA_CERES_SCAN_MATCHER.md` | Alta | No tocar sin repeticiones controladas |
| SLAM | El mapa baseline `stage3` (63.75 m²) se generó con el launch estándar del fabricante | EVIDENCIA INDIRECTA | Coincide el nombre de nodo `cartographer_scan_filter_chain`; comando exacto no recuperable (anterior al historial visible) | `AUDITORIA_CARTOGRAPHER_COMPLETA.md`, `TRAZABILIDAD_SLAM_CONFIGS.md` | Media | — |
| Pipeline | Frames (`map`/`odom`/`base_link`/`laser_frame`) consistentes en toda la cadena | CONFIRMADO | Comparación cruzada de todos los archivos de config | `AUDITORIA_PIPELINE_LOCALIZACION.md` | Alta | Ninguna |
| Pipeline | `map_server.yaml_filename` fijado a un mapa concreto | CONTRADICCIÓN (vacío) | Lectura directa del yaml: campo vacío | `navigation2_real.yaml`, `AUDITORIA_PIPELINE_LOCALIZACION.md` | Alta | Fijar una vez confirmado el mapa oficial |
| Hardware | Robot tiene cámara de profundidad RealSense R200 | DESCONOCIDO | Referenciada en `voxel_layer` inactivo del costmap global, sin aparecer en `serial_devices.txt` ni en ningún otro lugar del proyecto | `AUDITORIA_NAV2_CONFIG.md` | — | Preguntar al equipo (P2) |
| Sesiones previas | Existe una sesión de Claude Code anterior a la de hoy para este proyecto | CONTRADICCIÓN | Solo existe el directorio de la sesión actual en `~/.claude/projects/` | `TRAZABILIDAD_FOOTPRINT.md` §3 | Alta | — |
| Historial de shell | El historial de comandos cubre toda la sesión del 2026-09-15 | CONTRADICCIÓN | `HISTFILESIZE=2000`, archivo capado exactamente en 2000 líneas, comienza a mitad de un heredoc | `TRAZABILIDAD_SLAM_CONFIGS.md` §0 | Alta | Ninguna (limitación permanente de la fuente) |

---

## Leyenda de "Confianza"

- **Alta**: múltiples fuentes independientes coinciden, o la evidencia es un hecho verificable directamente (timestamp, contenido de archivo, salida de comando de solo lectura).
- **Media**: una sola fuente, o coincidencia parcial/circunstancial.
- **—**: no aplica (la fila documenta una ausencia de evidencia, no un grado de certeza sobre un hecho positivo).

---

*Matriz generada por consolidación de todas las auditorías del proyecto (previas y de esta sesión). No se ejecutó ROS, launch ni hardware. No se modificó ningún archivo.*
