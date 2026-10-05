# Auditoría del pipeline de localización — LIMO RoboCup SML

Fecha: 2026-09-17
Método: solo lectura y análisis estático de archivos existentes. No se ejecutó ROS, no se inició hardware. Consulta de `~/limoatwork` estrictamente en modo lectura.

Objetivo: rastrear inconsistencias documentales a lo largo de la cadena
**LiDAR → filtros → Cartographer (SLAM) → mapa → AMCL → costmaps → planner/controller**, sin ejecutar nada.

---

## 0. Diagrama de la cadena (según archivos existentes)

```
YDLiDAR (/scan, laser_frame)
   │   TF base_link→laser_frame = (0.180, 0.000, -0.090)   [lidar_real.launch.py / stage2_calibrated_lidar]
   ▼
laser_filters (scan_to_scan_filter_chain)  →  /scan_filtered
   │   proyecto: range_only  |  fabricante: range + speckle
   ├─────────────► Cartographer (SLAM, construye mapa)  →  /map (OccupancyGrid) + TF map→odom
   │                    map_frame=map, tracking_frame=base_link, odom_frame=odom
   │
   └─────────────► AMCL (localización en mapa conocido)  →  TF map→odom
                        scan_topic=/scan_filtered, global=map, odom=odom, base=base_link
                             │
                             ▼
                        map_server (/map desde .pgm+.yaml guardado)
                             │
                             ▼
                        global_costmap (map, 0.03)  +  local_costmap (odom, 0.05, rolling)
                             │
                             ▼
                        planner (NavFn) → controller (DWB) → /cmd_vel
```

Nota: Cartographer y AMCL **ambos** publican `map→odom`, pero operan en modos distintos (SLAM en construcción vs. localización en mapa conocido). No deben correr simultáneamente; la cadena de navegación de producción usa AMCL + map_server, no Cartographer.

## 1. Frames — consistencia

| Componente | map | odom | base | laser | Fuente |
|---|---|---|---|---|---|
| Cartographer (`limo_lds_2d_no_correlative.lua`) | `map` | `odom` | `base_link` (tracking) | — | proyecto |
| AMCL (`navigation2_real.yaml`) | `map` | `odom` | `base_link` | — | proyecto |
| local_costmap | — | `odom` (global_frame) | `base_link` | — | proyecto |
| global_costmap | `map` | — | `base_link` | — | proyecto |
| TF LiDAR | — | — | `base_link` | `laser_frame` | `lidar_real.launch.py`, `stage2_calibrated_lidar/tf_echo.txt` |

**Resultado: los frames son consistentes en toda la cadena.** No se detecta inconsistencia de nombres de frame. La cadena `map → odom → base_link → laser_frame` está bien definida y la TF del LiDAR (0.180, 0, −0.090) coincide entre el launch propio y la medición baseline.

Observación menor: en Cartographer `published_frame = "odom"` y `provide_odom_frame = false` con `use_odometry = true` → Cartographer publica `map→odom` y **consume** una odometría externa (la del `limo_base`), no genera su propio `odom→base`. Coherente con la arquitectura del proyecto (odometría del driver validada en `base_motion_r1`).

## 2. Producción de `/scan_filtered` — inconsistencia de filtros

- **Existen dos definiciones distintas del filtro** que produce `/scan_filtered`:
  - Proyecto: `config/cartographer/laser_filters_range_only.yaml` → **1 filtro** (range gate [0.12, 8.0]).
  - Fabricante: `limoatwork/.../config_files/laser_filters.yaml` → **2 filtros** (range gate + speckle filter).
- Todos los consumidores (`AMCL`, ambos costmaps, Cartographer) escuchan `/scan_filtered` — pero **el contenido de ese topic depende de cuál launch/filtro se haya arrancado**.
- **Inconsistencia documental**: no hay en `limo_sml_ws` un launch que arranque el filtro `range_only` del proyecto. El nodo de filtro visto en el baseline (`stage3_cartographer_static/nodes.txt`: `cartographer_scan_filter_chain`) corresponde al launch del fabricante (filtro de 2 etapas). Por lo tanto, **el `/scan_filtered` sobre el que se validó el baseline no es, necesariamente, el que produciría la config `range_only` del proyecto.** Esto afecta cualquier comparación entre resultados baseline y la config actual.

## 3. Resolución del mapa a lo largo de la cadena

| Etapa | Resolución | Fuente |
|---|---|---|
| Cartographer submaps | 0.03 m | `limo_lds_2d_no_correlative.lua` |
| Cartographer occupancy_grid (publicado) | 0.03 m | `cartographer.launch.py` default |
| Mapa baseline `stage3` (OccupancyGrid en vivo) | 0.03 m | `stage3/map_metadata.txt` |
| Mapa candidato `map.pgm` / `map.yaml` | 0.03 m | `AUDITORIA_MAPAS.md` |
| Otros mapas persistidos (`map1`, `map_20260718...`, etc.) | **0.05 m** | `AUDITORIA_MAPAS.md` |
| global_costmap | 0.03 m | `navigation2_real.yaml` |
| local_costmap | 0.05 m | `navigation2_real.yaml` |

**Hallazgos:**
- La rama de 0.03 m (Cartographer → occupancy_grid → `map.pgm` → global_costmap) es **internamente consistente**.
- Sin embargo, **coexisten mapas a 0.05 m** en el mismo directorio. Si por error se cargara uno de 0.05 m como mapa de navegación, seguiría funcionando (la resolución del costmap es independiente de la del mapa), pero introduciría una discretización distinta de la usada al construir/validar. Riesgo latente, no error confirmado.
- `local_costmap` a 0.05 m mientras `global_costmap` a 0.03 m: es una configuración válida y común (el local es rolling y de corto alcance), no una inconsistencia, pero conviene tenerlo presente.

## 4. Origen del mapa — no verificable sin mapa oficial confirmado

| Mapa | Origen (x, y) | Área |
|---|---|---|
| `stage3` (en vivo, no persistido) | (−1.32, −3.96) | 63.75 m² |
| `map.yaml` (candidato) | (−0.923, −0.645) | 17.5 m² |
| `map_20260718_185606.yaml` | (−1.11, −1.57) | 26.5 m² |

- El origen es un atributo por-mapa; no hay una "inconsistencia" intrínseca entre orígenes distintos de mapas distintos.
- **La inconsistencia real es que no se sabe cuál mapa es el oficial**, por lo que no se puede verificar que el origen usado por `map_server` en producción coincida con el que AMCL/costmaps esperan. `map_server.yaml_filename` está **vacío** en `navigation2_real.yaml` (se inyecta por launch) → el mapa efectivo depende de qué se pase en tiempo de lanzamiento, no está fijado ni documentado en el repo.

## 5. TF esperado vs. evidencia

- **`base_link→laser_frame`**: esperado (0.180, 0, −0.090); confirmado en `stage2_calibrated_lidar/tf_echo.txt`. ✔ Consistente.
- **`map→base_link`**: la única medición baseline (`stage3/map_to_base.txt`) fue generada por **Cartographer** (SLAM), no por AMCL — mostró un robot esencialmente estático (traslación ~−0.015 m, rotación <0.04°). **No existe evidencia baseline de la TF `map→odom` publicada por AMCL** (ver `AUDITORIA_AMCL.md`). Hueco de validación.
- **`map→odom`**: publicada tanto por Cartographer (`tf_broadcast` implícito por `published_frame=odom`) como por AMCL (`tf_broadcast: true`). Consistente en frames, pero no hay captura de la versión de AMCL.

## 6. Diferencia entre "mapa usado en SLAM" y "mapa usado en navegación"

Esta es la inconsistencia documental de mayor impacto potencial:

- **Mapa de SLAM**: construido en vivo por Cartographer. La config del proyecto para SLAM es `no_correlative` (correlative OFF), pero el mapa baseline `stage3` se generó con el launch del fabricante (correlative ON + filtro 2 etapas). Es decir, **si hoy se remapea con la config del proyecto, el mapa podría diferir del baseline** por cambio de algoritmo de scan matching y de filtro.
- **Mapa de navegación**: un `.pgm`+`.yaml` estático cargado por `map_server`. No está confirmado cuál es (candidato `map.pgm` 0.03 m), ni con qué config de SLAM fue generado.
- **Consecuencia**: no se puede garantizar, con la documentación existente, que el mapa que AMCL usaría en navegación haya sido generado con la misma configuración de SLAM que está hoy activa en el repo. Esto acopla el problema de "identidad del mapa" (pendiente) con el de "config de Cartographer" (pendiente).

## 7. Inconsistencias detectadas — resumen

| # | Inconsistencia | Severidad | Estado |
|---|---|---|---|
| 1 | Filtro `/scan_filtered`: proyecto (range) vs fabricante (range+speckle); baseline usó el del fabricante | Media | Documentar; no tocar |
| 2 | Config SLAM activa (`no_correlative`) ≠ config con que se generó el baseline (`stage3`, correlative ON) | Alta | Bloquea comparabilidad |
| 3 | Mapa oficial sin confirmar; `map_server.yaml_filename` vacío | Alta | Pendiente equipo |
| 4 | Coexisten mapas 0.03 y 0.05 m en el mismo directorio | Baja | Riesgo latente |
| 5 | Footprint del costmap no validado (ya documentado en auditorías previas) | Alta | Pendiente medición |
| 6 | Sin captura baseline de la TF `map→odom` de AMCL | Media | Hueco de validación |
| 7 | `min_range` 0.1 (Cartographer) vs 0.12 (filtro/AMCL) | Baja | Origen no documentado |

## 8. Qué NO debe tocarse todavía

- No fijar `map_server.yaml_filename` a ningún mapa hasta confirmar el mapa oficial con el equipo.
- No cambiar la resolución de costmaps ni de submapas para "unificar" — la rama 0.03 ya es consistente; el problema es la identidad del mapa, no la resolución.
- No unificar/alterar los filtros de `/scan_filtered` sin decidir cuál es la config de referencia (range vs range+speckle) y re-validar.
- No modificar frames ni la TF del LiDAR — están confirmados y consistentes.
- No cambiar la config de Cartographer (ver `AUDITORIA_CARTOGRAPHER_COMPLETA.md`).

## 9. Preguntas abiertas

1. ¿Cuál es el mapa oficial que `map_server` debe cargar, y con qué config de SLAM (correlative, filtro, resolución) fue generado?
2. ¿El filtro de producción para `/scan_filtered` debe ser el `range_only` del proyecto o el `range+speckle` del fabricante? El baseline se validó con el segundo.
3. ¿Existe una captura o prueba de AMCL en vivo (TF `map→odom`, `/amcl_pose`) que no esté en `results/baseline/`?
4. ¿Se pretende navegar con AMCL+map_server (mapa conocido) exclusivamente, o hay intención de usar Cartographer en modo localización? La cadena actual está documentada para lo primero.

---

*Informe generado por análisis estático de archivos existentes en `~/limo_sml_ws` y consulta de solo lectura de `~/limoatwork`. No se ejecutó ROS, launch ni hardware. No se propusieron cambios.*
