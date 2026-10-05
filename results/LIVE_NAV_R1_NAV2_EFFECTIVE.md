# LIVE_NAV_R1 — Nav2 readiness / parámetros efectivos en vivo (FASE 5) — LIMO RoboCup SML

Fecha: 2026-09-18
Dominio aislado `ROS_DOMAIN_ID=17`. Robot **quieto**. **Sin goals, sin `/cmd_vel`, sin recoveries/spin/backup.** `~/limoatwork` solo lectura.

## Aclaración de alcance (mapa incorrecto)
El mapa cargado (`map.pgm`) **NO corresponde al entorno físico actual** (dato conocido de antemano). Por lo tanto, **FASE 4 (map↔scan) = NOT_APPLICABLE — KNOWN MAP MISMATCH**, y esta FASE 5 **solo** verifica que el *stack esté correctamente configurado*. **No** se evalúa calidad de rutas, ni el costmap global como espacio válido, ni se hace plan-only. La pose de AMCL **no** se interpreta como localización correcta.

## Método (seguro)
- El launch del fabricante `limo_navigation.launch.py` usa la API rota en Humble (`nav2_common.launch.Node`) → no usable.
- `nav2_bringup/navigation_launch.py` incluye `velocity_smoother`, que **publica `/cmd_vel`** continuamente → riesgo. **No** se usó tal cual.
- Se usó un launch de inspección mínimo (temporal, en scratchpad, **sin** `velocity_smoother`), con `cmd_vel` de `controller_server` y `behavior_server` **remapeado a un sink `/cmd_vel_probe`**. Nodos: `controller_server` (+local_costmap), `planner_server` (+global_costmap), `behavior_server`, `bt_navigator`, gestionados por un `lifecycle_manager` (autostart).
- **Seguridad verificada antes y después: `/cmd_vel` Publisher count = 0.** Los 6 publishers de comando quedaron en `/cmd_vel_probe` (sink), nunca en `/cmd_vel`.

## Estado de nodos (lifecycle)
| Nodo | Estado |
|---|---|
| map_server | active ✅ |
| amcl | active ✅ |
| controller_server (+ /local_costmap/local_costmap) | active ✅ |
| planner_server (+ /global_costmap/global_costmap) | active ✅ |
| behavior_server (spin, backup, drive_on_heading, assisted_teleop, wait cargados) | active ✅ (idle, sin ejecutar) |
| bt_navigator | active ✅ (sin goals) |

## Parámetros efectivos EN VIVO vs config en disco (`navigation2_real.yaml`)

### AMCL
| Param | Live | Disco | ¿Coincide? |
|---|---|---|---|
| robot_model_type | `nav2_amcl::DifferentialMotionModel` | idem | ✅ |
| base_frame_id / odom_frame_id / global_frame_id | `base_link` / `odom` / `map` | idem | ✅ |
| scan_topic | `/scan_filtered` | idem | ✅ |
| set_initial_pose | False | (ausente→False) | ✅ |

### Planner
| Param | Live | ¿Coincide con disco? |
|---|---|---|
| planner_plugins | `['GridBased']` | ✅ |
| GridBased.plugin | `nav2_navfn_planner/NavfnPlanner` | ✅ |
| GridBased.tolerance | 0.2 | ✅ |
| GridBased.allow_unknown | False | ✅ |
| GridBased.use_astar | False | ✅ |

### Controller (DWB)
| Param | Live | ¿Coincide? |
|---|---|---|
| controller_plugins / FollowPath.plugin | `['FollowPath']` / `dwb_core::DWBLocalPlanner` | ✅ |
| max_vel_x / min_vel_x | 0.25 / 0.0 | ✅ |
| **max_vel_y / min_vel_y / vy_samples** | **0.0 / 0.0 / 1** (no-holonómico) | ✅ |
| max_vel_theta | 1.0 | ✅ |
| sim_time | 1.7 | ✅ |
| general_goal_checker.xy_goal_tolerance | 0.25 | ✅ |
| critics | `['Oscillation','BaseObstacle','GoalAlign','PathAlign','PathDist','GoalDist']` | ✅ (RotateToGoal ausente, como en disco) |
| Oscillation/BaseObstacle/PathAlign/GoalAlign/PathDist/GoalDist .scale | 1.0 / **0.1** / 32.0 / 24.0 / 32.0 / 24.0 | ✅ |

### Costmaps
| Param | Local (live) | Global (live) | ¿Coincide con disco? |
|---|---|---|---|
| global_frame / robot_base_frame | odom / base_link | map / base_link | ✅ |
| resolution | 0.05 | 0.03 | ✅ |
| **footprint** | `[[-0.17,-0.16],[-0.17,0.16],[0.22,0.16],[0.22,-0.16]]` | idem | ✅ (39×32 cm) |
| footprint_padding | 0.02 | 0.02 | ✅ |
| plugins | `['obstacle_layer','inflation_layer']` | `['static_layer','obstacle_layer','inflation_layer']` | ✅ |
| inflation_radius | 0.35 | 0.35 | ✅ |
| cost_scaling_factor | 2.0 | 2.0 | ✅ |
| obstacle observation_sources / scan.topic | scan / `/scan_filtered` | scan / `/scan_filtered` | ✅ |

## Conclusiones

1. **La config efectiva EN VIVO == config en disco (`navigation2_real.yaml`) en TODO lo verificado.** No hay ninguna deriva disco↔live. El stack carga correctamente los parámetros del proyecto.
2. **Cinemática no-holonómica correctamente reflejada en vivo** (DWB `max_vel_y=0`, `vy_samples=1`; AMCL DifferentialMotionModel). Coherente con `MODE_FOUR_DIFF` confirmado en FASE 1.
3. **`voxel_layer` NO está en `plugins` en vivo** (inerte) en ambos costmaps — confirma el hallazgo estático; la cámara RealSense referenciada no participa.
4. **Footprint efectivo = 39×32 cm, padding 0.02 (EN VIVO).** El problema de footprint no resuelto (origen sin confirmar, contradice la medición física 36×20) **persiste y ahora está confirmado en el sistema en ejecución**. No se modificó.
5. **Fuente de obstáculos = `/scan_filtered`** en ambos costmaps → depende del filtro (hoy 2 etapas fabricante). La decisión range_only vs range+speckle sigue abierta.

## Observación de seguridad (no crítica)
- `/odom` pasó de (0,0,0) al inicio a un valor **fijo (−0.039, −0.007)** (~4 cm), **estable** (3 muestras idénticas). Como `/cmd_vel` Publisher permaneció en **0** todo el tiempo, **no fue un comando de nuestro stack** — probable empujón físico puntual o asentamiento del odom. No es deriva activa. Se reporta por transparencia.

## Qué quedará listo al cargar un mapa correcto
- Localización (AMCL, DifferentialMotionModel, /scan_filtered) — maquinaria validada (FASE 3).
- Planificación (NavfnPlanner, allow_unknown=false) y control (DWB no-holonómico) — configurados y activos.
- Costmaps con footprint/inflación efectivos — **pendiente resolver el footprint** antes de confiar en clearance.
- **Bloqueante real para navegar: (a) mapa correcto del entorno, (b) footprint confirmado.**

---

*FASE 5 completada en dominio aislado, robot quieto, sin goals, `/cmd_vel` Publisher=0 antes y después. No se movió el robot. No se interpretó el mapa (incorrecto) como espacio válido. `~/limoatwork` intacto.*
