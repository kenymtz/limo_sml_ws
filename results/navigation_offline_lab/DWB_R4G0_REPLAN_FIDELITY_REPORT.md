# NAV-OFFLINE-R4-G0 — Fidelidad del replanning periódico (fixed_plan vs replan_1hz)

Fecha: 2026-10-03. 100 % offline (ROS_DOMAIN_ID=99, ROS_LOCALHOST_ONLY=1).
- Sin hardware, sin limo_base, sin LiDAR real, sin teleop, sin `/cmd_vel`. El simulador no tiene publishers Twist ni acciones, y la auditoría del grafo dio PASS en los 12 lanzamientos.
- `config/navigation/navigation2_real.yaml` sin cambios (sha256 `dfb36e108d6872a5…`). `~/limoatwork` sin cambios.
- Candidata evaluada: R4-F (`dwb_offline_R4F_010_sim.yaml`), con ObstacleFootprint, RotateToGoal, FollowPath xy 0.15, checker 0.25 m / 0.10 rad y footprint B con padding 0.02. No se tocó ningún parámetro.

## Resumen

```
R4-G0  (25 casos representativos, misma semilla por caso)
fixed_plan (binario nuevo) == R4-F:           25/25 bit a bit
fixed_plan vs replan_1hz:   idénticos 4/25 · mismo resultado 21/25 · resultado cambiado 4/25
fixed_plan vs replan_same:  idénticos 13/25 · mismo resultado 22/25 · resultado cambiado 3/25
ABORT_PROGRESS (total / cerca del goal):  fixed 8 / 3   replan_1hz 6 / 1   replan_same 5 / 1
Correctos: fixed 12 · replan_1hz 13 · replan_same 14     Contactos: 2 · 2 · 2
Cambios de signo de angular.z (suma de los 25 casos): fixed 336 · replan_1hz 553 (+65 %) · replan_same 509 (+51 %)
```

**Conclusión: PARTIAL.**
- El simulador de plan fijo **sigue siendo fiel para la navegación nominal**: los 12 casos correctos quedan correctos, con Δt p50 +0.05 s, |Δxy| ≤ 1.3 cm y |Δyaw| ≤ 0.012 rad.
- **No es fiel justo para lo que gobierna el progress checker**: atascos cerca del goal, oscilaciones y bloqueos. De los 3 aborts cerca del goal, solo 1 se comporta igual.
- **No tunear progress_checker con fixed_plan.** Usar `replan_1hz`.

## Qué hace Nav2 Humble real (CONFIRMADO en el código fuente 1.1.20)

Fuentes descargadas del tag `1.1.20` de ros-navigation/navigation2, que es la versión instalada (`dwb_core`, `nav2_bt_navigator` 1.1.20). La copia en `~/Downloads/LIMOPRO` es Foxy 0.4.7 y no se usó.

| Paso | Archivo:línea (1.1.20) | Efecto |
|---|---|---|
| BT por defecto (no hay `default_nav_to_pose_bt_xml` en `navigation2_real.yaml`) | `navigate_to_pose_w_replanning_and_recovery.xml` | `RateController hz=1.0` → `ComputePathToPose` desde la pose del robot |
| NavFn | `navfn_planner.cpp:157, 211` | `path.header.stamp = now()` en cada plan, así que cada plan es distinto como mensaje |
| FollowPath BT | `follow_path_action.cpp` (`on_wait_for_result`) | `if (goal_.path != new_path) goal_updated_ = true`: reenvía el goal en cada replan |
| controller_server | `controller_server.cpp:406` | `updateGlobalPath()` al inicio de cada iteración, antes de `computeAndPublishVelocity()` |
| controller_server | `controller_server.cpp:444-462` | `setPlannerPath`: `controller->setPlan(path)` **y** `goal_checker->reset()` |
| controller_server | `controller_server.cpp:382-383` | `progress_checker_->reset()` **solo** al comenzar una acción nueva, no al recibir un path nuevo |
| DWB | `dwb_local_planner.cpp` (`setPlan`) | `critic->reset()` en **todos** los critics y `traj_generator_->reset()` |
| Oscillation | `oscillation.cpp:201` | `reset()` borra las tendencias x/y/θ: la histéresis de ±0.2 rad se pierde |
| RotateToGoal | `rotate_to_goal.cpp:78-92` | `reset()` pone `in_window_ = rotating_ = false`; se vuelven a enganchar solo si d ≤ 0.15 m y la velocidad ≤ 0.03 |
| SimpleGoalChecker | `simple_goal_checker.cpp` | `reset()` re-arma el chequeo xy (stateful) |
| NavFn | `navfn_planner.cpp:232, 521` | `clearRobotCell()` → `setCost(FREE_SPACE)` sobre el **costmap maestro** |

## Implementación (simulador `dwb_closed_loop_sim`)

- **`plan_mode`**, parámetro nuevo; por defecto `fixed_plan`:
  - `fixed_plan`: comportamiento R4-B..R4-F intacto, verificado con **25/25 corridas bit a bit iguales a R4-F**.
  - `replan_1hz`: cada 1.0 s de tiempo simulado, en el borde de un ciclo de control y **antes** de computar (como `updateGlobalPath`), se calcula un plan con el **NavFn real** (`nav2_navfn_planner::NavfnPlanner`, mismo global costmap y parámetros GridBased que R3) desde la **pose estimada**, igual que `ComputePathToPose` con la pose del robot en `map`. Después `dwb->setPlan(path)` y el reset del goal checker, igual que `setPlannerPath`. El primer plan se calcula en t = 0, igual que el primer tick del BT.
  - `replan_same_1hz` (ablación): mismo ciclo, pero entrega el path **original recortado** desde la pose más cercana a la estimada. Misma geometría hacia adelante, así que el único efecto es el reset.
- **Registro** por corrida: `setplan` con [t, ok, n_poses, largo, primer punto, distancia a la pose estimada, causa], `n_setplan`, `n_plan_fail` y `plan0` (el primer plan completo).
- **Reset de critics**: no tiene una señal propia. Se observa con `debug_eval_runs` (evaluación DWB por ciclo) como la caída a 0 de los ilegales por Oscillation en el ciclo del `setPlan`.
- **Validación de la integración de NavFn**: en S1 (sin ruido), el `plan0` de NavFn coincide con `paths_R3.json` en **12/12 casos** (desvío máximo 0.0000 m). En escenarios con ruido difiere 0.03–0.09 m, lo esperado porque parte de la pose estimada.
- Config: `config/offline_lab/dwb_offline_R4G0_replan_sim.yaml` = R4-F 0.10 + GridBased y global_costmap de R3.
  - Único cambio de arnés: `update/publish_frequency` 0; el simulador actualiza el costmap de forma síncrona.
  - Verificador `verify_r4g0_configs.py`: PASS. Ninguna clave existente de R4-F cambia.
- Binario final `dwb_closed_loop_sim` sha256 `d7e44470…` (fuente `ebb17205…`, copia en `dwb_r4g0/dwb_closed_loop_sim.cpp.r4g0`). El binario de R4-B..R4-F (`63b484a0…`) queda en `dwb_r4g/provenance/`.

### Artefactos detectados y corregidos durante R4-G0

1. **v1, `replan_same_1hz` con el path completo**: reentregar el path desde su inicio cuando el robot ya avanzó más de 4 m produce "Resulting plan has 0 poses" (S1:T050, S2:T050). El sistema real nunca hace eso. Se corrigió en v2/v3 recortando el path. Archivado en `dwb_r4g0/v1_same_full_path_ARTIFACT/`.
2. **v2, `clearRobotCell` de NavFn**: en un global costmap estático, las celdas puestas en FREE_SPACE persistían, incluso entre corridas del mismo proceso. El plan inicial de T041/T043/T045 llegó a desviarse 0.03 m porque T039 había dejado un rastro en N1.
   - En el robot real, la capa de obstáculos del global costmap repinta la zona del robot en cada update. En v3 se restaura el costmap prístino antes de cada plan; se restauraron 9–154 celdas por proceso.
   - Con la corrección, `plan0` coincide 12/12 en S1.
   - v2 se detuvo y quedó archivada en `dwb_r4g0/v2_STOPPED_navfn_clearRobotCell_ARTIFACT/`.
   - Además, en v1 la "eliminación de contactos" de S4 era falsa: con v3 los contactos siguen en 2 → 2.

## Casos (25)

| grupo | casos |
|---|---|
| ABORT_PROGRESS cerca del goal (R4-F) | S1:T041, S2:T045, S5:T041 |
| Oscilación / bloqueo (R4-C) | S1:T023, S1:T028, S1:T025 (timeout), S2:T029, S4:T024, S5:T023 |
| N1 | S1:T039, S1:T043 (sin trayectorias), S1:T045, S2:T041, S4:T040, S4:T042 y S4:T045 (los 2 contactos) |
| Control (correctos) | S1:T005, S1:T015, S1:T033, S1:T050, S1:T060, S5:T019, S5:T033, S5:T055 |
| Falso éxito | S2:T050 |

Corridas: `dwb_r4g0/runs/runs_<S>.txt`, con las líneas RUN/PATH copiadas de `dwb_r4b/runs_<S>.txt`, es decir, la misma semilla y el mismo path.

## CONFIRMADO

- **fixed_plan no cambió**: 25/25 corridas bit a bit iguales a R4-F (log de control sin `ms`, estado, pose final).
- **setPlan a 1 Hz exacto**: 676 llamadas en replan_1hz, una por segundo en el borde de ciclo (Δt = 1.000 s), y 0 fallos de NavFn.
- **El reset de Oscillation se observa y es total**: cada vez que Oscillation estaba invalidando trayectorias en el ciclo anterior a un `setPlan`, en el ciclo del `setPlan` sus ilegales bajaron a 0. Ejemplos: S1:T023 18/18, S1:T041 19/19, S2:T029 43/43, S5:T023 25/25.
- **Navegación nominal fiel**:
  - los 12 casos correctos con fixed_plan siguen correctos con replan_1hz;
  - Δt de SUCCEEDED p50 +0.05 s (máx 0.6 s), |Δxy final| ≤ 1.3 cm, |Δyaw final| ≤ 0.012 rad;
  - Δ clearance mínima ≥ −0.024 m.
- **Contactos**: los mismos 2 (S4:T042, S4:T045) en los tres modos.
- **Aborts cerca del goal (el fenómeno de R4-G)**: solo 1 de 3 se comporta igual.

  | caso | fixed_plan | replan_1hz | replan_same_1hz |
  |---|---|---|---|
  | S1:T041 (sin ruido) | ABORT_PROGRESS t = 33.9 s | ABORT_PROGRESS t = 33.9 s | ABORT_PROGRESS t = 33.9 s |
  | S2:T045 | ABORT_PROGRESS t = 36.8 s (en la ventana del goal) | ABORT_CONTROLLER t = 4.9 s, "No valid trajectories" a la **entrada de N1** | igual, t = 4.7 s |
  | S5:T041 | ABORT_PROGRESS t = 34.6 s | **SUCCEEDED** t = 15.7 s (yaw 0.069) | **SUCCEEDED** t = 18.2 s |

  - **S1:T041**: el ciclo límite RotateToGoal × ObstacleFootprint × Oscillation de R4-G **sobrevive** al reset de Oscillation. La pose queda a 0.141 m (< 0.15), así que RotateToGoal se re-engancha en el primer ciclo después de cada `setPlan`. ObstacleFootprint sigue invalidando las rotaciones lentas hacia el goal.
  - **S5:T041**: en fixed_plan, RotateToGoal había quedado enganchado (`in_window_` es un latch) cuando la pose estimada pasó por d ≤ 0.15, y el ruido la llevó después a 0.173 m. El `reset()` libera el latch, el robot vuelve a trasladarse (d final 0.144 / 0.085), sale de la pose bloqueada y completa el giro.
  - **S2:T045**: la primera diferencia de comando aparece a 1.2 s, en el ciclo siguiente al primer `setPlan`, por el reset de critics y del generador de trayectorias. Esa perturbación mínima basta para que el robot entre en N1 de otra forma y se quede sin trayectorias legales. En fixed_plan, N1 ya funcionaba con 1 trayectoria legal de 411.
- **El factor dominante es el reset, no la geometría**: `replan_same_1hz` (misma geometría, solo reset) reproduce los mismos cambios en los 3 aborts.
- **Otros cambios de resultado con replan_1hz**, todos de falla a falla: S1:T025 de TIMEOUT a OSCILACIÓN, y S2:T029 de OSCILACIÓN a TRACKING. Con replan_same: S5:T023 de OSCILACIÓN a OK.
- **Más cambios de signo de angular.z con replan** (336 → 553, +65 %). Se concentran en los casos atascados u oscilantes (S1:T025 1 → 24 y 99 con same; S2:T029 13 → 99; S1:T041 14 → 40; S5:T023 19 → 51) y en algunos correctos largos con replan_1hz (S1:T015 14 → 26, S5:T019 27 → 45: corrección de rumbo sobre el path nuevo).

## EVIDENCIA

- La sensibilidad de N1 (S2:T045, y S1:T045 en v1) es la misma que se vio en R4-A2 y R4-G: con el padding de 0.02 y celdas de 0.05 m, en N1 quedan 1–2 trayectorias legales de 411. Cualquier perturbación (un reset, un path corrido una celda) puede cambiar el resultado. Esto pesa en cualquier conclusión estadística sobre N1.
- El aumento de cambios de signo indica que el reset periódico **debilita la histéresis de Oscillation** (vuelve a permitir invertir el giro cada ≤ 1 s). En los casos atascados se ve más vaivén de ω, no menos.

## HIPÓTESIS

- En el robot real, la capa de obstáculos (LiDAR) del local y del global costmap agrega variación en cada ciclo, que aquí no existe. Sería otra fuente de ruptura de latches, y no está medida.
- El tiempo exacto del replan dentro del ciclo (BT a 100 Hz, planificación de algunos ms) puede desplazar el `setPlan` hasta 1 ciclo. Aquí ocurre siempre en el borde.

## Recovery / BT (sin ejecutar, auditoría de código 1.1.20)

**Qué sigue a un fallo de FollowPath** (BT por defecto, `RecoveryNode` y `RoundRobin` según `recovery_node.cpp` y `round_robin_node.cpp`):
1. `RecoveryNode "FollowPath"` (1 reintento): ClearLocalCostmap-Context, luego un **segundo FollowPath**. Un goal nuevo reinicia también el progress checker (línea 383), así que un atasco cuesta otros 25 s.
2. Si falla de nuevo, falla `PipelineSequence` y el `RecoveryNode "NavigateRecovery"` (6 reintentos) ejecuta el `RoundRobin`. Si un hijo falla, el RoundRobin pasa al siguiente en el mismo tick.

   | intento | recovery |
   |---|---|
   | 1 | ClearingActions (local + global) |
   | 2 | Spin 1.57 rad |
   | 3 | Wait 5 s |
   | 4 | BackUp 0.30 m a 0.05 m/s |
   | 5 | ClearingActions |
   | 6 | Spin |
3. Si los 7 intentos de navegación fallan (2 FollowPath cada uno), NavigateToPose termina ABORTED. Para un atasco persistente como S1:T041, eso son unos 7 × 2 × 25 s ≈ **6 min** girando cerca de la salida de N1, más los recoveries.

**Chequeo de colisión de los recoveries** (`spin.cpp`, `drive_on_heading.hpp`, `costmap_topic_collision_checker.cpp`, `footprint_collision_checker.cpp`):
- **Footprint completo**: sí. Spin y BackUp consultan `CostmapTopicCollisionChecker` con `local_costmap/published_footprint` (footprint B con padding 0.02) sobre `local_costmap/costmap_raw`.
- **Criterio**: colisión si el costo del footprint es ≥ LETHAL (254), lo que incluye NO_INFORMATION (255). **INSCRIBED (253) no cuenta como colisión**: el footprint con padding puede entrar en la zona inscrita. Es el mismo margen que ObstacleFootprint, ni más ni menos conservador.
- **BackUp**: proyecta en línea recta según el rumbo actual, `x = v·k/10` para k = 0…19, con `simulate_ahead_time` 2.0 y `cycle_frequency` 10.
  - A 0.05 m/s la proyección máxima es **0.095 m**, recortada además a la distancia restante.
  - Si detecta colisión, `stopRobot()` y FAILED ("Collision Ahead").
- **Spin**: proyecta la rotación a la velocidad comandada. La velocidad es `sqrt(2·1.5·restante)`, limitada a [0.2, 0.6] rad/s, así que la proyección es de 0.38–1.14 rad, muestreada cada 0.02–0.06 rad. Gira siempre en sentido antihorario (`spin_dist` +1.57).

**Limitaciones de un lookahead de ~0.10 m**:
- Cinemáticamente alcanza: a 0.05 m/s, la distancia de frenado más ~50 ms de latencia es de unos pocos mm.
- El riesgo está en la **percepción y la geometría**:
  1. Supone movimiento recto con el rumbo actual, sin deriva.
  2. Solo considera LETHAL, no INSCRIBED.
  3. Depende de que el costmap local muestre la pared trasera. El costmap local real no tiene static layer (R4-A), así que depende del LiDAR, y la visibilidad trasera del LIMO no está verificada.
  4. Si un ClearEntireCostmap acaba de vaciar el costmap, hasta el siguiente update de LiDAR no se ve nada.
- **Spin** muestrea cada 0.06 rad. En R4-G se midieron ventanas angulares de casi-contacto de unos 0.06 rad en la salida de N1, que una rotación así podría cruzar entre dos muestras. En el interior de N1, un giro de 90° es geométricamente imposible: la esquina del footprint está a 0.272 m del centro y el medio ancho es 0.225 m. Solo el collision checker lo detendría.
- Geometría desde las poses de atasco (R4-G, `dwb_r4g/recovery_geometry.json`): con el rumbo de S2:T045 y S5:T041, un **BackUp de 0.30 m llegaría al contacto real a los 0.13 / 0.14 m** y entraría en el padding a 0.09–0.11 m. Con el chequeo de 0.095 m debería abortar al empezar, si el costmap ve la pared. Un Spin desde esas poses es geométricamente libre en S1 y S5; en S2, en sentido horario entra en el padding a los 0.38 rad.

## LIMITACIONES

- 25 casos elegidos a propósito entre los problemáticos (no representativos de la tasa global) y una sola semilla por caso.
- Sin BT real: se reproduce el replan a 1 Hz y su efecto en el controller (`setPlan`, reset del goal checker), pero no los recoveries ni el `ClearEntireCostmap` ni el reintento de FollowPath.
- Sin capa de obstáculos ni LiDAR. El global costmap se restaura en cada plan como aproximación al repintado real.
- Modelo cinemático mínimo, odometría perfecta, ruido de localización sintético y mapa sintético (las mismas de R4-B).

## Archivos

- **Simulador**: `src/limo_nav_offline_lab/src/dwb_closed_loop_sim.cpp`, con `plan_mode`, `replan_period`, `debug_eval_runs` y la restauración del global costmap.
- **Configs**: `config/offline_lab/dwb_offline_R4G0_replan_sim.yaml`, `dwb_offline_R4G0_manifest.json`.
- **Scripts**: `scripts/nav_offline_lab/{make_dwb_r4g0_config.py, verify_r4g0_configs.py, run_r4g0.sh, evaluate_dwb_r4g0.py}`. `dwb_r4a_audit.py` ahora permite `/global_costmap/global_costmap` en closedloop; el chequeo de Twist y acciones no cambió.
- **Resultados**: `results/navigation_offline_lab/dwb_r4g0/`: `sim_<S>_<modo>.jsonl` y `.eval.jsonl`, `audit_*`, `r4g0_cases.csv`, `r4g0_summary.json`, `config_verification.json`, más las versiones archivadas v1 y v2.
- **R4-G**: las fases 1–2 (`dwb_r4g/`: trazas, mecanismo y geometría) se hicieron con fixed_plan. El barrido de progress_checker se detuvo antes de terminar y está en `dwb_r4g/sweep_STOPPED_before_R4G0/`. **No usar.**

## Siguiente paso

1. **Rehacer R4-G sobre `replan_1hz`**:
   - re-correr la baseline R4-F completa (384) en replan_1hz;
   - después el barrido de progress_checker (radio 0.05 / 0.03; tiempo 35 / 45), de a una variable;
   - evaluar contra esa baseline, no contra fixed_plan.
2. Repetir N1 con varias semillas: un resultado único en N1 no es concluyente.
3. Revisar en R4-G el diagnóstico de mecanismo de S1:T041, que sí persiste con replan. La causa raíz sigue siendo el inicio del giro de RotateToGoal a 0.14 m del goal junto a la salida de N1, no el progress checker.
