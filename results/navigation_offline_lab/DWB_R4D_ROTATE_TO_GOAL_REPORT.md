# NAV-OFFLINE-R4-D — RotateToGoal en lazo cerrado (DWB real + simulador R4-B)

Fecha: 2026-09-29. 100 % offline (ROS_DOMAIN_ID=99, ROS_LOCALHOST_ONLY=1). Sin hardware, sin limo_base, sin LiDAR, sin `/cmd_vel` (simulador sin publishers Twist; auditoría del grafo en cada corrida). `navigation2_real.yaml` sin cambios (sha256 `dfb36e108d6872a5…`), `~/limoatwork` sin cambios.

## Resumen

```
R4-D
Sin RotateToGoal:  correct successes 193/384
Con RotateToGoal:  correct successes 269/384
Cambio yaw:        error yaw final (corridas que llegan en xy) mediana 0.194 → 0.212 rad, p90 2.229 → 0.271 rad
Cambio oscilaciones: inversiones de ω tras llegar en xy (media) 8.3 → 0.6; por minuto (mediana) 36.8 → 31.2
```

## Método

- **Config experimental** `config/offline_lab/navigation2_R4D.yaml`: copia completa de `navigation2_real.yaml`. Cambios verificados por diff de dicts: scale, scale, critics; critics = `['RotateToGoal', 'Oscillation', 'ObstacleFootprint', 'GoalAlign', 'PathAlign', 'PathDist', 'GoalDist']`. Es decir, footprint B (ObstacleFootprint, R4-A2) + `RotateToGoal` en la posición de la línea comentada original, con sus parámetros ya presentes en la config real (scale 32, slowing_factor 5, lookahead_time −1; usa `xy_goal_tolerance` 0.25 y `trans_stopped_velocity` 0.03 de FollowPath).
- **Simulador**: `config/offline_lab/dwb_offline_R4D_fp_rtg.yaml` = config R4-B variante B + RotateToGoal. Assert: su `controller_server` es idéntico al de `navigation2_R4D.yaml`.
- **BASELINE** = variante B de R4-B (ObstacleFootprint, sin RotateToGoal), **reutilizada**. Antes se verificó que el binario actual la reproduce idéntica paso a paso (16/16 corridas en S1 y S5, `dwb_r4d/sim_chk_base_*.jsonl`). **EXPERIMENTAL** = misma config + RotateToGoal, mismos 64 paths, mismos 6 escenarios, mismos archivos de corridas y mismas semillas de ruido (comparación pareada).
- **Clasificación**: reglas idénticas a R4-C (assert: la baseline reproduce exactamente las clases de R4-C para B). "Correcto" = SUCCEEDED, sin contacto, pose **verdadera** final ≤ 0.25 m y ≤ 0.25 rad. "Bloqueos" = OSCILACIÓN + TIMEOUT + sin trayectorias legales.

## CONFIRMADO

- `dwb_critics::RotateToGoalCritic` se carga con la config experimental (`Using critic "RotateToGoal"`), junto con los otros 6 critics.
- Baseline reproducible: 16/16 corridas idénticas a R4-B con el binario actual; clasificación de la baseline = R4-C (assert).
- **Mecanismo de RotateToGoal observado**: al entrar en `FollowPath.xy_goal_tolerance` (0.25 m, el mismo valor que el goal checker) deja de trasladar y gira hacia el yaw del goal a baja velocidad (`slowing_factor`). Consecuencias medidas abajo: el robot **se detiene en el borde de la ventana xy** (error xy final ≈ 0.22 m en lugar de ≈ 0.03 m) y dispara el goal checker con |ω| ≈ 0.16 rad/s en lugar de ≈ 0.64 rad/s. El sobrepaso de yaw al frenar prácticamente desaparece.
- **No cambia la seguridad ni los bloqueos**: mismos 2 contactos (mismas corridas, S4 latencia 0.6 s) y mismos 46 bloqueos (OSCILACIÓN 23, TIMEOUT 9, sin trayectorias 14: around_obstacle y corridor_N1). RotateToGoal actúa solo dentro de 0.25 m del goal; esos robots nunca llegan ahí.
- **Nuevo modo de falla**: al terminar en el borde de la tolerancia **estimada**, el ruido de localización deja la pose **verdadera** apenas afuera (máximo 0.31 m / 0.36 rad). 39 corridas OK pasan a "éxito aparente", todas en escenarios con ruido (S2, S3, S5). 7 corridas OK de corridor_N1 con ruido pasan a GOAL_TOLERANCE.


## MEDIDO

### Clases (384 corridas por variante)

| clase | sin RotateToGoal | con RotateToGoal |
|---|---|---|
| OK | 193 | 269 |
| GOAL_TOLERANCE | 99 | 13 |
| OSCILACION | 23 | 23 |
| TIMEOUT | 9 | 9 |
| OTRO_SIN_TRAYECT | 14 | 14 |
| OTRO_EXITO_APARENTE | 46 | 56 |
| **contactos** | 2 | 2 |
| **bloqueos** | 46 | 46 |

### Por escenario: correctos / GOAL_TOLERANCE / éxitos aparentes / bloqueos / contactos

| escenario | sin RTG | con RTG |
|---|---|---|
| S1_nominal | 15 / 41 / 0 / 8 / 0 | 52 / 4 / 0 / 8 / 0 |
| S2_ruido_xy | 53 / 1 / 1 / 9 / 0 | 42 / 2 / 11 / 9 / 0 |
| S3_ruido_yaw | 24 / 29 / 2 / 9 / 0 | 35 / 3 / 17 / 9 / 0 |
| S4_latencia_0p6 | 23 / 28 / 5 / 8 / 2 | 56 / 0 / 0 / 8 / 2 |
| S5_combinado_medido | 56 / 0 / 3 / 5 / 0 | 28 / 4 / 27 / 5 / 0 |
| S6_combinado_lat0p6 | 22 / 0 / 35 / 7 / 0 | 56 / 0 / 1 / 7 / 0 |

### Precisión final, tiempo, oscilaciones y comandos

| métrica | sin RotateToGoal | con RotateToGoal |
|---|---|---|
| error xy final, corridas que llegan en xy: mediana / p90 (m) | 0.026 / 0.180 | 0.217 / 0.242 |
| error yaw final, corridas que llegan en xy: mediana / p90 (rad) | 0.194 / 2.229 | 0.212 / 0.271 |
| error xy final, correctas: mediana / p90 (m) | 0.042 / 0.227 | 0.200 / 0.236 |
| error yaw final, correctas: mediana / p90 (rad) | 0.098 / 0.212 | 0.178 / 0.239 |
| tiempo hasta SUCCEEDED: mediana / p90 (s) | 29.5 / 65.2 | 23.4 / 59.6 |
| tiempo hasta llegada xy: mediana / p90 (s) | 16.0 / 55.7 | 16.0 / 55.7 |
| tiempo de SUCCEEDED − llegada xy: mediana / p90 (s) | 7.5 / 15.9 | 3.1 / 4.7 |
| inversiones de ω por minuto: mediana / p90 | 36.8 / 55.4 | 31.2 / 53.7 |
| inversiones de ω tras llegar en xy: media / p90 | 8.3 / 19.0 | 0.6 / 0.0 |
| |ω| real al disparar el goal checker: mediana / máx (rad/s) | 0.64 / 1.00 | 0.16 / 0.37 |
| sobrepaso de yaw al frenar: mediana / máx (rad) | -0.043 / 0.601 | -0.008 / 0.125 |
| error lateral medio por corrida: mediana (m) | 0.032 / 0.087 | 0.032 / 0.087 |
| tiempo DWB por ciclo, media por corrida: mediana / máx (ms) | 12.06 / 13.15 | 12.07 / 13.16 |
| clearance mínima (m) | -0.009 | -0.009 |
| corridas cuyo último comando no nulo es giro en el lugar | 278 | 318 |
| fallas de DWB (ciclos sin trayectoria legal) | 82 | 71 |
| tiempo DWB máximo por ciclo (ms) | 27.9 | 28.0 |

### Comparación pareada (mismo path, escenario y semilla)

- Pares: 384. Arreglados (no OK → OK): **122**. Rotos (OK → no OK): **46** (S2_ruido_xy:T005:OTRO_EXITO_APARENTE, S2_ruido_xy:T008:OTRO_EXITO_APARENTE, S2_ruido_xy:T016:OTRO_EXITO_APARENTE, S2_ruido_xy:T034:OTRO_EXITO_APARENTE, S2_ruido_xy:T037:OTRO_EXITO_APARENTE, S2_ruido_xy:T041:GOAL_TOLERANCE, S2_ruido_xy:T045:GOAL_TOLERANCE, S2_ruido_xy:T047:OTRO_EXITO_APARENTE, S2_ruido_xy:T048:OTRO_EXITO_APARENTE, S2_ruido_xy:T049:OTRO_EXITO_APARENTE, S2_ruido_xy:T057:OTRO_EXITO_APARENTE, S3_ruido_yaw:T007:OTRO_EXITO_APARENTE, S3_ruido_yaw:T012:OTRO_EXITO_APARENTE, S3_ruido_yaw:T036:OTRO_EXITO_APARENTE, S3_ruido_yaw:T045:GOAL_TOLERANCE, S3_ruido_yaw:T047:OTRO_EXITO_APARENTE, S3_ruido_yaw:T053:OTRO_EXITO_APARENTE, S5_combinado_medido:T003:OTRO_EXITO_APARENTE, S5_combinado_medido:T004:OTRO_EXITO_APARENTE, S5_combinado_medido:T006:OTRO_EXITO_APARENTE, S5_combinado_medido:T010:OTRO_EXITO_APARENTE, S5_combinado_medido:T011:OTRO_EXITO_APARENTE, S5_combinado_medido:T015:OTRO_EXITO_APARENTE, S5_combinado_medido:T016:OTRO_EXITO_APARENTE, S5_combinado_medido:T017:OTRO_EXITO_APARENTE, S5_combinado_medido:T018:OTRO_EXITO_APARENTE, S5_combinado_medido:T020:OTRO_EXITO_APARENTE, S5_combinado_medido:T022:OTRO_EXITO_APARENTE, S5_combinado_medido:T024:OTRO_EXITO_APARENTE, S5_combinado_medido:T033:OTRO_EXITO_APARENTE, S5_combinado_medido:T034:OTRO_EXITO_APARENTE, S5_combinado_medido:T036:OTRO_EXITO_APARENTE, S5_combinado_medido:T038:OTRO_EXITO_APARENTE, S5_combinado_medido:T041:GOAL_TOLERANCE, S5_combinado_medido:T042:GOAL_TOLERANCE, S5_combinado_medido:T043:OTRO_EXITO_APARENTE, S5_combinado_medido:T044:GOAL_TOLERANCE, S5_combinado_medido:T045:OTRO_EXITO_APARENTE, S5_combinado_medido:T046:GOAL_TOLERANCE, S5_combinado_medido:T052:OTRO_EXITO_APARENTE, S5_combinado_medido:T053:OTRO_EXITO_APARENTE, S5_combinado_medido:T056:OTRO_EXITO_APARENTE, S5_combinado_medido:T058:OTRO_EXITO_APARENTE, S5_combinado_medido:T060:OTRO_EXITO_APARENTE, S5_combinado_medido:T061:OTRO_EXITO_APARENTE, S5_combinado_medido:T063:OTRO_EXITO_APARENTE).
- Contactos nuevos con RotateToGoal: 0 (—); contactos que desaparecen: 0 (—).
- Pares OK en ambas: Δt de SUCCEEDED (RTG − base) mediana -2.6 s; Δ error xy final mediana 0.102 m (p90 0.214).

| transición (sin RTG → con RTG) | corridas |
|---|---|
| OK->OK | 147 |
| GOAL_TOLERANCE->OK | 82 |
| OTRO_EXITO_APARENTE->OK | 40 |
| OK->OTRO_EXITO_APARENTE | 39 |
| OSCILACION->OSCILACION | 23 |
| OTRO_SIN_TRAYECT->OTRO_SIN_TRAYECT | 14 |
| GOAL_TOLERANCE->OTRO_EXITO_APARENTE | 12 |
| TIMEOUT->TIMEOUT | 9 |
| OK->GOAL_TOLERANCE | 7 |
| GOAL_TOLERANCE->GOAL_TOLERANCE | 5 |
| OTRO_EXITO_APARENTE->OTRO_EXITO_APARENTE | 5 |
| OTRO_EXITO_APARENTE->GOAL_TOLERANCE | 1 |

### Por categoría de path: correctos sin RTG → con RTG

| categoría | correctos | GOAL_TOLERANCE | bloqueos | contactos |
|---|---|---|---|---|
| around_obstacle | 13 → 21 | 8 → 0 | 35 → 35 | 0 → 0 |
| corridor_N1 | 24 → 22 | 7 → 13 | 11 → 11 | 2 → 2 |
| corridor_N2 | 26 → 39 | 16 → 0 | 0 → 0 | 0 → 0 |
| corridor_WIDE | 30 → 39 | 13 → 0 | 0 → 0 | 0 → 0 |
| long | 36 → 49 | 14 → 0 | 0 → 0 | 0 → 0 |
| random | 31 → 51 | 19 → 0 | 0 → 0 | 0 → 0 |
| short | 33 → 48 | 22 → 0 | 0 → 0 | 0 → 0 |

Figura: `dwb_r4d/r4d_overview.png`. Datos por corrida: `dwb_r4d/r4d_runs.csv`.

## LIMITACIONES

- Mismas del simulador R4-B: modelo cinemático mínimo, odometría perfecta, percepción perfecta (mapa estático), sin BT (plan fijo, sin recuperaciones), lógica de controller_server replicada, una semilla por caso, ruido de localización sintético, mapa sintético.
- Solo se probó RotateToGoal con sus parámetros existentes (scale 32, slowing_factor 5, lookahead −1) y con las tolerancias actuales (xy 0.25, yaw 0.25). No se exploraron otros valores.
- Solo con ObstacleFootprint (footprint B). No se midió BaseObstacle + RotateToGoal.
- La precisión final depende de `xy_goal_tolerance`, que aquí vale lo mismo para RotateToGoal y para el goal checker (0.25 m).

## Archivos

- Configs: `config/offline_lab/navigation2_R4D.yaml` (candidata completa, NO producción), `dwb_offline_R4D_fp_rtg.yaml` (simulador), `dwb_offline_R4D_manifest.json`
- Scripts: `scripts/nav_offline_lab/{make_dwb_r4d_config.py, run_r4d_all.sh, evaluate_dwb_r4d.py, make_dwb_r4d_report.py}`
- Resultados: `results/navigation_offline_lab/dwb_r4d/` (sim_*_R.jsonl, sim_chk_*.jsonl, r4d_runs.csv, r4d_summary.json, r4d_overview.png, auditorías)

## Recomendación

**Adoptar RotateToGoal como parte de la configuración candidata, pero no llevarlo a producción todavía.** Resuelve la causa dominante de R4-C (GOAL_TOLERANCE 99 → 13; +76 éxitos correctos netos) sin costo de seguridad (mismos contactos, mismos bloqueos) y elimina las oscilaciones en el goal. Su efecto secundario es real: la precisión final en posición pasa a ser la tolerancia (~0.22 m), y con ruido de localización termina en el borde.

**Siguiente ajuste (R4-E, offline, mismo simulador)**: separar las dos tolerancias, bajando `FollowPath.xy_goal_tolerance` (la que usa RotateToGoal) a 0.10–0.15 m y manteniendo el goal checker en 0.25 m / 0.25 rad. Objetivo: que el robot traslade más cerca del goal antes de girar, deje margen dentro de la ventana del checker y recupere las 39 + 7 corridas rotas sin perder las 122 arregladas. Evaluarlo además con BaseObstacle + RotateToGoal para separar efectos.

Los 46 bloqueos (around_obstacle, corridor_N1) necesitan un trabajo aparte sobre los pesos GoalDist/GoalAlign vs PathDist/PathAlign. RotateToGoal no los toca. Cualquier cambio a `navigation2_real.yaml` requiere tu confirmación y una prueba física supervisada.

