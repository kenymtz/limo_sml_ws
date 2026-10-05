# NAV-OFFLINE-R4-E — FollowPath.xy_goal_tolerance de RotateToGoal (DWB real + simulador R4-B)

Fecha: 2026-10-02. 100 % offline (ROS_DOMAIN_ID=99, ROS_LOCALHOST_ONLY=1). Sin hardware, sin limo_base, sin LiDAR, sin teleop, sin `/cmd_vel` (simulador sin publishers Twist; auditoría del grafo PASS en las 18 tandas). `config/navigation/navigation2_real.yaml` sin cambios (sha256 `dfb36e108d6872a5…`, igual al registrado en R4-D). `~/limoatwork` sin cambios.

## Resumen

```
R4-E (384 corridas por variante; general_goal_checker 0.25 m / 0.25 rad sin cambios)
0.25 (R4-D): 269/384 correctos   GoalTol 13   falsos éxitos 56
0.20       : 291/384             GoalTol  4   falsos éxitos 43
0.15       : 310/384             GoalTol  1   falsos éxitos 27
0.10       : 303/384             GoalTol  0   falsos éxitos 35
Contactos 2 / 2 / 2 / 2 (mismas corridas). Bloqueos 46 / 46 / 46 / 46 (mismas corridas).
```

**Mejor compromiso: 0.15 m**, con 0.10 m estadísticamente indistinguible en correctos (ver *Selección*).

## Método

- **Variable única**: `controller_server.FollowPath.xy_goal_tolerance` ∈ {0.20, 0.15, 0.10}. Es el radio en el que `RotateToGoalCritic` pasa a "dejar de trasladar y girar". Verificado que en DWB Humble solo lo lee `RotateToGoal` (`libdwb_critics.so` contiene una sola referencia; `dwb_core` y `standard_traj_generator` ninguna; el goal checker lee `general_goal_checker.xy_goal_tolerance`, y el simulador también: `dwb_closed_loop_sim.cpp:139`).
- **Configs** (generadas por `make_dwb_r4e_config.py` desde las de R4-D, que no se tocan):
  - candidatas completas `config/offline_lab/navigation2_R4E_{020,015,010}.yaml` (sha256 `f063568b…`, `cc773d1c…`, `9951437c…`). NO son producción.
  - simulador `config/offline_lab/dwb_offline_R4E_{020,015,010}_sim.yaml`; manifest `dwb_offline_R4E_manifest.json`.
- **Verificador automático** `scripts/nav_offline_lab/verify_r4e_configs.py` (también corre como paso previo obligatorio de `run_r4e_all.sh`):
  - diff de dicts aplanados vs R4-D == `[controller_server.ros__parameters.FollowPath.xy_goal_tolerance]` para cada candidata completa y cada config de simulador;
  - checker 0.25/0.25 en todas;
  - `controller_server` del simulador == `controller_server` de la candidata completa;
  - producción con el sha256 de R4-D;
  - con `--runs`: `meta` de cada `sim_*.jsonl` idéntica a R4-D (escenario, ruido, latencia, k_v/k_w, timeout de cmd, límites, checker, progress checker), mismos 64 runs, mismo orden y mismos goals.

  Resultado: **TODO PASS** (`dwb_r4e/config_verification.json`, `dwb_r4e/run_verification.json`).
- **Baseline 0.25 = R4-D existente, no re-ejecutada.** El binario `dwb_closed_loop_sim` (sha256 `63b484a0…`, compilado 2026-09-29 18:52) es anterior a las corridas de R4-D (19:11–19:38); mismo binario. El evaluador nuevo recalcula las clases de R4-B/R4-D y hace assert de que coinciden con `dwb_r4d/r4d_runs.csv` (384+384, sin diferencias) y de que los conjuntos 46 rotas / 122 arregladas son los de R4-D.
- **Ejecución**: `run_r4e_all.sh` (3 × 6 tandas de 64, ~3.7 min por tanda). **Evaluación**: `evaluate_dwb_r4e.py` (reglas de clasificación idénticas a R4-C/R4-D). **Tablas**: `make_dwb_r4e_report.py`.
- Definiciones:
  - "Correcto" = SUCCEEDED, sin contacto, pose **real** final ≤ 0.25 m y ≤ 0.25 rad.
  - "Falso éxito" = SUCCEEDED con la pose estimada dentro de tolerancia y la real fuera (clase OTRO_EXITO_APARENTE).
  - "Bloqueos" = OSCILACIÓN + TIMEOUT + sin trayectorias legales.
  - Pose final **estimada** = pose real final + offset de localización del último ciclo de control (el ruido se congela durante el frenado, ≤ 1 s).
  - Pareo por `scenario + path_id`: la semilla es fija por path y escenario (`dwb_r4b/runs_<S>.txt`).

## CONFIRMADO

- **Solo cambia la variable pedida** (verificador PASS, 24 checks de config + 54 de corridas).
- **Seguridad sin cambios**:
  - mismos 2 contactos en las 4 variantes: S4_latencia_0p6, corridor_N1, T042/T045; mismo tiempo en contacto (0.64 s / 0.46 s) y misma clearance mínima (−0.009 m);
  - los contactos ocurren a t = 3.6 / 4.6 s y esas corridas nunca entran en la ventana xy, así que son independientes de la tolerancia;
  - contactos nuevos: 0 en todas;
  - mismos 46 bloqueos (las mismas corridas): bloqueos nuevos 0 y eliminados 0 en todas.
- **Mecanismo**: bajar la tolerancia hace que RotateToGoal empiece a girar más cerca del goal. Distancia real al goal al empezar el giro (p50): 0.232 → 0.188 → 0.141 → 0.095 m. El robot sigue trasladando dentro de la ventana del checker (p50 0.1 → 0.4 → 0.9 → 1.4 s) y después gira en el lugar.
- **Los falsos éxitos por posición prácticamente desaparecen** (23 con 0.25 → 3 / 2 / 2). Los que quedan son casi todos **por yaw**: 33 → 40 / 25 / 33. Con ruido de yaw (σ 0.05 rad), el checker dispara cuando el yaw **estimado** cruza 0.25 rad (yaw estimado final p95 ≈ 0.241 rad en todas las variantes) y el real queda afuera (p95 0.27–0.31). Esto **no depende de `FollowPath.xy_goal_tolerance`** y R4-E no puede resolverlo, porque tiene prohibido cambiar el checker.
- **Las "oscilaciones" que suben son correcciones de rumbo durante la traslación extra, no oscilación en el goal.** Las inversiones de ω después de entrar en la ventana de 0.25 m suben de 0.57 a 2.02 de media, incluso en S1 nominal sin ruido. Separadas (`dwb_r4e/osc_split.json`):
  - antes de empezar a girar: 0 → 0.5 → 1.2 → 2.1 (media de escenarios);
  - después de empezar a girar en el lugar: 0.67 → 0.29 → 0.09 → 0.03 por corrida.
- Corridas que vuelven a trasladar después de haber empezado el giro (señal de entrar y salir de la ventana de RTG): 84 → 68 → 68 → 37.

## MEDIDO
### Tabla principal (384 corridas por variante)

| Tolerance | Correct | GoalTol | False success | Contacts | Blocks | XY p90 | Yaw p90 |
|---|---|---|---|---|---|---|---|
| sin RTG (R4-B) | 193 | 99 | 46 | 2 | 46 | 0.180 | 2.229 |
| 0.25 (R4-D) | 269 | 13 | 56 | 2 | 46 | 0.242 | 0.271 |
| 0.20 | 291 | 4 | 43 | 2 | 46 | 0.225 | 0.264 |
| 0.15 | 310 | 1 | 27 | 2 | 46 | 0.193 | 0.242 |
| 0.10 | 303 | 0 | 35 | 2 | 46 | 0.181 | 0.246 |

XY / Yaw p90 = error final VERDADERO (m / rad) de las corridas que llegan en xy (ventana del checker, pose estimada). "False success" = SUCCEEDED según pose estimada con pose verdadera fuera de 0.25 m / 0.25 rad (clase OTRO_EXITO_APARENTE). "Blocks" = OSCILACIÓN + TIMEOUT + sin trayectorias legales.

### Clases

| clase | 0.25 (R4-D) | 0.20 | 0.15 | 0.10 |
|---|---|---|---|---|
| OK | 269 | 291 | 310 | 303 |
| GOAL_TOLERANCE | 13 | 4 | 1 | 0 |
| OSCILACION | 23 | 23 | 23 | 23 |
| TIMEOUT | 9 | 9 | 9 | 9 |
| OTRO_SIN_TRAYECT | 14 | 14 | 14 | 14 |
| OTRO_EXITO_APARENTE | 56 | 43 | 27 | 35 |

### Precisión final: p50 / p90 / p95 (corridas que llegan en xy)

| métrica | 0.25 (R4-D) | 0.20 | 0.15 | 0.10 |
|---|---|---|---|---|
| error XY final REAL (m) | 0.217 / 0.242 / 0.261 | 0.180 / 0.225 / 0.236 | 0.136 / 0.193 / 0.232 | 0.093 / 0.181 / 0.232 |
| error XY final ESTIMADO (m) | 0.219 / 0.246 / 0.262 | 0.181 / 0.229 / 0.236 | 0.136 / 0.212 / 0.231 | 0.094 / 0.202 / 0.229 |
| error yaw final REAL (rad) | 0.212 / 0.271 / 0.311 | 0.204 / 0.264 / 0.291 | 0.194 / 0.242 / 0.274 | 0.206 / 0.246 / 0.274 |
| error yaw final ESTIMADO (rad) | 0.219 / 0.240 / 0.242 | 0.220 / 0.240 / 0.241 | 0.213 / 0.237 / 0.240 | 0.216 / 0.238 / 0.241 |
| error XY REAL, solo correctas (m) | 0.200 / 0.236 / 0.242 | 0.176 / 0.225 / 0.236 | 0.135 / 0.193 / 0.232 | 0.092 / 0.185 / 0.232 |
| error yaw REAL, solo correctas (rad) | 0.178 / 0.239 / 0.241 | 0.184 / 0.237 / 0.241 | 0.184 / 0.237 / 0.240 | 0.188 / 0.239 / 0.242 |

n de corridas que llegan en xy: 0.25 (R4-D) 338, 0.20 338, 0.15 338, 0.10 338.

### Tiempo, comportamiento cerca del goal, oscilaciones y comandos finales

| métrica | 0.25 (R4-D) | 0.20 | 0.15 | 0.10 |
|---|---|---|---|---|
| tiempo hasta SUCCEEDED p50 / p90 / p95 (s) | 23.4 / 59.6 / 61.5 | 19.3 / 59.8 / 61.8 | 19.0 / 60.2 / 62.1 | 19.8 / 60.6 / 62.8 |
| SUCCEEDED − entrada en ventana xy p50 / p90 / p95 (s) | 3.1 / 4.7 / 5.0 | 3.5 / 5.2 / 5.5 | 4.0 / 5.6 / 6.0 | 4.6 / 6.5 / 6.8 |
| tiempo dentro de 0.25 m del goal (est.) p50 / p90 / p95 (s) | 2.9 / 4.8 / 5.6 | 3.5 / 5.3 / 5.7 | 4.1 / 5.7 / 6.2 | 4.7 / 6.6 / 6.9 |
|   … girando en el lugar p50 / p90 / p95 (s) | 2.8 / 4.7 / 5.4 | 3.0 / 4.8 / 5.1 | 3.1 / 4.9 / 5.0 | 3.2 / 4.9 / 5.1 |
|   … detenido (v y ω ≈ 0) p50 / p90 / p95 (s) | 0.0 / 0.0 / 0.0 | 0.0 / 0.0 / 0.0 | 0.0 / 0.0 / 0.0 | 0.0 / 0.0 / 0.0 |
|   … trasladando p50 / p90 / p95 (s) | 0.1 / 0.5 / 0.8 | 0.4 / 0.8 / 1.0 | 0.9 / 1.3 / 1.5 | 1.4 / 2.0 / 2.2 |
| distancia REAL al goal al empezar a girar p50 / p90 / p95 (m) | 0.232 / 0.263 / 0.285 | 0.188 / 0.210 / 0.231 | 0.141 / 0.161 / 0.172 | 0.095 / 0.111 / 0.123 |
| sobrepaso de yaw al frenar p50 / p90 / p95 (rad) | -0.008 / 0.000 / 0.003 | -0.008 / 0.000 / 0.003 | -0.008 / 0.000 / 0.003 | -0.008 / 0.000 / 0.003 |
| |ω| al disparar el checker p50 / p90 / p95 (rad/s) | 0.16 / 0.26 / 0.26 | 0.16 / 0.26 / 0.26 | 0.16 / 0.26 / 0.34 | 0.16 / 0.26 / 0.26 |
| inversiones de ω tras llegar en xy p50 / p90 / p95 | 0.0 / 0.0 / 1.1 | 0.0 / 2.0 / 3.0 | 1.0 / 3.0 / 5.0 | 1.0 / 5.0 / 7.0 |
| inversiones de ω por minuto p50 / p90 / p95 | 31.2 / 53.7 / 61.6 | 31.7 / 53.5 / 64.1 | 32.4 / 55.6 / 64.3 | 33.6 / 57.4 / 65.8 |
| inversiones de ω tras llegar en xy, media | 0.57 | 0.89 | 1.32 | 2.02 |
| corridas cuyo último comando no nulo es giro en el lugar | 318 | 316 | 315 | 314 |
| ciclos DWB sin trayectoria legal | 71 | 71 | 71 | 71 |
| tiempo DWB por ciclo p50 (ms) | 12.07 | 12.08 | 12.06 | 12.04 |

### Por escenario: correctos / GoalTol / falsos éxitos / bloqueos / contactos

| escenario | 0.25 (R4-D) | 0.20 | 0.15 | 0.10 |
|---|---|---|---|---|
| S1_nominal | 52 / 4 / 0 / 8 / 0 | 55 / 1 / 0 / 8 / 0 | 56 / 0 / 0 / 8 / 0 | 56 / 0 / 0 / 8 / 0 |
| S2_ruido_xy | 42 / 2 / 11 / 9 / 0 | 53 / 0 / 2 / 9 / 0 | 54 / 0 / 1 / 9 / 0 | 54 / 0 / 1 / 9 / 0 |
| S3_ruido_yaw | 35 / 3 / 17 / 9 / 0 | 40 / 1 / 14 / 9 / 0 | 49 / 0 / 6 / 9 / 0 | 43 / 0 / 12 / 9 / 0 |
| S4_latencia_0p6 | 56 / 0 / 0 / 8 / 2 | 56 / 0 / 0 / 8 / 2 | 56 / 0 / 0 / 8 / 2 | 56 / 0 / 0 / 8 / 2 |
| S5_combinado_medido | 28 / 4 / 27 / 5 / 0 | 31 / 2 / 26 / 5 / 0 | 39 / 1 / 19 / 5 / 0 | 39 / 0 / 20 / 5 / 0 |
| S6_combinado_lat0p6 | 56 / 0 / 1 / 7 / 0 | 56 / 0 / 1 / 7 / 0 | 56 / 0 / 1 / 7 / 0 | 55 / 0 / 2 / 7 / 0 |

### Corredores estrechos N1 / N2 (48 corridas cada uno = 8 paths × 6 escenarios)

| corredor | variante | correctos | GoalTol | falsos éxitos | bloqueos | contactos | tiempo en contacto (s) | corridas < padding | clearance mín (m) | clearance p05 (m) |
|---|---|---|---|---|---|---|---|---|---|---|
| corridor_N1 | 0.25 (R4-D) | 22 | 13 | 2 | 11 | 2 | 1.10 | 14 | -0.009 | 0.010 |
| corridor_N1 | 0.20 | 29 | 4 | 4 | 11 | 2 | 1.10 | 12 | -0.009 | 0.010 |
| corridor_N1 | 0.15 | 34 | 1 | 2 | 11 | 2 | 1.10 | 12 | -0.009 | 0.010 |
| corridor_N1 | 0.10 | 34 | 0 | 3 | 11 | 2 | 1.10 | 12 | -0.009 | 0.010 |
| corridor_N2 | 0.25 (R4-D) | 39 | 0 | 9 | 0 | 0 | 0.00 | 0 | 0.025 | 0.026 |
| corridor_N2 | 0.20 | 43 | 0 | 5 | 0 | 0 | 0.00 | 0 | 0.025 | 0.026 |
| corridor_N2 | 0.15 | 45 | 0 | 3 | 0 | 0 | 0.00 | 0 | 0.025 | 0.026 |
| corridor_N2 | 0.10 | 42 | 0 | 6 | 0 | 0 | 0.00 | 0 | 0.025 | 0.026 |

### Por categoría: correctos (GoalTol / falsos éxitos)

| categoría | 0.25 (R4-D) | 0.20 | 0.15 | 0.10 |
|---|---|---|---|---|
| around_obstacle | 21 (0 / 4) | 22 (0 / 3) | 22 (0 / 3) | 23 (0 / 2) |
| corridor_N1 | 22 (13 / 2) | 29 (4 / 4) | 34 (1 / 2) | 34 (0 / 3) |
| corridor_N2 | 39 (0 / 9) | 43 (0 / 5) | 45 (0 / 3) | 42 (0 / 6) |
| corridor_WIDE | 39 (0 / 9) | 44 (0 / 4) | 46 (0 / 2) | 46 (0 / 2) |
| long | 49 (0 / 11) | 51 (0 / 9) | 53 (0 / 7) | 51 (0 / 9) |
| random | 51 (0 / 9) | 52 (0 / 8) | 54 (0 / 6) | 54 (0 / 6) |
| short | 48 (0 / 12) | 50 (0 / 10) | 56 (0 / 4) | 53 (0 / 7) |

### Comparación pareada contra R4-D 0.25 (clave scenario + path_id; semilla fija por path y escenario)

| variante | FIXED | BROKEN | UNCHANGED_OK | UNCHANGED_FAIL | 46 rotas por R4-D recuperadas | 122 arregladas por R4-D que se rompen | contactos nuevos | bloqueos nuevos / eliminados |
|---|---|---|---|---|---|---|---|---|
| 0.20 | 32 | 10 | 259 | 83 | 20/46 | 6/122 | 0 | 0 / 0 |
| 0.15 | 46 | 5 | 264 | 69 | 30/46 | 3/122 | 0 | 0 / 0 |
| 0.10 | 46 | 12 | 257 | 69 | 32/46 | 5/122 | 0 | 0 / 0 |

**0.20**: de las 46 rotas por R4-D siguen fallando 26 (GOAL_TOLERANCE 2, OTRO_EXITO_APARENTE 24). Pares OK en ambas: Δ error xy REAL p50 -0.025 m, Δ yaw REAL p50 0.000 rad, Δ t SUCCEEDED p50 0.2 s. Δ clearance mínima por corrida en N1/N2 (variante − 0.25) p50 0.000 m, mín -0.021 m.
**0.15**: de las 46 rotas por R4-D siguen fallando 16 (GOAL_TOLERANCE 1, OTRO_EXITO_APARENTE 15). Pares OK en ambas: Δ error xy REAL p50 -0.056 m, Δ yaw REAL p50 0.000 rad, Δ t SUCCEEDED p50 0.5 s. Δ clearance mínima por corrida en N1/N2 (variante − 0.25) p50 0.000 m, mín -0.016 m.
**0.10**: de las 46 rotas por R4-D siguen fallando 14 (OTRO_EXITO_APARENTE 14). Pares OK en ambas: Δ error xy REAL p50 -0.084 m, Δ yaw REAL p50 0.000 rad, Δ t SUCCEEDED p50 1.1 s. Δ clearance mínima por corrida en N1/N2 (variante − 0.25) p50 0.000 m, mín -0.004 m.

<details><summary>0.20: transiciones y listas</summary>

| transición (0.25 → 0.20) | corridas |
|---|---|
| OK->OK | 259 |
| OTRO_EXITO_APARENTE->OTRO_EXITO_APARENTE | 30 |
| OTRO_EXITO_APARENTE->OK | 26 |
| OSCILACION->OSCILACION | 23 |
| OTRO_SIN_TRAYECT->OTRO_SIN_TRAYECT | 14 |
| OK->OTRO_EXITO_APARENTE | 10 |
| TIMEOUT->TIMEOUT | 9 |
| GOAL_TOLERANCE->OK | 6 |
| GOAL_TOLERANCE->GOAL_TOLERANCE | 4 |
| GOAL_TOLERANCE->OTRO_EXITO_APARENTE | 3 |

BROKEN: S3_ruido_yaw:T014:OTRO_EXITO_APARENTE, S3_ruido_yaw:T015:OTRO_EXITO_APARENTE, S3_ruido_yaw:T054:OTRO_EXITO_APARENTE, S3_ruido_yaw:T058:OTRO_EXITO_APARENTE, S3_ruido_yaw:T060:OTRO_EXITO_APARENTE, S3_ruido_yaw:T064:OTRO_EXITO_APARENTE, S5_combinado_medido:T007:OTRO_EXITO_APARENTE, S5_combinado_medido:T014:OTRO_EXITO_APARENTE, S5_combinado_medido:T030:OTRO_EXITO_APARENTE, S5_combinado_medido:T048:OTRO_EXITO_APARENTE

122 re-rotas: S3_ruido_yaw:T014:OTRO_EXITO_APARENTE, S3_ruido_yaw:T015:OTRO_EXITO_APARENTE, S3_ruido_yaw:T054:OTRO_EXITO_APARENTE, S3_ruido_yaw:T058:OTRO_EXITO_APARENTE, S3_ruido_yaw:T060:OTRO_EXITO_APARENTE, S5_combinado_medido:T014:OTRO_EXITO_APARENTE

Contactos nuevos: —; eliminados: —

Bloqueos nuevos: —; eliminados: —

</details>

<details><summary>0.15: transiciones y listas</summary>

| transición (0.25 → 0.15) | corridas |
|---|---|
| OK->OK | 264 |
| OTRO_EXITO_APARENTE->OK | 35 |
| OSCILACION->OSCILACION | 23 |
| OTRO_EXITO_APARENTE->OTRO_EXITO_APARENTE | 21 |
| OTRO_SIN_TRAYECT->OTRO_SIN_TRAYECT | 14 |
| GOAL_TOLERANCE->OK | 11 |
| TIMEOUT->TIMEOUT | 9 |
| OK->OTRO_EXITO_APARENTE | 5 |
| GOAL_TOLERANCE->GOAL_TOLERANCE | 1 |
| GOAL_TOLERANCE->OTRO_EXITO_APARENTE | 1 |

BROKEN: S3_ruido_yaw:T054:OTRO_EXITO_APARENTE, S3_ruido_yaw:T058:OTRO_EXITO_APARENTE, S3_ruido_yaw:T061:OTRO_EXITO_APARENTE, S5_combinado_medido:T014:OTRO_EXITO_APARENTE, S5_combinado_medido:T030:OTRO_EXITO_APARENTE

122 re-rotas: S3_ruido_yaw:T054:OTRO_EXITO_APARENTE, S3_ruido_yaw:T058:OTRO_EXITO_APARENTE, S5_combinado_medido:T014:OTRO_EXITO_APARENTE

Contactos nuevos: —; eliminados: —

Bloqueos nuevos: —; eliminados: —

</details>

<details><summary>0.10: transiciones y listas</summary>

| transición (0.25 → 0.10) | corridas |
|---|---|
| OK->OK | 257 |
| OTRO_EXITO_APARENTE->OK | 35 |
| OSCILACION->OSCILACION | 23 |
| OTRO_EXITO_APARENTE->OTRO_EXITO_APARENTE | 21 |
| OTRO_SIN_TRAYECT->OTRO_SIN_TRAYECT | 14 |
| OK->OTRO_EXITO_APARENTE | 12 |
| GOAL_TOLERANCE->OK | 11 |
| TIMEOUT->TIMEOUT | 9 |
| GOAL_TOLERANCE->OTRO_EXITO_APARENTE | 2 |

BROKEN: S3_ruido_yaw:T009:OTRO_EXITO_APARENTE, S3_ruido_yaw:T015:OTRO_EXITO_APARENTE, S3_ruido_yaw:T058:OTRO_EXITO_APARENTE, S3_ruido_yaw:T060:OTRO_EXITO_APARENTE, S3_ruido_yaw:T061:OTRO_EXITO_APARENTE, S3_ruido_yaw:T063:OTRO_EXITO_APARENTE, S5_combinado_medido:T001:OTRO_EXITO_APARENTE, S5_combinado_medido:T030:OTRO_EXITO_APARENTE, S5_combinado_medido:T048:OTRO_EXITO_APARENTE, S5_combinado_medido:T051:OTRO_EXITO_APARENTE, S5_combinado_medido:T054:OTRO_EXITO_APARENTE, S6_combinado_lat0p6:T002:OTRO_EXITO_APARENTE

122 re-rotas: S3_ruido_yaw:T009:OTRO_EXITO_APARENTE, S3_ruido_yaw:T015:OTRO_EXITO_APARENTE, S3_ruido_yaw:T058:OTRO_EXITO_APARENTE, S3_ruido_yaw:T060:OTRO_EXITO_APARENTE, S6_combinado_lat0p6:T002:OTRO_EXITO_APARENTE

Contactos nuevos: —; eliminados: —

Bloqueos nuevos: —; eliminados: —

</details>


### Oscilación separada: antes y después de empezar a girar (media por corrida, corridas que llegan en xy y giran)

| variante | antes de girar (traslación dentro de 0.25 m) S1/S2/S3/S4/S5/S6 | después de empezar a girar S1/S2/S3/S4/S5/S6 | después, media global | corridas que vuelven a trasladar tras empezar el giro |
|---|---|---|---|---|
| 0.25 (R4-D) | 0 / 0 / 0 / 0 / 0 / 0 | 1.21 / 0.63 / 0.89 / 0.04 / 1.12 / 0.10 | 0.67 | 84 |
| 0.20 | 0.74 / 0.77 / 0.48 / 0.26 / 0.69 / 0.29 | 0.53 / 0.02 / 0.46 / 0.04 / 0.53 / 0.17 | 0.29 | 68 |
| 0.15 | 1.49 / 1.47 / 1.60 / 0.61 / 1.53 / 0.54 | 0 / 0 / 0.02 / 0.09 / 0.27 / 0.17 | 0.09 | 68 |
| 0.10 | 2.85 / 2.52 / 2.66 / 1.15 / 2.40 / 1.19 | 0.06 / 0.02 / 0.07 / 0.04 / 0 / 0 | 0.03 | 37 |

### Causa de los falsos éxitos (pose real fuera de tolerancia)

| variante | total | solo xy | solo yaw | xy + yaw | por escenario |
|---|---|---|---|---|---|
| 0.25 (R4-D) | 56 | 18 | 33 | 5 | S2 11, S3 17, S5 27, S6 1 |
| 0.20 | 43 | 3 | 40 | 0 | S2 2, S3 14, S5 26, S6 1 |
| 0.15 | 27 | 2 | 25 | 0 | S2 1, S3 6, S5 19, S6 1 |
| 0.10 | 35 | 2 | 33 | 0 | S2 1, S3 12, S5 20, S6 2 |

### Significancia pareada de correctos (McNemar exacto, mismas 384 corridas)

| comparación | pasan a OK | dejan de ser OK | p |
|---|---|---|---|
| 0.25 → 0.20 | 32 | 10 | 0.0009 |
| 0.25 → 0.15 | 46 | 5 | < 0.0001 |
| 0.25 → 0.10 | 46 | 12 | < 0.0001 |
| 0.20 → 0.15 | 21 | 2 | 0.0001 |
| 0.15 → 0.10 | 9 | 16 | 0.23 |
| 0.20 → 0.10 | 23 | 11 | 0.058 |

Figura: `dwb_r4e/r4e_overview.png`. Datos por corrida: `dwb_r4e/r4e_runs.csv`. Resumen: `dwb_r4e/r4e_summary.json`.

## Selección (criterio en orden de prioridad)

1. **Contactos graves**: sin incremento en ninguna variante (2 = 2, mismas corridas, previos a la ventana del goal).
2. **Bloqueos**: sin incremento en ninguna (46 = 46, mismas corridas).
3. **Falsos éxitos**: 0.15 es el mínimo (27, contra 56 / 43 / 35).
4. **Mejora de yaw de R4-D**: se mantiene en todas.
   - Error de yaw real en corridas que llegan en xy, p90: 0.271 → 0.264 / 0.242 / 0.246 rad.
   - Sobrepaso de yaw al frenar: igual en todas (p95 0.003 rad).
   - Oscilación después de empezar a girar: mejora (0.67 → 0.09 con 0.15).
5. **Correctos**: 0.15 (310) y 0.10 (303) superan con significancia a 0.25 y 0.20; **entre ellas no hay diferencia significativa** (p = 0.23).
   - 0.15 rompe menos: 5 corridas rotas contra 12, y 3 de las 122 contra 5.
   - 0.10 recupera algo más de las 46 rotas: 32 contra 30.
6. **Clearance**:
   - mismo mínimo global (−0.009 m, de los contactos previos) y mismo p05 en N1/N2;
   - ninguna corrida nueva bajo el padding;
   - el peor descenso por corrida en N1/N2 es −0.021 m con 0.20, −0.016 m con 0.15 y −0.004 m con 0.10, todos con clearance final ≥ 0.022 m.
7. **Tiempo**: en pares OK en ambas, Δt de SUCCEEDED p50 es +0.2 / +0.5 / +1.1 s.

**Mejor compromiso: 0.15 m.** Ninguna variante empeora la seguridad. 0.15 tiene menos falsos éxitos y menos corridas rotas y cuesta 0.6 s menos que 0.10. 0.10 da mejor precisión xy (p50 0.093 contra 0.136 m) y GoalTol 0, pero tiene más falsos éxitos de yaw (diferencia no significativa) y más correcciones de rumbo dentro de la ventana. La elección entre 0.15 y 0.10 **no está resuelta por estos datos** (una sola semilla por caso). Si en la competencia importa más la posición final que el tiempo, 0.10 es defendible.

## LIMITACIONES

- Mismas del simulador R4-B:
  - modelo cinemático mínimo, odometría perfecta y percepción perfecta (mapa estático, sin LiDAR);
  - sin BT (plan fijo, sin recuperaciones) y lógica de controller_server replicada;
  - una semilla por caso, ruido de localización sintético Gauss-Markov y mapa sintético.
- **Una sola semilla**: diferencias de menos de ~10 corridas entre variantes vecinas (0.15 contra 0.10) no son concluyentes. Para separarlas haría falta repetir con más semillas.
- La pose final **estimada** se aproxima congelando el offset de localización del último ciclo de control durante el frenado.
- El sobrepaso de yaw y |ω| en el disparo salen iguales en todas las variantes: el giro final lo controla `slowing_factor`, que no se tocó. No se exploraron `RotateToGoal.scale`, `slowing_factor` ni `lookahead_time`.
- **El falso éxito por yaw (~25–40 corridas) no se puede abordar con esta variable**: lo produce el checker de yaw (0.25 rad sobre pose estimada con ruido σ 0.05 rad), y R4-E tiene prohibido cambiarlo. Su peso real depende del ruido de yaw de AMCL en la arena, que no está medido con un mapa de arena válido.
- `xy_goal_tolerance` de FollowPath es también el umbral interno de "goal alcanzado" de RTG en Humble. No se verificó su efecto con un BT real ni con `velocity_smoother`.
- Las 46 corridas bloqueadas (around_obstacle, corridor_N1) y los 2 contactos (S4, N1, latencia 0.6 s) siguen igual. Son trabajo aparte.

## Archivos

- **Configs** (NO producción): `config/offline_lab/navigation2_R4E_{020,015,010}.yaml`, `dwb_offline_R4E_{020,015,010}_sim.yaml`, `dwb_offline_R4E_manifest.json`.
- **Scripts**: `scripts/nav_offline_lab/{make_dwb_r4e_config.py, verify_r4e_configs.py, run_r4e_all.sh, evaluate_dwb_r4e.py, make_dwb_r4e_report.py}`.
- **Resultados**: `results/navigation_offline_lab/dwb_r4e/`:
  - `sim_<S>_<tag>.jsonl` (crudos), `audit_*.json`, `launch_*.log`, `run_all.log`;
  - `config_verification.json`, `run_verification.json`;
  - `r4e_runs.csv`, `r4e_summary.json`, `r4e_tables.md`, `osc_split.json`, `mcnemar.json`, `r4e_overview.png`.

## Recomendación

**Adoptar `FollowPath.xy_goal_tolerance: 0.15` en la configuración candidata** (`config/offline_lab/navigation2_R4E_015.yaml`, junto con RotateToGoal y ObstacleFootprint), **sin llevarla a producción todavía**.

Frente a R4-D: +41 correctos (269 → 310), falsos éxitos 56 → 27 y GoalTol 13 → 1, con los mismos contactos y bloqueos. Además recupera 30 de las 46 corridas rotas por R4-D y solo vuelve a romper 3 de las 122 que RotateToGoal había arreglado.

Próximos pasos, todos offline:
1. **Repetir 0.15 y 0.10 con 2–3 semillas adicionales** para resolver el empate.
2. **Abordar el falso éxito por yaw** como R4-F (variable distinta, fuera del alcance de R4-E):
   - evaluar `general_goal_checker.yaw_goal_tolerance` menor que la tolerancia de aceptación;
   - o separar la tolerancia de evaluación de la del checker;
   - medir con el ruido de yaw real de AMCL cuando haya mapa de arena.
3. Los bloqueos de around_obstacle/N1 y los contactos con latencia 0.6 s siguen pendientes.

Cualquier cambio en `navigation2_real.yaml` requiere confirmación explícita y una prueba física supervisada.
