# NAV-OFFLINE-R4-F — general_goal_checker.yaw_goal_tolerance (DWB real + simulador R4-B)

Fecha: 2026-10-02. 100 % offline (ROS_DOMAIN_ID=99, ROS_LOCALHOST_ONLY=1). Sin hardware, sin limo_base, sin LiDAR, sin teleop, sin `/cmd_vel`: el simulador no tiene publishers Twist y la auditoría del grafo dio PASS en las 18 tandas. `config/navigation/navigation2_real.yaml` sin cambios (sha256 `dfb36e108d6872a5…`). `~/limoatwork` sin cambios.

## Resumen

```
R4-F (384 corridas por variante; FollowPath.xy_goal_tolerance 0.15 y checker xy 0.25 sin cambios)
yaw 0.25 (R4-E 0.15): 310/384 correctos   falsos éxitos por yaw 25   GoalTol 1
yaw 0.20            : 326/384             falsos éxitos por yaw  9   GoalTol 1
yaw 0.15            : 331/384             falsos éxitos por yaw  2   GoalTol 3
yaw 0.10            : 334/384             falsos éxitos por yaw  0   GoalTol 3
Contactos 2 en todas (mismas corridas). Bloqueos 46 en todas (mismas corridas).
```

## Método

- **Variable única**: `controller_server.general_goal_checker.yaw_goal_tolerance` ∈ {0.20, 0.15, 0.10}.
  - En Humble la lee solo el goal checker: `libsimple_goal_checker.so`. `libdwb_critics` y `libdwb_core` no la referencian.
  - El simulador la lee en `dwb_closed_loop_sim.cpp:140` y la aplica en la línea 423 como `|yaw_est − yaw_goal| < tol`, sobre la pose **estimada**.
- **Configs**, generadas por `make_dwb_r4f_config.py` desde las de R4-E 0.15, que no se modifican:
  - candidatas completas `config/offline_lab/navigation2_R4F_{020,015,010}.yaml` (sha256 `3408a428…`, `059ed436…`, `bd0ff326…`). Fuente: `navigation2_R4E_015.yaml`, `cc773d1c…`. NO son producción.
  - simulador `dwb_offline_R4F_{020,015,010}_sim.yaml`;
  - manifest `dwb_offline_R4F_manifest.json`.
- **Verificador automático** `verify_r4f_configs.py`. Es un paso obligatorio de `run_r4f_all.sh` antes de lanzar:
  - el diff de dicts aplanados contra R4-E 0.15 es exactamente `[...general_goal_checker.yaw_goal_tolerance]`, en la candidata completa y en el simulador;
  - checker xy 0.25 y FollowPath xy 0.15 en todas;
  - `controller_server` del simulador == el de la candidata;
  - producción con el sha256 de R4-D;
  - con `--runs`: la `meta` de cada salida es idéntica a la de R4-E 0.15 salvo `goal_yaw`, que vale la tolerancia de la variante; mismos 64 runs, orden y goals.

  Resultado **TODO PASS**: 24 checks de config y 72 de corridas (`dwb_r4f/config_verification.json`, `run_verification.json`).
- **Prueba de causalidad** `check_r4f_prefix.py`. Para las 384 corridas de cada variante, el log de control (t, pose real, pose estimada, v, ω, comandos) coincide **bit a bit** con la baseline hasta el último ciclo de la baseline:
  - 337 corridas en que la baseline llegó a SUCCEEDED: prefijo idéntico;
  - 47 en que no llegó: corrida completa idéntica, incluidos contactos y bloqueos;
  - 0 discrepancias (`dwb_r4f/prefix_check.json`).

  Toda diferencia es posterior al momento en que la baseline habría declarado SUCCEEDED.
- **Baseline yaw 0.25 = R4-E 0.15 existente, no re-ejecutada**. Mismo binario (`dwb_closed_loop_sim` sha256 `63b484a0…`). Assert de que sus clases recalculadas == `dwb_r4e/r4e_runs.csv` (T015).
- **Criterio de "correcto" fijo e independiente del checker**: SUCCEEDED, sin contacto y pose **real** ≤ 0.25 m / 0.25 rad, igual que R4-C/D/E.
  - Una tolerancia más estricta no cambia la vara de evaluación; solo puede cambiar cuándo y si el robot declara éxito.
  - "FalseYaw" = SUCCEEDED con yaw real > 0.25 rad.
  - Los fallos tras llegar en xy se desglosan por estado final (ABORT_PROGRESS / TIMEOUT / ABORT_CONTROLLER).
- Scripts: `run_r4f_all.sh`, `evaluate_dwb_r4f.py`, `make_dwb_r4f_report.py`, `check_r4f_prefix.py`.

## CONFIRMADO

- **Solo cambia la variable pedida**: verificador PASS y prefijos idénticos en 3 × 384 corridas.
- **Los falsos éxitos por yaw se convierten en éxitos reales, no en fallos.** De los 25 de la baseline:

  | variante | pasan a OK | pasan a fallo | siguen como falso éxito |
  |---|---|---|---|
  | yaw 0.20 | 17 | 0 | 8 |
  | yaw 0.15 | 23 | 0 | 2 |
  | yaw 0.10 | 25 | 0 | 0 |

  Ninguno pasa a TIMEOUT ni a GOAL_TOLERANCE.
- **Seguridad sin cambios**:
  - mismos 2 contactos (S4, N1, T042/T045) y mismos 46 bloqueos (las mismas corridas) en todas;
  - contactos nuevos 0 y bloqueos nuevos 0;
  - clearance mínima global igual (−0.009 m);
  - violaciones nuevas de padding: 0.
  - El clearance mínimo por corrida baja en 4 / 6 / 7 corridas. Es el barrido de las esquinas del footprint rectangular durante el giro final adicional, siempre en zonas abiertas (T005 `short`, clearance final ≥ 0.178 m; peor Δ −0.039 m con 0.10).
- **Correctos que se rompen**:
  - 0.20: 1 (S3_ruido_yaw:T019 pasa a falso éxito por yaw por ruido).
  - 0.15 y 0.10: las mismas 2 (S1_nominal:T041 y S2_ruido_xy:T045, corridor_N1), que pasan a GOAL_TOLERANCE[ABORT_PROGRESS].
- **Mecanismo de esas 2 corridas: ciclo límite RotateToGoal × ObstacleFootprint en N1, no ruido** (S1 no tiene ruido).
  - RotateToGoal deja al robot a 0.14–0.16 m del goal, en la parte angosta del corredor.
  - Girar hacia el yaw del goal acerca el footprint a la pared. En T045 el clearance pasa de 0.09–0.10 m (error +0.3/+0.4 rad) a 0.04 m (error 0), y en un yaw intermedio a 0.0096 m (`dwb_r4f/stuck_rotation_clearance.json`).
  - ObstacleFootprint penaliza ese giro y RotateToGoal lo empuja. ω alterna entre ±0.158 rad/s y el error de yaw queda atrapado entre 0.16 y 0.40 rad (mínimo 0.158 / 0.170) durante 25 s, hasta que el progress checker (0.1 m / 25 s) aborta.
  - Con yaw 0.25 estas corridas terminaban en ~0.16 rad, que era correcto.
  - **El mecanismo ya existía en la baseline**: S5_combinado_medido:T041 (N1) aborta igual con yaw 0.25.
  - El giro atascado no reduce el clearance mínimo de esas corridas (igual a la baseline).
- **La mejora de RotateToGoal se mantiene**:
  - sobrepaso de yaw al frenar idéntico (p95 0.002–0.003 rad);
  - ω al disparar el checker igual o menor (p95 0.34 → 0.16 rad/s);
  - cambios de signo de ω después de empezar a girar: media 0.093 → 0.107 → 0.127 si se excluyen las 2 corridas atascadas, que aportan 13–15 cada una. No reaparecen oscilaciones en el goal.

## MEDIDO
### Tabla principal (384 corridas por variante)

| YawTol | Correct | FalseSuccess | FalseYaw | GoalTol | Timeout | Contacts | Blocks | Yaw p90 (rad) | Time p50 / p90 (s) |
|---|---|---|---|---|---|---|---|---|---|
| yaw 0.25 (R4-E 0.15) | 310 | 27 | 25 | 1 | 9 | 2 | 46 | 0.242 | 19.0 / 60.2 |
| yaw 0.20 | 326 | 11 | 9 | 1 | 9 | 2 | 46 | 0.199 | 19.3 / 60.3 |
| yaw 0.15 | 331 | 4 | 2 | 3 | 9 | 2 | 46 | 0.158 | 19.9 / 60.6 |
| yaw 0.10 | 334 | 1 | 0 | 3 | 9 | 2 | 46 | 0.105 | 20.2 / 61.0 |

Yaw p90 = error final REAL de las corridas que llegan en xy. Time = tiempo hasta SUCCEEDED. FalseYaw = SUCCEEDED con yaw REAL > 0.25 rad. Timeout = clase TIMEOUT (no llega a la ventana xy). GoalTol = llega en xy sin SUCCEEDED; su causa en todas las variantes es ABORT_PROGRESS (ver abajo). Criterio de "correcto" fijo: pose REAL ≤ 0.25 m / 0.25 rad.

### Clases extendidas

| clase | yaw 0.25 (R4-E 0.15) | yaw 0.20 | yaw 0.15 | yaw 0.10 |
|---|---|---|---|---|
| FALSE_XY | 2 | 2 | 2 | 1 |
| FALSE_YAW | 25 | 9 | 2 | 0 |
| GOAL_TOLERANCE[ABORT_PROGRESS] | 1 | 1 | 3 | 3 |
| OK | 310 | 326 | 331 | 334 |
| OSCILACION | 23 | 23 | 23 | 23 |
| OTRO_SIN_TRAYECT | 14 | 14 | 14 | 14 |
| TIMEOUT | 9 | 9 | 9 | 9 |

### Error final: p50 / p90 / p95 (corridas que llegan en xy, n = 338 en todas)

| métrica | yaw 0.25 (R4-E 0.15) | yaw 0.20 | yaw 0.15 | yaw 0.10 |
|---|---|---|---|---|
| XY REAL (m) | 0.136 / 0.193 / 0.232 | 0.135 / 0.189 / 0.231 | 0.135 / 0.173 / 0.221 | 0.133 / 0.168 / 0.206 |
| XY ESTIMADO (m) | 0.136 / 0.212 / 0.231 | 0.137 / 0.207 / 0.228 | 0.136 / 0.183 / 0.226 | 0.133 / 0.181 / 0.217 |
| yaw REAL (rad) | 0.194 / 0.242 / 0.274 | 0.158 / 0.199 / 0.238 | 0.117 / 0.158 / 0.193 | 0.071 / 0.105 / 0.138 |
| yaw ESTIMADO (rad) | 0.213 / 0.237 / 0.240 | 0.167 / 0.188 / 0.191 | 0.121 / 0.139 / 0.141 | 0.071 / 0.090 / 0.091 |
| yaw REAL, solo correctas (rad) | 0.184 / 0.237 / 0.240 | 0.155 / 0.191 / 0.209 | 0.115 / 0.151 / 0.177 | 0.070 / 0.100 / 0.130 |
| XY REAL, solo correctas (m) | 0.135 / 0.193 / 0.232 | 0.135 / 0.184 / 0.230 | 0.132 / 0.172 / 0.207 | 0.132 / 0.167 / 0.203 |

### Tiempo, giro cerca del goal, oscilaciones y comandos finales (p50 / p90 / p95)

| métrica | yaw 0.25 (R4-E 0.15) | yaw 0.20 | yaw 0.15 | yaw 0.10 |
|---|---|---|---|---|
| tiempo hasta SUCCEEDED (s) | 19.0 / 60.2 / 62.1 | 19.3 / 60.3 / 62.4 | 19.9 / 60.6 / 62.7 | 20.2 / 61.0 / 62.9 |
| SUCCEEDED − entrada en ventana xy (s) | 4.0 / 5.6 / 6.0 | 4.3 / 6.0 / 6.4 | 4.6 / 6.3 / 6.7 | 4.9 / 6.7 / 7.0 |
| tiempo girando en el lugar dentro de 0.25 m (s) | 3.1 / 4.9 / 5.0 | 3.4 / 5.2 / 5.4 | 3.7 / 5.5 / 5.7 | 4.0 / 5.8 / 6.0 |
| tiempo total dentro de 0.25 m (s) | 4.1 / 5.7 / 6.2 | 4.4 / 6.1 / 6.5 | 4.7 / 6.5 / 7.0 | 5.0 / 6.8 / 7.2 |
| cambios de signo de angular.z, corrida completa | 12 / 45 / 50 | 12 / 45 / 50 | 12 / 45 / 50 | 12 / 45 / 50 |
| cambios de signo antes de empezar a girar | 0 / 3 / 4 | 0 / 3 / 4 | 1 / 3 / 4 | 1 / 3 / 4 |
| cambios de signo después de empezar a girar | 0 / 0 / 0 | 0 / 0 / 0 | 0 / 0 / 0 | 0 / 0 / 0 |
| sobrepaso de yaw al frenar (rad) | -0.008 / 0.000 / 0.003 | -0.008 / 0.000 / 0.003 | -0.008 / -0.001 / 0.002 | -0.008 / -0.000 / 0.003 |
| |ω| real al disparar el checker (rad/s) | 0.16 / 0.26 / 0.34 | 0.16 / 0.26 / 0.26 | 0.16 / 0.16 / 0.24 | 0.16 / 0.16 / 0.16 |
| |ω| del último comando no nulo, SUCCEEDED (rad/s) | 0.158 / 0.158 / 0.158 | 0.158 / 0.158 / 0.158 | 0.158 / 0.158 / 0.158 | 0.158 / 0.158 / 0.158 |
| |v| del último comando no nulo, SUCCEEDED (m/s) | 0.000 / 0.132 / 0.145 | 0.000 / 0.124 / 0.145 | 0.000 / 0.118 / 0.136 | 0.000 / 0.039 / 0.132 |
| cambios de signo después de empezar a girar, media | 0.09 | 0.10 | 0.20 | 0.22 |
| corridas cuyo último comando no nulo es giro en el lugar | 315 | 316 | 326 | 335 |
| clearance mínima global (m) | -0.009 | -0.009 | -0.009 | -0.009 |

### Por escenario: correctos / FalseYaw / GoalTol / bloqueos / contactos

| escenario | yaw 0.25 (R4-E 0.15) | yaw 0.20 | yaw 0.15 | yaw 0.10 |
|---|---|---|---|---|
| S1_nominal | 56 / 0 / 0 / 8 / 0 | 56 / 0 / 0 / 8 / 0 | 55 / 0 / 1 / 8 / 0 | 55 / 0 / 1 / 8 / 0 |
| S2_ruido_xy | 54 / 0 / 0 / 9 / 0 | 54 / 0 / 0 / 9 / 0 | 53 / 0 / 1 / 9 / 0 | 53 / 0 / 1 / 9 / 0 |
| S3_ruido_yaw | 49 / 6 / 0 / 9 / 0 | 53 / 2 / 0 / 9 / 0 | 55 / 0 / 0 / 9 / 0 | 55 / 0 / 0 / 9 / 0 |
| S4_latencia_0p6 | 56 / 0 / 0 / 8 / 2 | 56 / 0 / 0 / 8 / 2 | 56 / 0 / 0 / 8 / 2 | 56 / 0 / 0 / 8 / 2 |
| S5_combinado_medido | 39 / 18 / 1 / 5 / 0 | 51 / 6 / 1 / 5 / 0 | 56 / 1 / 1 / 5 / 0 | 58 / 0 / 1 / 5 / 0 |
| S6_combinado_lat0p6 | 56 / 1 / 0 / 7 / 0 | 56 / 1 / 0 / 7 / 0 | 56 / 1 / 0 / 7 / 0 | 57 / 0 / 0 / 7 / 0 |

### Corredores estrechos: correctos / FalseYaw / GoalTol / bloqueos / contactos / clearance mín (m)

| corredor | yaw 0.25 (R4-E 0.15) | yaw 0.20 | yaw 0.15 | yaw 0.10 |
|---|---|---|---|---|
| corridor_N1 | 34 / 2 / 1 / 11 / 2 / -0.009 | 35 / 1 / 1 / 11 / 2 / -0.009 | 34 / 0 / 3 / 11 / 2 / -0.009 | 34 / 0 / 3 / 11 / 2 / -0.009 |
| corridor_N2 | 45 / 2 / 0 / 0 / 0 / 0.025 | 45 / 2 / 0 / 0 / 0 / 0.025 | 47 / 0 / 0 / 0 / 0 / 0.025 | 47 / 0 / 0 / 0 / 0 / 0.025 |

### Comparación pareada contra yaw 0.25 (clave scenario + path_id + seed)

| variante | FIXED | BROKEN | UNCHANGED_OK | UNCHANGED_FAIL | 25 FalseYaw → OK | → fallo | siguen FalseYaw | correctos rotos (destino) | contactos nuevos | bloqueos nuevos | Δ yaw REAL p50, pares OK (rad) | Δ t SUCCEEDED p50 / p90, pares OK (s) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| yaw 0.20 | 17 | 1 | 309 | 57 | 17 | 0 | 8 | 1 (FALSE_YAW 1) | 0 | 0 | -0.032 | 0.20 / 0.40 |
| yaw 0.15 | 23 | 2 | 308 | 51 | 23 | 0 | 2 | 2 (GOAL_TOLERANCE[ABORT_PROGRESS] 2) | 0 | 0 | -0.073 | 0.50 / 0.80 |
| yaw 0.10 | 26 | 2 | 308 | 48 | 25 | 0 | 0 | 2 (GOAL_TOLERANCE[ABORT_PROGRESS] 2) | 0 | 0 | -0.116 | 0.90 / 1.20 |

<details><summary>yaw 0.20: transiciones y listas</summary>

| transición (yaw 0.25 → yaw 0.20) | corridas |
|---|---|
| OK->OK | 309 |
| OSCILACION->OSCILACION | 23 |
| FALSE_YAW->OK | 17 |
| OTRO_SIN_TRAYECT->OTRO_SIN_TRAYECT | 14 |
| TIMEOUT->TIMEOUT | 9 |
| FALSE_YAW->FALSE_YAW | 8 |
| FALSE_XY->FALSE_XY | 2 |
| OK->FALSE_YAW | 1 |
| GOAL_TOLERANCE[ABORT_PROGRESS]->GOAL_TOLERANCE[ABORT_PROGRESS] | 1 |

BROKEN: S3_ruido_yaw:T019:FALSE_YAW

FIXED: S3_ruido_yaw:T001:FALSE_YAW, S3_ruido_yaw:T003:FALSE_YAW, S3_ruido_yaw:T011:FALSE_YAW, S3_ruido_yaw:T058:FALSE_YAW, S3_ruido_yaw:T061:FALSE_YAW, S5_combinado_medido:T004:FALSE_YAW, S5_combinado_medido:T010:FALSE_YAW, S5_combinado_medido:T014:FALSE_YAW, S5_combinado_medido:T015:FALSE_YAW, S5_combinado_medido:T017:FALSE_YAW, S5_combinado_medido:T018:FALSE_YAW, S5_combinado_medido:T022:FALSE_YAW, S5_combinado_medido:T038:FALSE_YAW, S5_combinado_medido:T043:FALSE_YAW, S5_combinado_medido:T056:FALSE_YAW, S5_combinado_medido:T061:FALSE_YAW, S5_combinado_medido:T063:FALSE_YAW

</details>

<details><summary>yaw 0.15: transiciones y listas</summary>

| transición (yaw 0.25 → yaw 0.15) | corridas |
|---|---|
| OK->OK | 308 |
| OSCILACION->OSCILACION | 23 |
| FALSE_YAW->OK | 23 |
| OTRO_SIN_TRAYECT->OTRO_SIN_TRAYECT | 14 |
| TIMEOUT->TIMEOUT | 9 |
| OK->GOAL_TOLERANCE[ABORT_PROGRESS] | 2 |
| FALSE_XY->FALSE_XY | 2 |
| FALSE_YAW->FALSE_YAW | 2 |
| GOAL_TOLERANCE[ABORT_PROGRESS]->GOAL_TOLERANCE[ABORT_PROGRESS] | 1 |

BROKEN: S1_nominal:T041:GOAL_TOLERANCE[ABORT_PROGRESS], S2_ruido_xy:T045:GOAL_TOLERANCE[ABORT_PROGRESS]

FIXED: S3_ruido_yaw:T001:FALSE_YAW, S3_ruido_yaw:T003:FALSE_YAW, S3_ruido_yaw:T011:FALSE_YAW, S3_ruido_yaw:T054:FALSE_YAW, S3_ruido_yaw:T058:FALSE_YAW, S3_ruido_yaw:T061:FALSE_YAW, S5_combinado_medido:T004:FALSE_YAW, S5_combinado_medido:T010:FALSE_YAW, S5_combinado_medido:T012:FALSE_YAW, S5_combinado_medido:T014:FALSE_YAW, S5_combinado_medido:T015:FALSE_YAW, S5_combinado_medido:T017:FALSE_YAW, S5_combinado_medido:T018:FALSE_YAW, S5_combinado_medido:T022:FALSE_YAW, S5_combinado_medido:T030:FALSE_YAW, S5_combinado_medido:T038:FALSE_YAW, S5_combinado_medido:T043:FALSE_YAW, S5_combinado_medido:T046:FALSE_YAW, S5_combinado_medido:T052:FALSE_YAW, S5_combinado_medido:T056:FALSE_YAW, S5_combinado_medido:T058:FALSE_YAW, S5_combinado_medido:T061:FALSE_YAW, S5_combinado_medido:T063:FALSE_YAW

</details>

<details><summary>yaw 0.10: transiciones y listas</summary>

| transición (yaw 0.25 → yaw 0.10) | corridas |
|---|---|
| OK->OK | 308 |
| FALSE_YAW->OK | 25 |
| OSCILACION->OSCILACION | 23 |
| OTRO_SIN_TRAYECT->OTRO_SIN_TRAYECT | 14 |
| TIMEOUT->TIMEOUT | 9 |
| OK->GOAL_TOLERANCE[ABORT_PROGRESS] | 2 |
| FALSE_XY->FALSE_XY | 1 |
| FALSE_XY->OK | 1 |
| GOAL_TOLERANCE[ABORT_PROGRESS]->GOAL_TOLERANCE[ABORT_PROGRESS] | 1 |

BROKEN: S1_nominal:T041:GOAL_TOLERANCE[ABORT_PROGRESS], S2_ruido_xy:T045:GOAL_TOLERANCE[ABORT_PROGRESS]

FIXED: S3_ruido_yaw:T001:FALSE_YAW, S3_ruido_yaw:T003:FALSE_YAW, S3_ruido_yaw:T011:FALSE_YAW, S3_ruido_yaw:T054:FALSE_YAW, S3_ruido_yaw:T058:FALSE_YAW, S3_ruido_yaw:T061:FALSE_YAW, S5_combinado_medido:T004:FALSE_YAW, S5_combinado_medido:T010:FALSE_YAW, S5_combinado_medido:T012:FALSE_YAW, S5_combinado_medido:T014:FALSE_YAW, S5_combinado_medido:T015:FALSE_YAW, S5_combinado_medido:T017:FALSE_YAW, S5_combinado_medido:T018:FALSE_YAW, S5_combinado_medido:T019:FALSE_XY, S5_combinado_medido:T022:FALSE_YAW, S5_combinado_medido:T030:FALSE_YAW, S5_combinado_medido:T033:FALSE_YAW, S5_combinado_medido:T038:FALSE_YAW, S5_combinado_medido:T043:FALSE_YAW, S5_combinado_medido:T046:FALSE_YAW, S5_combinado_medido:T052:FALSE_YAW, S5_combinado_medido:T056:FALSE_YAW, S5_combinado_medido:T058:FALSE_YAW, S5_combinado_medido:T061:FALSE_YAW, S5_combinado_medido:T063:FALSE_YAW, S6_combinado_lat0p6:T026:FALSE_YAW

</details>


### Significancia pareada de correctos (McNemar exacto, 384 pares)

| comparación | pasan a OK | dejan de ser OK | p |
|---|---|---|---|
| yaw 0.25 → 0.20 | 17 | 1 | 1.4e-4 |
| yaw 0.25 → 0.15 | 23 | 2 | 1.9e-5 |
| yaw 0.25 → 0.10 | 26 | 2 | 3.0e-6 |
| yaw 0.20 → 0.15 | 7 | 2 | 0.18 |
| yaw 0.15 → 0.10 | 3 | 0 | 0.25 |
| yaw 0.20 → 0.10 | 10 | 2 | 0.039 |

Datos por corrida: `dwb_r4f/r4f_runs.csv`. Resumen: `dwb_r4f/r4f_summary.json`. Tablas: `dwb_r4f/r4f_tables.md`.

## Selección (criterio en orden de prioridad)

1. **Contactos**: sin aumento en ninguna variante; las corridas y los tiempos de contacto son idénticos.
2. **Bloqueos**: 46 = 46 en todas. GOAL_TOLERANCE[ABORT_PROGRESS] (no cuenta como bloqueo, pero es un abort cerca del goal) es 1 → 1 → 3 → 3.
   - 0.15 y 0.10 suman las mismas 2 corridas de N1, por el ciclo límite preexistente.
   - 0.20 no agrega ninguna.
3. **Falsos éxitos reales**: 27 → 11 → 4 → 1. Por yaw: 25 → 9 → 2 → 0.
4. **Correctos**: 310 → 326 → 331 → 334.
   - Las tres variantes mejoran a la baseline con significancia.
   - 0.15 y 0.10 no se distinguen entre sí (p = 0.25). 0.10 domina débilmente: +3 correctos, −2 falsos éxitos de yaw, mismas 2 roturas.
5. **Yaw final robusto**: yaw REAL p90 0.242 → 0.199 → 0.158 → 0.105 rad; p95 0.274 → 0.238 → 0.193 → 0.138 rad.
6. **Oscilaciones**: sin reaparición fuera de las 2 corridas de N1 atascadas.
7. **Tiempo**: en pares OK en ambas, Δt de SUCCEEDED p50 es +0.2 / +0.5 / +0.9 s y p90 +0.4 / +0.8 / +1.2 s. El tiempo girando dentro de 0.25 m, p50, es 3.1 → 3.4 → 3.7 → 4.0 s. No es excesivo.

**Mejor compromiso medido: yaw 0.10.**
- Convierte los 25 falsos éxitos por yaw en éxitos reales y ninguno en fallo.
- Llega al máximo de correctos (334) con las mismas 2 roturas que 0.15.
- Su costo es de ~+0.9 s por goal.

Condición: 0.15 y 0.10 comparten las 2 roturas de N1, causadas por el ciclo límite RotateToGoal × ObstacleFootprint. Si antes de resolverlo no se acepta ningún abort nuevo cerca del goal, la alternativa conservadora es **0.20**: 326 correctos, 9 falsos éxitos por yaw y ningún abort nuevo. 0.15 no aporta ventaja sobre 0.10.

## LIMITACIONES

- **Las del simulador R4-B**:
  - modelo cinemático mínimo, odometría perfecta, percepción perfecta (mapa estático, sin LiDAR);
  - sin BT (plan fijo, sin recuperaciones) y lógica de controller_server replicada;
  - una semilla por caso, ruido sintético Gauss-Markov y mapa sintético.
- **Ruido de yaw**: el simulado es σ = 0.05 rad (τ = 1 s). Con yaw 0.10 la tolerancia es 2σ. El ruido de yaw real de AMCL en la arena no está medido, porque no hay mapa de arena válido. Si es mayor, 0.10 puede hacer que el robot gire más tiempo buscando la ventana, y quizás más aborts por progress checker. Esto no se probó.
- **El ABORT_PROGRESS con un BT real dispararía recuperaciones** (spin / backup / wait) dentro de corridor_N1. Aquí no se simulan, y en un corredor angosto pueden ser riesgosas. Es la razón principal para no llevar yaw ≤ 0.15 al robot antes de resolver el ciclo límite.
- **Interpretación del ciclo límite**: verificada con la geometría del footprint en la pose de atasco, no con los puntajes internos de cada critic. Pendiente confirmarla con el desglose de puntajes de DWB (`/evaluation`).
- No se probó `stateful: false`, ni otro `movement_time_allowance`, ni otros parámetros de RotateToGoal.
- Diferencias de 2–3 corridas entre variantes vecinas no son concluyentes con una semilla.

## Archivos

- **Configs** (NO producción): `config/offline_lab/navigation2_R4F_{020,015,010}.yaml`, `dwb_offline_R4F_{020,015,010}_sim.yaml`, `dwb_offline_R4F_manifest.json`.
- **Scripts**: `scripts/nav_offline_lab/{make_dwb_r4f_config.py, verify_r4f_configs.py, run_r4f_all.sh, evaluate_dwb_r4f.py, make_dwb_r4f_report.py, check_r4f_prefix.py}`.
- **Resultados**: `results/navigation_offline_lab/dwb_r4f/`:
  - `sim_<S>_<tag>.jsonl`, `audit_*.json`, `launch_*.log`, `run_all.log`;
  - `config_verification.json`, `run_verification.json`, `prefix_check.json`;
  - `r4f_runs.csv`, `r4f_summary.json`, `r4f_tables.md`, `mcnemar.json`, `stuck_rotation_clearance.json`.

## Recomendación

**Candidata offline: `general_goal_checker.yaw_goal_tolerance: 0.10`** (`config/offline_lab/navigation2_R4F_010.yaml`, sobre R4-E 0.15). Frente a R4-E: +24 correctos (310 → 334), falsos éxitos 27 → 1, por yaw 25 → 0, yaw real p90 0.242 → 0.105 rad. Contactos y bloqueos idénticos, con ~+0.9 s por goal.

**No llevar a producción todavía.** Antes:
1. **R4-G (offline)**: resolver el ciclo límite RotateToGoal × ObstacleFootprint en corridor_N1. Responsable de las 2 roturas nuevas y del abort que ya existía.
   - Confirmar con los puntajes de critics (`/evaluation`).
   - Probar, de a una variable: `RotateToGoal.scale`, `ObstacleFootprint.scale` y que RotateToGoal empiece donde el giro es libre.
2. Repetir 0.20 / 0.10 con 2–3 semillas más y con ruido de yaw mayor (σ 0.08–0.10), para medir la robustez de 0.10 frente a un AMCL peor que el simulado.
3. Medir el ruido de yaw real de AMCL cuando exista mapa de arena (requiere autorización para mapear).

Si hace falta una config candidata antes de R4-G, usar **yaw 0.20**: no agrega aborts. Cualquier cambio en `navigation2_real.yaml` requiere confirmación explícita y una prueba física supervisada.
