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

