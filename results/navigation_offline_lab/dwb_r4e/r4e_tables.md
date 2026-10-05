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

