# NAV-OFFLINE-R4-C — Análisis de terminaciones incorrectas (datos de R4-B, sin simulaciones nuevas)

Fecha: 2026-09-29. Análisis 100 % offline sobre las 768 corridas ya generadas en R4-B (`results/navigation_offline_lab/dwb_r4b/sim_*.jsonl`): 6 escenarios × 64 paths × 2 variantes (A = BaseObstacle, config real; B = ObstacleFootprint). **No se ejecutó ninguna simulación nueva, no se modificó ninguna configuración**, ni `navigation2_real.yaml` ni `~/limoatwork`.

## Resumen

```
R4-C
Corridas analizadas: 768
Fallos:              428  (A 237, B 191)
Causa principal:     llegada al goal sin orientación final. 200 corridas llegan en xy (error mediano 10 mm) y no cumplen el yaw
                     (error mediano 1.84 rad); con los 66 sobrepasos de yaw al frenar suman 266/428 fallos.
```

## Método

Una clase por corrida, asignada por reglas fijas en este orden (`scripts/nav_offline_lab/analyze_r4c_failures.py`):

| clase | definición |
|---|---|
| OK | SUCCEEDED correcto: sin contacto; error VERDADERO final ≤ 0.25 m y ≤ 0.25 rad |
| GOAL_TOLERANCE | goal tolerance failure: llegó en xy (etapa 1 del goal checker, stateful) y no cumplió el yaw |
| OSCILACION | oscilación: no llegó en xy; ABORT_PROGRESS con el robot quieto (< 0.1 m en 25 s) y ≥ 4 inversiones de ω |
| TIMEOUT | timeout: no llegó en xy; se agotó t_max = 3·L/0.25 + 30 s |
| TRACKING | tracking failure: no llegó en xy y el error lateral máximo superó 0.25 m |
| OTRO_SIN_TRAYECT | otro: DWB sin trayectorias legales más de 0.3 s (ABORT_CONTROLLER); el robot se detiene |
| OTRO_EXITO_CONTACTO | otro: SUCCEEDED, pero con contacto real del footprint durante la corrida |
| OTRO_EXITO_APARENTE | otro: SUCCEEDED sobre la pose ESTIMADA, con error VERDADERO final fuera de tolerancia |
| OTRO | resto |

- Errores finales = pose **verdadera** en reposo vs goal. Error lateral = distancia de la pose verdadera al path NavFn hasta la llegada xy (o hasta el fin). Clearance = footprint real sin padding vs geometría del mapa (mismas métricas que R4-B).
- **Contrafactual de tolerancias (exacto)**: el goal checker stateful se re-evalúa sobre la pose **estimada** registrada con tolerancias mayores. Parar antes no altera la trayectoria previa, así que el instante de disparo es exacto. Cuenta como resuelta solo si dispara antes del fin, sin contacto previo, y con la pose **verdadera** dentro de las nuevas tolerancias en ese instante.
- **Atribución** (reglas explícitas):

| atribución | regla | evidencia |
|---|---|---|
| PARÁMETRO: tolerancias | GOAL_TOLERANCE resuelta por el contrafactual con xy ≤ 0.40 m y yaw ≤ 0.785 rad | verificada (contrafactual exacto) |
| PARÁMETRO: critic de obstáculos | la corrida pareada (mismo path y escenario, misma semilla) con el otro critic termina OK | verificada (pareado) |
| PARÁMETRO (inferido): sin orientación final | GOAL_TOLERANCE que no se resuelve con xy ≤ 0.40 / yaw ≤ 0.785 (solo con yaw ≥ 1.57 o sin chequeo) | inferida: `RotateToGoal` está comentado en la config real y ningún critic activo orienta al yaw del goal; **no simulado** |
| DWB | falla en ambas variantes y no la resuelven las tolerancias | por exclusión; no atribuible a un parámetro con estos datos |
| DINÁMICA DE PARADA | SUCCEEDED con la pose verdadera dentro de tolerancia al detectar, fuera al detenerse | verificada (pose en el disparo vs pose final) |
| LOCALIZACIÓN | SUCCEEDED con la pose verdadera ya fuera de tolerancia al detectar | verificada |

## CONFIRMADO

- **GOAL_TOLERANCE (200)**: el robot está en la posición del goal (error xy verdadero mediano 9.5 mm), con el rumbo lejos del yaw del goal (mediana 1.84 rad, p90 2.89 rad). Tras llegar en xy sigue 27.7 s (mediana) invirtiendo el giro (≈14 inversiones de ω en los últimos 25 s, |ω| ≈ 0.47 rad/s) hasta que el progress checker aborta. Con estos critics no hay término que lleve el rumbo al yaw del goal: GoalAlign/PathAlign alinean con el path y `RotateToGoal` está comentado en `navigation2_real.yaml` (verificado leyendo la config).
- **El ruido XY enmascara esta falla**: GOAL_TOLERANCE es 77 en S1, 61 en S3 (yaw), 56 en S4 (latencia) y cae a 3, 3 y 0 en S2, S5 y S6 (los escenarios con ruido XY). No es una mejora: el ruido mueve el goal estimado, agita el rumbo y el yaw cruza la ventana de ±0.25 rad por azar.
- **Éxitos aparentes (101)**: 66 estaban dentro de tolerancia (pose verdadera) al detectar el goal y se salieron al frenar: el robot gira a ~0.77 rad/s (mediana) al detectar, el cero llega con la latencia de la base y la desaceleración de 1.5 rad/s² agrega el resto (sobrepaso mediano 0.24 rad, máx 0.60). 81/101 son de los escenarios con latencia 0.6 s (S4, S6). Los otros 35 ya estaban fuera de tolerancia al detectar (goal checker sobre la pose estimada con ruido).
- **OSCILACIÓN y TIMEOUT sin llegar** son mínimos locales: el robot se queda a ~1–2 m del goal. En around_obstacle (goal detrás de un obstáculo) pasa con ambas variantes. En corridor_N1 con BaseObstacle se debe al término de obstáculo sobre la última pose (mecanismo verificado en R4-B).
- **TRACKING failure = 0**: ninguna corrida falla por apartarse del path. El error lateral máximo de todas las fallas sin llegada es ≤ 0.22 m, y en las 768 corridas ≤ 0.37 m.
- **OTRO_SIN_TRAYECT (14)**: todas en la variante B (ObstacleFootprint), mayormente al entrar a corridor_N1 (t ≈ 3–7 s). El footprint con padding (0.36 m de ancho) en un pasillo de 0.45 m deja casos sin ninguna trayectoria legal. El robot se detiene (2 de ellas tras un contacto de < 1 cm con latencia 0.6 s).

## MEDIDO

### Clases por variante

| clase | A BaseObstacle | B ObstacleFootprint | total |
|---|---|---|---|
| OK | 147 | 193 | 340 |
| GOAL_TOLERANCE | 101 | 99 | 200 |
| OSCILACION | 68 | 23 | 91 |
| TIMEOUT | 8 | 9 | 17 |
| OTRO_SIN_TRAYECT | 0 | 14 | 14 |
| OTRO_EXITO_CONTACTO | 5 | 0 | 5 |
| OTRO_EXITO_APARENTE | 55 | 46 | 101 |

### Clases por escenario

| escenario | OK | GOAL_TOLERANCE | OSCILACION | TIMEOUT | OTRO_SIN_TRAYECT | OTRO_EXITO_CONTACTO | OTRO_EXITO_APARENTE |
|---|---|---|---|---|---|---|---|
| S1_nominal | 29 | 77 | 18 | 2 | 2 | 0 | 0 |
| S2_ruido_xy | 98 | 3 | 14 | 4 | 4 | 1 | 4 |
| S3_ruido_yaw | 36 | 61 | 17 | 3 | 3 | 0 | 8 |
| S4_latencia_0p6 | 37 | 56 | 17 | 2 | 3 | 0 | 13 |
| S5_combinado_medido | 100 | 3 | 10 | 4 | 0 | 3 | 8 |
| S6_combinado_lat0p6 | 40 | 0 | 15 | 2 | 2 | 1 | 68 |

### Clases por categoría de path

| categoría | OK | GOAL_TOLERANCE | OSCILACION | TIMEOUT | OTRO_SIN_TRAYECT | OTRO_EXITO_CONTACTO | OTRO_EXITO_APARENTE |
|---|---|---|---|---|---|---|---|
| around_obstacle | 27 | 15 | 51 | 17 | 3 | 0 | 7 |
| corridor_N1 | 26 | 7 | 40 | 0 | 11 | 5 | 7 |
| corridor_N2 | 47 | 34 | 0 | 0 | 0 | 0 | 15 |
| corridor_WIDE | 53 | 27 | 0 | 0 | 0 | 0 | 16 |
| long | 67 | 28 | 0 | 0 | 0 | 0 | 25 |
| random | 61 | 41 | 0 | 0 | 0 | 0 | 18 |
| short | 59 | 48 | 0 | 0 | 0 | 0 | 13 |

### Estadísticas por clase

| clase | n | error lateral máx (m) | error final xy med (m) | error final yaw med (rad) | t fin med (s) | clearance mín (m) |
|---|---|---|---|---|---|---|
| OK | 340 | 0.374 | 0.041 | 0.111 | 30.7 | 0.010 |
| GOAL_TOLERANCE | 200 | 0.355 | 0.010 | 1.840 | 46.2 | 0.010 |
| OSCILACION | 91 | 0.136 | 2.075 | 1.336 | 26.4 | 0.010 |
| TIMEOUT | 17 | 0.222 | 1.513 | 0.865 | 74.3 | 0.015 |
| OTRO_SIN_TRAYECT | 14 | 0.142 | 1.997 | 1.630 | 4.8 | -0.009 |
| OTRO_EXITO_CONTACTO | 5 | 0.103 | 0.039 | 0.089 | 29.2 | -0.057 |
| OTRO_EXITO_APARENTE | 101 | 0.242 | 0.039 | 0.465 | 31.3 | 0.018 |

## Respuestas

### 1. ¿Cuál es la causa dominante de fallos?

La llegada al goal sin orientación final. **200/428 fallos (47 %)** son GOAL_TOLERANCE: el robot llega en posición y no logra el yaw. Si se suman los 66 sobrepasos de yaw al frenar, el manejo del rumbo al final explica **266/428 (62 %)**. La segunda causa son los mínimos locales sin llegar (OSCILACIÓN 91 + TIMEOUT 17).

### 2. ¿Cuántos fallos son por DWB?

**105** fallan con ambas variantes y no los resuelven las tolerancias. Por clase: OSCILACION 73, OTRO_EXITO_CONTACTO 2, OTRO_SIN_TRAYECT 14, TIMEOUT 16. Son sobre todo los mínimos locales de around_obstacle (T023–T029) y las paradas sin trayectorias de B en N1. Es una atribución **por exclusión**: puede haber pesos de critics no probados que los resuelvan.

### 3. ¿Cuántos son por parámetros?

**74 verificados** (52 por tolerancias de goal, contrafactual; 22 por el critic de obstáculos, pareado) + **148 inferidos** (GOAL_TOLERANCE que no se resuelve con yaw ≤ 0.785 rad: falta un critic de orientación final, RotateToGoal comentado). Total atribuible a configuración: **222/428**. Aparte: 66 por dinámica de parada (latencia + ω alta al detectar) y 35 por error de localización.

### 4. ¿Cuántos desaparecerían aumentando tolerancias?

Contrafactual exacto sobre los fallos (se cuentan solo si la pose verdadera cumple la nueva tolerancia al disparar y no hubo contacto antes):

| xy tol (m) | yaw tol (rad) | GOAL_TOLERANCE resueltas | éxitos aparentes que pasan a correctos | otras clases |
|---|---|---|---|---|
| 0.25 | 0.350 | 17/200 | 75/101 | 0 |
| 0.25 | 0.500 | 20/200 | 71/101 | 0 |
| 0.25 | 0.785 | 51/200 | 73/101 | 0 |
| 0.25 | 1.571 | 117/200 | 69/101 | 0 |
| 0.25 | sin chequeo | 196/200 | 67/101 | 0 |
| 0.30 | 0.785 | 52/200 | 75/101 | 0 |
| 0.40 | 0.250 | 5/200 | 70/101 | 0 |
| 0.40 | 0.350 | 17/200 | 74/101 | 0 |
| 0.40 | 0.500 | 19/200 | 71/101 | 0 |
| 0.40 | 0.785 | 51/200 | 74/101 | 0 |
| 0.40 | 1.571 | 115/200 | 70/101 | 0 |
| 0.40 | sin chequeo | 196/200 | 63/101 | 0 |
| 0.50 | 0.250 | 3/200 | 72/101 | 0 |
| 0.50 | 0.350 | 17/200 | 75/101 | 0 |
| 0.50 | 0.500 | 22/200 | 76/101 | 0 |
| 0.50 | 0.785 | 51/200 | 78/101 | 0 |
| 0.50 | 1.571 | 120/200 | 75/101 | 0 |
| 0.50 | sin chequeo | 198/200 | 70/101 | 0 |

- Subir yaw a 0.35 / 0.50 / 0.785 rad resuelve pocas GOAL_TOLERANCE, porque el error de yaw al llegar es grande (mediana 1.84 rad). Solo desactivar el chequeo de yaw las elimina casi todas, y eso significa aceptar llegar con cualquier orientación.
- Subir la tolerancia xy (hasta 0.50 m) **no resuelve ninguna** OSCILACIÓN, TIMEOUT ni parada sin trayectorias: esos robots se quedan a 1–2 m.
- Los "éxitos aparentes" que pasan a correctos lo hacen porque se redefine qué se considera correcto, no porque el robot quede mejor.

### 5. Los 10 casos más difíciles

Ranking por path sobre sus 12 corridas (6 escenarios × 2 variantes): fallos sin contar éxitos aparentes ↓, contactos ↓, mejor error final ↓, clearance ↑.

| # | path | categoría | L (m) | fallos / 12 | contactos | clearance mín (m) | error lateral máx (m) | mejor error final xy (m) | clases |
|---|---|---|---|---|---|---|---|---|---|
| 1 | T029 | around_obstacle | 4.36 | 12 | 0 | 0.067 | 0.081 | 2.317 | OSCILACION 12 |
| 2 | T025 | around_obstacle | 3.67 | 12 | 0 | 0.015 | 0.222 | 1.392 | TIMEOUT 12 |
| 3 | T023 | around_obstacle | 2.24 | 12 | 0 | 0.010 | 0.128 | 1.274 | OSCILACION 10, TIMEOUT 1, OTRO_SIN_TRAYECT 1 |
| 4 | T028 | around_obstacle | 2.47 | 12 | 0 | 0.027 | 0.142 | 0.969 | OSCILACION 11, OTRO_SIN_TRAYECT 1 |
| 5 | T027 | around_obstacle | 3.34 | 12 | 0 | 0.019 | 0.112 | 0.779 | OSCILACION 10, OTRO_SIN_TRAYECT 1, TIMEOUT 1 |
| 6 | T042 | corridor_N1 | 2.23 | 11 | 2 | -0.011 | 0.099 | 0.009 | OSCILACION 5, OTRO_SIN_TRAYECT 4, GOAL_TOLERANCE 1, OTRO_EXITO_CONTACTO 1, OK 1 |
| 7 | T024 | around_obstacle | 2.28 | 11 | 0 | 0.044 | 0.153 | 0.016 | OSCILACION 8, TIMEOUT 3, OK 1 |
| 8 | T043 | corridor_N1 | 2.23 | 10 | 3 | -0.057 | 0.103 | 0.005 | OSCILACION 3, OTRO_SIN_TRAYECT 3, OTRO_EXITO_CONTACTO 3, GOAL_TOLERANCE 1, OK 1, OTRO_EXITO_APARENTE 1 |
| 9 | T046 | corridor_N1 | 2.23 | 10 | 1 | -0.054 | 0.103 | 0.007 | OSCILACION 5, OTRO_SIN_TRAYECT 3, GOAL_TOLERANCE 1, OTRO_EXITO_CONTACTO 1, OK 1, OTRO_EXITO_APARENTE 1 |
| 10 | T060 | random | 6.05 | 9 | 0 | 0.014 | 0.374 | 0.001 | GOAL_TOLERANCE 9, OK 2, OTRO_EXITO_APARENTE 1 |

- around_obstacle (goal detrás de un obstáculo): 6 de los 10 paths de la categoría (T023, T024, T025, T027, T028, T029) están en el top 10 y fallan en 11–12 de 12 corridas con ambas variantes. Mínimo local de DWB: el robot gira en el lugar o avanza sin acercarse.
- T042, T043, T046 (corridor_N1): concentran los contactos y las paradas sin trayectorias de B.
- T060 (random, 6 m): llega en xy y falla por yaw en 9/12.

### Detalle por fallo

Los 428 fallos, con path_id, variante, escenario, error lateral máximo, error final xy/yaw, tiempo, clearance mínima, último comando y último estado, están en `dwb_r4c/r4c_failures.csv`. Muestra (los primeros de cada clase):

| path | variante | escenario | clase | atribución | lat máx | final xy | final yaw | t (s) | clearance | último cmd (v, ω) | último estado |
|---|---|---|---|---|---|---|---|---|---|---|---|
| T001 | BaseObstacle | S1_nominal | GOAL_TOLERANCE | PARAMETRO (inferido) | 0.218 | 0.000 | 2.825 | 35.8 | 0.600 | (0.00, 0.00) | ABORT_PROGRESS (Failed to make progress); en goal (xy), girando en el lugar |
| T001 | BaseObstacle | S4_latencia_0p6 | GOAL_TOLERANCE | PARAMETRO (inferido) | 0.111 | 0.004 | 1.692 | 30.5 | 0.600 | (0.00, 0.00) | ABORT_PROGRESS (Failed to make progress); en goal (xy), girando en el lugar |
| T001 | ObstacleFootprint | S1_nominal | GOAL_TOLERANCE | PARAMETRO (inferido) | 0.218 | 0.000 | 2.825 | 35.8 | 0.600 | (0.00, 0.00) | ABORT_PROGRESS (Failed to make progress); en goal (xy), girando en el lugar |
| T023 | BaseObstacle | S1_nominal | OSCILACION | DWB | 0.010 | 1.395 | 0.696 | 25.1 | 0.118 | (0.00, 0.00) | ABORT_PROGRESS (Failed to make progress); a 1.39 m del goal, girando en el lugar |
| T023 | BaseObstacle | S2_ruido_xy | OSCILACION | DWB | 0.045 | 1.387 | 0.242 | 25.6 | 0.076 | (0.00, 0.00) | ABORT_PROGRESS (Failed to make progress); a 1.39 m del goal, girando en el lugar |
| T023 | BaseObstacle | S3_ruido_yaw | OSCILACION | DWB | 0.056 | 1.365 | 0.562 | 25.4 | 0.067 | (0.00, 0.00) | ABORT_PROGRESS (Failed to make progress); a 1.36 m del goal, girando en el lugar |
| T001 | BaseObstacle | S3_ruido_yaw | OTRO_EXITO_APARENTE | LOCALIZACION | 0.188 | 0.008 | 0.332 | 25.6 | 0.600 | (0.00, 0.16) | SUCCEEDED (—); en goal (xy), desplazándose |
| T001 | BaseObstacle | S6_combinado_lat0p6 | OTRO_EXITO_APARENTE | LOCALIZACION | 0.062 | 0.057 | 0.500 | 22.0 | 0.600 | (0.04, 0.89) | SUCCEEDED (—); en goal (xy), desplazándose |
| T001 | ObstacleFootprint | S3_ruido_yaw | OTRO_EXITO_APARENTE | LOCALIZACION | 0.188 | 0.008 | 0.332 | 25.6 | 0.600 | (0.00, 0.16) | SUCCEEDED (—); en goal (xy), desplazándose |
| T042 | BaseObstacle | S5_combinado_medido | OTRO_EXITO_CONTACTO | PARAMETRO | 0.073 | 0.054 | 0.026 | 26.2 | -0.011 | (0.03, -1.00) | SUCCEEDED (—); en goal (xy), desplazándose |
| T043 | BaseObstacle | S2_ruido_xy | OTRO_EXITO_CONTACTO | DWB | 0.103 | 0.040 | 0.103 | 33.2 | -0.057 | (0.03, 1.00) | SUCCEEDED (—); en goal (xy), desplazándose |
| T043 | BaseObstacle | S5_combinado_medido | OTRO_EXITO_CONTACTO | PARAMETRO | 0.079 | 0.030 | 0.141 | 29.2 | -0.018 | (0.00, 0.89) | SUCCEEDED (—); en goal (xy), desplazándose |
| T023 | ObstacleFootprint | S6_combinado_lat0p6 | OTRO_SIN_TRAYECT | DWB | 0.128 | 1.274 | 0.448 | 35.7 | 0.020 | (0.00, 0.00) | ABORT_CONTROLLER (No valid trajectories out of 411! ); a 1.27 m del goal, girando en el lugar |
| T027 | ObstacleFootprint | S2_ruido_xy | OTRO_SIN_TRAYECT | DWB | 0.112 | 0.779 | 3.126 | 45.2 | 0.026 | (0.00, 0.00) | ABORT_CONTROLLER (No valid trajectories out of 411! ); a 0.78 m del goal, girando en el lugar |
| T028 | ObstacleFootprint | S4_latencia_0p6 | OTRO_SIN_TRAYECT | DWB | 0.142 | 0.969 | 0.221 | 15.0 | 0.038 | (0.00, 0.00) | ABORT_CONTROLLER (No valid trajectories out of 411! ); a 0.97 m del goal, desplazándose |
| T023 | ObstacleFootprint | S2_ruido_xy | TIMEOUT | DWB | 0.111 | 1.296 | 0.527 | 57.2 | 0.016 | (0.00, 0.47) | TIMEOUT (t_max); a 1.30 m del goal, desplazándose |
| T024 | BaseObstacle | S3_ruido_yaw | TIMEOUT | DWB | 0.074 | 1.502 | 0.551 | 57.5 | 0.089 | (0.00, 0.16) | TIMEOUT (t_max); a 1.50 m del goal, desplazándose |
| T024 | BaseObstacle | S5_combinado_medido | TIMEOUT | PARAMETRO | 0.060 | 1.545 | 1.695 | 58.1 | 0.083 | (0.00, 0.79) | TIMEOUT (t_max); a 1.54 m del goal, desplazándose |

## LIMITACIONES

- Todo proviene de la simulación R4-B (modelo cinemático mínimo, percepción perfecta, sin BT ni recuperaciones, plan fijo, una semilla por caso). En el robot real, un ABORT dispara recuperaciones y replanificación que aquí no existen.
- La atribución "DWB" es **por exclusión** y la atribución "sin orientación final" es **inferida**: no se simuló RotateToGoal ni otros pesos de critics. Confirmarlas requiere simular (R4-D).
- El contrafactual de tolerancias evalúa la pose verdadera **en el instante del disparo**. No modela el sobrepaso al frenar posterior (que sí existe: 66 éxitos aparentes), así que puede sobreestimar las resoluciones en escenarios con latencia alta.
- Las fronteras entre clases dependen de umbrales elegidos (quieto < 0.1 m en 25 s, ≥ 4 inversiones de ω, tracking > 0.25 m). Los números por clase son sensibles a ellos; los estados de fin (ABORT_PROGRESS / TIMEOUT / ABORT_CONTROLLER / SUCCEEDED) no.
- El contrafactual **no es monótono** en la tolerancia xy (p. ej. yaw 0.25: 5 resueltas con xy 0.40 y 3 con 0.50). El goal checker es stateful: con más tolerancia xy la etapa 1 dispara antes y la condición de yaw y la pose verdadera se evalúan en otros instantes.
- La causa de que el ruido XY haga desaparecer GOAL_TOLERANCE está medida pero su mecanismo no se verificó.

## Archivos

- Script: `scripts/nav_offline_lab/analyze_r4c_failures.py`, reporte `scripts/nav_offline_lab/make_dwb_r4c_report.py`
- Resultados: `results/navigation_offline_lab/dwb_r4c/` (`r4c_all_runs.csv` 768 corridas, `r4c_failures.csv`, `r4c_summary.json`)

## Siguiente ajuste recomendado

**Activar `RotateToGoal` en una copia experimental de la config** (sus parámetros ya están en `navigation2_real.yaml`: scale 32, slowing_factor 5, lookahead_time -1; solo falta agregarlo a `critics`). Luego medirlo con el simulador R4-B en los mismos 6 escenarios × 2 variantes (R4-D). Ataca la causa dominante: 200 GOAL_TOLERANCE y, por su `slowing_factor`, probablemente parte de los 66 sobrepasos al frenar. Criterio de éxito: GOAL_TOLERANCE < 10/384 por variante sin aumentar contactos.

En segundo lugar, y por separado, los mínimos locales de around_obstacle (T023–T029): probar pesos GoalDist/GoalAlign vs PathDist/PathAlign. No se recomienda subir la tolerancia de yaw: resuelve pocas corridas, salvo desactivarla, y oculta el problema. Ningún cambio a producción sin tu confirmación.
