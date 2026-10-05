# NAV-OFFLINE-R4-G1 — Baseline completa en replan_1hz (candidata R4-F + re-validación de la cadena R4-B..R4-F)

Fecha: 2026-10-03. 100 % offline (ROS_DOMAIN_ID=99, ROS_LOCALHOST_ONLY=1).
- Sin hardware, sin limo_base, sin LiDAR real, sin teleop, sin `/cmd_vel`. El simulador no tiene publishers Twist ni acciones; la auditoría del grafo dio PASS en 30 de 30 tandas más el autotest.
- `config/navigation/navigation2_real.yaml` sin cambios (sha256 `dfb36e108d6872a5…`). `~/limoatwork` sin cambios.
- **No se tuneó nada.** progress_checker en 0.10 m / 25.0 s. No se ejecutaron recoveries.

## Resumen

```
R4-G1 — candidata R4-F (F010), 384 corridas (64 paths × 6 escenarios, mismas semillas que R4-F)
                         fixed_plan (R4-F)   replan_1hz (R4-G1)
correct successes            334                332        (pareado: +6 / −8, McNemar p = 0.79)
false successes                1                  3        (1 por yaw)
GOAL_TOLERANCE                 3                  3
ABORT_PROGRESS                26 (3 cerca)       32 (2 cerca del goal + 30 lejos, todos en around_obstacle)
no valid trajectories         14                 15
timeout                        9                  2
contactos                      2                  2        (mismas corridas: S4:T042, S4:T045)
bloqueos (OSC+TIMEOUT+SIN)    46                 41        (+5 TRACKING nuevos: fallas totales ≈ iguales)
cambios de signo angular.z   6967               8997       (+29 %; por minuto p50 31.9 → 40.7)
setPlan por corrida            1           p50 27 (5–84)
```

**La candidata R4-F sigue siendo válida bajo replan_1hz: YES.** Hay tres problemas abiertos, que **no dependen de la candidata** porque aparecen en toda la cadena: la sensibilidad de N1, 2 aborts cerca del goal y fallos de NavFn en N1 con ruido.

## Validación previa (automática, todo PASS)

| # | verificación | resultado |
|---|---|---|
| 1 | config = R4-F | `verify_r4g1.py static` (25 checks). `dwb_offline_R4G1_F010_replan_sim.yaml` = `dwb_offline_R4F_010_sim.yaml` + solo `GridBased.*` y `global_costmap.*`, ninguna clave existente cambia. Coincide con la config ya validada en R4-G0. Se verificó explícitamente: ObstacleFootprint + RotateToGoal, FollowPath xy 0.15, checker 0.25 / 0.10, progress 0.10 / 25.0, footprint B, padding 0.02 |
| 2 | solo cambia el modo | parámetros de escenario de `run_r4g1.sh` == `run_r4f_all.sh` (comparación automática). Post-corrida, la `meta` de las 30 tandas == la de fixed_plan salvo `{plan_mode: replan_1hz, replan_period: 1}` |
| 3 | bug de NavFn corregido | la fuente restaura el global costmap prístino **antes** de cada `createPlan`; binario más nuevo que la fuente. Autotest: T039 recorre N1 justo antes de T041/T043/T045, y los planes iniciales de los tres siguen idénticos a R3 |
| 4 | costmap limpio por corrida | autotest: las 5 corridas son bit a bit idénticas a las de R4-G0 v3 del mismo caso, que tenían otras corridas previas. Las celdas restauradas crecen de forma monótona: los residuos se detectan y se eliminan |
| 5 | plan inicial = benchmark | las 64 corridas de S1 (sin ruido) de cada eslabón tienen el mismo número de poses y desvío ≤ 1e-4 m respecto de `paths_R3.json` |
| 6 | sin Twist físico | auditoría del grafo PASS en 31/31 lanzamientos |

Nota sobre la verificación: la primera versión del check 5 usaba 1e-5 m y marcó T048/T050/T052/T054 (N2) con un desvío de **1.6e-5 m**, misma geometría y mismas 907 poses. Es la precisión del JSON (6 cifras significativas) frente a los float32 de `paths_R3`. Se fijó en 1e-4 m (0.1 mm, contra una celda de 30 mm) y se documentó en el script.

## CONFIRMADO

### Candidata R4-F (F010) bajo replan_1hz

- **Rendimiento global estable**: 334 → 332 correctos, sin diferencia significativa (6 pasan a OK, 8 dejan de serlo; p = 0.79). En 354/384 corridas la clase es la misma.
- **Seguridad estable**:
  - los mismos 2 contactos (S4, N1, latencia 0.6 s), con el mismo tiempo en contacto (1.1 s);
  - contactos nuevos 0;
  - clearance mínima global −0.0095 → −0.0078 m (en las corridas con contacto) y p05 0.0158 → 0.0155 m;
  - corridas bajo el padding: 29 → 30.
- **Precisión final** (corridas que llegan en xy):
  - XY p50 / p90: 0.133 / 0.168 → 0.126 / 0.154 m;
  - yaw p50 / p90 / p95: 0.071 / 0.105 / 0.138 → 0.074 / 0.112 / 0.157 rad;
  - pares OK en ambos modos: |Δxy| p90 0.008 m, |Δyaw| p90 0.022 rad.
- **Tiempo hasta SUCCEEDED**: p50 20.2 → 20.1 s, p90 61.0 → 61.1 s. Pares OK: Δt p50 0.0 s, p90 +1.05 s, extremos de −12 a +11 s.
- **Los 9 TIMEOUT desaparecen**: 7 pasan a OSCILACIÓN (ABORT_PROGRESS) y otros 2 a sin trayectorias o TRACKING, **a los ~30 s en lugar de ~74 s**. Con replan, el robot atascado en around_obstacle deja de deambular más de 0.1 m cada 25 s, y el progress checker lo detecta antes.
- **ABORT_PROGRESS = 32**: 30 lejos del goal, **todos en around_obstacle** (los mínimos locales de R4-C), y 2 cerca del goal, S1:T041 (el de R4-G, persiste) y S3:T041 (nuevo). S2:T045 y S5:T041 ya no abortan cerca del goal (lo mismo que en R4-G0).
- **Los 46 bloqueos originales** (R4-B variante B, fixed): en F010 replan siguen fallando **41**, como OSCILACIÓN 26, sin trayectorias 10 y TRACKING 5. Pasan a OK 5. Aparecen 5 bloqueos nuevos (4 en N1, sin trayectorias). Las fallas por bloqueo o seguimiento quedan prácticamente iguales.
- **N1**: correctos 34 → 35, bloqueos 11 → 11, sin trayectorias 11 → 11, contactos 2 → 2. Pero 8 corridas cambian de clase (4 mejoran, 4 empeoran): el saldo es nulo con mucha variación. N1 sigue al límite, con 1 trayectoria legal de 411.
- **Oscilación**: el reset periódico aumenta los cambios de signo de angular.z en **todos** los eslabones.

  | eslabón | fixed → replan | aumento |
  |---|---|---|
  | F010 | 6967 → 8997 | +29 % |
  | B_B | 9261 → 12867 | +39 % |
  | B_A | 9247 → 13263 | +43 % |

  En F010: por minuto, p50 31.9 → 40.7; después de empezar el giro final, media 0.22 → 1.30 por corrida. Es consistente con R4-G0: setPlan borra la histéresis de Oscillation cada 1 s.

### Robustez de cada paso de la cadena (McNemar pareado, 384 corridas por eslabón y modo)

| pregunta / paso | fixed_plan: correctos (+/−, p) | replan_1hz: correctos (+/−, p) | ¿se mantiene? |
|---|---|---|---|
| 1. ObstacleFootprint (B_A → B_B) | 147 → 193 (+58/−12, p = 2e-8); contactos 5 → 2; bloqueos 76 → 46 | 163 → 189 (+49/−23, p = 0.003); contactos 3 → 2 (tiempo en contacto 2.8 → 1.1 s); bloqueos 72 → 41 | **Sí**, con menor ganancia (+26 contra +46). Con replan, BaseObstacle mejora sola (contactos 5 → 3, correctos 147 → 163) |
| 2. RotateToGoal (B_B → D) | 193 → 269 (+122/−46, p = 4e-9); GoalTol 99 → 13 | 189 → 278 (+121/−32, p = 2e-13); GoalTol 93 → 14; aborts cerca del goal 91 → 4 | **Sí**, más fuerte. Cambios de signo después de llegar en xy 14.0 → 1.3 |
| 3. FollowPath xy 0.15 (D → E015) | 269 → 310 (+46/−5, p = 2e-9) | 278 → 306 (+38/−10, p = 6e-5); GoalTol 14 → 0; falsos éxitos 46 → 32 | **Sí** |
| 4. yaw 0.10 (E015 → F010) | 310 → 334 (+26/−2, p = 3e-6); falsos por yaw 25 → 0 | 306 → 332 (+30/−4, p = 6e-6); falsos por yaw 29 → 1 | **Sí** |

Todas las conclusiones R4-A2 → R4-F se mantienen bajo replan, en dirección y con significancia. La candidata R4-F es la mejor de la cadena en ambos modos.

### Fallos de NavFn (hallazgo nuevo)

- **5 planes vacíos de NavFn en 3 corridas**, S6:T042 (2), S6:T044 (2) y S6:T046 (1), todas en N1 con ruido y latencia 0.6 s. Ocurren **igual** en F010, E015, D y B_B, y no en B_A.
- **Causa verificada**: la y estimada estaba en 5.085–5.099 m, es decir, a 0.136–0.150 m de la pared, dentro del radio inscrito del global costmap (0.16 + 0.02 = 0.18 m). NavFn trata la celda como obstáculo. La y **real** estaba en 5.045–5.055 m, dentro de la franja libre.
  - En N1 (0.45 m) la franja que NavFn considera libre mide solo 0.45 − 2 × 0.18 ≈ **9 cm**.
  - Con σ_xy = 0.03 m de ruido de localización, la pose estimada sale de esa franja.
- En el simulador se conserva el plan anterior y las 3 corridas terminan SUCCEEDED. **En el BT real**, ComputePathToPose fallaría: ClearGlobalCostmap-Context y reintento, y si vuelve a fallar se detiene FollowPath y empieza el recovery (Clearing / Spin / Wait / BackUp) **dentro de N1**. Estas 3 corridas no deben contarse como éxitos limpios.

## EVIDENCIA

- La caída de ganancia de ObstacleFootprint bajo replan (+46 → +26) se debe sobre todo a que **BaseObstacle mejora con replan**: el reset de critics y la replanificación desde la pose actual sacan a BaseObstacle de bloqueos en la entrada de N1. ObstacleFootprint sigue siendo mejor en correctos, contactos y bloqueos.
- ObstacleFootprint tiene **más** corridas bajo el padding (22 → 30 en fixed, 24 → 32 en replan) pero menos contactos reales. La explicación probable: BaseObstacle se traba antes en la entrada de N1 y no llega a los pasajes estrechos. Es un efecto de selección, no de seguridad peor.
- Los 30 ABORT_PROGRESS lejos del goal son los **mínimos locales de around_obstacle** (T023–T029, ya identificados en R4-C). El progress checker no los causa, solo decide cuándo se declaran. Cambiar sus parámetros cambiaría el momento del abort, no el resultado.

## HIPÓTESIS

- Los fallos de NavFn en N1 aumentarían con un ruido de AMCL real mayor que 0.03 m. Cualquier navegación por N1 con localización ruidosa puede activar recoveries por fallo del planner, independientemente del controlador.
- S3:T041 (abort nuevo cerca del goal) tiene el mismo mecanismo que S1:T041 (R4-G): RotateToGoal empieza a 0.14 m del goal junto a la salida de N1. Falta confirmarlo con `debug_eval_runs`.

## Recoveries (hallazgo documentado, no ejecutado)

- Spin y BackUp chequean el footprint con padding contra `costmap_raw`, con criterio ≥ LETHAL (INSCRIBED no cuenta).
- BackUp a 0.05 m/s durante `simulate_ahead_time` 2.0 s proyecta ~0.10 m (0.095 m).
- Desde las poses de atasco S2:T045 y S5:T041, un BackUp de 0.30 m intersectaría la pared real después de 0.13–0.14 m (`dwb_r4g/recovery_geometry.json`).
- Detalle completo en `DWB_R4G0_REPLAN_FIDELITY_REPORT.md`.

## LIMITACIONES

- Las del simulador R4-B: modelo cinemático mínimo, odometría perfecta, ruido de localización sintético, mapa sintético, sin LiDAR ni capa de obstáculos y una semilla por caso.
- Se replanifica a 1 Hz como el BT por defecto, pero **sin BT**:
  - un fallo de NavFn no dispara recoveries; se conserva el plan anterior;
  - ABORT_PROGRESS y ABORT_CONTROLLER terminan la corrida, sin reintentos.
- N1 está al límite: 1 de 411 trayectorias legales en tramos, y perturbaciones mínimas cambian el resultado. Conclusiones de N1 con una sola semilla no son robustas.
- El global costmap se restaura antes de cada plan, como aproximación al repintado de la capa de obstáculos real.

## Archivos

- **Configs** (NO producción): `config/offline_lab/dwb_offline_R4G1_{B_A,B_B,D,E015,F010}_replan_sim.yaml`, `dwb_offline_R4G1_manifest.json`.
- **Scripts**: `scripts/nav_offline_lab/{make_dwb_r4g1_configs.py, verify_r4g1.py, run_r4g1.sh, run_r4g1_resume.sh, evaluate_dwb_r4g1.py}`.
- **Resultados**: `results/navigation_offline_lab/dwb_r4g1/`:
  - `sim_<S>_<TAG>.jsonl`: 30 × 64 corridas replan, con el registro de cada setPlan en `setplan`;
  - `audit_*`, `launch_*`;
  - `verify_static.json`, `verify_selftest.json`, `verify_runs.json`, `selftest/`;
  - `r4g1_runs.csv` (3840 filas: 5 eslabones × 2 modos × 384), `r4g1_summary.json`;
  - `run_all_part1.log` (autotest + F010, E015, D) y `run_all_part2.log` (B_B, B_A). La tarea original se detuvo al empezar B_B para no pasar el límite de tiempo y se reanudó con `run_r4g1_resume.sh`, que salta las tandas completas. No hubo tandas parciales.
- `dwb_r4g/sweep_STOPPED_before_R4G0/` tiene un `README` y el marcador `NO_VALIDO_PARA_CONCLUSIONES`. Se conserva, **no se usa**.

## Siguiente paso recomendado

1. **R4-G2: barrido de progress_checker sobre esta baseline replan_1hz** (radio 0.05 / 0.03; tiempo 35 / 45, de a una variable), comparando pareado contra F010 replan. Hay que tener presentes dos cosas:
   - solo 2 aborts son cerca del goal y 30 son mínimos locales de around_obstacle;
   - el criterio es no convertir aborts en giros o deambulación más largos.
2. **El fallo de NavFn en N1 con ruido** es un problema aparte y probablemente más importante para el robot real. Interrumpiría FollowPath y lanzaría recoveries dentro de un corredor de 0.45 m. Analizarlo offline (franja libre de 9 cm frente al ruido de AMCL) antes de cualquier prueba física en N1.
3. Repetir N1 con 3+ semillas antes de sacar conclusiones sobre ese corredor.
4. Confirmar con `debug_eval_runs` el mecanismo de S3:T041.
