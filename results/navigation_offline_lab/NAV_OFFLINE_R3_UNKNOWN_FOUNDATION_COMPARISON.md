# NAV_OFFLINE_R3_UNKNOWN_FOUNDATION_COMPARISON

Fecha: 2026-09-29. Modo: **100 % offline**. Sin hardware y sin nodos físicos. Solo se levantó `map_server`, `planner_server` (NavFn real), 2 TF estáticas y `lifecycle_manager` con `ROS_LOCALHOST_ONLY=1` y `ROS_DOMAIN_ID=99`.
- La auditoría del grafo en cada corrida no encontró topics ni acciones de movimiento. Solo había `/compute_path_to_pose` y `/compute_path_through_poses`.
- No quedaron procesos residuales.

Leyenda: **[C]** CONFIRMADO (medido aquí) · **[E]** EVIDENCIA · **[H]** HIPÓTESIS

> **Todo se midió sobre mapas SINTÉTICOS de desarrollo. Nada de esto es evidencia de desempeño físico.**

---

## 0. Qué se comparó

| Tag | Mapa | Config | `track_unknown_space` | Footprint |
|---|---|---|---|---|
| A | synthetic_arena | `planner_offline_A.yaml` | sin definir (= false) | A, padding 0.03 |
| B | synthetic_arena | `planner_offline_B.yaml` | sin definir (= false) | B, padding 0.02 |
| B_rerun | synthetic_arena | `planner_offline_B.yaml` | sin definir | B (control de determinismo) |
| **R3** | synthetic_arena | `planner_offline_R3.yaml` | **true** | B, padding 0.02 |
| SUP_B | **synthetic_arena_unknown** | `planner_offline_B.yaml` | sin definir | B |
| **SUP_R3** | **synthetic_arena_unknown** | `planner_offline_R3.yaml` | **true** | B |

- `planner_offline_R3.yaml` difiere de `planner_offline_B.yaml` **en una sola línea** (`track_unknown_space: true`), verificado con diff de texto y de campos.
- Planner NavFn, inflación (0.35 / 2.0), resolución 0.03 y los 64 pares start/goal (`synthetic_benchmark_pairs.json`, seed 42) son idénticos en todas las corridas.
- Los dos mapas usan `free_thresh: 0.10` y pasan `validate_ros_map.py --strict --expect-resolution 0.03`. `map_server` entrega el UNKNOWN como −1 en todas las corridas: 34 848 celdas en el sintético y 35 868 en el suplementario.

**Mapa suplementario** (`synthetic_arena_unknown/`, generado por `make_synthetic_arena_unknown.py`; el original no se modificó, su sha256 no cambió):
- **G1** y **G2**: huecos de 0.60 m en la pared superior, a cada lado del divisor, que representan tramos de pared no observados.
- **U1**: zona interna no observada de 0.90 × 0.90 m en x ∈ [6.06, 6.96], y ∈ [5.52, 6.42].
  - Se ubicó automáticamente donde más rutas baseline cruzaban (14), con cualquier start/goal a ≥ 0.45 m.
  - Ningún par se invalidó.

Evaluación: `path_eval.py`, la misma lógica que R2; reproduce el CSV de R2 con 0 diferencias en 128 filas. Comparador: `compare_r3.py`. "Ruta cambiada" significa Hausdorff ≥ 1 cm.

Salidas:
- `r3_comparison.csv`, `r3_comparison_summary.json`;
- figuras `r3_*.png`;
- corridas en `planner_runs/paths_{A,B,B_rerun,R3,SUP_B,SUP_R3}.json` y `audit_*.json`.

---

## 1. Resultados

### 1.1 Control de determinismo [C]
B frente a B_rerun: **64/64 rutas idénticas** (Hausdorff < 1 cm, longitud total 353.858 m en ambas). NavFn es determinista, así que las diferencias que siguen se deben a la configuración.

### 1.2 Estado del costmap [C]
| Corrida | Celdas LETALES | Valor del UNKNOWN exterior |
|---|---|---|
| B / SUP_B | 8 656 / 8 536 (solo paredes) | **0 (libre)** |
| R3 / SUP_R3 | 43 504 / 44 404 (paredes + todo el UNKNOWN) | **100 (letal)** |

Los valores dentro de la arena (N1 86, N2 99, WIDE 0) son idénticos entre B y R3.

### 1.3 Planificación

| Comparación | Qué aísla | Encontradas antes → después | Cambiadas | Perdidas | Nuevas | Longitud total (rutas comunes) |
|---|---|---|---|---|---|---|
| B → R3 (sintético) | solo `track_unknown_space` | 64 → 64 | **0** | 0 | 0 | 353.86 → 353.86 m |
| A → R3 (sintético) | footprint A→B + unknown | 64 → 64 | **22** | 0 | 0 | 262.34 → 353.86 m (+91.5) |
| **SUP_B → SUP_R3** | solo `track_unknown_space`, UNKNOWN expuesto | 64 → 64 | **15** | **0** | 0 | 296.16 → 406.64 m (+110.5) |

Tiempo de planificación en ms (mediana / p95 / máx) [C]:
- A: 3.7 / 9.4 / 12.5
- B: 4.4 / 9.4 / 10.1
- B_rerun: 3.0 / 7.2 / 9.6
- R3: 3.6 / 9.3 / 9.7
- SUP_B: 2.9 / 6.9 / 7.7
- SUP_R3: 3.7 / 9.0 / 9.8

Sin diferencia relevante. Todo queda por debajo de 13 ms.

### 1.4 Detalle SUP_B → SUP_R3 (el caso donde UNKNOWN influye) [C]
| Rutas | Antes (UNKNOWN libre) | Después (UNKNOWN bloqueado) | Δ longitud |
|---|---|---|---|
| 8 × corridor_N2 (T047–T054) | **salen de la arena por G1, recorren ~6 m de exterior no observado y reentran por G2** (195–200 poses en UNKNOWN) | rodean por WIDE y bordean U1 | **+11.5 a +11.6 m** (6.9 → 18.5 m) |
| T011 (long) | sale por los huecos (198 poses en UNKNOWN) | por WIDE | +4.67 m |
| T014, T056, T058 | cruzan U1 (60 poses en UNKNOWN) | bordean U1 | +3.59, +2.15, +3.68 m |
| T028, T030 (around_obstacle) | cruzan U1 (35 poses) | bordean U1 | +3.81, +0.17 m |
| T025 (around_obstacle) | no toca UNKNOWN | desvío mínimo (Hausdorff 1 cm, −3 cm) | −0.03 m |

- Rutas que atraviesan UNKNOWN: **14 → 0**.
- Rutas perdidas: **0**. Siempre existió una alternativa por espacio observado.
- T025 [E]: probablemente cambia por el campo de coste alterado por la inflación alrededor de U1, que ahora es letal. El cambio es insignificante.

### 1.5 Clearance y clases [C]
Obstáculo = ocupado ∪ UNKNOWN. MARGINAL: < padding + 0.05 m, un valor de trabajo no medido. Swept con el modelo de giro en el sitio supuesto en R2.

| Comparación | Clearance mín. del centro antes → después | Clearance media del centro | Footprint B | Footprint A |
|---|---|---|---|---|
| B → R3 | 0.205 → 0.205 m | 0.565 → 0.565 m | 54 P / 10 M → sin cambios | sin cambios |
| A → R3 | 0.125 → 0.205 m | 0.514 → 0.565 m | 46 P / 10 M / **8 C** → 54 P / 10 M / **0 C** | 55 P / 9 M → 63 P / 1 M |
| SUP_B → SUP_R3 | **−0.005** → 0.205 m | 0.515 → 0.557 m | 40 P / 10 M / **14 C** → 54 P / 10 M / **0 C** | 49 P / 1 M / **14 C** → 63 P / 1 M |

- En SUP_B, las 14 "COLLISION" son las 14 rutas que atraviesan UNKNOWN. Con la definición conservadora usada, entrar en espacio no observado cuenta como colisión.
  - **[H]** No significa que en la realidad hubiera choque: significa que la ruta pasa por espacio del que el mapa **no sabe nada**, fuera del perímetro o por una zona no escaneada.
- Las 10 MARGINAL de B (rutas por N1 y dos giros en el sitio cerca de paredes) **no cambian** con R3. `track_unknown_space` no resuelve pasos angostos entre obstáculos conocidos.

---

## 2. Respuestas

**1. ¿`track_unknown_space=true` cambia realmente la planificación?**
- **[C]** En el mapa sintético de perímetro cerrado, **no**: 0/64 rutas cambian. Ahí el UNKNOWN solo existe fuera de las paredes y la inflación de las paredes ya domina.
- **[C]** En cuanto el UNKNOWN toca el espacio libre (paredes no observadas o zonas no escaneadas), **sí**: 15/64 rutas cambian. Todas las que usaban espacio no observado (14) dejan de hacerlo.

**2. ¿Cuántas rutas cambian?**
- [C] 0 (sintético, solo unknown).
- [C] 15 (suplementario, solo unknown): 14 por dejar de atravesar UNKNOWN y 1 desvío de 3 cm.
- [C] 22 (A → R3, que incluye el cambio de footprint): todas se deben al footprint, porque el cambio de unknown aporta 0 en ese mapa.

**3. ¿Cuántas desaparecen?** [C] **0** en todas las comparaciones: 64/64 antes y después. Todas las metas tenían una alternativa por espacio observado.
- [H] En un mapa donde la **única** conexión pasa por UNKNOWN, R3 haría fallar la planificación. Ese es el comportamiento deseado (no se ha probado aquí).

**4. ¿Cuánto cambia la longitud?** [C]
- Suplementario: +110.5 m sobre 64 rutas (+37 %), concentrado en 15 rutas. Las 8 rutas N2 pasan de 6.9 a 18.5 m (×2.7); el resto crece entre +0.17 y +4.7 m.
- A → R3: +91.5 m, casi todo por las 8 rutas N2 que con footprint B no caben (2.24 → 13.67 m).

**5. ¿Qué pasa con la clearance?** [C]
- Con UNKNOWN libre, la clearance mínima respecto a lo no observado cae a −0.005 m: las rutas lo atraviesan.
- Con R3 queda en 0.205 m, la misma que en el mapa cerrado.
- La clearance media sube de 0.515 a 0.557 m.
- Las COLLISION por UNKNOWN pasan de 14 a 0. Las MARGINAL (10) no cambian.

**6. ¿Es recomendable para la navegación real?**
- **[E] Sí, como parte de la configuración base**, junto con cargar solo mapas validados (`free_thresh` ≤ 0.196). Motivos:
  - evita rutas por espacio no observado sin coste medible de tiempo de planificación;
  - en mapas cerrados no altera ninguna ruta;
  - el mapa real disponible (arena_20260918) tiene el 85 % de la frontera del libre en contacto con UNKNOWN, el caso "suplementario", no el "cerrado".
- **[H] Riesgos a vigilar antes de usarlo en el robot:**
  - (a) Con `inflate_around_unknown: true`, cualquier hueco de escaneo cerca del camino se infla como obstáculo. En mapas reales ruidosos, con muchas manchas UNKNOWN pequeñas (R1 midió 203 componentes internos), podría cerrar pasos o alargar rutas.
  - (b) Un robot cuyo start cae en UNKNOWN no podrá planificar.
  - (c) No se probó la interacción con `obstacle_layer`: el raytrace del LiDAR puede limpiar UNKNOWN en el costmap en marcha.
  - Esto debe medirse offline con un mapa real válido antes de adoptarlo.
- El cambio **no** se ha aplicado a ninguna configuración real. `navigation2_real.yaml` sigue intacto.

**7. Limitaciones**
- Mapas sintéticos: la geometría y los tamaños de UNKNOWN los elegí yo. U1 se colocó **a propósito** donde cruzaban rutas baseline (sesgo de selección intencional). Mide el mecanismo, no la frecuencia real.
- Solo el global costmap con `static_layer` + `inflation_layer`. Sin `obstacle_layer`, local costmap, controller ni smoother.
- Un solo planner (NavFn) y una sola resolución (0.03).
- La clasificación de footprint usa un modelo de ejecución supuesto (giro en el sitio, rumbo tangente) y un margen de 0.05 m no medido. Cuenta el UNKNOWN como obstáculo.
- 64 pares y una semilla. No hay mapa real válido de la arena para repetirlo.
- No se probaron metas inalcanzables sin atravesar UNKNOWN (el caso "ruta perdida").

---

## 3. Conclusión
- **[C]** `track_unknown_space: true` (con footprint B) **elimina el 100 % de las rutas que atravesaban espacio no observado** (14 → 0) sin perder ninguna ruta.
  - Coste: rutas más largas solo donde antes "atajaban" por lo desconocido (+11.6 m como máximo).
  - En un mapa cerrado no tiene ningún efecto (0/64 cambios).
- **[C]** El footprint es el factor dominante en pasos angostos: A → B elimina las 8 colisiones de N2 alargando esas rutas ×6. Es independiente del ajuste de unknown.

## 4. Siguiente paso
Obtener un mapa real válido de la arena (mapeo completo guardado con `save_map_checked.sh`; requiere autorización de movimiento). Sobre ese mapa, repetir offline B frente a R3 para medir el efecto del UNKNOWN real (manchas, huecos de escaneo) antes de proponer `track_unknown_space: true` para el robot.
