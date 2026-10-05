# NAV_OFFLINE_R2_REPORT

Fecha: 2026-09-29. Modo: **100 % offline**. Sin hardware, sin limo_base, sin LiDAR, sin teleop, sin `/cmd_vel`. El robot no se movió.
Toda prueba ROS usó `ROS_LOCALHOST_ONLY=1` y `ROS_DOMAIN_ID=99`. No quedan procesos residuales (verificado con `ps`).

Integridad verificada al cierre:
- `~/limoatwork`: 0 archivos modificados hoy.
- arena_20260918_1900: sha256 y md5 del bag OK.
- `config/navigation/navigation2_real.yaml`: sin cambios (mtime 2026-09-15).

Leyenda: **[C]** CONFIRMADO · **[E]** EVIDENCIA · **[H]** HIPÓTESIS / supuesto

---

## 0. Artefactos

| Fase | Archivos |
|---|---|
| 1 Procedencia | `MAP_THRESHOLD_PROVENANCE.md`, `provenance/` · script `provenance_map_saver_experiment.sh` |
| 2 Validador | `scripts/nav_offline_lab/validate_ros_map.py`, `tests/test_validate_ros_map.py` (**21/21 PASS**) |
| 3 Pipeline | `save_map_checked.sh`, `save_map_helper.py`, `tests/test_save_map_checked.sh` (**10/10 PASS**), `tests/fake_map_publisher.py`, `FUTURE_MAP_SAVE_PROTOCOL.md`, `save_pipeline_test/` |
| 4 Mapa sintético | `synthetic_arena/synthetic_arena.{pgm,yaml,png}`, `synthetic_arena_geometry.json` · script `make_synthetic_arena.py` |
| 5 Footprints | `footprint_analysis_r2/footprint_analysis.json`, `footprint_cspace.png`, `cspace_masks.npz` · scripts `footprint_eval.py`, `footprint_analysis.py` |
| 6 Planner | `config/offline_lab/planner_offline.launch.py`, `planner_offline_{A,B,B_trackunknown}.yaml` · `run_planner_benchmark.sh`, `planner_benchmark_client.py` · `planner_runs/audit_*.json`, `paths_*.json` |
| 7 Benchmark | `synthetic_benchmark_pairs.json`, **`synthetic_planner_benchmark.csv`** (128 filas), `synthetic_planner_benchmark_summary.json`, `synthetic_benchmark_paths.png`, `synthetic_swept_examples.png` · `make_benchmark_pairs.py`, `evaluate_planner_benchmark.py` |

---

# PARTE I — RESULTADOS SOBRE MAPA SINTÉTICO
> El mapa sintético es **solo de desarrollo**. No representa la arena ni el robot físico. Nada de esta parte es evidencia de desempeño físico.

## I.1 Mapa sintético [C]
- 387 × 387 px a 0.03 m. Interior libre de 9.99 × 9.99 m, perímetro de 0.09 m, 4 esquinas limpias y UNKNOWN exterior de 0.72 m.
- Divisor central con tres pasos de 1.50 m de largo: **WIDE 1.20 m**, **N1 0.45 m** y **N2 0.33 m**. Los anchos se midieron en el raster.
- 4 obstáculos: caja, caja rectangular, columna circular r 0.30 y pared en L.
- yaml trinary con `free_thresh: 0.10`. `validate_ros_map.py --strict --expect-resolution 0.03` → **PASS**:
  - FREE 70.95 %, OCC 5.78 %, UNKNOWN 23.27 %;
  - 0 celdas libres adyacentes a UNKNOWN (perímetro cerrado).
- nav2_map_server lo carga con 34 848 celdas −1, idéntico al validador.

## I.2 Footprints (geométrico, C-space de 1 cm, 72 orientaciones) [C sobre el sintético]

| | A raw | A padded (0.03) | B raw | B padded (0.02) |
|---|---|---|---|---|
| Ancho × largo | 0.20 × 0.36 | 0.26 × 0.42 | 0.32 × 0.39 | 0.36 × 0.43 |
| Área donde cabe con algún yaw | 86.38 m² | 83.59 m² | 80.85 m² | 79.01 m² |
| Área donde puede girar en el sitio | 75.30 m² | 71.75 m² | 70.88 m² | 68.97 m² |
| WIDE (margen lateral) | pasa (0.50) | pasa (0.47) | pasa (0.44) | pasa (0.42) |
| N1 | pasa (0.125) | pasa (0.095) | pasa (0.065) | pasa (0.045) |
| **N2** | pasa (0.065) | pasa (0.035) | pasa (**0.005**) | **no pasa** (−0.015) |

- Zonas donde A cabe y B no (con padding): 4.58 m². Donde A gira en el sitio y B no: 2.78 m². No existe ninguna zona donde B quepa y A no.
- **Corredor crítico: N2.**
  - B sin padding "pasa" con 5 mm por lado, un margen menor que la resolución del mapa (3 cm) y que cualquier error de localización: **no es concluyente**.
  - B con padding no pasa.
- Autocomprobación del C-space frente a la clearance directa: 100 % de acuerdo (440 muestras fuera de la banda de cuantización).

## I.3 Nav2 planner REAL (NavFn), offline [C]
- **Launch auditado**, `config/offline_lab/planner_offline.launch.py`. Aborta si no se cumple dominio 99 + localhost. Nodos en ejecución:
  - `map_server`, `planner_server` (con `global_costmap` interno), 2 TF estáticas y `lifecycle_manager_offline`.
  - Acciones: solo `/compute_path_to_pose` y `/compute_path_through_poses`.
  - **Topics o acciones de movimiento: ninguno.** No hay controller, bt_navigator, behavior_server, AMCL ni `/cmd_vel`.
- Configuración: copia de `navigation2_real.yaml` (**sin modificarlo**). Únicos cambios:
  - (a) quitar `obstacle_layer`/`voxel_layer`, que requieren sensores;
  - (b) footprint y padding de A en la copia A.
  - `planner_server` (NavFn, `allow_unknown: false`, tolerance 0.2), inflación (0.35 / 2.0) y resolución (0.03) quedan idénticos.
- **Hallazgo: el global costmap convierte UNKNOWN en LIBRE** (ver I.5).

## I.4 Benchmark (64 pares × 2 configuraciones de planner = 128 planificaciones) [C]
Categorías: 10 cortas, 10 largas, 10 alrededor de obstáculos, 8 WIDE, 8 N1, 8 N2 y 10 aleatorias. Orientaciones variadas y seed 42. Start y goal válidos para A y B con padding (clearance > 3 cm).

| | Planner con costmap A | Planner con costmap B |
|---|---|---|
| Éxito | **64/64 (100 %)** | **64/64 (100 %)** |
| planning_time mediana / p95 / máx | 3.7 / 9.4 / 12.5 ms | 4.4 / 9.4 / 10.1 ms |
| Error final al goal | 0.0 m | 0.0 m |
| Clearance mínima del centro | 0.125 m | 0.205 m |
| **Footprint A** (swept) | 55 PASS · 9 MARGINAL · 0 COLLISION | 63 PASS · 1 MARGINAL · 0 COLLISION |
| **Footprint B** (swept) | 46 PASS · 10 MARGINAL · **8 COLLISION** | 54 PASS · 10 MARGINAL · 0 COLLISION |
| Rutas N2 | cruzan **N2** (L mediana 2.24 m) | rodean por **WIDE** (L mediana 13.67 m) |
| Rutas N1 | cruzan N1: A PASS, B MARGINAL (3.5–3.6 cm) | cruzan N1: A PASS, B MARGINAL (4.5 cm) |

Definición de clases (polígono sin padding; obstáculo = ocupado ∪ unknown):
- COLLISION: ≤ 0 m.
- MARGINAL: < padding + 0.05 m.
- PASS: el resto.

El 0.05 m es una reserva de trabajo para localización y tracking, **no medida**. Swept = giro en el sitio en el start → ruta con rumbo tangente → giro en el sitio en el goal [H: modelo de ejecución supuesto; el controller real puede no girar en el sitio].

Lecturas:
1. **Planificar con A y ejecutar con B → 8/8 colisiones en N2** [C sobre el sintético]. Con el costmap A, N2 es transitable y el planner lo usa; el footprint B no cabe (−0.005 m).
2. Con el costmap B, el planner **nunca** entra en N2 [C]:
   - el eje central de N2 vale 99 (inscrito) en el costmap B;
   - el rodeo por WIDE multiplica ×6 la longitud.
   - Que prefiera WIDE a N1 se explica por el coste de inflación en N1 (86/100 en el eje), comparable al rodeo [E].
3. Todas las rutas por N1 son **MARGINAL para B** (4.5 cm de clearance con el costmap B) [C].
4. **Las rutas de NavFn no van centradas** [C]:
   - desvío lateral sistemático de −1.5 cm, media celda;
   - hasta −3.8 cm con rumbos de ±4.5° en la entrada del corredor;
   - en N2, con el costmap A, el margen de A baja de 6.5 cm (centrado) a **1.5–2.5 cm**.
5. Giros en el sitio cerca de paredes (T060, T010) reducen la clearance de B a 0.5–1.5 cm aunque el start y el goal eran válidos. Es el swept de la rotación, no de la traslación [C sobre el modelo supuesto].

## I.5 UNKNOWN en el global costmap (experimento controlado) [C]
El mismo `/map` (34 848 celdas −1, verificado en el topic) se probó con dos configuraciones:

| Config | `track_unknown_space` | Costmap: celdas −1 | Valor de UNKNOWN exterior |
|---|---|---|---|
| B (fiel a `navigation2_real.yaml`) | sin definir (default false) | 0 | **0 = LIBRE** |
| B_trackunknown | true | 0 | **100 = LETAL** (inflado por `inflate_around_unknown: true`) [E sobre el mecanismo] |

→ Con la configuración actual, **aunque el yaml del mapa sea correcto, el global costmap trata lo desconocido como libre**. El `allow_unknown: false` de NavFn no tiene nada que filtrar. En el sintético no afectó a las rutas porque el perímetro está cerrado.

---

# PARTE II — CONCLUSIONES APLICABLES AL ROBOT REAL
> Solo lo que no depende de la geometría sintética.

| # | Conclusión | Nivel | Aplica al robot porque… |
|---|---|---|---|
| R1 | Todos los mapas existentes (9 yaml) cargan UNKNOWN como FREE. Causa: default de `map_saver_cli` (`free_thresh: 0.25`) | **C** | Es comportamiento de Nav2 Humble sobre los archivos reales |
| R2 | `navigation2_real.yaml` **no define `track_unknown_space`**: el global costmap convierte UNKNOWN en libre **incluso con un yaml corregido** | **C** en la copia offline (static + inflation) · **E** para la configuración completa con `obstacle_layer` (mismo parámetro, no probado) | Los parámetros son idénticos a la copia |
| R3 | `amcl_params.yaml` (fabricante) con `map_server.free_thresh: 0.196` no tiene efecto: el parámetro no existe | **C** | Lista de parámetros del nodo real |
| R4 | El `--free` de `map_saver_cli` cambia el contenido del PGM, no solo el yaml | **C** | Experimento con el guardador real |
| R5 | Si el costmap se configura con un footprint más pequeño que el real, el planner generará rutas por pasos donde el robot real no cabe | **C** como mecanismo; **no se mide cuál footprint es el correcto** | Lógica del costmap/planner, independiente del mapa |
| R6 | NavFn produce rutas desplazadas ~1.5 cm (media celda) y no centradas en pasos angostos. Con resolución 0.03 no hay que contar con margen centrado | **C** sobre el sintético · **E** para mapas reales | Propiedad del algoritmo y la resolución |
| R7 | Con B + padding 0.02, un paso < ~0.40 m queda bloqueado (inscrito) o con clearance < 5 cm | **E** | Geometría del footprint B; la arena real no se conoce |
| R8 | El tiempo de planificación NavFn (< 13 ms en ~11.6 × 11.6 m a 0.03) no es un cuello de botella | **E** | CPU real del robot, pero mapa sintético |

**No se puede concluir** sobre la arena real: no existe un mapa válido de ella; v1 y run3 no existen; stage3 solo tiene metadata. Tampoco sobre qué footprint es el físico, el error real de localización y tracking, el comportamiento del controller ni rutas reales.

---

## III. Anomalías importantes
1. **Doble conversión UNKNOWN → FREE**: en la carga del mapa (R1) y en el global costmap (R2). Cualquiera de las dos basta para que el planner vea libre lo no observado.
2. Yaml antiguos apuntando a un `map.pgm` compartido con resoluciones 0.05 y 0.03 (escala ×1.67), y dos yaml con indicios de edición manual.
3. B raw en N2: margen de 5 mm, no concluyente a esta resolución.
4. Rutas de NavFn no centradas (−1.5 a −3.8 cm).
5. Planner con footprint A ejecutado por un robot B: colisiones.

## IV. Próximo paso recomendado
1. **Decisión del equipo** (no aplicada): en una copia de la config de navegación real dentro de `~/limo_sml_ws/config/`, añadir `track_unknown_space: true` al global costmap y cargar solo mapas con `free_thresh ≤ 0.196`, validados con `validate_ros_map.py`.
   - Validar el cambio offline con este mismo banco: `run_planner_benchmark.sh` sobre una variante.
2. Medir físicamente el footprint (A vs B), una tarea de medición sin movimiento, y cerrar la contradicción antes de cualquier prueba de navegación.
3. Nuevo mapeo de arena con `save_map_checked.sh`, con autorización de movimiento.
4. R3 del laboratorio: el mismo benchmark con `SmacPlanner2D`/`Hybrid` o con `use_astar`, y una evaluación de los controllers en simulación cinemática, sin hardware.
