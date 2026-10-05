# UNKNOWN_THRESHOLD_COMPARISON — free_thresh 0.25 vs 0.196

Fecha: 2026-09-29. Modo: offline. Continúa `NAVIGATION_OFFLINE_LAB_R1` (ver `DATASET_REPORT.md` §1.3).
El dataset arena_20260918_1900 **no** es un baseline de navegación. Aquí solo se estudia la **interpretación del desconocido**. No se hizo ningún benchmark de Nav2.

Leyenda: **[C]** CONFIRMADO · **[E]** EVIDENCIA · **[H]** HIPÓTESIS

Archivos:
- `maps_test/map_test_unknown.yaml`: copia derivada. Solo cambian `free_thresh` e `image` (ruta absoluta al PGM original, que **no se copió ni se modificó**).
- `maps_test/map_server_original.json`, `maps_test/map_server_test_unknown.json`: salida del nav2_map_server real.
- `existing_maps_audit.json`: auditoría de todos los yaml de mapa en disco.
- Scripts: `scripts/nav_offline_lab/verify_map_server_load.sh` (map_server aislado + auditoría del grafo) y `scripts/nav_offline_lab/audit_map_yamls.py` (histograma y réplica de la clasificación trinary).

Originales: sha256 de `map.yaml` y `map.pgm` re-verificados **OK** después de la prueba.

---

## 1. Diff de la copia derivada [C]
```
< image: map.pgm
> image: /home/limopro/limo_sml_ws/results/maps/arena_20260918_1900/map.pgm
< free_thresh: 0.25
> free_thresh: 0.196
```
El resto de campos es idéntico: `mode: trinary`, `resolution: 0.03`, `origin [-3.32,-4.88,0]`, `negate: 0`, `occupied_thresh: 0.65`.

## 2. Resultado con nav2_map_server real [C]
382 × 336 = 128 352 celdas. OccupancyGrid recibido del `nav2_map_server` de Humble.

| Valor OccupancyGrid | `map.yaml` original (free 0.25) | `map_test_unknown.yaml` (free 0.196) | Diferencia |
|---|---|---|---|
| Libre (0) | **126 444** (98.51 %) | **34 676** (27.02 %) | −91 768 |
| Ocupado (100) | 1 908 (1.49 %) | 1 908 (1.49 %) | 0 |
| Desconocido (−1) | **0** (0 %) | **91 768** (71.50 %) | +91 768 |
| Otros valores | 0 | 0 | 0 |

- Las 91 768 celdas que cambian son exactamente los píxeles PGM = 205 (el desconocido que escribió `map_saver`). Pasan de libre a desconocido. **Lo ocupado no cambia.**
- Con 0.196 la carga coincide celda a celda con la semántica del PGM: 254 → libre, 205 → desconocido, 0 → ocupado.

### Por qué funciona y con qué margen
- nav2 calcula occ = (255 − px)/255. Para px 205, occ = 50/255 = **0.196078**.
- Con free_thresh 0.25, 0.196 < 0.25, así que la celda sale **libre** (bug de configuración).
- Con free_thresh 0.196, 0.196078 **no** es < 0.196, así que sale **desconocida**. El margen es de solo 7.8e-5.
- [E] Cualquier free_thresh en (0.0039, 0.196] da el mismo resultado para PGM trinary con valores {0, 205, 254}; 254 da occ 0.0039. Todos los PGM auditados contienen **solo** esos tres valores.
- [H] Un valor intermedio como 0.1 sería más robusto al redondeo que 0.196. 0.196 es la convención de ROS1 `map_saver`. La decisión queda para quien apruebe el cambio.

## 3. Auditoría del grafo ROS durante la prueba [C]
Condiciones:
- `ROS_LOCALHOST_ONLY=1`, `ROS_DOMAIN_ID=99`.
- Sin hardware y sin ningún otro stack.
- Se lanzó solo `/opt/ros/humble/lib/nav2_map_server/map_server`, renombrado a `offline_map_check`. Se configuró y activó vía el servicio lifecycle desde rclpy.

| Elemento | Resultado |
|---|---|
| Nodos en dominio 99 | `/offline_map_check` (map_server), `/offline_map_reader` (lector), `/_ros2cli_daemon_99_…` (daemon del CLI residual de un intento previo; detenido al terminar) |
| Publishers de map_server | `/offline_map_check/map` (OccupancyGrid), `/offline_map_check/transition_event`, `/bond` (heartbeat de lifecycle), `/parameter_events`, `/rosout` |
| Subscribers de map_server | `/bond`, `/parameter_events` |
| Servicios de map_server | lifecycle + parámetros + `load_map` + `map` (GetMap) |
| Acciones | **ninguna** |
| Topics de movimiento (`cmd_vel`, `joint`, `trajectory`, `controller`, `gripper`, `pump`, `twist`) | **ninguno** |

- Conclusión [C]: map_server **solo publicó el mapa**, más el tráfico estándar de lifecycle, bond y logging. No hubo riesgo de movimiento.
- Al finalizar no quedan procesos `map_server` ni `ros2-daemon` del dominio 99 (verificado con `ps`).

## 4. Revisión de los mapas existentes [C]

### 4.1 v1, run3, stage3
| Mapa | Resultado de la búsqueda |
|---|---|
| **v1** | **No existe ningún archivo.** Búsqueda en todo el filesystem, sin build/install/log ni paquetes del sistema. Coincide con `CONTRADICCIONES_PENDIENTES.md` |
| **run3** | **No existe ningún archivo.** Mismo resultado |
| **stage3** | Solo metadata: `results/baseline/stage3_cartographer_static/map_metadata.txt` (0.03 m, 220 × 322 px = 6.6 × 9.66 m, origin [-1.32, -3.96]). **No hay .yaml ni .pgm ni .pbstream**, así que su threshold **no se puede auditar** |

### 4.2 Todos los yaml de mapa en disco (solo lectura; 8 en `~/limoatwork`, que es inmutable y no se tocó)

| yaml | PGM que carga | res | px | free_thresh | Desconocido en PGM (205) | Desconocido tras carga Nav2 | Desconocido perdido |
|---|---|---|---|---|---|---|---|
| `limo_sml_ws/results/maps/arena_20260918_1900/map.yaml` | map.pgm | 0.03 | 382×336 | 0.25 | 91 768 | 0 | **71.5 %** del mapa |
| `limoatwork/mimap.yaml` | mimap.pgm | 0.05 | 118×190 | 0.25 | 14 775 | 0 | **65.9 %** |
| `limo_bringup/maps/map.yaml` | map.pgm | 0.03 | 129×151 | 0.25 | 6 761 | 0 | **34.7 %** |
| `limo_bringup/maps/map1.yaml` | **map.pgm** (≠ map1.pgm) | 0.05 | 129×151 | 0.25 | 6 761 | 0 | **34.7 %** |
| `limo_bringup/maps/map_20260718_185606.yaml` | **map.pgm** (≠ su pgm) | 0.05 | 129×151 | 0.25 | 6 761 | 0 | **34.7 %** |
| `limo_bringup/maps/map_20260720_160000.yaml` | **map.pgm** (≠ su pgm) | 0.05 | 129×151 | 0.25 | 6 761 | 0 | **34.7 %** |
| `limo_bringup/maps/map_20260723_193027.yaml` | map_20260723_193027.pgm | 0.05 | 86×95 | 0.25 | 3 287 | 0 | **40.2 %** |
| `limo_bringup/maps/map_20260723_195338.yaml` | map_20260723_195338.pgm | 0.05 | 85×95 | 0.25 | 3 295 | 0 | **40.8 %** |
| `limo_bringup/maps/.yaml` (nombre vacío) | .pgm | 0.05 | 103×107 | 0.25 | 4 552 | 0 | **41.3 %** |

- **Los 9 yaml existentes tienen `free_thresh: 0.25` y el mismo problema: el 100 % de su desconocido se carga como libre.** [C]
  - Las columnas de carga en Nav2 salen de la réplica numérica de `audit_map_yamls.py`.
  - La réplica se validó contra el map_server real en el mapa arena: coincidencia exacta en las dos configuraciones.
- Con free_thresh 0.196, la réplica recupera en todos los casos exactamente el desconocido del PGM [C, por réplica].

### 4.3 Hallazgo adicional: yaml que apuntan al PGM equivocado [C]
- `map1.yaml`, `map_20260718_185606.yaml` y `map_20260720_160000.yaml` declaran `image: map.pgm`, pero cada uno tiene su propio PGM en el directorio: `map1.pgm` 94×104, `map_20260718_185606.pgm` 101×105 y `map_20260720_160000.pgm` 88×96.
  - Los tres cargan en realidad el **mismo** `map.pgm` (129×151, mtime 2026-07-23 20:13, sha256 `cc5c8a0e7d5f…`).
- Ese `map.pgm` lo usa también `map.yaml` con **res 0.03**, mientras que los otros tres lo cargan con **res 0.05**.
  - El mismo dibujo queda en 3.87 × 4.53 m o en 6.45 × 7.55 m: un **error de escala de ×1.67** según qué yaml se use.
- [H] `map.pgm` coincide en fecha con `map_20260723_201330.pbstream`, que no tiene yaml propio. Probablemente es ese mapa, sobrescrito por sucesivos `map_saver -f map`, y los yaml viejos quedaron apuntando al nombre reutilizado.
- `.yaml` / `.pgm` con nombre vacío [E]: probablemente un `map_saver` lanzado sin `-f`.

## 5. ¿Puede afectar a la navegación futura?

**Sí [C/E].** Todos los mapas guardados hasta hoy tienen el problema, y el launch por defecto lo arrastra:
- `limoatwork/.../launch/navigation2.launch.py` usa por defecto `limo_bringup/maps/map.yaml`, que tiene free_thresh 0.25 y además la ambigüedad de escala de §4.3 [C, leído del src].
- `limo_localization.launch.py` usa por defecto `maps/limo.yaml`, que **no existe** en src [C]. Obliga a pasar `map:=` explícito.

Efectos esperados si se navega con un yaml de free_thresh 0.25:
| Componente | Efecto | Nivel |
|---|---|---|
| Static layer / global costmap | No hay celdas desconocidas: el espacio nunca observado es costo 0. `track_unknown_space` y `unknown_cost_value` quedan sin efecto | [C] (mapa cargado sin −1) · efecto en costmap [E] |
| Planner (`allow_unknown`) | El parámetro no tiene nada que filtrar: el planner puede trazar rutas por zonas nunca vistas, sobre todo donde no se mapeó pared (en arena_20260918 el 85 % del borde del libre era desconocido) | [E] |
| AMCL | Las partículas aleatorias o de reinicialización se generan sobre celdas "libres", incluidas las no observadas. El likelihood field no cambia (solo depende de lo ocupado) | [H] (según el diseño de nav2_amcl; no probado) |
| Análisis offline (clearance, rutas, footprint) | Sobreestima el espacio transitable. En arena_20260918: 113.8 m² libres con 0.25 contra 31.2 m² con 0.196 | [C] |
| Seguridad en marcha | El LiDAR y el local costmap detectarían obstáculos reales, pero la ruta global puede ser irrealizable y provocar replanificaciones o bloqueos | [H] |

Mitigación (no aplicada fuera de `maps_test/`; requiere decisión):
1. Crear en `~/limo_sml_ws` copias derivadas de los yaml a usar, con free_thresh ≤ 0.196 y `image` explícito y correcto. Verificar cada una con `verify_map_server_load.sh`.
2. Guardar los mapas nuevos con `map_saver_cli --free 0.196` o corregir el yaml tras guardar. Nunca con `-f map` repetido.
3. Pasar siempre `map:=<yaml verificado>` a los launch. No depender de defaults de `limoatwork`.
