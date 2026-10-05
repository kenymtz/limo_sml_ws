# MAP_THRESHOLD_PROVENANCE — origen de `free_thresh: 0.25`

Fecha: 2026-09-29 · NAV-OFFLINE-R2 · FASE 1 · offline.

Experimento:
- Script: `scripts/nav_offline_lab/provenance_map_saver_experiment.sh`, aislado (`ROS_LOCALHOST_ONLY=1`, `ROS_DOMAIN_ID=99`). Solo usa nodos de mapa y un publisher de OccupancyGrid sintético.
- Salidas: `provenance/` (`prov_*.yaml/pgm`, `provenance_experiment.json`, `map_server_declared_params.txt`).

Leyenda: **CONFIRMADO** (medido o leído) · **EVIDENCIA** (indicio consistente) · **NO DETERMINADO**

---

## 1. Conclusión

| Pregunta | Respuesta | Nivel |
|---|---|---|
| ¿Quién escribe `free_thresh: 0.25`? | `nav2_map_server/map_saver_cli` sin `--free` escribe `occupied_thresh: 0.65` y `free_thresh: 0.25` | **CONFIRMADO** (experimento) |
| ¿Carga Nav2 bien su propio mapa con esos valores? | **No.** Un mapa guardado y recargado con defaults convierte el desconocido (205) en libre | **CONFIRMADO** (R1 + este experimento) |
| ¿Qué se usó para arena_20260918_1900? | `map_saver_cli -f <dir>/map`, sin umbrales | **CONFIRMADO** (comando en su README) |
| ¿Qué se usó para los mapas de `~/limoatwork`? | Un guardador de Nav2 con defaults: formato de yaml y PGM idéntico byte a byte a la salida de map_saver_cli. No se puede saber si fue el CLI, el servidor `map_saver` (param `free_thresh_default: 0.25` en `navigation2.yaml` del fabricante) o slam_toolbox (`use_map_saver: true`, que delega en el mismo guardador) | **EVIDENCIA**. El comando exacto: **NO DETERMINADO** (no hay rastro en `~/.bash_history`, 2 000 líneas) |
| ¿El `free_thresh: 0.196` de `limoatwork/.../amcl_params.yaml` lo corrige? | **No.** `map_server` solo declara `yaml_filename`, `topic_name`, `frame_id` y `use_sim_time`; el parámetro se ignora. Los umbrales se leen **solo** del yaml del mapa | **CONFIRMADO** (lista de parámetros del nodo real) |
| ¿El paquete `limo_map_saver` fija umbrales? | No. Lanza `map_saver_cli -f /home/solverbot/mapas/...`, una ruta de otra máquina | **CONFIRMADO** (lectura del script). Que se usara aquí: **NO DETERMINADO** |

**Causa raíz: CONFIRMED.** El valor 0.25 es el default de `map_saver_cli` en Nav2 Humble, y es inconsistente con la clasificación de carga de `map_server` para el píxel 205. Queda **parcial** solo el comando exacto usado en cada mapa antiguo de `~/limoatwork`.

## 2. Qué hace el guardador (experimento) [CONFIRMADO]
Entrada: un OccupancyGrid con todas las celdas −1, 0, 1, …, 100.

| Guardado con | yaml escrito | Celdas → 254 (libre) | Celdas → 205 (desconocido) | Celdas → 0 (ocupado) |
|---|---|---|---|---|
| defaults | `free_thresh: 0.25` | 0..25 | −1 y 26..64 | 65..100 |
| `--free 0.196` | `free_thresh: 0.196` | 0..20 | −1 y 21..64 | 65..100 |
| `--free 0.10` | `free_thresh: 0.1` | 0..10 | −1 y 11..64 | 65..100 |

- **El `--free` del guardado no solo cambia el yaml: cambia el contenido del PGM.** Con 0.196, las celdas de probabilidad 21–25 se guardan como desconocidas.
- Por eso el **umbral de guardado** (qué probabilidad cuenta como libre) y el **umbral de carga** (cómo se lee un PGM trinary) son decisiones distintas. El pipeline futuro las separa (ver `FUTURE_MAP_SAVE_PROTOCOL.md`).
- La salida trinary contiene **solo 3 niveles**: 0, 205 y 254. Nunca 255 ni grises intermedios.

## 3. Mapas existentes [CONFIRMADO salvo indicación]
Distribución real de píxeles (de `existing_maps_audit.json`): **todos** los PGM contienen únicamente {0, 205, 254}.

| yaml | Formato vs map_saver | Observación |
|---|---|---|
| arena_20260918_1900/map.yaml | idéntico | comando documentado en el README |
| limoatwork/mimap.yaml | idéntico | — |
| maps/map1.yaml, maps/map_20260718_185606.yaml | idéntico | `image: map.pgm` aunque existen `map1.pgm` y `map_20260718_185606.pgm`. **EVIDENCIA** de un guardado con `-f map` seguido de renombrado del yaml sin actualizar `image` |
| maps/map_20260720_160000.yaml | **editado** (termina en `\n`; map_saver no escribe salto final) | `image: map.pgm` · **EVIDENCIA** de edición manual |
| maps/map.yaml | **editado** (termina en `\n`; mtime 07-24 15:17, posterior al pgm 07-23 20:13) | res 0.03 sobre el mismo `map.pgm` que otros yaml leen a 0.05. Cuál es la resolución correcta: **NO DETERMINADO**. `map.pgm` coincide en fecha con `map_20260723_201330.pbstream` (EVIDENCIA) |
| maps/map_20260723_193027.yaml, maps/map_20260723_195338.yaml | idéntico | image coherente |
| maps/.yaml | idéntico | nombre vacío: **EVIDENCIA** de `-f` vacío |

## 4. Consecuencias de cada `free_thresh` de CARGA (trinary, negate 0, occupied_thresh 0.65)
Nav2 calcula occ = (255 − px)/255:
- **ocupado** si occ > 0.65, o sea px ≤ 89;
- **libre** si occ < free_thresh;
- **desconocido** en el resto.

| Nivel de píxel | occ | free 0.25 | free 0.196 | free 0.10 |
|---|---|---|---|---|
| 0 (ocupado guardado) | 1.000 | OCUPADO | OCUPADO | OCUPADO |
| 1..89 | > 0.65 | OCUPADO | OCUPADO | OCUPADO |
| 90..191 | 0.25..0.647 | DESCONOCIDO | DESCONOCIDO | DESCONOCIDO |
| 192..205 | 0.196..0.247 | **LIBRE** | DESCONOCIDO | DESCONOCIDO |
| **205** (desconocido guardado) | 0.19608 | **LIBRE ✗** | DESCONOCIDO ✓ (margen 7.8e-5) | DESCONOCIDO ✓ (margen 0.096) |
| 206..229 | 0.102..0.192 | LIBRE | LIBRE | DESCONOCIDO |
| 230..253 | 0.008..0.098 | LIBRE | LIBRE | LIBRE |
| **254** (libre guardado) | 0.0039 | LIBRE | LIBRE | LIBRE |
| 255 | 0 | LIBRE | LIBRE | LIBRE |

Lectura:
- **0.25 es incorrecto para cualquier PGM trinary de map_saver**, porque el 205 cae en libre.
- **0.196 y 0.10 dan resultados idénticos para los PGM existentes**, que solo tienen {0, 205, 254}. Solo difieren si aparecieran grises 206–229, que map_saver trinary no produce. Podrían aparecer en PGM editados a mano, reescalados o interpolados (por ejemplo, redimensionar con un filtro no-nearest).
- 0.196 depende de un margen de 7.8e-5 en coma flotante. Funcionó en el map_server real (R1), pero es frágil ante cualquier implementación que redondee distinto.
- 0.10 tiene un margen amplio respecto a 205 y a 254. Clasificaría como desconocido cualquier gris 206–229 de un PGM editado; es la opción conservadora.
- **No se fija un valor único para todos los mapas.**
  - Recomendación: carga con **0.10** para PGM trinary validados (solo {0, 205, 254}).
  - Cualquier PGM con otros niveles debe rechazarse o revisarse a mano. El validador lo reporta.
  - Esto no es una decisión cerrada: la decisión final queda al equipo.
