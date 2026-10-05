# Auditoría de mapas — LIMO RoboCup SML

Fecha: 2026-09-17
Método: solo lectura e inspección de metadatos (nombre, tamaño de imagen, resolución, origen, fechas de modificación/borrado). No se ejecutó ROS, no se inició hardware, no se modificó, renombró ni copió ningún archivo.

Alcance de la búsqueda: `~/limo_sml_ws` (workspace del proyecto), `~/limoatwork` (solo lectura, workspace base del fabricante), `~/` (home), `~/bags/`, `~/.local/share/Trash/` (papelera), `~/Downloads/LIMOPRO/` (descargas del fabricante).

---

## 1. Mapas encontrados

### 1.1 `~/limo_sml_ws` (workspace del proyecto)

**No se encontró ningún mapa (`.pgm`/`.yaml`) dentro de `limo_sml_ws`.** El workspace de competición no contiene mapas propios guardados — solo `config/missions/test_arena.yaml`, que referencia un `map.name: test_map` sin archivo `.pgm`/`.yaml` asociado localizable.

### 1.2 `~/limoatwork/src/limo_ros2/limo_bringup/maps/` (solo lectura, directorio activo)

| Archivo | Tamaño (px) | Resolución | Origen | Área física | `.pbstream` asociado | Última modificación |
|---|---|---|---|---|---|---|
| `map1.pgm/.yaml` | 94×104 | 0.05 m | [-1.17, -1.02] | 4.70×5.20 m = 24.4 m² | no | 2026-07-20 16:00 |
| `map_20260718_185606.pgm/.yaml` | 101×105 | 0.05 m | [-1.11, -1.57] | 5.05×5.25 m = 26.5 m² | no | 2026-07-18 18:56 |
| `map_20260720_160000.pgm/.yaml` | 88×96 | 0.05 m | [-1.08, -0.848] | 4.40×4.80 m = 21.1 m² | no | 2026-07-20 16:00 |
| `map_20260723_193027.pgm/.yaml` | 86×95 | 0.05 m | [-1.04, -0.859] | 4.30×4.75 m = 20.4 m² | **sí** (3.65 MB) | 2026-07-23 19:30/31 |
| `map_20260723_195338.pgm/.yaml` | 85×95 | 0.05 m | [-1.05, -0.76] | 4.25×4.75 m = 20.2 m² | **sí** (2.03 MB) | 2026-07-23 19:53 |
| **`map.pgm`** / **`map.yaml`** | 129×151 | **0.03 m** | [-0.923, -0.645] | 3.87×4.53 m = 17.5 m² | **sí** (`map_20260723_201330.pbstream`, 1.58 MB) | `.pgm`/`.pbstream`: 2026-07-23 20:13 — **`.yaml`: 2026-07-24 15:17 (el más reciente de todo el directorio)** |
| `.pgm` / `.yaml` (nombre oculto, sin prefijo) | 103×107 | 0.05 m | [-1.17, -1.48] | 5.15×5.35 m = 27.6 m² | no | 2026-07-18 18:27 |

### 1.3 Papelera del sistema (`~/.local/share/Trash/`) — versiones históricas descartadas

Todas corresponden a sobrescrituras sucesivas del **mismo archivo** `limoatwork/.../limo_bringup/maps/map.pgm` / `map.yaml`, es decir, versiones anteriores de lo que hoy es `map.pgm`:

| Copia en papelera | Tamaño (px) | Resolución | Origen | Fecha de borrado (= fecha en que fue reemplazado) |
|---|---|---|---|---|
| `map.pgm/.yaml` | 98×106 | 0.05 m | [-1.01, -1.56] | 2026-03-05 20:50 |
| `map.2.pgm/.yaml` | 102×106 | 0.05 m | [-1.07, -1.53] | 2026-03-14 15:25 |
| `map.3.pgm/.yaml` | 107×115 | 0.05 m | [-1.36, -1.8] | 2026-03-21 20:28 |
| `map.4.pgm/.yaml` | 106×112 | 0.05 m | [-1.31, -1.74] | 2026-03-21 21:55 |

Esto confirma que `map.pgm` es un **nombre reutilizado iterativamente** desde marzo de 2026 (al menos 4 generaciones descartadas antes de la versión actual de julio, la única que pasó de 0.05 a 0.03 m de resolución).

### 1.4 Mapa Cartographer en vivo (no persistido como archivo)

`results/baseline/stage3_cartographer_static/map_metadata.txt` documenta un `OccupancyGrid` publicado en vivo por `/occupancy_grid_node` durante una prueba de Cartographer:

- **220×322 px, resolución 0.03 m/px → 6.6×9.66 m ≈ 63.7 m²**
- Origen: `[-1.32, -3.96]`

Esto es **significativamente más grande** que cualquier mapa `.pgm` guardado en disco (máximo encontrado: 27.6 m²). No se localizó ningún `.pgm`/`.pbstream` en el filesystem que corresponda a esta sesión — es un dato de metadata capturado en el momento, sin archivo de mapa persistido asociado disponible para inspección.

### 1.5 Copia suelta en el home (`~/mi_mapa.pgm` / `.yaml`)

- 88×96 px, 0.05 m, origen `[-1.08, -0.848]` — **idéntico en tamaño y origen a `map_20260720_160000.yaml`**. Es una copia de ese mapa, guardada fuera del árbol de `limoatwork` con nombre genérico, no un mapa nuevo.

### 1.6 Fuera de alcance — mapas de ejemplo del SDK del fabricante

`~/Downloads/LIMOPRO/` contiene múltiples `.pgm` (`map11`, `map1021`, `zhiyuan`, `limo`, `rtabmap`, `map01`-`map05`, `testmap`, mapas de test de `nav2_system_tests`, `turtlebot3_world`, etc.) dentro de copias descargadas de `agilex_ws`/`limo_ros2_ws` (repositorios de ejemplo/SDK). **No hay indicio de que estos correspondan a mapas generados por el equipo para este proyecto** — parecen mapas de ejemplo/demostración incluidos en las descargas originales del fabricante. Se excluyen de la clasificación como candidatos.

---

## 2. Candidatos posibles a "v1" y "run3"

No existe ningún archivo llamado literalmente `v1` ni `run3` en ninguna ubicación revisada. Los candidatos se infieren por heurística de nombre, fecha, resolución y evolución histórica:

| Candidato | Argumento a favor | Argumento en contra |
|---|---|---|
| **`map1.pgm`** → posible "v1" | El nombre `map1` es la coincidencia textual más cercana a "v1" encontrada en todo el sistema. | También podría ser simplemente "mapa número 1" de una serie sin relación con el apodo "v1 producción". No tiene `.pbstream` ni es el más antiguo (hay 4 generaciones previas en la papelera con el nombre `map.pgm`, más antiguas que `map1.pgm`). |
| **`map.pgm`/`map.yaml`** (resolución 0.03 m) → posible "v1 producción" o "run3" | Es el archivo modificado más recientemente (yaml del 2026-07-24, un día después de su `.pbstream`), tiene `.pbstream` propio, y su resolución (0.03 m) coincide exactamente con la nota de CLAUDE.md: *"Algunos mapas de evaluación pueden utilizar: 0.03 m"*. Es además la 5ª generación conocida de un archivo con ese nombre (histórico en papelera desde marzo), consistente con la idea de ser el resultado final de iteraciones sucesivas. | Su área (17.5 m²) es la **más pequeña** de todos los mapas activos — contraintuitivo si "run3" implica "full arena" (se esperaría el mapa más grande, no el más chico). |
| **`map_20260718_185606.pgm`** → posible "run3 full arena" | Es el mapa **más grande** de todos los persistidos en disco (26.5 m², sin contar el `.pgm` oculto). "Full arena" sugiere el de mayor cobertura. | No tiene `.pbstream` asociado, y su nombre (timestamp automático) no sugiere una selección deliberada como versión "final" o "de producción". |
| **`.pgm` oculto (sin nombre)** | Es el de mayor área física de todo el directorio (27.6 m²). | Nombre de archivo vacío/oculto sugiere un artefacto accidental (posible error al guardar sin especificar nombre), no una versión deliberadamente conservada. |
| Mapa Cartographer en vivo de `stage3_cartographer_static` (63.7 m²) | Es, por lejos, el área más grande registrada en todo el proyecto — el único candidato cuyo tamaño se acerca a lo que razonablemente sería una "full arena" de RoboCup SML. | **No existe como archivo persistido** — solo hay metadata de un topic en un momento dado. No se puede confirmar si fue guardado con `map_saver` bajo algún nombre, y de ser así, ese archivo no fue localizado. |

## 3. Cuál tiene evidencia más fuerte

**Ningún candidato tiene evidencia concluyente**, pero ordenados por fuerza de evidencia disponible:

1. **`map.pgm` / `map.yaml` (resolución 0.03 m, en `limoatwork/.../limo_bringup/maps/`)** — el más fuerte entre los archivos persistidos: es la versión final de una serie de al menos 5 generaciones del mismo nombre, tiene `.pbstream` propio, coincide con la resolución de evaluación documentada en CLAUDE.md, y es el archivo modificado más recientemente de todo el directorio. Es el mejor candidato a "mapa de producción actual", aunque su tamaño reducido no encaja bien con la idea de "full arena".
2. **El mapa Cartographer en vivo de 63.7 m² (`stage3_cartographer_static`)** tiene la evidencia más fuerte en términos de *escala* para ser "run3 full arena" (por mucho el más grande registrado), pero **no tiene archivo persistido localizable** — es evidencia de que ese mapeo ocurrió, no evidencia de dónde quedó guardado el resultado.
3. Los demás candidatos (`map1.pgm`, `map_20260718_185606.pgm`, el `.pgm` oculto) tienen solo coincidencias débiles (nombre parcial o tamaño relativo) sin ningún otro dato que los confirme.

## 4. Qué información falta confirmar

1. **Confirmación directa del equipo**: qué archivo(s) llaman internamente "v1" y "run3" — es la única forma de resolver esta ambigüedad con certeza, ya que no hay metadatos en disco que lo indiquen.
2. **Destino del mapa Cartographer de 63.7 m²** (`stage3_cartographer_static`): si se guardó con `map_saver` en algún momento, bajo qué nombre y en qué ruta — no localizado en esta auditoría.
3. **Relación entre las 5 generaciones de `map.pgm`** (papelera: marzo 2026 ×4, más la actual de julio) — no hay documentación de por qué se regeneró repetidamente ni qué cambió entre versiones (¿misma arena remapeada?, ¿arena distinta?).
4. **Identidad del archivo `.pgm`/`.yaml` sin nombre** (oculto, 2026-07-18 18:27) — no está claro si es un artefacto accidental o un mapa válido con nombre no asignado.
5. **Si algún mapa de `~/Downloads/LIMOPRO/` fue efectivamente usado por el equipo** en algún momento, o si son exclusivamente material de ejemplo del fabricante sin relación con las pruebas documentadas en `results/baseline/`.

---

*Informe generado por inspección de solo lectura de nombres de archivo, metadatos `.yaml`, tamaños de imagen `.pgm` (vía `file`) y registros de papelera (`.trashinfo`). No se abrió el contenido de píxeles de ningún mapa nuevo en esta auditoría (ya se hizo para `map.pgm` y `map_20260718_185606.pgm` en `results/footprint_analysis/`, ver auditorías previas). No se modificó, renombró ni copió ningún archivo.*
