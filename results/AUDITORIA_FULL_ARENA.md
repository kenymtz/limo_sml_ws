# Auditoría del mapa candidato "full arena" — LIMO RoboCup SML

Fecha del informe: 2026-09-17
Fuente única: `results/baseline/stage3_cartographer_static/` (todos sus archivos: `map_metadata.txt`, `map_info.txt`, `map_to_base.txt`, `nodes.txt`, `scan_filtered_info.txt`, `scan_filtered_hz.txt`, `cmd_vel_info.txt`)
Método: solo lectura de archivos de texto ya existentes. No se ejecutó ROS, no se inició hardware, no se copió ningún mapa, no se modificó ningún archivo.

---

## 1. Ubicación exacta del mapa encontrado

El dato **no corresponde a un archivo de mapa persistido** (no hay `.pgm`/`.pbstream` en disco para esta sesión). Es un `nav_msgs/msg/OccupancyGrid` capturado en vivo desde el topic `/map`, documentado como texto en:

```
~/limo_sml_ws/results/baseline/stage3_cartographer_static/map_metadata.txt
```

Contexto de la captura (según `nodes.txt` de la misma carpeta): la prueba tenía activos `/cartographer_node`, `/cartographer_scan_filter_chain`, `/occupancy_grid_node`, `/imu_link_broadcaster`, `/static_tf_pub_laser`, `/ydlidar_ros2_driver_node` y `/limo/limo_base`. **No hay nodo `amcl`** en esta lista — el mapa fue generado por Cartographer (SLAM en construcción), no leído de un `map_server` con un mapa ya guardado.

`map_info.txt` confirma: `Type: nav_msgs/msg/OccupancyGrid`, `Publisher count: 1`, `Subscription count: 0` — el topic `/map` tenía un único publisher (el `occupancy_grid_node` de Cartographer) y ningún subscriptor activo capturado en ese momento.

Timestamp de la captura: `map_load_time.sec = 1789496823` (misma sesión que `map_to_base.txt`, cuyo primer log es `[1789496823.770...]`) — coincide con la fecha de modificación de los archivos en disco: 2026-09-15, ~15:26-15:27, la misma sesión en que se calibró el LiDAR (`stage2_calibrated_lidar`, 15:16) y antes de `base_motion_r1` (17:07) y del cambio de footprint (21:03).

## 2. Resolución

```
resolution: 0.029999999329447746  →  0.03 m/px (redondeo de punto flotante estándar)
```

Coincide con la nota de CLAUDE.md: *"Algunos mapas de evaluación pueden utilizar: 0.03 m"*, y con la resolución del candidato `map.pgm` identificado en `AUDITORIA_MAPAS.md`.

## 3. Dimensiones físicas

```
width:  220 px
height: 322 px
```

→ **220 × 0.03 = 6.60 m** de ancho, **322 × 0.03 = 9.66 m** de alto → **área total ≈ 63.75 m²** (incluye celdas libres, ocupadas y desconocidas; no se puede separar el área "libre" real sin el archivo de imagen, que no existe en disco).

Es, por un margen amplio, el área más grande registrada en todo el proyecto: 2.4× más grande que el mayor mapa `.pgm` persistido encontrado (`map_20260718_185606.pgm`, 26.5 m², ver `AUDITORIA_MAPAS.md`) y 3.6× más grande que el candidato `map.pgm` de 0.03 m (17.5 m²).

## 4. Origen del mapa

```
origin.position: x = -1.32 m, y = -3.96 m, z = 0.0
orientation: identidad (0,0,0,1) — sin rotación
```

El origen del mapa está desplazado sustancialmente en Y (-3.96 m) respecto a todos los demás mapas encontrados (cuyos orígenes Y rondan entre -0.65 y -1.8 m, ver `AUDITORIA_MAPAS.md`) — consistente con un área de cobertura mucho más extendida en esa dirección, como cabría esperar de un mapa de mayor tamaño.

## 5. Relación posible con v1 / run3 / full arena

**No se puede establecer una correspondencia confirmada.** No existe ningún nombre de archivo, etiqueta o metadato en `stage3_cartographer_static/` que mencione "v1", "run3" o "full arena" — esos términos provienen únicamente de CLAUDE.md, sin vínculo textual directo a este dato.

Lo único que se puede afirmar con la evidencia disponible:
- Es, **por escala**, el candidato más consistente con la idea de "full arena" que se ha encontrado en todo el proyecto (ver comparación en la sección 6).
- No hay evidencia de que este `OccupancyGrid` en particular haya sido guardado como archivo (`map_saver`) bajo algún nombre — pudo haberse descartado al terminar la prueba, o guardado en una ubicación no localizada en las auditorías previas (`AUDITORIA_MAPAS.md`, búsqueda exhaustiva fuera de `limoatwork`/`limo_sml_ws`/`limo_bringup/maps`/papelera).
- No hay ningún indicio de que corresponda específicamente a "v1" (primera versión) o a "run3" (tercera corrida) — el nombre stage3_cartographer_static hace referencia a la etapa de prueba documentada en el propio proyecto (stage3 de una secuencia de baseline), no a una versión de mapa.

## 6. Comparación contra mapas anteriores

| Mapa | Área física | Resolución | Persistido en disco |
|---|---|---|---|
| `map_20260723_195338.pgm` | 20.2 m² | 0.05 m | sí |
| `map_20260723_193027.pgm` | 20.4 m² | 0.05 m | sí |
| `map_20260720_160000.pgm` (= `mi_mapa.pgm`) | 21.1 m² | 0.05 m | sí |
| `map1.pgm` | 24.4 m² | 0.05 m | sí |
| `map_20260718_185606.pgm` | 26.5 m² | 0.05 m | sí |
| `.pgm` oculto (sin nombre) | 27.6 m² | 0.05 m | sí |
| `map.pgm` (candidato principal, ver `AUDITORIA_MAPAS.md`) | 17.5 m² | 0.03 m | sí |
| **Este mapa (`stage3_cartographer_static`)** | **63.75 m²** | **0.03 m** | **no localizado** |

El salto de escala es claro: todos los mapas persistidos en disco están en el rango 17.5-27.6 m², mientras que este dato de metadata triplica al mayor de ellos. Comparte la resolución de 0.03 m únicamente con `map.pgm`, pero no hay relación de tamaño entre ambos que sugiera que uno sea un recorte o versión reducida del otro sin más evidencia.

## 7. ¿Sirve para análisis de footprint y clearance?

**No, no con lo disponible actualmente.** El script `footprint_clearance_analysis.py` (usado en auditorías previas) requiere un archivo de imagen `.pgm` con valores de píxel (libre/ocupado/desconocido) para calcular el *distance transform*. Aquí solo se cuenta con **metadata textual** (resolución, dimensiones, origen) — no con la imagen en sí. Sin el archivo `.pgm` (o el `.pbstream` de Cartographer del que se podría regenerar), **no es posible ejecutar el análisis de clearance sobre este mapa**.

## 8. Qué falta confirmar

1. **Si este `OccupancyGrid` fue guardado en algún momento con `map_saver`**, y en tal caso, dónde — no localizado en la búsqueda de `AUDITORIA_MAPAS.md` (que cubrió `limo_sml_ws`, `limoatwork`, home, bags, papelera y Downloads).
2. **Confirmación directa del equipo** sobre si esta sesión de Cartographer corresponde a lo que llaman internamente "run3" o "full arena", dado que por escala es el candidato más fuerte encontrado hasta ahora.
3. **Si existe un `.pbstream` de esta sesión específica** (Cartographer permite regenerar el `.pgm` desde un `.pbstream` guardado) en alguna ubicación no revisada aún.
4. **Relación (si la hay) entre este mapa y el candidato `map.pgm`** (mismo día, misma resolución 0.03 m, pero área muy distinta) — no se puede determinar sin más información si son la misma arena mapeada en distinto momento/alcance, o arenas distintas.
5. Sin la imagen del mapa, **no se puede confirmar si este mapa contiene información útil de clearance/corredores** para la arena real de RoboCup SML — solo se conoce su extensión, no su contenido (obstáculos, pasillos, forma).

---

*Informe generado exclusivamente a partir de los archivos de texto en `results/baseline/stage3_cartographer_static/`. No se inventó ninguna correspondencia con "v1"/"run3"/"full arena" — donde no hay evidencia directa, se indica explícitamente como no confirmado. No se ejecutó ningún comando ROS, launch, hardware, ni se copió o modificó ningún mapa.*
