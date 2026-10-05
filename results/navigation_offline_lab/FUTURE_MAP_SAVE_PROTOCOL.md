# FUTURE_MAP_SAVE_PROTOCOL — guardado verificado de mapas (NAV-OFFLINE-R2)

Estado: **READY** (probado solo offline, con un publicador sintético en el dominio 99). **Nunca se ha ejecutado contra el robot.**

Script: `~/limo_sml_ws/scripts/nav_offline_lab/save_map_checked.sh`
Dependencias: `save_map_helper.py`, `validate_ros_map.py`, `verify_map_server_load.sh` (misma carpeta).
Test: `scripts/nav_offline_lab/tests/test_save_map_checked.sh`, resultado **10/10 PASS**. Salidas en `results/navigation_offline_lab/save_pipeline_test/`.

---

## 1. Por qué existe
- `map_saver_cli` sin opciones escribe `free_thresh: 0.25`. Con ese yaml, `map_server` carga el desconocido (205) como libre (CONFIRMADO; ver `MAP_THRESHOLD_PROVENANCE.md`).
- Guardados repetidos con `-f map` dejaron varios yaml apuntando a un `map.pgm` compartido, con resoluciones distintas (CONFIRMADO; ver `UNKNOWN_THRESHOLD_COMPARISON.md`).
- El `--free` del guardado también cambia el contenido del PGM. Por eso **el umbral de guardado y el de carga se tratan por separado**.

## 2. Qué hace el script (y qué NO)
| Paso | Acción |
|---|---|
| 0 | Valida el nombre (sin genéricos como `map`) y el destino: solo dentro de `~/limo_sml_ws/results`; rechaza `~/limoatwork` **sin crear nada**; nunca sobrescribe |
| 1 | Toma una instantánea del OccupancyGrid (solo suscripción) |
| 2 | Ejecuta `map_saver_cli -t <topic> -f <dir>/<name> --occ 0.65 --free 0.25 --fmt pgm --mode trinary` (umbrales de **guardado** explícitos, iguales a los históricos) |
| 3 | Toma una segunda instantánea |
| 4 | Renombra el yaml del saver a `<name>.saver_raw.yaml`, **sin tocarlo**, y genera `<name>.yaml` de **carga**, idéntico salvo `free_thresh: 0.10` |
| 5 | `validate_ros_map.py --strict` sobre el yaml de carga (y con `--expect-resolution` si se da) |
| 6 | Compara el PGM celda a celda con las instantáneas: −1 → 205, ≤ 25 → 254, ≥ 65 → 0; misma resolución, dimensiones y origin. **Falla si el mapa cambió durante el guardado** |
| 7 | (con `--verify-map-server`) Carga el yaml en un `nav2_map_server` aislado (`ROS_LOCALHOST_ONLY=1`, `ROS_DOMAIN_ID=99`) y exige conteos idénticos a los del validador |
| fin | Escribe `save_report.json` y `SHA256SUMS`; pone pgm y yaml en solo lectura |

**NO** publica nada fuera del paso 7 aislado. **NO** llama servicios de SLAM ni envía `/cmd_vel`. **NO** mueve el robot. El guardado de `.pbstream` queda fuera del script (ver §4).

Exit: `0` mapa verificado · `2` condición peligrosa (el mapa no debe usarse).

## 3. Procedimiento en una futura sesión física
Requiere autorización previa para la sesión de mapeo. El guardado en sí no mueve el robot.

1. **Terminar el recorrido y dejar el robot detenido.** El script no lo comprueba.
2. **Congelar el mapa antes de guardar.** Con Cartographer:
   `ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}"`.
   Si el grid sigue cambiando, el paso 6 fallará. Ese fallo es el comportamiento deseado.
3. En el mismo `ROS_DOMAIN_ID` de la sesión (por ejemplo 17):
   ```bash
   ~/limo_sml_ws/scripts/nav_offline_lab/save_map_checked.sh \
       --name arena_$(date +%Y%m%d_%H%M) --topic /map \
       --expect-resolution 0.03 --verify-map-server
   ```
4. Si sale `OK`, usar **siempre** `results/maps/<name>/<name>.yaml` con `map:=<ruta absoluta>`. No usar `<name>.saver_raw.yaml`: su validación da FAIL a propósito, porque documenta el problema de origen.
5. Guardar el pbstream por separado, en el mismo directorio:
   `ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '<dir>/<name>.pbstream', include_unfinished_submaps: true}"`.
6. Anotar en `<dir>/README.md` el recorrido, la duración y la configuración de SLAM. Grabar `/cmd_vel` desde el inicio del bag.

## 4. Fallos y qué hacer
| Mensaje | Causa probable | Acción |
|---|---|---|
| `sin OccupancyGrid en /map` | SLAM o map_server no activo, o `ROS_DOMAIN_ID` distinto | Verificar con `ros2 topic info /map` |
| `el PGM no corresponde a ninguna instantánea` | El mapa cambió durante el guardado | `finish_trajectory` y repetir con otro `--name` |
| `resolución X != esperada` | Configuración de SLAM distinta a la esperada | Revisar el .lua antes de usar el mapa |
| `píxeles 205 ... se cargan como FREE` | yaml de carga mal generado | No usar el mapa; revisar `--load-free` |
| `destino ya existe` | Nombre repetido | Usar otro nombre; nunca borrar mapas previos |
| `niveles de gris fuera de {0,205,254}` (WARN → exit 2 en strict) | PGM no trinary | Revisión manual |

## 5. Umbrales usados y por qué
- **Guardado 0.25 / 0.65**: mantiene el mismo criterio que todos los mapas históricos (celda ≤ 25 → libre, ≥ 65 → ocupada, resto desconocida).
  - Cambiarlo altera qué probabilidades se consideran libres. Es una decisión de SLAM, no de formato, y queda abierta.
- **Carga 0.10**: con PGM trinary {0, 205, 254}, da el mismo resultado que 0.196, pero con margen amplio (0.096 frente a 7.8e-5).
  - Cualquier gris 206–229 inesperado queda como desconocido (conservador).
  - El script rechaza `--load-free` > 0.196.

## 6. Limitaciones conocidas
- No comprueba que el robot esté detenido ni que el SLAM haya terminado. Solo detecta cambios del mapa durante el guardado.
- La comparación con instantáneas asume que el saver recibe el mismo mensaje que ellas. Si el SLAM publica a alta frecuencia, puede fallar aun con el robot quieto; en ese caso, congelar el mapa (§3.2).
- Probado solo con un publicador sintético en el dominio 99. **La primera ejecución real debe revisarse a mano.**
