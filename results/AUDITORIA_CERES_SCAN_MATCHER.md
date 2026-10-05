# Auditoría del Ceres scan matcher (Cartographer) — LIMO RoboCup SML

Fecha: 2026-09-17
Fuentes: `config/cartographer/limo_lds_2d_no_correlative.lua`, `config/cartographer/laser_filters_range_only.yaml`, `results/baseline/slam_ab_r1/RESULTS.txt`, y (solo lectura, referencia) `limoatwork/src/limo_ros2/limo_bringup/config_files/limo_lds_2d.lua` + `/opt/ros/humble/share/cartographer/configuration_files/trajectory_builder_2d.lua` (defaults del paquete Cartographer instalado del sistema).
Método: solo lectura y análisis estático. No se ejecutó ROS, no se inició Cartographer, no se modificó ningún archivo.

---

## 1. Configuración actual del Ceres scan matcher

El único archivo de configuración de Cartographer en el proyecto es:

```
config/cartographer/limo_lds_2d_no_correlative.lua
```

Este archivo **no sobreescribe explícitamente ningún parámetro de `ceres_scan_matcher`** (`occupied_space_weight`, `translation_weight`, `rotation_weight`). Esos valores se heredan sin modificar desde el `trajectory_builder_2d.lua` base que se incluye vía `include "trajectory_builder.lua"`. En el paquete Cartographer instalado del sistema (`/opt/ros/humble/share/cartographer/configuration_files/trajectory_builder_2d.lua`), esos valores por defecto son:

```lua
ceres_scan_matcher = {
  occupied_space_weight = 1.,
  translation_weight = 10.,
  rotation_weight = 40.,
  ...
}
```

**Esto confirma textualmente lo que ya señalaba `slam_ab_r1/RESULTS.txt`: "El Ceres scan matcher permanece activo incluso con online correlative scan matching desactivado"** — el proyecto nunca ha tocado sus pesos, sigue en los valores de fábrica de Cartographer, independientemente de qué se probó en las sesiones A/B.

## 2. Parámetros relevantes encontrados en `limo_lds_2d_no_correlative.lua`

| Parámetro | Valor | Notas |
|---|---|---|
| `use_online_correlative_scan_matching` | **`false`** | Ver hallazgo crítico en sección 3. |
| `min_range` / `max_range` | 0.1 / 8 m | Coherente con `laser_min_range`/`laser_max_range` de AMCL (0.12/8.0), pequeña discrepancia en el mínimo (0.1 vs 0.12) sin evidencia de que sea relevante. |
| `motion_filter.max_angle_radians` | `math.rad(0.1)` ≈ 0.0017 rad | Umbral muy bajo — casi cualquier cambio angular genera un nuevo nodo en el grafo de pose. |
| `submaps.grid_options_2d.resolution` | **0.03 m** | Coincide exactamente con la resolución del mapa candidato `map.pgm` identificado en `AUDITORIA_MAPAS.md` — refuerza que ese mapa fue generado con este archivo de configuración. |
| `hit_probability` / `miss_probability` | 0.57 / 0.495 | Valores por defecto de Cartographer, cercanos a 0.5 (bajo contraste hit/miss) — no calibrados para el proyecto. |
| `num_range_data` (por submapa) | 120 | Tamaño de submapa antes de cerrarlo e iniciar uno nuevo. |
| `constraint_builder.min_score` / `global_localization_min_score` | 0.65 / 0.65 | Umbral de aceptación de restricciones de loop-closure. |
| `optimize_every_n_nodes` | **comentado (`--`)** | Optimización periódica del pose graph deshabilitada explícitamente (queda en el valor por defecto del include, no en 30 como sugiere el comentario). |
| `insert_free_space` | `true` | Por defecto. |
| Pesos de `ceres_scan_matcher` | **no definidos aquí — heredados del default** (`occupied_space_weight=1`, `translation_weight=10`, `rotation_weight=40`) | Nunca ajustados para el LIMO. |

`laser_filters_range_only.yaml`: define un único filtro (`LaserScanRangeFilter`) que recorta el rango del scan a [0.12, 8.0] m, reemplazando valores fuera de rango por `NaN`. Es, casi con certeza, el filtro que produce `/scan_filtered` a partir del scan crudo del YDLiDAR — coherente con que AMCL y los costmaps usen `/scan_filtered` en vez del topic crudo.

## 3. Relación con los hallazgos de `slam_ab_r1/RESULTS.txt`

### Hallazgo crítico: el archivo de configuración guardado es la variante que el propio test recomendó **no adoptar**

- `slam_ab_r1/RESULTS.txt` documenta explícitamente una comparación A/B: **TEST A = online correlative scan matching ON**, **TEST B = online correlative scan matching OFF**. Conclusión textual: *"Desactivar online correlative scan matching redujo parcialmente la discrepancia lateral, pero empeoró la discrepancia longitudinal. No adoptar SLAM-B como configuración definitiva."* (repetida más adelante: *"No adoptar configuración B"*).
- El archivo `limo_lds_2d_no_correlative.lua`, que es **el único archivo de Cartographer que existe hoy en el proyecto**, tiene `use_online_correlative_scan_matching = false` — es decir, **es la configuración B, la que el test recomendó no adoptar**.
- Comparado contra la referencia de fábrica en `limoatwork/src/limo_ros2/limo_bringup/config_files/limo_lds_2d.lua` (idéntica en todos los demás parámetros), la única diferencia es exactamente esa línea: `use_online_correlative_scan_matching = true` en la referencia del fabricante vs `false` en la copia del proyecto.
- El nombre del archivo (`..._no_correlative.lua`) es autodescriptivo y coherente con su contenido — no es un error de nombre. El problema es que **es la única variante presente**, sin que exista en el repo (fuera de la referencia de fábrica en `limoatwork`, que es de solo lectura y no debe usarse como config activa del proyecto) una copia equivalente con correlative **ON**, que es la configuración que el propio equipo concluyó que debía mantenerse.
- Cronología: el archivo del proyecto está fechado 2026-09-15 17:32, **antes** de que se corriera el test `slam_ab_r1` (19:42) que descartó esta misma configuración. No hay evidencia de que, tras la conclusión del test, se haya creado o restaurado una variante con correlative ON dentro de `limo_sml_ws/config/cartographer/`.

### Relación con el sesgo lateral detectado

- El sesgo lateral sistemático (`MAP - ODOM` lateral +1.4 a +2.8 cm en todos los tests, A y B por igual) **persiste independientemente del estado de `use_online_correlative_scan_matching`**, según los propios datos de `slam_ab_r1`. Esto es consistente con que los pesos del `ceres_scan_matcher` (que sí operan en ambos casos, A y B) nunca fueron ajustados — el correlative scan matching es solo una etapa de búsqueda gruesa previa al refinamiento por Ceres; si el sesgo persiste con y sin esa etapa, la causa más probable señalada por el propio equipo (`RESULTS.txt`: "Se necesitan repeticiones y/o auditoría del Ceres scan matcher antes de modificar pesos") recae en el optimizador Ceres o en otra fuente (calibración del sensor, `motion_filter`, `hit/miss_probability`), no en el correlative matching en sí.
- Esta auditoría **no encontró ningún ajuste de `ceres_scan_matcher`** en el repo que pudiera explicar o corregir el sesgo — los pesos siguen en su valor de fábrica.

### Relación con las diferencias RAW vs `scan_filtered`

- `slam_ab_r1/RESULTS.txt` (tests C1/C2) comparó scan RAW vs filtrado con correlative **ON**, y encontró que RAW reduce la discrepancia media (0.895 cm vs 1.605 cm en avance) pero no la elimina — con solo 2 repeticiones, conclusión marcada como no definitiva en el propio resultado.
- El archivo `laser_filters_range_only.yaml` solo aplica un filtro de rango (recorte min/max), no hay evidencia en este archivo de otro tipo de filtrado (por ejemplo, filtrado angular o de ruido) que pudiera introducir distorsión adicional al comparar RAW vs filtrado — el filtro es simple y no debería, en principio, introducir sesgo lateral por sí mismo, pero esta auditoría no puede confirmarlo sin una prueba dedicada.

## 4. Parámetros que NO deben modificarse todavía

- **`occupied_space_weight`, `translation_weight`, `rotation_weight` (Ceres scan matcher)** — no ajustar sin antes tener una prueba repetida y controlada, tal como pide explícitamente `slam_ab_r1/RESULTS.txt`. Esta auditoría documenta que siguen en su valor de fábrica, pero no determina cuál sería el ajuste correcto.
- **`use_online_correlative_scan_matching`** — no cambiarlo unilateralmente a `true` sin decidir primero si se reemplaza el archivo completo o se crea una variante nueva; ya existe evidencia (`slam_ab_r1`) de que `true` (config A) tuvo mejor resultado que `false` (config B, la actual), pero el cambio de archivo de configuración activa es una decisión que debe quedar documentada explícitamente, no aplicada como corrección silenciosa.
- **`hit_probability` / `miss_probability`** — valores de fábrica, sin prueba de calibración específica para el LIMO/YDLiDAR.
- **`submaps.grid_options_2d.resolution` (0.03 m)** — cambiarlo afecta directamente la resolución del mapa resultante y rompe la relación ya identificada con el candidato `map.pgm`; no modificar sin coordinar con la resolución de decisión pendiente sobre mapas v1/run3.
- **`min_range` (0.1 m) vs `laser_min_range` de AMCL (0.12 m)** — pequeña discrepancia detectada, no modificar sin antes confirmar si es intencional o relevante.

## 5. Preguntas abiertas

1. **¿Por qué el archivo de configuración activo en el proyecto (`limo_lds_2d_no_correlative.lua`) es la variante que el propio test `slam_ab_r1` recomendó no adoptar?** ¿Fue un descuido al no revertir tras el test, o hay una razón no documentada para mantener correlative OFF a pesar de la conclusión del test?
2. **¿Existe en algún lugar no localizado una variante con `use_online_correlative_scan_matching = true` generada por el equipo** (distinta de la referencia de fábrica en `limoatwork`), usada durante los tests A/A2/C1/C2 de `slam_ab_r1`? Esta auditoría solo encontró la variante OFF en el proyecto.
3. **¿Se ha intentado alguna vez ajustar los pesos del `ceres_scan_matcher`** fuera de este repo (por ejemplo, en una sesión no documentada), o siguen realmente en el valor de fábrica desde el inicio del proyecto?
4. **¿La discrepancia `min_range` 0.1 (Cartographer) vs 0.12 (AMCL, laser_filters) es intencional?**
5. **¿Qué repeticiones adicionales de SLAM A/B/C se planean** para llegar a una conclusión estadísticamente más sólida, dado que varias comparaciones en `slam_ab_r1` se basaron en solo 1-2 repeticiones?

---

*Informe generado a partir de `config/cartographer/`, `results/baseline/slam_ab_r1/RESULTS.txt`, y consulta de solo lectura de `limoatwork` (archivo de referencia de fábrica) y del paquete Cartographer del sistema (`/opt/ros/humble/share/cartographer/`, valores por defecto documentales, no un archivo del proyecto). No se propusieron cambios, no se ejecutó ningún comando ROS, launch, ni Cartographer.*
