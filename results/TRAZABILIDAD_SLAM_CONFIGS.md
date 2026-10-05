# Trazabilidad de configuraciones SLAM (Cartographer) — LIMO RoboCup SML

Fecha: 2026-09-17 (sesión de trazabilidad autónoma)
Método: solo lectura. Fuentes: `~/.bash_history`, `results/baseline/slam_ab_r1/RESULTS.txt`, `config/cartographer/`, lectura de código de `limoatwork/src/limo_ros2/limo_bringup/launch/cartographer.launch.py` (solo lectura). No se ejecutó ROS, no se inició Cartographer, no se modificó nada.

Convención de certeza usada en todo el documento: **CONFIRMADO** / **EVIDENCIA INDIRECTA** / **HIPÓTESIS** / **DESCONOCIDO**.

---

## 0. Advertencia sobre el alcance del historial disponible

`~/.bash_history` tiene `HISTSIZE=1000` / `HISTFILESIZE=2000` (`~/.bashrc`). El archivo actual contiene exactamente 2000 líneas y **comienza a mitad de un heredoc de Python** (no en el inicio de una sesión) — **CONFIRMADO**: el historial está truncado por rotación; todo comando anterior al inicio del archivo visible se perdió. No existe ninguna sesión previa de Claude Code registrada en `~/.claude/projects/` (solo existe el directorio de la sesión actual) — **CONFIRMADO**: esta es la única sesión de Claude Code registrada para el proyecto; el trabajo del 2026-09-15 (tests, `CLAUDE.md`, ediciones de yaml) fue realizado por el usuario directamente en shell (parcialmente capturado) y/o por herramientas no interactivas no registradas en bash_history.

**Consecuencia práctica**: se pudo reconstruir con alta confianza la secuencia de comandos de una sesión de pruebas SLAM (ver §1), pero no la de `stage3_cartographer_static` (15:26, anterior al inicio del historial visible) ni, con certeza, la de los tests C1/C2 de `slam_ab_r1` (ver §2).

## 1. Hallazgo estructural: el filtro de `/scan_filtered` está hardcodeado, no es parametrizable

**CONFIRMADO** (lectura de código, `limoatwork/src/limo_ros2/limo_bringup/launch/cartographer.launch.py`, líneas 51-55):

```python
laser_filters_yaml = os.path.join(
    limo_cartographer_prefix,   # = get_package_share_directory('limo_bringup')  → limoatwork
    'config_files',
    'laser_filters.yaml'        # ← ruta fija, NO es un LaunchConfiguration
)
```

Este `laser_filters_yaml` **no está expuesto como argumento de launch** (`DeclareLaunchArgument`). Es una ruta fija dentro del árbol de `limoatwork`. **Consecuencia CONFIRMADA**: cualquier invocación de `ros2 launch limo_bringup cartographer.launch.py`, con o sin overrides de `configuration_basename`/`cartographer_config_dir`, **siempre** arranca el filtro `limoatwork/.../config_files/laser_filters.yaml` (2 etapas: range + speckle) — **nunca** el `config/cartographer/laser_filters_range_only.yaml` del proyecto (1 etapa), a menos que se pase `use_scan_filter:=false` (lo que apaga el nodo de filtro por completo) y además se remapee `scan_topic` a algo distinto de `/scan_filtered`.

Esto reduce drásticamente la ambigüedad de auditorías previas sobre "qué filtro se usó" — con evidencia de código, no solo de nombres de archivo.

## 2. Reconstrucción de la sesión "SLAM ESTATICO R1" (evidencia en `~/.bash_history`, líneas ~1700-1757)

Bloque localizado, marcado explícitamente en el propio historial con comentarios `echo`:

| Línea | Comando | Interpretación |
|---|---|---|
| 1709 | `echo "=== INICIANDO SLAM ESTATICO R1 ==="` | Marcador de inicio de sesión |
| 1710 | `ros2 launch limo_bringup cartographer.launch.py use_sim_time:=false open_rviz:=false` | Launch **sin overrides** → `configuration_basename=limo_lds_2d.lua` (default), `cartographer_config_dir`=share de `limo_bringup` (limoatwork) → **correlative ON** (valor por defecto de ese `.lua`, ver `AUDITORIA_CARTOGRAPHER_COMPLETA.md`), filtro 2 etapas (§1). Seguido inmediatamente de verificación de nodos — patrón compatible con un arranque de verificación rápida, no necesariamente una captura de datos completa. |
| 1712-1715 | `ros2 node list`, `ros2 topic info /cmd_vel` | Chequeo "antes de levantar" |
| 1721 | `ros2 launch limo_bringup cartographer.launch.py use_sim_time:=false open_rviz:=false` | Igual que 1710: **sin overrides** → correlative ON, filtro 2 etapas. |
| 1727 | `echo "=== PRECHECK SLAM-B ==="` | Marcador explícito: lo que sigue es "SLAM-B" |
| 1728-1730 | `ros2 node list \| grep ...`, `ros2 topic info /cmd_vel` | Precheck antes de B |
| 1732 | `ros2 launch limo_bringup cartographer.launch.py use_sim_time:=false open_rviz:=false cartographer_config_dir:=/home/limopro/limo_sml_ws/config/cartographer configuration_basename:=limo_lds_2d_no_correlative.lua` | **Override explícito** → usa el `.lua` del proyecto (`no_correlative`, correlative **OFF**). El filtro sigue siendo el de 2 etapas del fabricante (§1, no hay override de `laser_filters_yaml`). |
| 1739 | `ros2 launch limo_bringup cartographer.launch.py use_sim_time:=false open_rviz:=false` | Sin overrides de nuevo → correlative ON (repetición, consistente con una segunda corrida "A2"). |
| 1741-1750 | Verificación de limpieza (procesos, nodos, `/cmd_vel`) | Chequeo post-test |
| 1756 | `ros2 launch limo_bringup cartographer.launch.py use_sim_time:=false open_rviz:=false cartographer_config_dir:=/home/limopro/limo_sml_ws/config/cartographer configuration_basename:=limo_lds_2d_no_correlative.lua` | Override de nuevo → correlative OFF (repetición, consistente con "B2"). |

Inmediatamente después de la línea 1756, el historial pasa a comandos no relacionados (`claude --version`, instalación de Claude Code, creación de `CLAUDE.md`) — **no hay ningún comando de Cartographer después de esto en el historial visible.**

### Mapeo propuesto a las secciones de `slam_ab_r1/RESULTS.txt`

**EVIDENCIA INDIRECTA** (correlación por orden y por los propios marcadores `echo` del usuario, no verificación exacta 1:1):

| Sección de `RESULTS.txt` | Comando candidato | Correlative | Filtro | Certeza |
|---|---|---|---|---|
| TEST A | línea 1721 (línea 1710 pudo ser un arranque de verificación previo, descartado) | ON | 2 etapas (fabricante) | EVIDENCIA INDIRECTA |
| TEST B | línea 1732 | OFF | 2 etapas (fabricante) | EVIDENCIA INDIRECTA |
| TEST A2 | línea 1739 | ON | 2 etapas (fabricante) | EVIDENCIA INDIRECTA |
| TEST B2 | línea 1756 | OFF | 2 etapas (fabricante) | EVIDENCIA INDIRECTA |
| TEST C1 / C2 ("RAW SCAN") | **ningún comando localizado** | ON (según el propio título de la sección en RESULTS.txt) | **DESCONOCIDO** | Ver §3 |

## 3. Tests C1/C2 ("RAW SCAN") — configuración real DESCONOCIDA

`slam_ab_r1/RESULTS.txt` titula estas pruebas "CORRELATIVE ON + RAW SCAN" y concluye que "RAW reduce la discrepancia... frente al filtro completo". Dado el hallazgo de §1 (el filtro está hardcodeado en `cartographer.launch.py` y no es parametrizable), lograr un escenario "RAW" genuino requeriría **necesariamente** uno de estos caminos, ninguno de los cuales aparece en el historial visible:

- Pasar `use_scan_filter:=false` **y** `scan_topic:=/scan` (para que Cartographer consuma el scan crudo directamente, saltándose el nodo de filtro por completo).
- Ejecutar `cartographer_node` de forma manual/independiente (sin el launch file), con remapeos propios.
- Usar un launch distinto no localizado en el proyecto ni en `limoatwork`.

**No se encontró ninguna de estas variantes en `~/.bash_history`.** Es posible que:
(a) se haya ejecutado en una shell/terminal distinta cuyo historial no fue volcado a este archivo (el comportamiento por defecto de bash sin `histappend` puede hacer que una shell sobrescriba el historial de otra al salir), o
(b) el comando exacto haya quedado fuera de la ventana de 2000 líneas conservadas, o
(c) "RAW" se refiera a otra cosa no relacionada con el flag `use_scan_filter` (por ejemplo, una interpretación distinta del propio equipo al nombrar la prueba).

**Estado: DESCONOCIDO.** No se debe asumir que los tests C1/C2 usaron el archivo `laser_filters_range_only.yaml` del proyecto — no hay ningún launch conocido que lo referencie (ver también `AUDITORIA_PIPELINE_LOCALIZACION.md`, que ya señalaba esta ambigüedad; esta auditoría la profundiza pero no la resuelve).

## 4. `laser_filters_range_only.yaml` — propósito real sin confirmar

**CONFIRMADO** por timestamp: el archivo se creó a las 2026-09-15 19:44, **2 minutos después** de que se generaran los resultados de `slam_ab_r1` (19:42, según el timestamp embebido en `map_to_base.txt` de sesiones relacionadas — nota: el timestamp exacto de `slam_ab_r1/RESULTS.txt` en sí no está disponible como metadata de mensaje ROS, se infiere de la cercanía de archivos).

**HIPÓTESIS** (no confirmada): este archivo se creó *después* de concluir que correlative OFF no debía adoptarse, posiblemente como preparación para un test posterior (C1/C2) que finalmente se ejecutó por un método no capturado en el historial, o que quedó sin completar con esta configuración específica.

## 5. Resumen de certeza por afirmación

| Afirmación | Estado |
|---|---|
| El filtro de `cartographer.launch.py` es siempre el de 2 etapas del fabricante, salvo `use_scan_filter:=false` | **CONFIRMADO** (lectura de código) |
| TEST A y A2 de `slam_ab_r1` usaron correlative ON | **EVIDENCIA INDIRECTA** (comandos sin override + defecto del `.lua` de fábrica) |
| TEST B y B2 usaron correlative OFF vía el `.lua` del proyecto | **CONFIRMADO** que el comando existe y usa ese archivo; **EVIDENCIA INDIRECTA** de que corresponde exactamente a esas secciones del RESULTS.txt |
| Todos los tests A/B/A2/B2 usaron el filtro de 2 etapas del fabricante (no el `range_only` del proyecto) | **CONFIRMADO** por código, dado que ningún comando localizado pasó `use_scan_filter:=false` |
| Configuración real de los tests C1/C2 ("RAW") | **DESCONOCIDO** |
| El mapa de `stage3_cartographer_static` (63.75 m²) se generó con alguna variante de este mismo launch | **EVIDENCIA INDIRECTA** (coincide el nombre de nodo `cartographer_scan_filter_chain`, pero el comando exacto no está en el historial visible — es anterior a su inicio) |
| `laser_filters_range_only.yaml` fue usado en algún test real | **DESCONOCIDO** |

---

*Documento generado por análisis de solo lectura de `~/.bash_history`, `results/baseline/`, y código fuente de `limoatwork` (lectura). No se ejecutó ningún comando ROS, launch ni Cartographer. No se modificó ningún archivo.*
