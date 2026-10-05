# Trazabilidad del cambio de footprint — LIMO RoboCup SML

Fecha: 2026-09-17
Método: solo lectura — forense de timestamps (`stat`), `~/.bash_history`, historial de sesiones Claude Code, git de `limoatwork` (solo lectura), comparación con archivos de parámetros del fabricante. No se ejecutó ROS, no se modificó ningún archivo.

---

## 1. Los hechos ya conocidos (recordatorio, sin repetir análisis)

- Footprint "medido" (`base_motion_r1`): `[[-0.16,-0.10],[-0.16,0.10],[0.20,0.10],[0.20,-0.10]]`, padding 0.03.
- Footprint "actual" (`navigation2_real.yaml`): `[[-0.17,-0.16],[-0.17,0.16],[0.22,0.16],[0.22,-0.16]]`, padding 0.02.

## 2. Forense de timestamps — momento exacto del cambio

**CONFIRMADO** (vía `stat`, filesystem ext4/local, timestamps de creación y modificación):

| Archivo | Birth (creación) | Modify (última escritura) |
|---|---|---|
| `navigation2_real.yaml` | 2026-09-15 **15:43:51.296** | 2026-09-15 **21:03:03.733** |
| `.before_kinematics.yaml` (backup) | 2026-09-15 15:52:24.758 | igual (instante único) |
| `.pre_diff_amcl` (backup) | 2026-09-15 20:54:58.065 | igual (instante único) |
| `.pre_footprint_real` (backup) | 2026-09-15 **21:02:23.295** | 2026-09-15 **21:03:03.649** |

**Momento exacto del cambio de footprint: 2026-09-15, entre 21:02:23 y 21:03:03.733 (hora local, ventana de ~40 segundos).** Es la última modificación registrada del archivo `navigation2_real.yaml` — no hay ediciones posteriores a esa fecha.

### Anomalía detectada en el patrón de backups

**CONFIRMADO por comparación directa de timestamps**: los dos backups anteriores del mismo día (`.before_kinematics.yaml` a las 15:52 y `.pre_diff_amcl` a las 20:54) tienen **Birth = Modify en el mismo instante exacto** — patrón consistente con una copia simple (`cp origen destino`), que no vuelve a tocar el archivo después de crearlo.

El backup `.pre_footprint_real`, en cambio, tiene **Birth (21:02:23.295) distinto de Modify (21:03:03.649)** — una diferencia de **~40 segundos** durante los cuales el archivo fue escrito una segunda vez. Y el archivo final `navigation2_real.yaml` fue modificado **84 milisegundos después** de esa segunda escritura del backup (21:03:03.733 vs 21:03:03.649).

**EVIDENCIA INDIRECTA (forense de timestamps, no una prueba directa)**: esta secuencia (backup escrito dos veces con 40s de diferencia, luego el archivo final modificado 84ms después de la segunda escritura del backup) **no es el patrón típico de una edición manual con `cp` + `nano`/editor de texto**, que dejaría un backup de instante único (como los otros dos) y un intervalo de segundos-a-minutos (tiempo humano de edición) antes de guardar el archivo final, no de 84 milisegundos. Es más compatible con una herramienta que automatiza "leer → respaldar → escribir" como una operación casi atómica (por ejemplo, una herramienta de edición de archivos usada por un agente, o un script propio no localizado).

## 3. ¿Quién/qué lo hizo? — búsqueda de autoría

### En `~/.bash_history`
**CONFIRMADO (ausencia)**: no se encontró ningún `cp`, `sed -i`, invocación de editor (`nano`/`vim`/`code`), ni ejecución de script (`.sh`/`.py`) relacionado con `navigation2_real.yaml` o con la palabra "footprint" en todo el historial disponible (2000 líneas). Esto es notable porque el historial **sí** registra actividad de shell en ese mismo rango horario (la sesión "SLAM ESTATICO R1" cerca de las 19:42-21:0x, ver `TRAZABILIDAD_SLAM_CONFIGS.md`) — es decir, la shell estaba activa y registrando comandos alrededor de la misma ventana de tiempo, pero ninguno de esos comandos tocó el archivo de footprint.

### En sesiones previas de Claude Code
**CONFIRMADO (ausencia)**: `~/.claude/projects/-home-limopro-limo-sml-ws/` contiene un único directorio de sesión, correspondiente a la sesión actual (identificable por el ID de sesión que aparece en las rutas de scratchpad del entorno actual). **No existe ningún registro de una sesión de Claude Code anterior** que pudiera haber hecho este cambio mediante herramientas de edición de archivos.

### En git de `limoatwork`
No aplica directamente — `navigation2_real.yaml` vive en `limo_sml_ws`, que **no es un repositorio git** (`git status` devuelve "not a git repository"). No hay historial de versiones para este archivo específico.

### Conclusión de autoría
**DESCONOCIDO.** No se pudo identificar, con las fuentes disponibles, qué herramienta o persona realizó el cambio. Las dos hipótesis más plausibles, ninguna confirmable con la evidencia actual:

- **HIPÓTESIS A**: se hizo con un editor de texto gráfico (ej. VSCode) fuera de la shell registrada en `~/.bash_history` — no dejaría rastro en ese archivo. El patrón de timestamps (backup + escritura rápida) no es típico de este flujo salvo que el editor tuviera un mecanismo de auto-backup no identificado.
- **HIPÓTESIS B**: se hizo mediante una herramienta o script que automatiza backup+edición (similar en comportamiento a una operación de "leer, guardar copia de seguridad, escribir nuevo contenido" ejecutada programáticamente) — consistente con el patrón de timestamps casi atómico, pero sin sesión de Claude Code registrada que lo explique, y sin script `.sh`/`.py` localizado que lo haga.

## 4. ¿Existe explicación escrita?

**CONFIRMADO (ausencia)**: no existe, en ningún archivo de `limo_sml_ws` (`CLAUDE.md`, `results/`, comentarios en el propio yaml), ninguna nota, comentario o justificación escrita para el cambio de footprint. El nombre del archivo de backup (`.pre_footprint_real`) es autodescriptivo ("antes del footprint real") pero no constituye una justificación — solo indica la intención de quien lo nombró (creer que el nuevo valor era "el real"), no evidencia que lo respalde.

## 5. Posible origen parcial del valor — comparación con archivo del fabricante

**EVIDENCIA INDIRECTA nueva** (hallazgo de esta sesión): existe un **tercer** valor de footprint, distinto de los dos ya conocidos, en un archivo de parámetros del fabricante (solo lectura, `limoatwork/src/limo_ros2/limo_bringup/param/costmap_common.yaml`):

```yaml
footprint: [[-0.16, -0.11], [-0.16, 0.11], [0.16, 0.11], [0.16, -0.11]]
footprint_padding: 0.02
```

Esto es 32×22 cm, padding 0.02.

Comparación de los tres valores:

| Fuente | Polígono | Ancho×Largo | Padding |
|---|---|---|---|
| Medido (`base_motion_r1`) | `[[-0.16,-0.10],[-0.16,0.10],[0.20,0.10],[0.20,-0.10]]` | 20×36 cm | 0.03 |
| **Vendor** (`costmap_common.yaml`, limoatwork) | `[[-0.16,-0.11],[-0.16,0.11],[0.16,0.11],[0.16,-0.11]]` | 22×32 cm | **0.02** |
| Actual (`navigation2_real.yaml`) | `[[-0.17,-0.16],[-0.17,0.16],[0.22,0.16],[0.22,-0.16]]` | 32×39 cm | **0.02** |

**Observación (EVIDENCIA INDIRECTA, no CONFIRMADA como causa)**: el `footprint_padding = 0.02` del valor "actual" coincide **exactamente** con el del archivo del fabricante, mientras que el valor "medido" usa 0.03. Esto sugiere que quien fijó el footprint "actual" pudo haber partido del padding del fabricante (0.02) — pero el polígono en sí (32×39 cm) **no coincide** con ninguno de los otros dos valores conocidos (ni el medido, 20×36 cm, ni el del fabricante, 22×32 cm). Es decir, el footprint "actual" no parece ser una copia directa de ninguna fuente conocida — es mayor que ambas en el eje Y, y mayor que el vendor también en X. **No se puede confirmar el origen exacto del polígono; el padding compartido con el fabricante es la única coincidencia parcial encontrada.**

## 6. ¿Fue deliberado o un cambio sin documentación?

Con la evidencia disponible, **no se puede determinar si fue una decisión deliberada** (alguien decidió ampliar el margen de seguridad conscientemente) **o un error** (por ejemplo, una confusión de unidades, una copia incorrecta desde otra fuente, o una generación automática incorrecta). Lo que sí es **CONFIRMADO**:

- No hay documentación escrita que lo explique.
- No hay comando de shell que lo explique.
- No hay sesión de Claude Code previa que lo explique.
- El patrón de timestamps es distinto del usado para los otros dos cambios del mismo día (que sí fueron simples `cp`).
- El polígono resultante no coincide exactamente con ninguna fuente conocida en el proyecto, aunque comparte el padding con un archivo del fabricante.

**Veredicto: DESCONOCIDO si fue deliberado.** Dado que viola la propia regla del proyecto ("Nunca modificar footprint sin justificar", CLAUDE.md), y no hay registro de justificación en ningún lugar accesible, **esta auditoría no puede resolver la pregunta por trazabilidad local — requiere respuesta humana directa** (ver `PREGUNTAS_EQUIPO_PENDIENTES_V2.md`).

---

*Documento generado por forense de timestamps y búsqueda de solo lectura en `~/.bash_history`, `~/.claude/projects/`, y archivos del fabricante (lectura). No se modificó ningún archivo de configuración, footprint ni mapa. No se ejecutó ROS ni hardware.*
