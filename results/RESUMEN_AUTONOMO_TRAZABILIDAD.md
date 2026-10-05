# Resumen autónomo de trazabilidad — LIMO RoboCup SML

Fecha: 2026-09-17 (sesión de trazabilidad autónoma, continuación de `CIERRE_SESION_2026_09_17.md` y `RESUMEN_TRABAJO_2026_09_17.md`)

Modo de trabajo: análisis estático y forense de solo lectura. **No se ejecutó ROS, no se inició hardware, no se movió el robot, no se ejecutó ningún launch, no se modificó ninguna configuración, mapa ni código.** Fuentes nuevas explotadas: `~/.bash_history`, forense de timestamps (`stat`), historial git de `~/limoatwork` (solo lectura: `log`/`status`/`show`, sin checkout ni fetch), código fuente de launch files (lectura), historial de sesiones Claude Code.

---

## 1. Hallazgos nuevos

1. **El filtro de `/scan_filtered` en `cartographer.launch.py` está hardcodeado**, no es un argumento de launch parametrizable. Consecuencia: todo el baseline histórico de Cartographer conocido (tests A/A2/B/B2 de `slam_ab_r1`, y muy probablemente `stage3_cartographer_static`) usó **siempre** el filtro de 2 etapas del fabricante (range+speckle) — nunca el `range_only` de 1 etapa del proyecto.
2. **Se reconstruyó con evidencia indirecta sólida la secuencia de comandos de los tests A/B/A2/B2** de `slam_ab_r1`, directamente desde `~/.bash_history` — incluyendo los overrides exactos usados para activar la config `no_correlative` del proyecto.
3. **Los tests C1/C2 ("RAW SCAN") no tienen comando reconstruible.** Dado el hallazgo #1, un escenario "RAW" genuino con el launch estándar requeriría flags que no aparecen en ningún comando localizado — la configuración real de C1/C2 queda como incógnita más acotada mas no resuelta.
4. **Ningún mapa candidato a "v1"/"run3" está protegido por control de versiones en `limoatwork`.** Todos son `untracked` o modificados-sin-commit según `git status`. El único mapa alguna vez commiteado (18 de julio, autor IgorNunnes) es una versión antigua y más pequeña, hoy conservada por separado como `map_20260718_185606` — nuevo candidato a "v1" con un criterio distinto (formalidad de versionado) al que ya se tenía (recencia/resolución).
5. **El cambio de footprint (2026-09-15, 21:02-21:03) no tiene rastro en `~/.bash_history` ni en ninguna sesión previa de Claude Code — porque no existe ninguna sesión previa registrada.** El patrón de timestamps del backup asociado (`.pre_footprint_real`) es distinto y más "atómico" que el de los otros dos backups del mismo día, que sí siguen el patrón de una copia manual simple.
6. **Existe un tercer valor de footprint**, en `limoatwork/.../param/costmap_common.yaml` (del fabricante): 22×32 cm, padding 0.02. Comparte el padding exacto (0.02) con el footprint "actual" del proyecto, pero no el polígono — coincidencia parcial, no una copia directa confirmada.
7. **Ni "v1" ni "run3" aparecen en ningún mensaje de commit de todo el historial git de `limoatwork`** (todas las ramas, incluida `origin/robocup-sim`), ni en `~/.bash_history`. Refuerza, con una fuente adicional independiente, que son apodos verbales del equipo sin registro técnico.
8. **El repositorio `limoatwork` fue clonado de `https://github.com/UruBots/limoatwork.git`** — identifica el nombre del equipo/organización, útil como contexto no técnico.

## 2. Correcciones a supuestos anteriores

- **`AUDITORIA_PIPELINE_LOCALIZACION.md` (sesión anterior) planteaba como ambigüedad abierta** "si el proyecto usa `range_only` o `range+speckle`". Esta sesión **la resuelve para todo el baseline conocido excepto C1/C2**: fue siempre `range+speckle` (documento anterior decía "ambigüedad sin resolver"; nueva evidencia indica "resuelto para A/A2/B/B2, sigue abierto solo para C1/C2"). Ver `TRAZABILIDAD_SLAM_CONFIGS.md` §1.
- **`AUDITORIA_CERES_SCAN_MATCHER.md` y `AUDITORIA_CARTOGRAPHER_COMPLETA.md` planteaban como pregunta abierta** "¿con qué launch se ejecutó Cartographer en los tests?". Esta sesión **la responde parcialmente** con comandos concretos reconstruidos de `~/.bash_history` (documento anterior decía "desconocido"; nueva evidencia indica "reconstruido para A/A2/B/B2 con evidencia indirecta, desconocido para C1/C2"). Ver `TRAZABILIDAD_SLAM_CONFIGS.md` §2.
- **`AUDITORIA_MAPAS.md` señalaba `map.pgm` (0.03 m) como "el candidato más fuerte" a mapa oficial**, basado en recencia y resolución. Esta sesión **no lo contradice, pero añade un candidato alternativo de igual peso** (`map_20260718_185606`, único mapa versionado) bajo un criterio distinto. No se puede decir que uno sea "más correcto" — son dos hipótesis con evidencia de tipo diferente, ambas sin confirmar.
- **`RELECTURA_SLAM_AB_R1.md`** debilita (sin invalidar) la interpretación causal de la comparación "RAW vs filtrado" (tests C) de `slam_ab_r1/RESULTS.txt`, por la razón del hallazgo #3. El documento histórico original **no fue modificado**.

## 3. Qué sabemos con certeza (CONFIRMADO)

- Cinemática diferencial del robot y corrección del driver de yaw (`base_motion_r1`, PASS).
- Los dos valores de footprint en conflicto, y el momento exacto (segundo a segundo) en que cambió uno al otro.
- Que el filtro de scan en todo el launch estándar de Cartographer es fijo (2 etapas, fabricante).
- Que los pesos de Ceres nunca fueron modificados en ningún archivo del proyecto.
- Que ningún mapa candidato está protegido por control de versiones.
- Que no existe ninguna sesión de Claude Code anterior a la actual para este proyecto.
- Que "v1"/"run3" no aparecen en ningún artefacto técnico (archivos, commits, comandos) de todo el sistema accesible.

## 4. Qué sigue siendo hipótesis (no confirmado, pero con algún respaldo)

- Que las secciones A/B/A2/B2 de `slam_ab_r1/RESULTS.txt` corresponden exactamente (en ese orden) a los 4 comandos de `cartographer.launch.py` reconstruidos.
- Que el footprint "actual" tomó su `footprint_padding` del archivo del fabricante (`costmap_common.yaml`).
- Que "v1" se refiere al mapa commiteado por IgorNunnes (18 de julio) — o alternativamente, al `map.pgm` actual más reciente. Dos hipótesis en competencia.
- Que el cambio de footprint fue hecho con una herramienta de edición automatizada (por el patrón de timestamps) en vez de manualmente.
- Que `stage3_cartographer_static` se generó con el launch estándar del fabricante (por coincidencia del nombre de nodo del filtro).

## 5. Qué requiere respuesta del equipo

Consolidado completo en `PREGUNTAS_EQUIPO_PENDIENTES_V2.md`, 12 preguntas (6 en P0, 4 en P1, 2 en P2). Las de mayor impacto:
- Configuración exacta de los tests C1/C2.
- Identidad real de "v1" y "run3"/"full arena".
- Existencia de un backup externo del mapa de 63.75 m².
- Autoría y justificación del cambio de footprint.

## 6. Qué requiere robot físico

- **Re-mapear la arena** (si no aparece un backup externo del mapa de 63.75 m² ni un rosbag reconstruible) — ya identificado en el cierre anterior, sigue siendo la única vía confirmada.
- **Re-medir el footprint físico del robot**, incluyendo protuberancias, para resolver la discrepancia de una vez.
- **Grabar bags completos** (`/scan`, `/tf`, `/tf_static`, `/odom`, `/imu`) en la próxima sesión con hardware, para no repetir la pérdida de datos de reconstrucción offline.
- **Validar AMCL en vivo** (captura de `/amcl_pose`, TF `map→odom`) — no existe ningún artefacto de esto en el repo.

Ninguna de estas tareas se ejecutó ni se intentó ejecutar en esta sesión.

## 7. Qué tareas offline todavía pueden hacerse

- Decidir (con el equipo, pero documentar la decisión una vez tomada) cuál config de Cartographer usar como referencia (correlative ON/OFF, filtro 1 o 2 etapas) — es una decisión, no requiere robot.
- Preparar, offline, el comando exacto que se debería usar para regenerar el mapa (una vez decidida la config), para tenerlo listo antes de la sesión con hardware.
- Considerar (proponer al equipo, no ejecutar sin autorización) commitear en git los mapas actuales de `limoatwork` para eliminar el riesgo de pérdida por no estar versionados — esto es una escritura en `limoatwork` y **requiere autorización explícita**, ya que las reglas de esta sesión prohíben tocar ese árbol.
- Diseñar (documentar, sin ejecutar) el procedimiento de prueba end-to-end de Nav2, como ya estaba planificado en el cierre anterior.

## 8. Qué archivos/configuraciones NO deben cambiarse todavía

- `navigation2_real.yaml` (footprint y cualquier otro parámetro) — hasta resolver autoría/justificación del cambio de footprint.
- `config/cartographer/limo_lds_2d_no_correlative.lua` y su relación con los launches — hasta decidir con el equipo la config de referencia.
- `config/cartographer/laser_filters_range_only.yaml` — su propósito real sigue sin confirmar; no asumir que reemplaza al filtro del fabricante en producción.
- Cualquier archivo dentro de `~/limoatwork`, incluidos los mapas sin versionar — **regla absoluta de esta sesión, sin excepción**, a pesar del riesgo identificado en el hallazgo #4 (la mitigación de ese riesgo requiere autorización explícita separada).
- `map_server.yaml_filename` — hasta confirmar el mapa oficial.
- `PREGUNTAS_EQUIPO_PENDIENTES.md` (v1) — se conserva sin modificar como registro histórico; la v2 es el documento vigente.
- `results/baseline/slam_ab_r1/RESULTS.txt` — documento histórico, no se reescribe; las reinterpretaciones viven en `RELECTURA_SLAM_AB_R1.md`.

---

## Nota de honestidad metodológica

Todas las afirmaciones marcadas como CONFIRMADO en este documento y en los generados durante esta sesión provienen de evidencia verificable directamente (contenido de archivo, salida de comando de solo lectura, timestamp de filesystem). Las marcadas EVIDENCIA INDIRECTA o HIPÓTESIS se señalan explícitamente como tales y no deben tratarse como hechos establecidos. Ninguna conclusión de sesiones anteriores fue alterada silenciosamente — donde esta sesión aporta una corrección o matiz, se documenta explícitamente en la sección 2.

---

*Sesión de trazabilidad autónoma completada. No se ejecutó ningún comando ROS, launch, ni hardware. No se movió el robot. No se modificó ninguna configuración, mapa ni código. No se tocó `~/limoatwork`.*
