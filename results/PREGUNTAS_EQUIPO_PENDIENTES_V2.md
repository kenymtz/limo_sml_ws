# Preguntas pendientes para el equipo — v2 (post-trazabilidad) — LIMO RoboCup SML

Fecha: 2026-09-17
Reemplaza en vigencia a `PREGUNTAS_EQUIPO_PENDIENTES.md` (v1, **no eliminado, se conserva como registro histórico**). Esta versión refleja qué preguntas siguen sin respuesta después de una sesión completa de trazabilidad local (bash_history, timestamps, git de `limoatwork`, código de launch).

---

## P0 — Bloquea trabajo (sin poder avanzar sin esto)

1. **¿Con qué comando exacto se ejecutaron los tests C1/C2 ("RAW SCAN") de `slam_ab_r1`?** Se confirmó que el launch estándar (`cartographer.launch.py`) tiene el filtro de scan hardcodeado (no parametrizable) — un escenario "RAW" genuino requeriría flags (`use_scan_filter:=false` + remapeo de `scan_topic`) que no aparecen en ningún comando localizado. *(Nuevo, refinado en esta sesión — antes solo se sabía que "RAW vs filtrado" era ambiguo; ahora se sabe específicamente qué se necesitaría para lograrlo y que no hay rastro de ello.)*

2. **¿Cuál archivo corresponde a "v1"?** Ahora hay dos candidatos con razones distintas, sin poder elegir entre ellos: (a) `map.pgm` actual (0.03 m, más reciente, coincide con nota de CLAUDE.md sobre resolución de evaluación) o (b) `map_20260718_185606` (el único mapa que alguna vez fue commiteado en git de `limoatwork`, por IgorNunnes el 18 de julio). *(Reformulado — antes solo había un candidato débil por nombre; ahora hay dos candidatos con evidencia de distinto tipo.)*

3. **¿Cuál corresponde a "run3"/"full arena"?** Sigue sin resolverse. El candidato por escala (mapa de 63.75 m², `stage3_cartographer_static`) no tiene archivo persistido en ningún lugar accesible del sistema, ni rosbag que permita reconstruirlo. *(Sin cambios respecto a v1 del documento de preguntas.)*

4. **¿Existe una copia del mapa de 63.75 m² (u otro "full arena") en algún backup externo** (disco externo, otra máquina, repositorio remoto no clonado localmente, nube)? La búsqueda local fue exhaustiva y no encontró nada; solo un backup externo podría resolverlo sin remapear físicamente. *(Sin cambios.)*

5. **¿Quién cambió el footprint el 2026-09-15 entre las 21:02 y 21:03, y con qué herramienta?** Se descartó que fuera mediante un comando de shell registrado o una sesión previa de Claude Code (ninguna existe). El patrón de timestamps sugiere una herramienta de edición automatizada, no una edición manual típica, pero no se pudo identificar cuál. *(Refinado — antes solo se sabía "no hay prueba física"; ahora se sabe que tampoco hay rastro de shell ni de agente, lo cual acota más las hipótesis pero no las resuelve.)*

6. **¿Por qué se cambió el footprint a 39×32 cm?** ¿Existe una medición nueva no documentada, o fue una decisión de margen de seguridad, o un error? *(Sin cambios — sigue sin explicación escrita en ningún lugar.)*

## P1 — Importante (no bloquea todo, pero condiciona decisiones de diseño)

7. **¿Con qué configuración de Cartographer (correlative ON/OFF, filtro) se debe generar el mapa oficial?** Dado que la config activa del proyecto (`no_correlative.lua`) es la que el propio equipo descartó en `slam_ab_r1`, y el launch por defecto usa la del fabricante (correlative ON), conviene decidir explícitamente cuál usar antes de cualquier remapeo futuro.

8. **¿El filtro de producción para `/scan_filtered` debe ser el de 1 etapa (`range_only`, proyecto) o el de 2 etapas (`range+speckle`, fabricante)?** El baseline completo (`base_motion_r1`, `slam_ab_r1` A/B/A2/B2, y muy probablemente `stage3`) se generó con el de 2 etapas — cambiar a 1 etapa sin revalidar rompería la comparabilidad con todo el histórico.

9. **¿Cuál `.pgm`+`.yaml` debe cargar `map_server` en producción?** Hoy `yaml_filename` está vacío en `navigation2_real.yaml`.

10. **¿Existe alguna prueba de AMCL en vivo** (captura de `/amcl_pose`, TF `map→odom` de AMCL específicamente) fuera de `results/baseline/`? Sigue sin encontrarse ningún artefacto — el estado "AMCL activo" de CLAUDE.md no tiene respaldo empírico documentado en el repo.

## P2 — Deseable (contexto útil, no urgente)

11. **¿El robot tiene físicamente una cámara de profundidad** (RealSense R200 u otra)? Referenciada en un `voxel_layer` inactivo del costmap global, sin aparecer en ningún inventario de hardware del proyecto.

12. **¿Se ha considerado respaldar (commitear) en git los mapas actuales de `limoatwork/.../limo_bringup/maps/`?** Se confirmó que ninguno de los mapas candidatos actuales está protegido por control de versiones — es un riesgo operativo nuevo identificado en esta sesión, independiente de la identidad de "v1"/"run3".

---

## Preguntas resueltas por evidencia local

Estas preguntas de `PREGUNTAS_EQUIPO_PENDIENTES.md` (v1) fueron total o parcialmente resueltas por la trazabilidad de esta sesión:

### "¿Con qué launch y argumentos se ejecutó Cartographer en `slam_ab_r1`?" (v1, pregunta C12)
**Parcialmente resuelta.** Se reconstruyó con evidencia indirecta la secuencia completa de comandos para (con alta probabilidad) los tests A, B, A2, B2: todos usan `ros2 launch limo_bringup cartographer.launch.py`, con y sin overrides de `configuration_basename`/`cartographer_config_dir`. Ver `TRAZABILIDAD_SLAM_CONFIGS.md` §2. **Sigue sin resolverse** la parte de los tests C1/C2 (ver P0-1 arriba).

### "¿Por qué está activa la configuración no-correlative?" (v1, pregunta C10)
**Parcialmente resuelta.** Se confirmó que existe un comando explícito de override (`configuration_basename:=limo_lds_2d_no_correlative.lua`) usado deliberadamente durante los tests SLAM-B — es decir, el archivo no quedó ahí "por accidente de instalación", sino que fue creado y usado a propósito **para el test comparativo**. Lo que sigue sin explicación es por qué, **después** de que el test concluyera "no adoptar B", el archivo quedó como el único `.lua` del proyecto sin que se agregara o restaurara una variante equivalente a la config A recomendada.

### "¿El filtro de producción debe ser range_only o range+speckle?" — aporte parcial
No se resolvió cuál *debe* ser, pero se **confirmó con certeza técnica (lectura de código)** que **todo el baseline histórico disponible** (A/A2/B/B2, muy probablemente también `stage3`) se generó con el filtro de 2 etapas del fabricante, nunca con el `range_only` del proyecto — esto convierte la pregunta 8 (P1) en una decisión informada, no en una incógnita total.

### "¿Existe evidencia de que alguien haya tocado los pesos de Ceres?" (v1, pregunta C16)
**Resuelta: no.** Confirmado por comparación directa contra los defaults del paquete Cartographer instalado (`AUDITORIA_CERES_SCAN_MATCHER.md`) — los pesos nunca fueron sobreescritos en ningún `.lua` del proyecto ni de la referencia del fabricante.

---

*Documento de consolidación v2. No contiene decisiones ni cambios. `PREGUNTAS_EQUIPO_PENDIENTES.md` (v1) se conserva sin modificar como registro histórico de la sesión anterior.*
