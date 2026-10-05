# Contradicciones pendientes — LIMO RoboCup SML

Fecha: 2026-09-18
Propósito: registrar contradicciones internas detectadas al revisar el conjunto de documentos del proyecto (los de trazabilidad y los generados en esta sesión), **sin corregir ningún archivo previo**. Cada entrada solo documenta el choque y qué evidencia lo resolvería. La resolución depende de confirmación humana o de nueva evidencia; hasta entonces quedan abiertas.

Formato por entrada: **Documento A dice / Documento B dice / Evidencia disponible / Estado.**

---

## C1 — Footprint "real": CLAUDE.md lo da por definido; la trazabilidad lo da por injustificado

**Documento A dice** (`CLAUDE.md`, sección "Footprint real del robot"):
> Usar footprint real: `[[-0.17,-0.16],[-0.17,0.16],[0.22,0.16],[0.22,-0.16]]`, padding 0.02 m. "Nunca modificar footprint sin justificar."

Es decir, presenta el valor **39×32 (padding 0.02)** como el footprint real y correcto a usar.

**Documento B dice** (`TRAZABILIDAD_FOOTPRINT.md`, `ESTADO_PROYECTO.md` §4.1, `MATRIZ_EVIDENCIA_PROYECTO.md`):
El valor 39×32 **no tiene medición física de respaldo ni justificación escrita**, su origen es DESCONOCIDO, cambió el 2026-09-15 sin rastro de autoría, y **contradice la única medición física documentada** (36×20, padding 0.03, prueba `base_motion_r1`, 4 h antes en la misma sesión).

**Evidencia disponible:** prueba física `base_motion_r1` (respalda 36×20). Forense de timestamps (cambio a 39×32 el 2026-09-15, sin autor identificado). Ausencia total de justificación escrita. El padding 0.02 coincide con el del fabricante, pero el polígono no coincide con ninguna fuente conocida.

**Estado:** ABIERTA. CLAUDE.md afirma como "real" un valor que la trazabilidad no puede respaldar. No se puede saber si CLAUDE.md refleja una decisión deliberada del equipo (que quedaría como justificación válida) o si arrastra el valor sin respaldo. **Resuelve:** P0-1/2/3 del mensaje al equipo + medición física.

---

## C2 — "run3" como mejor mapa global: CLAUDE.md lo afirma; la trazabilidad no encuentra el archivo

**Documento A dice** (`CLAUDE.md`, sección "Mapas"):
> Mapas importantes: v1 producción, run3 full arena. "run3 fue evaluado como mejor mapa global."

Presupone que "run3" existe como mapa concreto y que fue evaluado y elegido.

**Documento B dice** (`AUDITORIA_MAPAS.md`, `TRAZABILIDAD_MAPAS_V1_RUN3.md`, `MATRIZ_EVIDENCIA_PROYECTO.md`):
"v1", "run3" y "full arena" **no corresponden a ningún archivo** localizable (cero coincidencias en disco, git y `~/.bash_history`). El candidato por escala ("full arena", 63.75 m², `stage3_cartographer_static`) **no tiene archivo persistido** (.pgm/.yaml/.pbstream) ni rosbag reconstruible.

**Evidencia disponible:** búsqueda exhaustiva de filesystem + git (`log --all`) + historial de shell: sin coincidencias. Metadata de `stage3` existe (63.75 m²) pero sin el mapa. Rosbags disponibles solo tienen `/cmd_vel`.

**Estado:** ABIERTA. CLAUDE.md registra "run3" como una evaluación ya hecha, pero no existe el artefacto que la respalde ni permita reproducirla. **Resuelve:** P0-4/5/7/8 (identidad del archivo real de run3 / respaldo de full arena).

---

## C3 — Candidato para "v1": dos hipótesis distintas (aparente, ya reconciliada)

**Documento A dice** (`AUDITORIA_MAPAS.md`, `ESTADO_PROYECTO.md` §4.2):
El candidato más fuerte para "v1" es **`map.pgm` actual (0.03 m)**, por recencia de modificación y coincidencia de resolución con la nota de CLAUDE.md.

**Documento B dice** (`TRAZABILIDAD_MAPAS_V1_RUN3.md` §3):
El candidato con más peso pasa a ser **`map_20260718_185606` (0.05 m)**, por ser el único mapa alguna vez commiteado en git (criterio de formalidad de versionado).

**Evidencia disponible:** ambos criterios son válidos y apuntan a archivos distintos. `TRAZABILIDAD_MAPAS_V1_RUN3.md` §3 **explícitamente aclara** que "no contradice pero reencuadra" al anterior, y que ambas hipótesis "conviven sin resolverse; ninguna es más correcta sin confirmación del equipo".

**Estado:** NO es contradicción real — es una **doble hipótesis reconocida y reconciliada** dentro de los propios documentos. Se registra aquí solo para que un lector nuevo no la confunda con un choque. **Resuelve:** P0-4 (qué archivo es "v1").

---

## C4 — Convención de rótulo del footprint (Ancho×Largo vs Largo×Ancho) — cosmética

**Documento A dice** (`TRAZABILIDAD_FOOTPRINT.md` §5, tabla):
Footprint actual = "**32×39 cm**" (rotulado Ancho×Largo, Y×X).

**Documento B dice** (`ESTADO_PROYECTO.md`, `MATRIZ_EVIDENCIA_PROYECTO.md`, `MENSAJE_EQUIPO_PENDIENTES.md`):
Footprint actual = "**39×32 cm**" (rotulado Largo×Ancho, X×Y).

**Evidencia disponible:** el polígono es el mismo en ambos: `[[-0.17,-0.16],[-0.17,0.16],[0.22,0.16],[0.22,-0.16]]` → X (largo) = 0.39 m, Y (ancho) = 0.32 m. Los números son idénticos; solo cambia el orden de los ejes en el rótulo.

**Estado:** NO es contradicción de datos — es una **inconsistencia de convención de rótulo** entre documentos. Sin impacto en decisiones, pero puede confundir. Se recomienda (a futuro, sin corregir lo ya escrito) fijar una convención única (p. ej. siempre Largo×Ancho / X×Y) en documentos nuevos.

---

## Resumen

| ID | Tipo | Prioridad | Resuelve con |
|---|---|---|---|
| C1 | Contradicción real (CLAUDE.md vs trazabilidad) | Alta | P0 footprint + medición física |
| C2 | Contradicción real (CLAUDE.md vs trazabilidad) | Alta | P0 mapa / full arena |
| C3 | Doble hipótesis ya reconciliada (no es choque) | Media | P0 identidad de "v1" |
| C4 | Convención de rótulo (cosmética) | Baja | Fijar convención en docs futuros |

---

*Documento de registro. No se corrigió ni modificó ningún archivo previo (incluido `CLAUDE.md`, que además pertenece a las instrucciones del proyecto). No ejecuta ROS ni hardware.*
