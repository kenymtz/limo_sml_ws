# Relectura de `slam_ab_r1/RESULTS.txt` a la luz de la trazabilidad — LIMO RoboCup SML

Fecha: 2026-09-17
Fuente: `results/baseline/slam_ab_r1/RESULTS.txt` (documento histórico, **no modificado**) + `TRAZABILIDAD_SLAM_CONFIGS.md` (esta sesión).
Método: solo lectura y análisis cruzado. No se ejecutó ROS, no se modificó ningún archivo, incluyendo el propio `RESULTS.txt` original.

**Principio de este documento**: no se reescribe ni se corrige `slam_ab_r1/RESULTS.txt`. Se documenta explícitamente qué conclusiones siguen sosteniéndose, cuáles quedan debilitadas por la nueva evidencia de trazabilidad, y cuáles ya no pueden sostenerse tal como estaban formuladas.

---

## 1. Conclusiones que SIGUEN VÁLIDAS

### "No adoptar configuración B (correlative OFF)"
**Sigue válida.** Esta conclusión compara A vs B, y ambas, según `TRAZABILIDAD_SLAM_CONFIGS.md` §2, compartieron el mismo filtro (2 etapas, fabricante) — la única variable que cambió entre A y B fue el flag de correlative scan matching, que es exactamente la comparación que el test pretendía hacer. La comparación A vs B es internamente válida.

### "El sesgo lateral persiste con correlative ON y OFF"
**Sigue válida**, con el mismo respaldo anterior: A, A2, B y B2 comparten filtro; el sesgo aparece en las cuatro. No hay evidencia nueva que lo contradiga.

### "El Ceres scan matcher permanece activo incluso con correlative OFF"
**Sigue válida** y reforzada: confirmado en `AUDITORIA_CERES_SCAN_MATCHER.md` que los pesos de Ceres nunca fueron tocados (siguen en valores de fábrica) en ninguna de las variantes de `.lua` disponibles.

## 2. Conclusiones DEBILITADAS por la nueva evidencia

### "RAW reduce la discrepancia media respecto al filtro completo" (comparación C vs A)

**Debilitada.** La comparación A/B fue entre correlative ON/OFF **con el mismo filtro** (2 etapas). Pero la comparación C1/C2 vs A pretende aislar el efecto del **filtro** (RAW vs "filtro completo"), y aquí la trazabilidad no pudo confirmar qué configuración de filtro se usó realmente en C1/C2 (`TRAZABILIDAD_SLAM_CONFIGS.md` §3): el launch estándar del proyecto no permite un escenario "RAW" sin pasar flags adicionales que no aparecen en el historial disponible.

Esto no significa que la medición en sí sea falsa — los datos numéricos de C1/C2 en `RESULTS.txt` son observaciones reales de una prueba que ocurrió. Lo que se debilita es la **interpretación causal**: no se puede confirmar con certeza que la mejora observada (avance MAP-ODOM medio: 0.895 cm en C vs 1.605 cm en A) se deba específicamente a usar scan crudo en lugar del filtro de 2 etapas, porque no se sabe qué configuración de filtro tenían realmente C1/C2. Podría deberse a esa diferencia, o a otra variable no controlada de esa corrida particular (deriva, condición de la arena, timing).

### Comparabilidad numérica entre "A" y "C" como si solo variara el filtro

**Debilitada** por la misma razón: la conclusión "RAW reduce la discrepancia... pero no elimina el sesgo lateral" asume implícitamente que A y C comparten todo excepto el tipo de scan. Sin confirmar la configuración exacta de C, esa comparación pierde parte de su rigor — sigue siendo la mejor evidencia disponible, pero con una reserva metodológica que antes no estaba documentada.

## 3. Conclusiones que YA NO PUEDEN SOSTENERSE tal como estaban formuladas

Ninguna de las conclusiones centrales de `slam_ab_r1/RESULTS.txt` queda invalidada por completo. La trazabilidad no contradice ningún resultado numérico ni la comparación A/B (que sí quedó bien reconstruida). Lo único que "ya no puede sostenerse sin matiz" es:

- **La premisa implícita de que "RAW" en C1/C2 significa scan crudo sin ningún filtro.** Dado el hallazgo estructural de `TRAZABILIDAD_SLAM_CONFIGS.md` §1 (el filtro está hardcodeado en el launch estándar), si C1/C2 se ejecutaron con ese mismo launch sin flags adicionales, **"RAW" no podría haber sido scan verdaderamente sin filtrar** — habría seguido pasando por el filtro de 2 etapas del fabricante, y "RAW" tendría que referirse a otra cosa (quizás una comparación mental del equipo, o un método de ejecución distinto no documentado). Esto es una alerta metodológica, no una refutación de los números.

## 4. Preguntas todavía abiertas (específicas de esta relectura)

1. ¿Con qué comando exacto se ejecutaron los tests C1/C2? (ver `PREGUNTAS_EQUIPO_PENDIENTES_V2.md`, pregunta de prioridad P0)
2. Si C1/C2 usaron el launch estándar sin flags adicionales, ¿qué quiso decir el equipo con "RAW SCAN" en el título de esa sección?
3. ¿Existe alguna otra forma de lanzar Cartographer en el proyecto (script propio, launch no localizado) que sí permita bypass del filtro?
4. Dado que A/A2/B/B2 comparten filtro (2 etapas, fabricante) y **no** el `range_only.yaml` del proyecto: ¿el archivo `laser_filters_range_only.yaml` fue usado en algún test real documentado, o quedó sin uso confirmado?

## 5. Nota de trazabilidad honesta

Este documento **no cambia ningún número ni conclusión** de `results/baseline/slam_ab_r1/RESULTS.txt`. Añade una capa de contexto sobre **qué tan bien se puede reconstruir la configuración real** detrás de cada sección del test, usando evidencia externa (historial de shell, código de launch) que no estaba disponible o no se había cruzado en auditorías previas. La comparación A/B es la que queda mejor respaldada; la comparación con C1/C2 es la que queda con mayor incertidumbre metodológica.

---

*Documento generado por análisis cruzado de solo lectura. El archivo histórico `results/baseline/slam_ab_r1/RESULTS.txt` no fue modificado. No se ejecutó ROS, launch ni Cartographer.*
