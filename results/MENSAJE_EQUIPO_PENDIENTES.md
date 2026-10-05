# Mensaje para el equipo — Confirmaciones pendientes (listo para enviar)

> Este archivo es un mensaje redactado para copiar y enviar al equipo. No contiene decisiones ni cambios técnicos. Todo lo que aparece aquí está respaldado por la trazabilidad de solo lectura documentada en `results/` (ver `MATRIZ_EVIDENCIA_PROYECTO.md`).

---

**Asunto:** Confirmaciones necesarias para cerrar decisiones de navegación — LIMO RoboCup SML

Hola equipo:

Antes de tocar footprint, mapas o configuración de SLAM/Nav2, hicimos una revisión de **trazabilidad** para no cambiar nada sin evidencia. La conclusión es que hay varias decisiones que dependen de información que solo ustedes pueden confirmar (no quedó registrada en archivos, commits ni historial de comandos del workspace). No es que haya errores: son puntos donde el repositorio no alcanza para decidir solo, y queremos confirmarlos antes de avanzar.

Agrupé las preguntas por prioridad. Las **P0** bloquean trabajo: sin ellas no podemos cerrar nada.

---

## P0 — Bloqueantes

### Footprint
1. **¿Cuál es el footprint que consideramos oficial para competencia?**
2. El footprint activo hoy en `navigation2_real.yaml` es **39×32 cm, padding 0.02**. ¿Fue una **decisión intencional** (margen de seguridad) o hay que revisarlo? No encontramos justificación escrita ni comando que lo explique.
3. La única medición **física** documentada en el repo es **36×20 cm, padding 0.03** (prueba `base_motion_r1`). ¿Existe una **medición física más reciente** que respalde el valor de 39×32 (incluyendo protuberancias: soporte de LiDAR, cables, antena)?

### Mapa
4. **¿Qué archivo corresponde realmente a "v1"?** Tenemos dos candidatos y no podemos elegir sin ustedes: (a) `map.pgm` actual (resolución 0.03 m, el más reciente) o (b) `map_20260718_185606` (resolución 0.05 m, el único que alguna vez estuvo commiteado en git).
5. **¿Qué archivo corresponde realmente a "run3"?** No encontramos ningún archivo con ese nombre ni referencia en commits/historial.
6. **¿Existe el mapa oficial de competencia exportado** (`.pgm` + `.yaml` o equivalente), y cuál es? Hoy `map_server.yaml_filename` está **vacío** en `navigation2_real.yaml`.

### Full arena
7. **¿Existe un respaldo del mapa "full arena"** generado en la sesión `stage3_cartographer_static` (~63.75 m²)? Solo encontramos su metadata, no el mapa en sí.
8. **¿Hay un `.pbstream`, `.pgm/.yaml` o rosbag completo de esa sesión** guardado en algún lado (disco externo, otra máquina, nube, repo remoto)? Localmente no está persistido, y los rosbags disponibles solo tienen `/cmd_vel` (sin `/scan` ni `/tf`), así que no alcanzan para reconstruirlo offline.

### Cartographer
9. **¿Con qué configuración exacta se generó el mapa final** que usaremos en competencia?
10. **¿Se usó correlative scan matching** (ON/OFF)?
11. **¿Qué filtro de LiDAR estaba activo** al generar ese mapa: el de 2 etapas del fabricante (`range+speckle`) o el de 1 etapa del proyecto (`range_only`)? Todo el histórico disponible (A/A2/B/B2, y muy probablemente `stage3`) se generó con el del fabricante.

---

## P1 — Importantes (condicionan diseño, no bloquean todo)

12. **¿La configuración `no_correlative.lua`** debe quedar como config de SLAM del proyecto? Fue la variante que el propio test `slam_ab_r1` recomendó **no adoptar**, pero hoy es el único `.lua` propio del workspace.
13. **¿Los tests C1/C2 ("RAW SCAN") de `slam_ab_r1`** se ejecutaron con algún launch o flag especial? El launch estándar tiene el filtro de scan hardcodeado, así que un "RAW" genuino habría necesitado algo extra que no aparece en el historial.
14. **¿Existe alguna prueba de AMCL en vivo** (captura de `/amcl_pose` o TF `map→odom` de AMCL) fuera de `results/baseline/`? No la encontramos.

---

## P2 — Deseables (contexto útil, no urgente)

15. **¿El robot tiene físicamente una cámara de profundidad** (RealSense R200 u otra)? Aparece referenciada en un `voxel_layer` inactivo del costmap, sin figurar en ningún inventario de hardware.
16. **¿Respaldamos (commiteamos) en git los mapas actuales?** Hoy ninguno de los mapas candidatos está bajo control de versiones — es un riesgo si se limpia `~/limoatwork`.

---

Con las respuestas P0 podemos desbloquear el análisis de clearance sobre el mapa correcto y cerrar la decisión de footprint. Gracias.
