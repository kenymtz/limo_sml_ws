# Estado actual del proyecto — resumen de entrada — LIMO RoboCup SML

Fecha: 2026-09-18
Para: alguien que entra nuevo al proyecto y necesita entender en 5 minutos qué pasa. Máx. 2 páginas. Detalle completo en `MATRIZ_EVIDENCIA_PROYECTO.md` y en los `AUDITORIA_*` / `TRAZABILIDAD_*`.

---

## En una frase

Robot AgileX LIMO (ROS2 Humble) para navegación autónoma RoboCup SML. La **base del robot está validada** (cinemática, driver, odometría); lo que está **frenado** es footprint, mapa y configuración de SLAM, porque el repositorio no alcanza para decidir solo y hace falta que el equipo confirme varias cosas.

---

## ✅ Qué funciona / está validado (con prueba física)

- **Cinemática:** robot diferencial puro (`MODE_FOUR_DIFF`), sin movimiento lateral. Confirmado con prueba física.
- **Driver de yaw:** corregido (se eliminó un deadband). Giros ±90° con error ≤0.1°. PASS.
- **Odometría lineal:** validada (objetivo 50 cm, error ~1 cm, sin sesgo). No necesita factor de calibración.
- **AMCL — modelo de movimiento:** corregido a `DifferentialMotionModel` (coherente con la cinemática real).
- **Controller (DWB):** corregido a no-holonómico (antes estaba mal, como mecanum).
- **TF `base_link → laser_frame`:** medido y estable (0.180, 0, −0.090 m).
- **SLAM (Cartographer) — TF `map → base_link`:** estable, error 1–3 cm XY, <1° yaw.

Fuente: `results/baseline/base_motion_r1/`, `slam_ab_r1/`, `ESTADO_PROYECTO.md`.

## 🟡 Qué existe pero NO está cerrado

- **Nav2:** nodos activos confirmados (amcl, map_server, controller, planner, bt_navigator), pero **no hay prueba end-to-end** documentada (planner+controller siguiendo una ruta real).
- **SLAM — sesgo lateral:** hay un sesgo sistemático de ~1.4–2.8 cm que no desaparece con ninguna config probada. Causa raíz sin confirmar.
- **AMCL en vivo:** no hay ninguna captura real (`/amcl_pose`, TF de AMCL) que respalde que "AMCL está validado". Solo está configurado.
- **Mapas:** hay varios archivos candidatos, pero **ninguno confirmado** como el oficial, y `map_server` no tiene mapa asignado (campo vacío).

## 🔴 Qué está bloqueado (no tocar hasta confirmar)

Detalle y motivos en `DECISIONES_BLOQUEADAS.md`. En corto:

1. **Footprint** — hay 3 valores distintos y no se sabe cuál es el oficial.
2. **Mapa de producción** — no se sabe cuál archivo es "v1"/"run3"/"full arena".
3. **Pesos de Ceres** — nunca tocados; no tocar sin baseline.
4. **Correlative scan matching** — la config activa es la que el equipo recomendó NO usar.
5. **Filtro de LiDAR** — cambiar rompe comparabilidad con todo el histórico.
6. **AMCL** — sin baseline en vivo, cambiarlo sería a ciegas.
7. **Planner / controller** — depende de footprint y mapa, aún sin resolver.

---

## Los 3 problemas raíz (lo importante)

### 1. Footprint sin origen confirmado
Tres valores conviven:

| Fuente | Tamaño | Padding |
|---|---|---|
| Medición física (`base_motion_r1`) | 36×20 cm | 0.03 |
| **Activo hoy** (`navigation2_real.yaml`) | **39×32 cm** | **0.02** |
| Referencia fabricante (`costmap_common.yaml`) | 22×32 cm | 0.02 |

El valor **activo (39×32) no tiene medición física ni justificación escrita** y es más grande que los otros dos. Cambió el 2026-09-15 (21:02–21:03) sin rastro de quién ni por qué. Impacto medido: el footprint grande "come" ~8–9% del espacio libre en los mapas de prueba.

### 2. Mapa oficial no identificado
Los nombres "v1", "run3" y "full arena" **no corresponden a ningún archivo** encontrado (ni en disco, ni en git, ni en historial). Son, con alta probabilidad, apodos verbales del equipo. El candidato "full arena" (~63.75 m²) **no tiene archivo persistido** ni rosbag para reconstruirlo.

### 3. Config de SLAM ambigua
La config activa (`no_correlative.lua`) es la que el propio test del equipo (`slam_ab_r1`) recomendó **no adoptar**. Además, el filtro de LiDAR de todos los tests históricos fue el del fabricante (2 etapas), no el del proyecto — así que la etiqueta "RAW" de algunos tests no está confirmada como scan realmente sin filtrar.

---

## Qué se está esperando

Respuestas del equipo a `MENSAJE_EQUIPO_PENDIENTES.md` (prioridad P0: footprint, mapa, full arena, config de Cartographer). Con esas respuestas se desbloquea casi todo (ver `PLAN_POST_RESPUESTA_EQUIPO.md`).

## Reglas duras del proyecto (no negociables)

- `~/limoatwork` es **solo lectura**. Nunca modificar.
- Todo código nuevo vive en `~/limo_sml_ws`.
- **Nada de movimiento físico** sin autorización explícita (ver reglas de seguridad en `CLAUDE.md`).
- Ante duda: analizar, explicar el riesgo, pedir confirmación. No ejecutar.

---

*Resumen de entrada. No ejecuta ROS, launch ni hardware. No modifica configuración, mapas ni `~/limoatwork`.*
