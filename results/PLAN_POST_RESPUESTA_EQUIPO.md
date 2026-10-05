# Plan post-respuesta del equipo — LIMO RoboCup SML

Fecha: 2026-09-18
Propósito: definir **de antemano** qué acción tomar según cada respuesta posible del equipo a `MENSAJE_EQUIPO_PENDIENTES.md`, para no improvisar cuando lleguen las respuestas. Ninguna acción listada aquí se ejecuta hasta recibir la confirmación correspondiente.

**Regla transversal:** toda acción es análisis / documentación / preparación offline hasta que exista autorización explícita para tocar hardware, ejecutar ROS o modificar configuración. Nada en este plan modifica `~/limoatwork` (solo lectura). Ver `DECISIONES_BLOQUEADAS.md`.

---

## 1. Footprint

| Si el equipo confirma... | Acción |
|---|---|
| **Footprint oficial = 36×20 (medido), padding 0.03** | 1) Documentar en `CLAUDE.md` y en el yaml el origen (prueba `base_motion_r1`). 2) Preparar (sin aplicar) el diff de `navigation2_real.yaml`. 3) Re-correr el análisis offline de clearance con 36×20 sobre el mapa oficial. 4) Aplicar el cambio solo tras validación offline y con autorización. |
| **Footprint oficial = 39×32, padding 0.02** | 1) Pedir/registrar la **medición física** que lo respalde (hoy no existe). 2) Documentar el origen del polígono en `CLAUDE.md` (hoy DESCONOCIDO). 3) Mantener el valor actual sin cambios; cerrar el hallazgo #1 de `ESTADO_PROYECTO.md`. |
| **Ninguno de los dos / hay un tercer valor** | 1) Registrar el nuevo valor y su fuente. 2) Repetir análisis offline de clearance con ese valor. 3) No aplicar hasta re-medición física documentada. |
| **No hay medición física reciente** | Mantener bloqueo del footprint (ver `DECISIONES_BLOQUEADAS.md`); proponer una sesión de re-medición física como próximo hito. |

## 2. Mapa "v1" / mapa oficial

| Si el equipo... | Acción |
|---|---|
| **Identifica el archivo real de "v1"** | 1) Fijar ese archivo como referencia en la documentación. 2) Verificar su resolución/origen y consistencia con `map_server`. 3) Preparar (sin aplicar) el `yaml_filename` de `map_server`. |
| **Entrega el mapa oficial de competencia (.pgm/.yaml)** | 1) Ejecutar **análisis offline de clearance** (`distance_transform_edt`) con el footprint confirmado. 2) Reportar corredores con ancho 0.26–0.36 m (zona crítica según footprint). 3) Recién entonces evaluar si el footprint afecta rutas reales. |
| **Confirma que `map.pgm` (0.03 m) es el oficial** | Igual que arriba, usando ese archivo; documentar la elección y respaldarlo (commit) por el riesgo de versionado. |
| **No hay mapa oficial exportado** | Mantener bloqueo de regeneración de mapa; proponer sesión de remapeo físico (requiere autorización) como hito, con la config de Cartographer que se confirme (ver §4). |

## 3. Full arena (mapa ~63.75 m²)

| Si el equipo... | Acción |
|---|---|
| **Entrega un `.pbstream` de la sesión stage3** | Evaluar reconstrucción offline del mapa a `.pgm/.yaml` desde el `.pbstream` (proceso offline, sin hardware); luego análisis de clearance. |
| **Entrega un rosbag completo (`/scan` + `/tf` + `/odom`)** | Evaluar **reconstrucción SLAM offline** con la config confirmada; comparar con la metadata existente (63.75 m²). |
| **Entrega solo `.pgm/.yaml` (mapa ya exportado)** | Ir directo a análisis de clearance; saltar reconstrucción. |
| **No existe respaldo en ningún lado** | Cerrar la hipótesis "full arena reconstruible offline" como imposible con los datos actuales; proponer remapeo físico como única vía (requiere autorización). |

## 4. Cartographer / configuración de SLAM

| Si el equipo confirma... | Acción |
|---|---|
| **La config exacta con que se generó el mapa final** | 1) Cerrar la hipótesis abierta sobre "qué generó el mapa" (hoy EVIDENCIA INDIRECTA). 2) Documentar la config oficial. 3) Alinear (o dejar registrado el desalineamiento con) el `.lua` del proyecto. |
| **Correlative scan matching = ON** | Confirmar que `no_correlative.lua` **no** debe ser la config de producción; documentar y dejar preparada (sin aplicar) la config con correlative ON. |
| **Correlative scan matching = OFF** | Registrar la decisión contra la recomendación de `slam_ab_r1`; pedir la justificación para el registro. |
| **Filtro activo = 2 etapas (fabricante)** | Consistente con todo el baseline; ninguna acción de cambio, solo documentar. |
| **Filtro activo = 1 etapa (`range_only`, proyecto)** | Marca una divergencia con el baseline histórico; documentar y advertir que rompe comparabilidad; no cambiar sin revalidar. |
| **Explica qué fue "RAW" en C1/C2** | Actualizar `RELECTURA_SLAM_AB_R1.md` con la aclaración; cerrar la reserva metodológica sobre la comparación A vs C. |

## 5. AMCL / cámara / versionado (P1–P2)

| Si el equipo... | Acción |
|---|---|
| **Aporta una prueba de AMCL en vivo** | Incorporarla al baseline; cerrar la fila "AMCL validado" (hoy DESCONOCIDO). |
| **No la tiene** | Proponer diseñar una prueba dedicada de AMCL (captura `/amcl_pose`, TF `map→odom`) como hito, análogo a `base_motion_r1`. |
| **Confirma/descarta la cámara de profundidad** | Documentar en inventario de hardware; decidir si el `voxel_layer` debe quedar inactivo o eliminarse. |
| **Aprueba respaldar los mapas en git** | Preparar (sin ejecutar sobre `limoatwork`) un plan de respaldo externo, ya que `limoatwork` es solo lectura. |

---

## Secuencia recomendada una vez lleguen respuestas P0

1. Fijar footprint oficial → 2. Fijar mapa oficial → 3. Análisis offline de clearance (footprint × mapa) → 4. Fijar config de Cartographer/filtro → 5. Recién entonces evaluar cambios en Nav2/costmaps, siempre offline primero y con autorización antes de cualquier ejecución física.

---

*Plan preparatorio. No ejecuta ROS, launch ni hardware. No modifica configuración, mapas ni `~/limoatwork`.*
