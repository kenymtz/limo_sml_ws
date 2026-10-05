# Decisiones bloqueadas — LIMO RoboCup SML

Fecha: 2026-09-18
Propósito: listar explícitamente las decisiones que **NO deben tomarse todavía**, por qué están bloqueadas y qué evidencia falta para desbloquearlas. Sirve como salvaguarda: si algo de esta lista se propone ejecutar sin la evidencia indicada, hay que detenerse.

Fuente de respaldo: `MATRIZ_EVIDENCIA_PROYECTO.md`, `TRAZABILIDAD_*`, `AUDITORIA_*`, `RELECTURA_SLAM_AB_R1.md`, `ESTADO_PROYECTO.md`.

Estados usados: **CONFIRMADO / EVIDENCIA INDIRECTA / HIPÓTESIS / PENDIENTE DE CONFIRMACIÓN**.

---

## 1. Cambiar el footprint de `navigation2_real.yaml`

- **Estado:** BLOQUEADA.
- **Por qué:** el footprint activo (39×32, padding 0.02) no tiene medición física de respaldo ni justificación escrita; contradice la única medición física documentada (36×20, padding 0.03, prueba `base_motion_r1`). El origen del polígono actual es DESCONOCIDO.
- **Evidencia que falta (PENDIENTE DE CONFIRMACIÓN):** cuál footprint es el oficial (P0-1/2/3 del mensaje al equipo) y una medición física actualizada.
- **Desbloquea con:** confirmación del equipo + medición física + análisis offline de clearance sobre el mapa oficial.

## 2. Regenerar / cambiar el mapa de producción

- **Estado:** BLOQUEADA.
- **Por qué:** no está confirmado cuál archivo es el mapa oficial ("v1"/"run3"/"full arena" no corresponden a ningún archivo identificado). `map_server.yaml_filename` está vacío. El candidato "full arena" (63.75 m²) no tiene archivo persistido ni rosbag reconstruible.
- **Evidencia que falta:** identidad del mapa oficial y/o un `.pbstream`/`.pgm`/rosbag completo (P0-4 a P0-8).
- **Desbloquea con:** el equipo entrega el mapa oficial o un respaldo reconstruible; o se autoriza un remapeo físico.

## 3. Cambiar los pesos de Ceres (`ceres_scan_matcher`)

- **Estado:** BLOQUEADA.
- **Por qué:** CONFIRMADO que los pesos nunca fueron modificados (están en valores de fábrica). Cambiarlos alteraría el comportamiento de SLAM sin baseline comparativo. El sesgo lateral observado no tiene causa raíz confirmada.
- **Evidencia que falta:** repeticiones controladas que aíslen el efecto de los pesos; decisión del equipo sobre si tocar SLAM.
- **Desbloquea con:** auditoría dedicada de Ceres + repeticiones + autorización (es cambio de SLAM).

## 4. Activar / desactivar correlative scan matching

- **Estado:** BLOQUEADA.
- **Por qué:** `slam_ab_r1` ya evaluó y recomendó **no adoptar** correlative OFF (config B); sin embargo, la config activa del proyecto (`no_correlative.lua`) es justamente esa. No está confirmado con qué config se generó el mapa oficial.
- **Evidencia que falta:** confirmación de la config con que se generó/generará el mapa oficial (P0-9/10).
- **Desbloquea con:** respuesta del equipo sobre correlative ON/OFF para el mapa oficial.

## 5. Cambiar el filtro de LiDAR (1 etapa `range_only` vs 2 etapas fabricante)

- **Estado:** BLOQUEADA.
- **Por qué:** CONFIRMADO que todo el baseline histórico (A/A2/B/B2, muy probablemente `stage3`) usó el filtro de 2 etapas del fabricante. Cambiar a 1 etapa rompería la comparabilidad con todo el histórico. Además, "RAW" en C1/C2 no está confirmado como scan realmente sin filtrar.
- **Evidencia que falta:** qué filtro debe ser el de producción (P0-11) y qué fue realmente "RAW" en C1/C2 (P1-13).
- **Desbloquea con:** decisión del equipo + revalidación si se cambia.

## 6. Modificar AMCL

- **Estado:** BLOQUEADA.
- **Por qué:** no existe una prueba de AMCL en vivo documentada (equivalente a `base_motion_r1`/`slam_ab_r1`). El estado "AMCL activo/validado" no tiene respaldo empírico en el repo. Sin baseline, cualquier cambio sería a ciegas.
- **Evidencia que falta:** una captura de `/amcl_pose` / TF `map→odom` de AMCL (P1-14).
- **Desbloquea con:** diseñar y ejecutar una prueba dedicada de AMCL (requiere autorización de ejecución).

## 7. Ajustar planner / controller (Nav2)

- **Estado:** BLOQUEADA.
- **Por qué:** no existe prueba end-to-end de Nav2 documentada (planner+controller siguiendo una ruta real). Ajustar sin baseline ni footprint/mapa confirmados sería prematuro y dependiente de decisiones aún bloqueadas (footprint, mapa).
- **Evidencia que falta:** footprint oficial + mapa oficial + un baseline end-to-end de navegación.
- **Desbloquea con:** resolver footprint y mapa (P0), luego diseñar la prueba end-to-end.

---

## Nota de dependencias

Las decisiones **1 (footprint)** y **2 (mapa)** son las raíz: casi todo lo demás (análisis de clearance, planner/controller, evaluación de rutas) depende de ellas. Priorizar las respuestas P0 de footprint y mapa desbloquea la mayor parte del árbol.

---

*Documento de salvaguarda. No ejecuta ni modifica nada. `~/limoatwork` permanece solo lectura.*
