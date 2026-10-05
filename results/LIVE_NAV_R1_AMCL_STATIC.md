# LIVE_NAV_R1 — AMCL estático (FASE 3) — LIMO RoboCup SML

Fecha: 2026-09-18
Dominio aislado `ROS_DOMAIN_ID=17`. Robot **quieto** durante toda la fase. Se publicó **una única** `/initialpose` en (0,0,0) con autorización explícita del usuario (no mueve el robot; solo fija la creencia de AMCL). No se movió el robot. No se tocó `/cmd_vel`. `~/limoatwork` solo lectura.

## Veredicto FASE 3: ✅ **PASS (maquinaria)** — con salvedad de covarianza (ver §5)

Es el **primer artefacto de AMCL en vivo con datos propios** del proyecto (todas las auditorías previas notaron su ausencia).

## 0. Cómo se llegó aquí (hallazgo de infraestructura)

- El launch del fabricante `limo_localization.launch.py` **NO funciona en Humble**: importa `Node` desde `nav2_common.launch` (API de Foxy/Galactic, eliminada en Humble) → `ImportError`. **Inutilizable tal cual.**
- Workaround Humble-nativo usado: `ros2 launch nav2_bringup localization_launch.py map:=<map.pgm> params_file:=~/limo_sml_ws/config/navigation/navigation2_real.yaml`. Levanta `map_server` + `amcl` + `lifecycle_manager`, sin movimiento.
- Requisito previo: se arrancó el filtro `scan_to_scan_filter_chain` (2 etapas, fabricante) para producir `/scan_filtered` (~10.1 Hz), que es el topic que AMCL consume.

## 1. Estado y lifecycle

| Nodo | Estado | 
|---|---|
| `map_server` | **active [3]** ✅ (mapa `map.pgm` cargado, `/map` publicado) |
| `amcl` | **active [3]** ✅ |
| `lifecycle_manager_localization` | activo, bond OK |

## 2. Parámetros efectivos de AMCL (LIVE, confirmados con `ros2 param get`)

| Parámetro | Valor live | Coincide con config proyecto |
|---|---|---|
| `robot_model_type` | `nav2_amcl::DifferentialMotionModel` | ✅ |
| `scan_topic` | `/scan_filtered` | ✅ |
| `set_initial_pose` | `False` | ✅ (por eso esperaba `/initialpose`) |

→ Confirma que la localización corre con la **config del proyecto** (`navigation2_real.yaml`), no con la del fabricante.

## 3. Diagnóstico de "AMCL no publicaba" (antes del initialpose)

- Log: `AMCL cannot publish a pose or update the transform. Please set the initial pose...`
- **Causa (sin mover el robot):** `set_initial_pose=false` + ausencia de `/initialpose`. No es un fallo de AMCL; es su comportamiento por diseño.
- Tras publicar `/initialpose (0,0,0)`: log `initialPoseReceived` → `Setting pose: 0.000 0.000 0.000`. (Hubo un warning transitorio de extrapolación TF al momento exacto del set, sin efecto posterior.)

## 4. Medición estática (robot quieto)

| Métrica | Valor | Nota |
|---|---|---|
| `/amcl_pose` pose | **x=−0.0534 m, y=0.0304 m**, yaw ≈ 0.23° | offset ~6.1 cm respecto al (0,0) asumido; coherente con robot en el origen |
| `/amcl_pose` tasa (robot quieto) | **~1 mensaje** en 12 s | actualización gateada por `update_min_d=0.25 m` / `update_min_a=0.2 rad`: sin movimiento no re-actualiza |
| TF `map→base_link` | **[−0.053, 0.030, 0.000]** constante en 4 muestras (~12 s) | ✅ **perfectamente estable** con robot quieto |
| Variación X/Y/yaw estática | ~0 (no hay nuevas estimaciones sin movimiento) | estable por diseño |

## 5. Covarianza (salvedad importante)

| Componente | Valor | std |
|---|---|---|
| `cov_xx` | 0.242 m² | ~49 cm |
| `cov_yy` | 0.209 m² | ~46 cm |
| `cov_yaw` | 0.0662 rad² | ~14.7° |

**La covarianza reportada ≈ la sembrada en el `/initialpose` (0.25 / 0.25 / 0.0685).** Con el robot quieto, AMCL **no ejecuta actualización de medición** (motion-gated), por lo que **no encoge la covarianza**. Es decir, este número refleja la incertidumbre *inicial impuesta*, **no** la confianza real del scan-matching. La confianza real de localización solo puede evaluarse **con el robot en movimiento** (dispara updates y encoge la covarianza) → prueba futura de movimiento.

## 6. Criterios del veredicto

| Criterio | Resultado |
|---|---|
| map_server + amcl activos, `/map` cargado | ✅ |
| Config del proyecto efectiva (DifferentialMotionModel, /scan_filtered) | ✅ |
| AMCL publica `/amcl_pose` y TF `map→odom` tras initialpose | ✅ |
| Pose converge cerca del origen esperado (robot en (0,0)) | ✅ (offset ~6 cm) |
| TF `map→base_link` estable con robot quieto | ✅ |
| Sin errores en lifecycle ni caídas | ✅ |
| Covarianza reducida por scan-matching | ⚠️ No evaluable en estático (motion-gated) |

**PASS de la maquinaria de AMCL.** La reducción de covarianza / confianza real de localización queda **PENDIENTE de una prueba con movimiento**.

## 7. Notas

- Filtro usado: **2 etapas del fabricante** (baseline-consistente). La decisión range_only vs range+speckle sigue abierta (no afecta este PASS de maquinaria).
- Mapa usado: `map.pgm` (0.03 m) como **candidato**, no confirmado como oficial. El offset de ~6 cm no permite, por sí solo, certificar el mapa.

---

*FASE 3 completada en dominio aislado, robot quieto. Un único `/initialpose` autorizado. No se movió el robot. `~/limoatwork` intacto.*
