# MAP_ANALYSIS_R1 — arena_20260918_1900

Script: `scripts/nav_offline_lab/map_analysis.py [semantic|loaded]`
Salidas: `map_analysis.json`, `map_overview.png` (semántico) · `map_analysis_as_loaded.json`, `map_overview_as_loaded.png`
El mapa original no se modifica (solo lectura).

Leyenda: **[C]** CONFIRMADO · **[E]** EVIDENCIA · **[H]** HIPÓTESIS

Radios del footprint real (CLAUDE.md) + padding 0.02 m:
- r_inscrito = 0.16 + 0.02 = **0.18 m**
- r_circunscrito = √(0.22² + 0.16²) + 0.02 = **0.292 m**

Clearance calculado con la transformada de distancia euclídea en dos escenarios:
- **OPT**: solo lo ocupado cuenta como obstáculo (optimista).
- **CONS**: ocupado + desconocido cuentan como obstáculo (conservador).

---

## 1. Dos lecturas del mismo mapa [C]

| Clase | Semántica PGM (205 = desconocido) | Como lo carga Nav2 (yaml actual) |
|---|---|---|
| Libre | 27.0 % (31.2 m²) | **98.5 %** (113.8 m²) |
| Ocupado | 1.49 % (1.72 m²) | 1.49 % |
| Desconocido | 71.5 % | **0 %** |

La columna "Nav2" está verificada con el `nav2_map_server` real (ver DATASET_REPORT §1.3). **Todo lo que sigue usa la semántica PGM**, que es la interpretación correcta del contenido.

## 2. Estructura del espacio libre (semántico) [C]
- Componentes libres (8-conexión): 56.
  - Principal: 30.80 m², el 98.7 % del libre.
  - 47 componentes son < 0.01 m² (ruido).
- BBox del componente principal: x ∈ [-2.72, 7.06], y ∈ [-4.34, 4.27] m.
- **Frontera del componente principal: solo el 14.6 % toca ocupado y el 85.4 % toca desconocido.**
  - [E] El espacio libre **no está cerrado por paredes**.
  - [E] La forma es de "estrella": rayos libres radiales desde una zona de ~1 × 1 m (ver `map_overview.png`), la huella típica de un mapeo desde casi un solo punto de vista. Es coherente con la trayectoria: bbox en map de ~1.06 × 0.95 m.
- Desconocido interno: 203 componentes que no tocan el borde de la imagen.
  - Suman 0.89 m²; el mayor mide 0.18 m² y solo 1 supera 0.05 m².
  - [E] Son sombras entre rayos y detrás de obstáculos cercanos, no huecos de un área barrida.

## 3. Clearance (componente libre principal) [C]

| Métrica | OPT | CONS |
|---|---|---|
| Clearance mediana / p90 / máx | 0.36 / 0.81 / 1.33 m | 0.17 / 0.66 / 1.10 m |
| Área válida para el centro con d ≥ r_ins | 23.9 m² (14 componentes; mayor 20.7) | 15.2 m² (7; mayor 14.1) |
| Área válida para el centro con d ≥ r_circ (cualquier yaw) | 18.4 m² (12; mayor 15.0) | 10.1 m² (7; mayor 9.8) |
| "Cuellos" detectados (r_ins ≤ d < r_circ uniendo 2 regiones ≥ 0.05 m²) | 3: (3.07, 3.33), (−0.23, 1.59), (−0.53, −0.33) | 1: (1.57, 0.40) |

- [E] Estos cuellos **no deben leerse como pasillos reales de la arena**. Salen de la geometría radial (rayos angostos) y de obstáculos cercanos al punto de observación. No se puede identificar ningún cuello de botella real con este mapa.
- d ≥ r_circ es condición suficiente para el footprint rectangular en cualquier orientación. d ≥ r_ins es solo necesaria. Una evaluación exacta del footprint en SE2 queda fuera de R1.

## 4. Consistencia trayectoria real ↔ mapa [C]
- 17 028 muestras de la trayectoria en frame map (cada 10 de /odom): **100 % sobre celdas libres**, 0 % sobre desconocido u ocupado.
- Clearance mínimo al ocupado en la trayectoria: **0.09 m**. El 0.9 % de las muestras tiene d < r_ins (0.18 m).
  - [E] Físicamente, el centro del robot no puede estar a 9 cm de un obstáculo real: el semiancho es 0.16 m.
  - [H] Causas posibles, no discriminadas:
    - (a) error de la pose SLAM en esos instantes;
    - (b) celdas ocupadas por objetos transitorios, como personas junto al robot;
    - (c) el mapa final se deformó respecto a la pose registrada.

## 5. Conclusión del mapa
| Criterio | Estado |
|---|---|
| Integridad de archivos | **PASS** [C] |
| Carga correcta en Nav2 (desconocido preservado) | **FAIL** [C]: free_thresh 0.25 |
| Utilidad como mapa de navegación de arena | **FAIL** [E]: mapa radial desde un solo punto, sin perímetro cerrado (85 % de la frontera del libre es desconocido) |
| Utilidad como baseline para herramientas offline | **PASS** [E]: sirve para validar scripts y pipeline, no para concluir sobre la arena |

**No se recomienda usar este mapa para evaluar rutas, clearance de arena ni AMCL**. No es "mapa oficial", y además tampoco es un mapa de área barrida.
