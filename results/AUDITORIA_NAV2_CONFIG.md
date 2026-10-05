# Auditoría de configuración Nav2 — LIMO RoboCup SML

Fecha: 2026-09-17
Archivo auditado: `config/navigation/navigation2_real.yaml`
Método: solo lectura y análisis estático. No se ejecutó ningún launch, nodo ROS ni comando de movimiento.

---

## 1. Parámetros importantes encontrados

### AMCL
- `robot_model_type: nav2_amcl::DifferentialMotionModel` — coherente con la cinemática real confirmada (`MODE_FOUR_DIFF`, ver `results/baseline/base_motion_r1/RESULTS.txt`).
- `scan_topic: /scan_filtered` — AMCL usa el scan filtrado, no el crudo.
- `max_particles: 2000`, `min_particles: 500`.
- `update_min_a: 0.2` rad (~11.5°), `update_min_d: 0.25` m — umbrales de actualización de partículas.
- `transform_tolerance: 1.0` s — tolerancia alta para la transformada `map→odom`.
- `laser_max_range: 8.0`, `laser_min_range: 0.12`.
- `alpha1..alpha5: 0.2` (todos iguales) — modelo de ruido de odometría sin diferenciar traslación/rotación.
- `tf_broadcast: true`.

### controller_server
- `controller_frequency: 10.0` Hz.
- Plugin: `FollowPath` → `dwb_core::DWBLocalPlanner`.
- `holonomic: False` — correcto para robot diferencial (corregido respecto a versión histórica que tenía `True`, ver `.before_kinematics.yaml`).
- Límites de velocidad: `max_vel_x: 0.25`, `max_vel_theta: 1.0`, `min_vel_x: 0.0`.
- `max_vel_y: 0.0`, `min_vel_y: 0.0`, `vy_samples: 1` — consistente con robot no-holonómico.
- `acc_lim_x: 1.0`, `decel_lim_x: -1.0`, `acc_lim_theta: 1.5`.
- `xy_goal_tolerance: 0.25` m (en `general_goal_checker` **y** repetido dentro de `FollowPath`).
- Critics activos: `["Oscillation", "BaseObstacle", "GoalAlign", "PathAlign", "PathDist", "GoalDist"]`. **`RotateToGoal` está comentado/excluido** de la lista activa aunque sus parámetros (`RotateToGoal.scale`, `RotateToGoal.slowing_factor`) siguen definidos en el yaml.
- `BaseObstacle.scale: 0.10` (bajo) vs `PathAlign.scale: 32.0`, `PathDist.scale: 32.0`, `GoalAlign.scale: 24.0`, `GoalDist.scale: 24.0` (altos) — fuerte preferencia por seguir el path/alinear con el goal sobre evitar obstáculos activamente en el costo del DWB.

### planner_server
- Plugin: `nav2_navfn_planner/NavfnPlanner` (no A*: `use_astar: false`).
- `tolerance: 0.2` m, `allow_unknown: false` — el planner **no** puede planear a través de zonas desconocidas del mapa.
- `expected_planner_frequency: 20.0` Hz.

### local_costmap
- `global_frame: odom`, `rolling_window: true`, `width/height: 3x3` m, `resolution: 0.05`.
- `footprint: "[[-0.17,-0.16],[-0.17,0.16],[0.22,0.16],[0.22,-0.16]]"`, `footprint_padding: 0.02`.
- `inflation_layer`: `cost_scaling_factor: 2.0`, `inflation_radius: 0.35` m.
- `obstacle_layer` y `voxel_layer` ambos escuchan `/scan_filtered`; el `voxel_layer` está definido pero **no aparece en `plugins:`** (solo `["obstacle_layer", "inflation_layer"]`) → queda inerte, no se usa.
- `update_frequency: 5.0`, `publish_frequency: 2.0`.

### global_costmap
- `global_frame: map`, `resolution: 0.03` (distinta a la del local, que usa 0.05).
- Mismo `footprint` y `footprint_padding: 0.02` que el local costmap.
- `plugins: ["static_layer", "obstacle_layer", "inflation_layer"]`.
- `inflation_layer`: mismos valores que local (`cost_scaling_factor: 2.0`, `inflation_radius: 0.35`).
- `voxel_layer` definido con fuente `/intel_realsense_r200_depth/points` (cámara de profundidad) pero, igual que en local, **no está en la lista `plugins:`** → inerte. Además referencia un sensor (RealSense R200) no mencionado en ningún otro punto de la documentación del proyecto.
- `update_frequency: 1.0`, `publish_frequency: 1.0` — más lento que el local costmap (razonable, ya que es global y de mayor extensión).

---

## 2. Relación footprint / inflation / clearance

- **Footprint activo** (local y global, idéntico): `[[-0.17,-0.16],[-0.17,0.16],[0.22,0.16],[0.22,-0.16]]` + `footprint_padding: 0.02` → extensión efectiva aprox. 39 cm (largo) × 32 cm (ancho) + padding.
- **Inflation radius: 0.35 m** en ambos costmaps, con `cost_scaling_factor: 2.0`.
- Según el análisis offline ya realizado (`results/footprint_analysis/`), este footprint es ~60% más ancho que el footprint medido físicamente en `base_motion_r1` (`[[-0.16,-0.10],[-0.16,0.10],[0.20,0.10],[0.20,-0.10]]`, padding 0.03, ancho total 26 cm vs 36 cm actual). Esa auditoría cuantificó una pérdida de 8-9% del espacio libre transitable en dos mapas de prueba, concentrada como margen adicional alrededor de obstáculos y paredes, sin confirmar aún si bloquea corredores reales de la arena de competencia.
- El `inflation_radius` de 0.35 m se suma, en términos de costo (no de colisión dura), por fuera del footprint ya inflado por el padding — es decir, hay dos capas de margen superpuestas: (a) el footprint+padding (límite de colisión real usado por el chequeo de footprint) y (b) la zona de costo creciente por inflación (usada por el planner/controller para preferir rutas alejadas de obstáculos, no un límite duro). Esto es la arquitectura estándar de Nav2 y no es en sí mismo un error, pero **el tamaño combinado depende directamente de qué footprint sea el correcto** — si el footprint base está sobreestimado (hallazgo ya documentado en `ESTADO_PROYECTO.md`), el problema se propaga multiplicado a cualquier análisis de clearance que use inflación.

---

## 3. Posibles riesgos

1. **Footprint no validado físicamente** (ya documentado en auditorías previas) — cualquier parámetro de clearance/inflación calculado sobre este footprint hereda esa incertidumbre.
2. **`voxel_layer` inerte en ambos costmaps** — está completamente parametrizado (incluyendo, en el global, una fuente de point cloud de una cámara RealSense) pero no se activa porque no está en la lista `plugins:`. Si en algún momento alguien asume que el voxel layer está funcionando (p. ej. para detectar obstáculos por altura), esa suposición sería falsa con la configuración actual.
3. **Referencia a sensor no documentado** (`/intel_realsense_r200_depth/points` en `global_costmap.voxel_layer`) — no hay mención de esta cámara en CLAUDE.md ni en el resto del proyecto auditado. Podría ser un remanente de una plantilla/ejemplo (p. ej. TurtleBot3) no limpiado, dado que el archivo tiene comentarios que referencian explícitamente turtlebot3 (`# Add high threshold velocity for turtlebot 3 issue`).
4. **Resolución distinta entre local costmap (0.05) y global costmap (0.03)** — no es necesariamente un error (es común tener distinta resolución), pero introduce una fuente adicional de discretización distinta entre ambos costmaps que conviene tener presente al depurar comportamientos cerca de obstáculos.
5. **`RotateToGoal` definido pero no activo** en `controller_plugins` de DWB — sus parámetros están en el yaml pero no tienen efecto; puede llevar a confusión si alguien ajusta `RotateToGoal.scale` esperando que tenga impacto.
6. **`BaseObstacle.scale` muy bajo (0.10) frente a critics de seguimiento de path (32.0) y alineación (24.0)** — el controlador prioriza fuertemente seguir el path/alinear con el goal sobre el costo de evitar obstáculos en el cálculo de trayectoria local. Combinado con un `inflation_radius` de 0.35 m, el sistema depende en gran medida de que la capa de inflación (no el critic de obstáculo) sea la que efectivamente aleje al robot de obstáculos.
7. **`transform_tolerance: 1.0 s` en AMCL** es relativamente alto — no es necesariamente incorrecto, pero vale la pena confirmar que no está enmascarando problemas de sincronización de tiempo/TF en vez de ser una elección deliberada.
8. **Comentarios y elecciones de parámetros con referencias directas a TurtleBot3** (`# tb3_simulation_launch.py`, `# turtlebot 3 issue`) en varios bloques del yaml — sugiere que el archivo partió de una plantilla de TurtleBot3 y fue adaptado parcialmente; no se puede asumir que todos los valores no mencionados en las pruebas documentadas (`base_motion_r1`, `slam_ab_r1`) hayan sido validados específicamente para el LIMO.

---

## 4. Inconsistencias detectadas

- **Footprint del costmap vs footprint medido físicamente** (ya reportado en `ESTADO_PROYECTO.md`, hallazgo #1) — se reitera aquí porque es el eje central de esta configuración.
- **`voxel_layer` completamente configurado pero no incluido en `plugins:`** en local y global costmap — configuración muerta, no coherente con el resto de la config activa.
- **Sensor RealSense referenciado sin evidencia de que exista en el robot real** — no aparece en ningún otro archivo del proyecto (`serial_devices.txt`, `limo_packages.txt`, CLAUDE.md).
- **Comentarios de plantilla TurtleBot3 no limpiados** — indicio de que el archivo no ha sido revisado línea por línea para el LIMO real, más allá de los parámetros ya corregidos y documentados (cinemática, AMCL, footprint).

---

## 5. Cosas que NO deben modificarse todavía

- **No modificar el footprint** hasta resolver la discrepancia ya documentada en `ESTADO_PROYECTO.md` (pendiente de re-medición física y confirmación del equipo).
- **No activar el `voxel_layer`** (agregándolo a `plugins:`) sin antes confirmar si existe o no una cámara de profundidad físicamente instalada en el robot — hacerlo sin verificar podría introducir una fuente de datos inexistente o mal configurada al costmap.
- **No eliminar ni "limpiar" los comentarios/parámetros de TurtleBot3** sin revisar primero si algún valor heredado de esa plantilla sigue siendo necesario (p. ej. el ajuste de velocidad umbral mencionado en el comentario).
- **No reactivar `RotateToGoal`** en `controller_plugins` sin evaluar su efecto, ya que fue deliberadamente excluido (no hay evidencia documentada de por qué, pero está fuera de la lista activa mientras sus parámetros permanecen — cambiarlo sin prueba sería una modificación de comportamiento de navegación no validada).
- **No cambiar `inflation_radius` ni `cost_scaling_factor`** de forma aislada del footprint — ambos están acoplados; cualquier cambio debe evaluarse junto con la decisión pendiente sobre el footprint correcto.
- **No ejecutar pruebas de navegación real** con esta configuración hasta resolver el hallazgo de footprint y confirmar la identidad del mapa de producción (pendientes ya documentados en `ESTADO_PROYECTO.md`).

---

*Informe generado a partir de la lectura de `config/navigation/navigation2_real.yaml` y de los hallazgos ya documentados en `results/baseline/` y `results/footprint_analysis/`. No se ejecutó ningún comando ROS, launch, ni de movimiento para su elaboración.*
