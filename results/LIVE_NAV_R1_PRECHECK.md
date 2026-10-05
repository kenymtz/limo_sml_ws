# LIVE_NAV_R1 — Precheck (FASE 0) — LIMO RoboCup SML

Fecha: 2026-09-18
Modo: diagnóstico ROS de **solo lectura**. No se lanzó ningún nodo/launch, no se publicó ningún topic, no se movió el robot, no se modificó configuración ni `~/limoatwork`.
Entorno: `ROS_DISTRO=humble`, `ROS_DOMAIN_ID=5`, `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` (leído de `~/.bashrc`).

## VEREDICTO FASE 0: **STOP — NO CONTINUAR CON BRINGUP LIMO**

El dominio ROS está **compartido y contaminado por un sistema ajeno con DDS incompatible**, y el stack del LIMO **no está corriendo**. Lanzar componentes del LIMO en este estado es inseguro y produciría conflictos de TF/odom/cmd_vel. Se requiere decisión del usuario antes de avanzar (ver final).

---

## 1. Comandos ejecutados (todos read-only)

- `ros2 node list` (×2)
- `ros2 topic list`
- `ros2 topic info /cmd_vel -v`, `/scan`, `/odom -v`, `/imu/data -v`, `/tf -v`, `/clock -v`
- `ros2 topic echo --once` sobre `/robot_description`, `/tf`, `/odom` (una muestra, con timeout)
- `ps -eo ... | grep ros/nav/...`

## 2. Procesos ROS en la máquina local

```
Solo 1 proceso ROS: /usr/bin/python3 ... ros2-daemon --ros-domain-id 5   (el daemon lo generó NUESTRA propia CLI)
```
**No hay ningún nodo del robot corriendo localmente** (ni driver `limo_base`, ni YDLiDAR, ni Cartographer, ni Nav2, ni AMCL). El proceso `nxexec` visto es NoMachine (escritorio remoto), no ROS.

## 3. Grafo ROS observado — anomalías

### 3.1 `node list` FALLA
Segunda ejecución aborta con:
```
RCLError: Failed to get node names: empty node name returned by the RMW layer, at ./src/rcl/graph.c:360
```
→ Hay un **participante DDS publicando con nombre de nodo vacío**, lo que rompe la enumeración de nodos. **Todos** los endpoints de todos los topics resuelven a `_NODE_NAME_UNKNOWN_` → no se puede atribuir ningún topic a un nodo concreto por tooling estándar.

### 3.2 DDS de vendor/serialización incompatible
`ros2 topic echo /robot_description` dispara en CycloneDDS:
```
'invalid data size, at ./src/serdata.cpp:384'
'string data is not null-terminated, at ./src/serdata.cpp:384'
```
→ Síntoma clásico de **mismatch entre vendors DDS / serialización** en el mismo dominio. Un participante ajeno (no CycloneDDS-compatible, o mal configurado) está en `ROS_DOMAIN_ID=5`. Esto degrada todo el tooling ROS2 CLI y haría poco fiable cualquier diagnóstico o navegación mientras persista.

### 3.3 Topics de un sistema AJENO al LIMO
`ros2 topic list` incluye topics que **no pertenecen al LIMO**:
```
/ainex/kick/left        /ainex/kick/right      /ainex/planar_distance
/ainex/reset_pose       /soccer/ball_confidence /soccer/ball_detection
/soccer/ball_visible    /camera/camera_info    /camera/image_raw
/clock                  /joint_states          /robot_description
```
`/ainex/*` + `/soccer/*` = robot humanoide de fútbol **"AiNex"** (Hiwonder). `/clock` con 1 publisher = **tiempo de simulación** activo en el dominio. Interpretación dominante: **hay un AiNex (muy probablemente en simulación) compartiendo `ROS_DOMAIN_ID=5`**.

## 4. Estado de topics clave (FASE 0 requerida)

| Topic | Pub | Sub | Nota |
|---|---|---|---|
| **`/cmd_vel`** | **0** | **1** | `geometry_msgs/Twist`. **0 publishers → nadie comanda movimiento ahora (OK).** Pero hay **1 suscriptor activo** (nodo no identificable) esperando comandos. |
| `/scan` | — | — | **NO EXISTE** el topic → LiDAR del LIMO NO corriendo. |
| `/odom` | 1 | 0 | `odom→base_link`. Origen no identificable (probable AiNex). |
| `/imu/data` | 1 | 0 | Origen no identificable. |
| `/tf` | 2 | 0 | `odom→base_link` en la muestra. 2 publishers. |
| `/tf_static` | — | — | Presente en la lista. |
| `/clock` | 1 | 2 | Tiempo de simulación activo. |
| `/amcl_pose` | — | — | **NO EXISTE** → AMCL no corriendo. |
| `/limo_status` | — | — | **NO EXISTE** → el driver `limo_base` (que lo publica) **no está corriendo**. |
| `/map` | — | — | **NO EXISTE** → map_server no corriendo. |

## 5. Chequeo específico de `/cmd_vel` (regla de seguridad)

- **Publisher count = 0** → **no hay ningún publisher inesperado comandando `/cmd_vel`**. Ningún actor está enviando velocidades ahora.
- **Subscription count = 1** → existe un suscriptor de `/cmd_vel` de identidad no verificable (tooling roto por el DDS ajeno). No se puede confirmar si es el driver del LIMO o el sistema AiNex.
- **Implicación:** publicar en `/cmd_vel` en este dominio podría mover *algún* robot. **PROHIBIDO publicar `/cmd_vel`** (regla del proyecto + este prompt). No se hizo ni se hará sin autorización explícita nueva.

## 6. ¿Está el LIMO corriendo? — evidencia

**Evidencia de que el stack del LIMO NO está corriendo:**
- Ausencia de `/scan` (YDLiDAR).
- Ausencia de `/limo_status` (lo publica `limo_base`).
- Ausencia de `/map`, `/amcl_pose`, `/plan`, y de nodos `controller_server`/`planner_server`/`bt_navigator`.
- Ningún proceso ROS del robot en `ps`.

**Ambigüedad no resuelta:** `/odom`, `/imu/data`, `/cmd_vel`(sub), `/tf`(odom→base_link) *podrían* teóricamente ser de un driver de base, pero por el DDS ajeno no se puede atribuir a un nodo. Dada la ausencia de `/limo_status` y `/scan`, la hipótesis más fuerte es que **estos topics son del sistema AiNex/sim, no del LIMO** (un humanoide también expone odom/imu/cmd_vel/tf).

## 7. Clasificación de hallazgos

| Hallazgo | Estado |
|---|---|
| Stack ROS del LIMO no corriendo | **CONFIRMADO** (ausencia de /scan, /limo_status, /map, /amcl_pose + sin procesos) |
| Dominio `ROS_DOMAIN_ID=5` compartido con sistema ajeno (AiNex) | **CONFIRMADO** (topics /ainex, /soccer, /clock) |
| Participante DDS de vendor incompatible en el dominio | **CONFIRMADO** (errores serdata.cpp + node list roto) |
| `/cmd_vel` sin publisher activo (nadie comanda movimiento) | **CONFIRMADO** |
| `/cmd_vel` con 1 suscriptor activo (robot "en caliente" esperando comandos) | **CONFIRMADO** (existe el sub); identidad del robot = **DESCONOCIDA** |
| Los topics odom/imu/tf son del AiNex y no del LIMO | **EVIDENCIA INDIRECTA** (fuerte, por ausencia de /limo_status y /scan) |

---

## 8. Por qué se DETIENE la auditoría live (FASES 1–10)

La instrucción FASE 0 indica: *"Si existe cualquier publisher inesperado: DETENER LA AUDITORÍA LIVE y explicarlo."* Se cumplen múltiples condiciones de parada:

1. **Publishers inesperados** en el dominio (`/ainex/*`, `/soccer/*`, `/clock`, `/odom`, `/imu/data`, `/tf`) que **no** son el stack del LIMO.
2. **DDS contaminado / incompatible** → el tooling ROS2 (node list, echo limpio) no es fiable; cualquier diagnóstico live sería de baja confianza.
3. **`/cmd_vel` con suscriptor activo de identidad desconocida** → riesgo latente si algo publicara velocidades.
4. Levantar el LIMO en `ROS_DOMAIN_ID=5` mezclaría TF (`odom→base_link` ya publicado por 2 fuentes), `/odom`, `/cmd_vel` con el sistema ajeno → **conflicto de TF y ambigüedad de comando**, inseguro para navegación.

No se lanzó ningún componente del LIMO. Las FASES 1–10 quedan **BLOQUEADAS** hasta resolver el entorno.

## 9. Decisiones que necesito del usuario (bloqueantes)

1. **¿El AiNex / la simulación de fútbol debe estar en `ROS_DOMAIN_ID=5`?** Para trabajar el LIMO limpio, lo recomendable es **aislar el dominio**: usar un `ROS_DOMAIN_ID` distinto para el LIMO (o apagar el sistema AiNex/sim). *No cambiaré variables de entorno ni configuración sin tu confirmación.*
2. **¿El robot LIMO está físicamente encendido y con motores energizados?** Estando ustedes junto al robot, conviene confirmarlo antes de energizar nada, dado que hay un suscriptor `/cmd_vel` activo en el dominio.
3. **Autorización para levantar componentes del LIMO** (driver base, YDLiDAR, TF, map_server, AMCL — todos sin movimiento) **una vez el dominio esté aislado**. Cada launch será auditado (SAFE TO START / DO NOT START) antes de ejecutarse.

---

*Precheck FASE 0. Solo lectura. No se ejecutó ningún launch, no se publicó ningún topic, no se movió el robot, no se modificó configuración ni `~/limoatwork`.*
