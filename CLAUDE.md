# LIMO RoboCup SML - Claude Instructions

## Contexto del proyecto

Este workspace corresponde al desarrollo del robot AgileX LIMO para RoboCup Small Size / SML.

Sistema:
- ROS2 Humble
- Navegación autónoma con Nav2
- SLAM / AMCL
- LiDAR
- Mapas conocidos
- Manipulación separada

Workspace principal de competición:
~/limo_sml_ws

Workspace base oficial:
~/limoatwork

IMPORTANTE:
~/limoatwork es INMUTABLE.
Nunca modificar, borrar, mover ni escribir archivos dentro de:
~/limoatwork

Todo código nuevo o modificado debe vivir dentro de:
~/limo_sml_ws


# REGLAS DE SEGURIDAD DEL ROBOT

## Prohibido ejecutar movimientos

Nunca ejecutar ni generar comandos que puedan mover físicamente el robot sin confirmación explícita.

Prohibido:

- ros2 topic pub /cmd_vel
- teleop_twist_keyboard
- comandos de velocidad
- MoveIt
- FollowJointTrajectory
- joint trajectory controllers
- controladores de motores
- manipulación del brazo
- scripts de movimiento físico

Nunca asumir que un launch es seguro.


## Revisión obligatoria antes de ejecutar cualquier launch

Antes de ejecutar:

ros2 launch paquete archivo.launch.py

debe:

1. Abrir y revisar el launch completo.
2. Identificar todos los nodos que inicia.
3. Revisar publishers, subscribers, servicios y acciones.
4. Confirmar si puede producir movimiento físico.
5. Solo ejecutar si no existe riesgo de movimiento.


# Comandos permitidos

Permitido ejecutar:

## Diagnóstico ROS2

- ros2 node list
- ros2 topic list
- ros2 topic info
- ros2 topic echo
- ros2 topic hz
- ros2 param list
- ros2 param get
- ros2 lifecycle get
- ros2 bag info


## Desarrollo

Permitido:

- leer código
- modificar scripts
- crear archivos
- hacer análisis
- ejecutar grep/find/sed
- colcon build
- pruebas offline


## Navegación segura

Permitido:

- análisis de Nav2
- análisis de mapas
- RViz
- SLAM
- AMCL
- costmaps

No ejecutar navegación física sin autorización.


# Optimización de tokens

Para ahorrar contexto y tokens:

- No leer carpetas build/.
- No leer carpetas install/ salvo necesidad explícita.
- No leer carpetas log/.
- No abrir archivos completos si grep, sed o búsqueda permiten obtener la información necesaria.
- Antes de explorar muchos archivos, explicar qué se busca.
- Priorizar archivos fuente sobre archivos generados.
- Mantener resúmenes de estado.
- Evitar repetir información ya conocida.


# Forma de trabajo

Antes de realizar cambios:

1. Explicar qué se va a modificar.
2. Indicar archivos afectados.
3. Evaluar riesgos.
4. Realizar cambios pequeños.
5. Verificar resultados.


# Memoria de proyecto

Mantener actualizado un resumen del estado:

- qué funciona
- qué fue probado
- qué falló
- próximos pasos

No repetir análisis ya realizados si existe información previa.


# Estructura importante

Paquetes relevantes:

- limo_bringup
- limo_base
- limo_description
- limo_slam

Archivos importantes deben mantenerse documentados.


# Estilo de respuesta

Ser preciso y técnico.

Cuando exista incertidumbre:
- no asumir
- pedir datos
- inspeccionar primero

Nunca inventar resultados de pruebas.
# Estado técnico actual del proyecto LIMO SML

## Robot

Plataforma:
- AgileX LIMO
- ROS2 Humble

Objetivo:
Navegación autónoma RoboCup SML.

Arquitectura general:

Sensores:
- LiDAR para percepción y seguridad.
- Odometría.
- TF.
- AMCL para localización.

Navegación:
- Nav2.
- Planner.
- Controller.
- Costmaps.
- Behavior Tree Navigator.

Manipulación:
Separada de navegación.
Nunca asumir que manipulación está disponible o segura.


# Restricción fundamental del proyecto

La navegación debe basarse únicamente en observables reales:

Permitido:
- /scan
- /odom
- /tf
- /tf_static
- /map
- AMCL
- costmaps

Prohibido usar para control:
- SDF de Gazebo
- Ground truth de simulación
- información perfecta del simulador

SDF solamente puede utilizarse como evaluación offline.


# Estado Nav2 actual

Nav2 funcionando:

Nodos confirmados:

- amcl
- map_server
- controller_server
- planner_server
- bt_navigator

Estado probado:

amcl:
- activo
- tf_broadcast = true
- robot_model_type:
  nav2_amcl::DifferentialMotionModel

controller_server:
- activo


# Footprint real del robot

Usar footprint real:

[
[-0.17, -0.16],
[-0.17, 0.16],
[0.22, 0.16],
[0.22, -0.16]
]

Padding:

0.02 m


Nunca modificar footprint sin justificar.


# Mapas

Resolución habitual:

0.05 m

Algunos mapas de evaluación pueden utilizar:

0.03 m


Mapas importantes:

- v1 producción
- run3 full arena

run3 fue evaluado como mejor mapa global.


# Estrategia de navegación

Modo principal:

Known map navigation.

Concepto:

1. Usar mapa conocido.
2. Planificar ruta segura.
3. Usar LiDAR para detectar cambios.
4. Si existe mismatch:
   - detener
   - analizar
   - replantear


# Hallazgos importantes

La ruta debe considerar:

- footprint completo
- clearance real
- incertidumbre de localización
- error de tracking


Pruebas previas:

Localización:
- error XY evaluado aproximadamente centímetros.

Tracking:
- error evaluado en centímetros.


No aceptar rutas solamente porque el centro del robot pasa.
Debe pasar el footprint completo.


# Workflow esperado de Claude

Cuando analice navegación:

Primero revisar:

1. mapa
2. footprint
3. costmaps
4. planner
5. controller
6. sensores


No proponer cambios sin verificar:

- archivos existentes
- parámetros actuales
- resultados de pruebas anteriores


# Historial de hitos

DEPLOY-0:
Checkpoint base del proyecto.

ROUTE-0:
Grid / A*

ROUTE-1:
Handoff de planificación.

ROUTE-1.5:
Swept SE2.

ROUTE-2:
Seguridad LiDAR.

DOCK:
Pruebas de docking aproximado.

LOCALIZATION-R1:
Ajustes AMCL.


# Próximos trabajos esperados

Prioridad:

1. Mejorar navegación segura.
2. Validar mapas.
3. Mejorar docking.
4. Integrar percepción.
5. Separar completamente percepción LEGO de navegación.


# Regla final

Ante cualquier duda:

No ejecutar.

Primero:
- analizar
- explicar riesgo
- pedir confirmación



# Zonas críticas del sistema


## Workspace protegido

Nunca modificar:

~/limoatwork

Motivo:

Es el workspace base del fabricante.
Debe considerarse solamente lectura.


## Carpetas generadas

Evitar modificar directamente:

- build/
- install/
- log/

Modificar siempre código fuente.


# Paquetes de riesgo alto

Estos paquetes pueden producir movimiento físico.

Nunca ejecutar launches sin revisión completa:

- mycobot_*
- myarm_*
- pro450_moveit2
- mycobot_*_moveit2
- ros2_control
- controller_manager


# Launch desconocidos

Si un launch contiene alguno de estos elementos:

- MoveGroup
- MoveIt
- controller_manager
- joint_trajectory_controller
- FollowJointTrajectory
- gripper
- pump
- slider_control
- teleop

Debe detenerse y pedir confirmación.


# Separación del proyecto

Navegación y manipulación son sistemas independientes.

La navegación puede trabajar con:

- LiDAR
- odometría
- AMCL
- Nav2

La manipulación no debe activarse durante pruebas de navegación.


# Ejecución autónoma prolongada

Cuando trabaje sin supervisión:

Debe priorizar:

1. análisis
2. documentación
3. propuestas
4. código seguro
5. pruebas offline

No debe realizar pruebas físicas automáticamente.


# Optimización de uso de tokens


## Antes de responder

Siempre leer primero:

1. CLAUDE.md
2. estado actual del proyecto
3. archivos relevantes


No volver a explicar información ya documentada.


## Uso eficiente de contexto

Evitar:

- repetir historia completa del proyecto
- volver a investigar decisiones ya tomadas
- leer carpetas completas sin necesidad


Preferir:

- comandos específicos
- grep
- sed
- búsqueda dirigida
- análisis incremental


## Antes de modificar código

Primero:

1. localizar archivo exacto
2. leer contexto necesario
3. explicar cambio propuesto
4. recién modificar


## Sesiones largas

Mantener un resumen incremental:

- cambios realizados
- pruebas ejecutadas
- errores encontrados
- próximos pasos


Si el contexto empieza a crecer demasiado:

crear resumen compacto y continuar desde ese resumen.


## Trabajo autónomo

Cuando se solicite avanzar sin supervisión:

Prioridad:

1. analizar
2. documentar
3. crear scripts seguros
4. ejecutar únicamente pruebas offline

No gastar tokens preguntando cosas ya definidas.


# Revisión obligatoria de launch

Antes de ejecutar cualquier archivo launch desconocido:

1. Leer completamente el archivo .launch.py.
2. Identificar todos los nodos que inicia.
3. Revisar si alguno puede:
   - publicar /cmd_vel
   - activar controladores de movimiento
   - enviar acciones al robot
   - mover articulaciones
   - activar actuadores

Si existe cualquier duda:

NO ejecutar.

Solicitar confirmación.

Está permitido levantar:
- sensores
- LiDAR
- TF
- robot_state_publisher
- map_server
- AMCL
- Nav2
- herramientas de diagnóstico

siempre que no produzcan movimiento físico.
