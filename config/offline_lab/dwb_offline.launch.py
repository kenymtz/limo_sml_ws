"""NAV-OFFLINE-R4-A — DWB real de Nav2, OFFLINE. EJECUTAR SOLO CON ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=99.

mode:=server  (verificación DWB CONFIG con el controller_server REAL)
  map_server, controller_server (cmd_vel REMAPEADO a /offline_lab/cmd_vel_sink, sin suscriptores),
  static TF map->odom y odom->base_link (identidad), lifecycle_manager_offline [map_server, controller_server].
  NUNCA se envía un goal FollowPath: el servidor solo se configura/activa (sin ciclo de control).

mode:=closedloop  (NAV-OFFLINE-R4-B, simulador cinemático en lazo cerrado)
  map_server (frame_id map_true), lifecycle_manager_offline [map_server], dwb_closed_loop_sim (limo_nav_offline_lab).
  SIN TF estáticas (el simulador escribe TF en su propio buffer), SIN publisher Twist, SIN acciones.

mode:=eval    (evaluación de paths)
  map_server, static TF map->odom, lifecycle_manager_offline [map_server],
  dwb_offline_eval (limo_nav_offline_lab): clase real dwb_core::DWBLocalPlanner + Costmap2DROS real
  llamados directamente; SIN publisher Twist; publica solo /tf odom->base_link del caso evaluado.

NO incluye: bt_navigator, planner, behavior_server, velocity_smoother, AMCL, robot_state_publisher,
limo_base, drivers, sensores, teleop, MoveIt, controller_manager.
"""
import os
import tempfile

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    if os.environ.get('ROS_LOCALHOST_ONLY') != '1' or os.environ.get('ROS_DOMAIN_ID') != '99':
        raise RuntimeError('dwb_offline.launch.py requiere ROS_LOCALHOST_ONLY=1 y ROS_DOMAIN_ID=99')
    mode = LaunchConfiguration('mode').perform(context)
    params = LaunchConfiguration('params_file').perform(context)
    map_yaml = LaunchConfiguration('map').perform(context)
    lm_nodes = ['map_server']
    acts = [
        Node(package='nav2_map_server', executable='map_server', name='map_server', output='log',
             parameters=[params, {'yaml_filename': map_yaml}]),
    ]
    if mode != 'closedloop':
        acts.append(Node(package='tf2_ros', executable='static_transform_publisher', name='static_tf_map_odom',
                         output='log', arguments=['--frame-id', 'map', '--child-frame-id', 'odom']))
    if mode == 'server':
        lm_nodes.append('controller_server')
        acts += [
            Node(package='nav2_controller', executable='controller_server', name='controller_server',
                 output='log', parameters=[params],
                 remappings=[('cmd_vel', '/offline_lab/cmd_vel_sink')]),
            Node(package='tf2_ros', executable='static_transform_publisher', name='static_tf_odom_base',
                 output='log', arguments=['--frame-id', 'odom', '--child-frame-id', 'base_link']),
        ]
    elif mode == 'eval':
        # Mismos parámetros que controller_server, bajo el nombre del nodo evaluador (copia temporal).
        doc = yaml.safe_load(open(params))
        doc['dwb_offline_eval'] = doc.pop('controller_server')
        tmp = tempfile.NamedTemporaryFile('w', prefix='dwb_eval_params_', suffix='.yaml', delete=False)
        yaml.safe_dump(doc, tmp)
        tmp.close()
        acts.append(
            Node(package='limo_nav_offline_lab', executable='dwb_offline_eval', name='dwb_offline_eval',
                 output='log', parameters=[tmp.name, {
                     'cases_file': LaunchConfiguration('cases_file').perform(context),
                     'out_file': LaunchConfiguration('out_file').perform(context),
                     'settle_ms': int(LaunchConfiguration('settle_ms').perform(context))}]))
    elif mode == 'closedloop':
        import json
        doc = yaml.safe_load(open(params))
        doc['dwb_closed_loop_sim'] = doc.pop('controller_server')
        tmp = tempfile.NamedTemporaryFile('w', prefix='dwb_sim_params_', suffix='.yaml', delete=False)
        yaml.safe_dump(doc, tmp)
        tmp.close()
        extra = json.loads(LaunchConfiguration('sim_params').perform(context) or '{}')
        extra.update({'runs_file': LaunchConfiguration('cases_file').perform(context),
                      'out_file': LaunchConfiguration('out_file').perform(context)})
        acts.append(
            Node(package='limo_nav_offline_lab', executable='dwb_closed_loop_sim', name='dwb_closed_loop_sim',
                 output='log', parameters=[tmp.name, extra]))
    else:
        raise RuntimeError(f'mode desconocido: {mode}')
    acts.append(Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                     name='lifecycle_manager_offline', output='log',
                     parameters=[{'use_sim_time': False, 'autostart': True, 'node_names': lm_nodes}]))
    return acts


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mode', description='server | eval'),
        DeclareLaunchArgument('map', description='yaml del mapa validado'),
        DeclareLaunchArgument('params_file', description='config/offline_lab/dwb_offline_R4A*.yaml'),
        DeclareLaunchArgument('cases_file', default_value=''),
        DeclareLaunchArgument('out_file', default_value=''),
        DeclareLaunchArgument('settle_ms', default_value='300'),
        DeclareLaunchArgument('sim_params', default_value='{}', description='JSON con parámetros del escenario R4-B'),
        OpaqueFunction(function=_setup),
    ])
