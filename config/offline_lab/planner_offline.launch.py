"""NAV-OFFLINE-R2 — planner Nav2 REAL, mínimo y offline, solo para ComputePathToPose.

EJECUTAR SOLO CON:  ROS_LOCALHOST_ONLY=1  ROS_DOMAIN_ID=99   (el launch aborta si no)

Nodos (y únicamente estos):
  map_server                (nav2_map_server)  -> publica /map (OccupancyGrid)
  planner_server            (nav2_planner)     -> acción /compute_path_to_pose (+ through_poses),
                                                  global_costmap interno (static + inflation), publica /plan
  static_tf_map_odom        (tf2_ros)          -> TF estática map->odom (identidad)
  static_tf_odom_base       (tf2_ros)          -> TF estática odom->base_link (identidad; requerida por el costmap)
  lifecycle_manager_offline (nav2_lifecycle_manager) -> configura/activa map_server y planner_server

NO incluye: controller_server, bt_navigator, behavior_server, smoother, velocity_smoother,
waypoint_follower, AMCL, limo_base, drivers, teleop. Ningún nodo publica /cmd_vel.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _guard(context, *args, **kwargs):
    if os.environ.get('ROS_LOCALHOST_ONLY') != '1' or os.environ.get('ROS_DOMAIN_ID') != '99':
        raise RuntimeError('planner_offline.launch.py requiere ROS_LOCALHOST_ONLY=1 y ROS_DOMAIN_ID=99')
    return []


def generate_launch_description():
    map_yaml = LaunchConfiguration('map')
    params = LaunchConfiguration('params_file')
    return LaunchDescription([
        DeclareLaunchArgument('map', description='yaml del mapa (validado)'),
        DeclareLaunchArgument('params_file', description='config/offline_lab/planner_offline_{A,B}.yaml'),
        OpaqueFunction(function=_guard),
        Node(package='nav2_map_server', executable='map_server', name='map_server', output='log',
             parameters=[params, {'yaml_filename': map_yaml}]),
        Node(package='nav2_planner', executable='planner_server', name='planner_server', output='log',
             parameters=[params]),
        Node(package='tf2_ros', executable='static_transform_publisher', name='static_tf_map_odom', output='log',
             arguments=['--frame-id', 'map', '--child-frame-id', 'odom']),
        Node(package='tf2_ros', executable='static_transform_publisher', name='static_tf_odom_base', output='log',
             arguments=['--frame-id', 'odom', '--child-frame-id', 'base_link']),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_offline',
             output='log',
             parameters=[{'use_sim_time': False, 'autostart': True,
                          'node_names': ['map_server', 'planner_server']}]),
    ])
