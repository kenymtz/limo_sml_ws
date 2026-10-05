#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    ydlidar_share = get_package_share_directory(
        'ydlidar_ros2_driver'
    )

    ydlidar_launch = os.path.join(
        ydlidar_share,
        'launch',
        'ydlidar_launch.py'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ydlidar_launch),
            launch_arguments={
                'base_frame': 'base_link',
                'lidar_frame': 'laser_frame',

                # Geometria medida en el LIMO real
                'lidar_x': '0.180',
                'lidar_y': '0.000',
                'lidar_z': '-0.090',

                'lidar_roll': '0.0',
                'lidar_pitch': '0.0',
                'lidar_yaw': '0.0',
            }.items()
        )
    ])
