#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Get package share directory
    pkg_share = get_package_share_directory('vodom_first')
    
    # 1. Visual Odometry Simple (solo cámara)
    visual_odometry_node = Node(
        package='vodom_first',
        executable='visual_odo_simple',
        name='visual_odo_simple',
        output='screen'
    )
    
    # 2. EKF que toma VO + IMU
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(pkg_share, 'config', 'ekf_config.yaml')
        ],
        remappings=[
            ('/imu/data', '/vectornav/imu'),          # IMU input
            ('/odometry/filtered', '/vo_ekf_path')     # EKF output
        ]
    )
    
    # 3. Converter: EKF odometry → path para visualización
    ekf_to_path_node = Node(
        package='vodom_first',
        executable='odometry_to_path',
        name='ekf_path_converter',
        output='screen'
    )
    
    return LaunchDescription([
        visual_odometry_node,
        ekf_node,
        ekf_to_path_node
    ])