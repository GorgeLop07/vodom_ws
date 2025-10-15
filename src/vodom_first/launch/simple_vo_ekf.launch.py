#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Get package share directory
    pkg_share = get_package_share_directory('vodom_first')
    
    # Path to EKF config file
    ekf_config_file = os.path.join(pkg_share, 'config', 'ekf_config.yaml')
    
    return LaunchDescription([
        # Launch argument for simulation time
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        
        # 1. Visual Odometry Node (KITTI Pipeline)
        Node(
            package='vodom_first',
            executable='visual_odo_simple',
            name='visual_odometry',
            output='screen',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
            remappings=[
                # Output for EKF fusion
                ('/odometry/visual', '/odometry/visual'),
                # Output for visualization
                ('/vo_path', '/vo_path')
            ]
        ),
        
        # 2. EKF Localization Node (Fuses VO + IMU)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                ekf_config_file,
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
            remappings=[
                # IMU input from VectorNav
                ('/imu0', '/vectornav/imu'),
                # VO input from visual odometry
                ('/odom0', '/odometry/visual'),
                # Fused output
                ('/odometry/filtered', '/odometry/filtered')
            ]
        ),

        # 3. EKF Path Converter (For RViz visualization)
        Node(
            package='vodom_first',
            executable='odometry_to_path',
            name='ekf_path_converter',
            output='screen',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
            remappings=[
                # Input from EKF
                ('/input_odom', '/odometry/filtered'),
                # Output path for RViz
                ('/output_path', '/ekf_fused_path')
            ]
        )
    ])