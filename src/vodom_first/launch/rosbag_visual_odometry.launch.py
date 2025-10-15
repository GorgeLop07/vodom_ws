#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Get package share directory
    pkg_share = get_package_share_directory('vodom_first')
    
    # Declare launch arguments
    rosbag_path_arg = DeclareLaunchArgument(
        'rosbag_path',
        default_value='',
        description='Path to the rosbag file with /camara/image_raw and /vectornav/imu topics'
    )
    
    # Visual Odometry Simple Node (produces /vo_odometry)
    visual_odometry_node = Node(
        package='vodom_first',
        executable='visual_odo_simple',
        name='visual_odo_simple',
        output='screen',
        parameters=[
            # Add any parameters here if needed
        ],
        remappings=[
            # Remap to provide odometry for EKF
            ('/vo_path', '/vo_odometry_path')
        ]
    )
    
    # EKF Node - Fuses Visual Odometry + IMU
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(pkg_share, 'config', 'ekf_config.yaml')
        ],
        remappings=[
            # Map IMU topic from VectorNav
            ('/imu/data', '/vectornav/imu'),
            # Map visual odometry input
            ('/odometry/visual', '/vo_odometry'),
            # Output fused odometry
            ('/odometry/filtered', '/odometry/ekf_fused')
        ]
    )
    
    # Path Recorder Node (to save the fused trajectory)
    path_recorder_node = Node(
        package='vodom_first',
        executable='path_recorder',
        name='path_recorder',
        output='screen',
        parameters=[
            {'output_file': '/tmp/vo_imu_fused_path.txt'},
            {'topic_names': ['/odometry/ekf_fused']},
            {'topic_types': ['nav_msgs/msg/Odometry']}
        ]
    )
    
    return LaunchDescription([
        rosbag_path_arg,
        visual_odometry_node,
        ekf_node,
        path_recorder_node
    ])