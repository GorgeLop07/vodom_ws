#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    rosbag_path_arg = DeclareLaunchArgument(
        'rosbag_path',
        default_value='',
        description='Path to the rosbag file (.db3)'
    )
    
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/image_color',
        description='Image topic name in the rosbag'
    )
    
    # Get launch configuration
    rosbag_path = LaunchConfiguration('rosbag_path')
    image_topic = LaunchConfiguration('image_topic')
    
    return LaunchDescription([
        rosbag_path_arg,
        image_topic_arg,
        
        # Pure Visual Odometry Node
        Node(
            package='vodom_first',
            executable='visual_odo_Uli',
            name='pure_visual_odometry',
            output='screen',
            parameters=[],
            remappings=[
                ('/camera/image_color', image_topic),
            ]
        ),
        
        # RViz2 for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(os.path.dirname(__file__), '..', 'config', 'vo_visualization.rviz')],
            condition=None  # Always launch RViz
        ),
        
        # Rosbag playback (only if rosbag_path is provided)
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', rosbag_path, '--topics', image_topic],
            output='screen',
            condition=None  # User will run this manually
        )
    ])