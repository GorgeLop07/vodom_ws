#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, LogInfo
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
        
        # System startup info
        LogInfo(msg="🚀 Starting Complete VO + IMU + GT System"),
        LogInfo(msg="=========================================="),
        LogInfo(msg="📍 GT Publisher: LIO-SAM poses → /gt_pose"),
        LogInfo(msg="🎥 Visual Odometry: Camera → /odometry/visual"),
        LogInfo(msg="🤖 EKF Filter: Fusion → /odometry/filtered"),
        LogInfo(msg="🛤️  Path Visualization: Multiple trajectory topics"),
        LogInfo(msg=""),
        LogInfo(msg="💡 After launch: ros2 bag play your_rosbag.bag"),
        LogInfo(msg="💡 Visualization: rviz2"),
        LogInfo(msg="=========================================="),
        
        # 1. GT Pose Publisher (LIO-SAM poses) - START FIRST
        Node(
            package='vodom_first',
            executable='gt_pose_publisher',
            name='gt_pose_publisher',
            output='screen',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
            remappings=[
                # Output for EKF fusion
                ('/gt_pose', '/gt_pose'),
                # Output for visualization
                ('/gt_path', '/gt_path')
            ],
            # Add logging prefix
            arguments=['--ros-args', '--log-level', 'INFO']
        ),
        
        # 2. Visual Odometry Node (KITTI Pipeline) - START AFTER 2 SECONDS
        TimerAction(
            period=2.0,
            actions=[
                LogInfo(msg="🎥 Starting Visual Odometry..."),
                Node(
                    package='vodom_first',
                    executable='visual_odo_simple',
                    name='visual_odometry',
                    output='screen',
                    parameters=[
                        {'use_sim_time': LaunchConfiguration('use_sim_time')}
                    ],
                    remappings=[
                        # Input from camera
                        ('/camera/image_raw', '/camera/image_raw'),
                        # Output for EKF fusion
                        ('/odometry/visual', '/odometry/visual'),
                        # Output for visualization
                        ('/vo_path', '/vo_path')
                    ],
                    arguments=['--ros-args', '--log-level', 'INFO']
                )
            ]
        ),
        
        # 3. EKF Localization Node (Fuses VO + IMU + GT) - START AFTER 4 SECONDS
        TimerAction(
            period=4.0,
            actions=[
                LogInfo(msg="🤖 Starting EKF Filter..."),
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
                        # GT input from LIO-SAM
                        ('/pose0', '/gt_pose'),
                        # Fused output
                        ('/odometry/filtered', '/odometry/filtered')
                    ],
                    arguments=['--ros-args', '--log-level', 'INFO']
                )
            ]
        ),

        # 4. EKF Path Converter (For RViz visualization) - START AFTER 5 SECONDS
        TimerAction(
            period=5.0,
            actions=[
                LogInfo(msg="🛤️  Starting Path Converters..."),
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
                    ],
                    arguments=['--ros-args', '--log-level', 'WARN']
                )
            ]
        ),

        # 5. Path Recorder (Optional - for saving trajectories) - START AFTER 6 SECONDS
        TimerAction(
            period=6.0,
            actions=[
                LogInfo(msg="📊 Starting Trajectory Recorder..."),
                Node(
                    package='vodom_first',
                    executable='path_recorder',
                    name='trajectory_recorder',
                    output='screen',
                    parameters=[
                        {'use_sim_time': LaunchConfiguration('use_sim_time')}
                    ],
                    remappings=[
                        # Record EKF fused path
                        ('/input_path', '/ekf_fused_path')
                    ],
                    arguments=['--ros-args', '--log-level', 'WARN']
                )
            ]
        ),

        # Final system ready message
        TimerAction(
            period=8.0,
            actions=[
                LogInfo(msg=""),
                LogInfo(msg="✅ Complete System Ready!"),
                LogInfo(msg="🎯 Topics Available:"),
                LogInfo(msg="   /gt_pose - Ground truth from LIO-SAM"),
                LogInfo(msg="   /odometry/visual - Visual odometry output"),
                LogInfo(msg="   /odometry/filtered - EKF fused result (MAIN OUTPUT)"),
                LogInfo(msg="   /vo_path - VO trajectory"),
                LogInfo(msg="   /gt_path - GT trajectory"),
                LogInfo(msg="   /ekf_fused_path - Final fused trajectory"),
                LogInfo(msg=""),
                LogInfo(msg="🎮 Next Steps:"),
                LogInfo(msg="   1. ros2 bag play your_rosbag.bag"),
                LogInfo(msg="   2. rviz2 (visualize trajectories)"),
                LogInfo(msg="   3. ros2 topic hz /odometry/filtered (monitor fusion)"),
                LogInfo(msg=""),
                LogInfo(msg="🎉 Ready for GT-scaled visual odometry!")
            ]
        )
    ])