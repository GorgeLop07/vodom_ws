import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare arguments
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RViz'
    )
    
    use_tkinter_arg = DeclareLaunchArgument(
        'use_tkinter',
        default_value='false',
        description='Whether to start Tkinter visualizer'
    )

    # Get package directory
    pkg_dir = get_package_share_directory('vodom_first')
    
    # Ground truth node (simulates GPS)
    ground_truth_node = Node(
        package='vodom_first',
        executable='ground_truth_path_node',
        name='ground_truth_path_node',
        output='screen'
    )
    
    # Visual odometry node with EKF
    visual_odometry_node = Node(
        package='vodom_first',
        executable='visual_odo_Uli',
        name='visual_odo_uli',
        output='screen',
        parameters=[],
        remappings=[]
    )
    
    # RViz2 node with 2D config
    rviz_config_path = PathJoinSubstitution([
        FindPackageShare('vodom_first'),
        'config',
        'vo_2d_visualization.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        output='screen'
    )
    
    # Tkinter visualizer node (no NumPy dependency)
    tkinter_visualizer = Node(
        package='vodom_first',
        executable='tkinter_visualizer_2d.py',
        name='tkinter_visualizer_2d',
        condition=IfCondition(LaunchConfiguration('use_tkinter')),
        output='screen'
    )

    return LaunchDescription([
        use_rviz_arg,
        use_tkinter_arg,
        ground_truth_node,
        visual_odometry_node,
        rviz_node,
        tkinter_visualizer
    ])