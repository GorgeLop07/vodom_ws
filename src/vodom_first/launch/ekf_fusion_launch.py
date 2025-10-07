from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Path to the EKF config file
    config_file = PathJoinSubstitution([
        FindPackageShare('vodom_first'),
        'config',
        'ekf_config.yaml'
    ])

    return LaunchDescription([
        # Visual Odometry Node (simplified for robot_localization)
        Node(
            package='vodom_first',
            executable='visual_odo_simple',
            name='visual_odo_simple',
            output='screen',
            parameters=[{
                'image_directory': '/home/jorgelop/Documents/VantTec_SDV_SWARM/SDV_Proyect/SDV_Software_Workspaces/vo_ws/src/vodom_first/Kitti_Sequence_Larga/image_0/'
            }]
        ),

        # Ground Truth GPS Simulation Node
        Node(
            package='vodom_first',
            executable='ground_truth_path_node',
            name='ground_truth_path_node',
            output='screen',
            parameters=[{
                'pose_file_path': '/home/jorgelop/Documents/VantTec_SDV_SWARM/SDV_Proyect/SDV_Software_Workspaces/vo_ws/src/vodom_first/Kitti_Sequence_Larga/poses.txt'
            }]
        ),

        # Extended Kalman Filter from robot_localization
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_localization_node',
            output='screen',
            parameters=[config_file],
            remappings=[
                ('odometry/filtered', '/odometry/filtered')
            ]
        ),

        # Transform publisher for map->odom
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
        )
    ])