from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Package directory
    pkg_dir = get_package_share_directory('vodom_first')
    
    # Visual Odometry 2D Node
    visual_odometry_node = Node(
        package='vodom_first',
        executable='visual_odo_Uli',
        name='visual_odometry_2d',
        output='screen',
        parameters=[{
            'sequence_path': 'src/vodom_first/Kitti_Sequence_Larga',
            'frame_id': 'map'
        }]
    )
    
    # Ground Truth 2D Node
    ground_truth_node = Node(
        package='vodom_first',
        executable='ground_truth_2d_node',
        name='ground_truth_2d',
        output='screen',
        parameters=[{
            'poses_file': 'src/vodom_first/Kitti_Sequence_Larga/poses.txt',
            'frame_id': 'map'
        }]
    )
    
    # RViz Node
    rviz_config = os.path.join(pkg_dir, 'config', 'vo_2d_comparison.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )
    
    return LaunchDescription([
        visual_odometry_node,
        ground_truth_node,
        rviz_node
    ])