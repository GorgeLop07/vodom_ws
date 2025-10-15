#!/bin/bash

echo "🚀 LAUNCHING COMPLETE VO + EKF + PATH SYSTEM"
echo "=============================================="

# Source the workspace
source install/setup.bash

echo "📋 System Components:"
echo "  📷 Visual Odometry: /camera/image_raw → /odometry/visual"
echo "  🧭 IMU Input: /vectornav/imu"
echo "  🔧 EKF Fusion: VO + IMU → /vo_ekf_path"
echo "  🛤️  Path Visualization: /vo_path"
echo ""
echo "🎯 Expected Input Topics:"
echo "  - /camera/image_raw (sensor_msgs/msg/Image)"
echo "  - /vectornav/imu (sensor_msgs/msg/Imu)"
echo ""
echo "📡 Output Topics:"
echo "  - /odometry/visual (nav_msgs/msg/Odometry) - for EKF"
echo "  - /vo_path (nav_msgs/msg/Path) - VO path for RViz"
echo "  - /vo_ekf_path (nav_msgs/msg/Odometry) - fused odometry"
echo "  - /ekf_fused_path (nav_msgs/msg/Path) - fused path for RViz"
echo ""

# Launch the complete system
echo "🚀 Starting VO + EKF system..."
ros2 launch vodom_first simple_vo_ekf.launch.py &
LAUNCH_PID=$!

echo ""
echo "🎮 NEXT STEPS:"
echo "1. ▶️  Run rosbag: ros2 bag play your_rosbag.bag"
echo "2. 🔍 Check VO path: ros2 topic echo /vo_path"
echo "3. 🔍 Check EKF fused path: ros2 topic echo /ekf_fused_path"
echo "4. 📊 Visualize in RViz:"
echo "   - Run: rviz2"
echo "   - Add > Path (Topic: /vo_path) - Raw VO"
echo "   - Add > Path (Topic: /ekf_fused_path) - Fused VO+IMU"
echo "   - Fixed Frame: odom"
echo ""
echo "Press Ctrl+C to stop the system"

wait $LAUNCH_PID