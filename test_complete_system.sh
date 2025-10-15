#!/bin/bash

echo "🚀 SISTEMA COMPLETO: VO + EKF + PATH VISUALIZATION"
echo "=================================================="

source install/setup.bash

echo "✅ SISTEMA INTEGRADO:"
echo "   1. 🎥 Visual Odometry (KITTI Pipeline)"
echo "      • Input: /camera/image_raw"
echo "      • Output: /odometry/visual + /vo_path"
echo ""
echo "   2. 🔗 EKF Fusion (VO + IMU)"
echo "      • Inputs: /odometry/visual + /vectornav/imu"
echo "      • Output: /odometry/filtered"
echo ""
echo "   3. 🛤️  Path Converter"
echo "      • Input: /odometry/filtered"
echo "      • Output: /ekf_fused_path"
echo ""

echo "🚀 Launching integrated system..."
ros2 launch vodom_first simple_vo_ekf.launch.py &
LAUNCH_PID=$!

sleep 5

echo ""
echo "📊 SYSTEM STATUS:"
echo "================="
echo "🔍 Active nodes:"
ros2 node list | grep -E "(visual|ekf|path)" | head -5

echo ""
echo "📡 Published topics:"
ros2 topic list | grep -E "(vo_path|ekf_fused_path|odometry)" | head -10

echo ""
echo "🎨 RVIZ SETUP:"
echo "=============="
echo "1. 🚀 Open RViz: rviz2"
echo "2. 🔧 Set Fixed Frame: 'odom'"
echo "3. ➕ Add paths:"
echo "   • /vo_path (Visual Odometry - Green)"
echo "   • /ekf_fused_path (EKF Fused - Blue)"
echo "4. 📊 Optional: Add /odometry/filtered as Odometry display"
echo ""

echo "▶️  NOW START YOUR ROSBAG:"
echo "   ros2 bag play your_rosbag.bag"
echo ""
echo "💡 EXPECTED BEHAVIOR:"
echo "   • Green path: Raw visual odometry"
echo "   • Blue path: EKF fused (smoother, more accurate)"
echo "   • Both should follow camera movement"
echo ""

echo "Press Ctrl+C to stop"

wait $LAUNCH_PID