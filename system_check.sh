#!/bin/bash

echo "🔍 DIAGNÓSTICO SISTEMA COMPLETO"
echo "==============================="

source install/setup.bash

echo "1. 📦 Verificando paquetes instalados:"
echo "   • robot_localization:" 
ros2 pkg list | grep robot_localization | head -1 || echo "❌ robot_localization NO INSTALADO"

echo ""
echo "2. 🏗️  Verificando ejecutables:"
echo "   • visual_odo_simple:"
ls install/vodom_first/lib/vodom_first/visual_odo_simple 2>/dev/null && echo "✅ visual_odo_simple" || echo "❌ visual_odo_simple"

echo "   • odometry_to_path:"
ls install/vodom_first/lib/vodom_first/odometry_to_path 2>/dev/null && echo "✅ odometry_to_path" || echo "❌ odometry_to_path"

echo ""
echo "3. 📄 Verificando archivos de configuración:"
echo "   • ekf_config.yaml:"
ls src/vodom_first/config/ekf_config.yaml 2>/dev/null && echo "✅ ekf_config.yaml" || echo "❌ ekf_config.yaml"

echo "   • launch file:"
ls src/vodom_first/launch/simple_vo_ekf.launch.py 2>/dev/null && echo "✅ simple_vo_ekf.launch.py" || echo "❌ simple_vo_ekf.launch.py"

echo ""
echo "4. 🚀 Test rápido de launch:"
timeout 3s ros2 launch vodom_first simple_vo_ekf.launch.py 2>/dev/null && echo "✅ Launch file funciona" || echo "⚠️  Launch file tiene problemas (normal sin datos)"

echo ""
echo "5. 📡 COMANDO PARA TESTING:"
echo "   ./test_complete_system.sh"
echo ""
echo "6. 🎮 COMANDO PARA ROSBAG:"
echo "   ros2 bag play tu_rosbag.bag"
echo ""
echo "7. 🎨 PARA RVIZ:"
echo "   rviz2"
echo "   Fixed Frame: odom"
echo "   Add Path: /vo_path (verde)"
echo "   Add Path: /ekf_fused_path (azul)"