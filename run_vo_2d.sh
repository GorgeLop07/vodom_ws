#!/bin/bash

# Script para ejecutar visual odometry 2D con diferentes visualizadores

echo "=== Visual Odometry 2D - VantTec SDV ==="
echo ""
echo "Selecciona el tipo de visualización:"
echo "1) Solo RViz2 (recomendado para desarrollo)"
echo "2) Solo Tkinter (ligero, sin dependencias adicionales)"
echo "3) Tkinter + RViz2 (mejor opción para depuración)"
echo "4) Solo ejecutar VO (sin visualización)"
echo ""

read -p "Ingresa tu opción (1-4): " choice

cd /home/jorgelop/Documents/VantTec_SDV_SWARM/SDV_Proyect/SDV_Software_Workspaces/vo_ws

# Source el workspace
source install/setup.bash

case $choice in
    1)
        echo "Ejecutando con RViz2..."
        ros2 launch vodom_first vo_2d_visualization.launch.py use_rviz:=true
        ;;
    2) 
        echo "Ejecutando con Tkinter..."
        ros2 launch vodom_first vo_2d_visualization.launch.py use_tkinter:=true
        ;;
    3)
        echo "Ejecutando con Tkinter + RViz2..."
        ros2 launch vodom_first vo_2d_visualization.launch.py use_rviz:=true use_tkinter:=true
        ;;
    4)
        echo "Ejecutando solo Visual Odometry..."
        ros2 run vodom_first visual_odo_Uli
        ;;
    *)
        echo "Opción inválida. Ejecutando con Tkinter por defecto..."
        ros2 launch vodom_first vo_2d_visualization.launch.py use_tkinter:=true
        ;;
esac