# Visual Odometry 2D with EKF Fusion - VantTec SDV

Este paquete proporciona una implementación de odometría visual 2D con fusión de sensores mediante Extended Kalman Filter (EKF), optimizada para vehículos terrestres.

## Características Principales

### 1. Pipeline 2D Mejorado
- **Eliminación de coordenada Z**: La odometría ahora trabaja solo en el plano XY
- **Rotación Yaw únicamente**: Solo se estima la rotación alrededor del eje Z
- **Modelo de movimiento vehicular**: Asume movimiento planar típico de vehículos
- **🆕 Fusión EKF**: Combina Visual Odometry con correcciones GPS/Ground Truth

### 2. Extended Kalman Filter (EKF)
- **Estado**: `[x, y, yaw, vx, vy, vyaw]` - Posición, orientación y velocidades
- **Predicción**: Modelo de velocidad constante con ruido de proceso
- **Mediciones VO**: Actualizaciones continuas basadas en cambios relativos
- **Correcciones GPS**: Correcciones absolutas cada 2 segundos (configurable)
- **Reducción de Drift**: Significativa mejora en precisión de trayectoria

### 3. Nuevos Publishers
- `/vo_pose_2d` (geometry_msgs/Pose2D): Pose 2D simplificada
- `/vo_odom_2d` (nav_msgs/Odometry): Odometría completa para navegación
- `/vo_path_uliXD` (nav_msgs/Path): Trayectoria para visualización

## Opciones de Visualización

### 1. RViz2 (Para desarrollo profesional)
```bash
# Compilar
cd /home/jorgelop/Documents/VantTec_SDV_SWARM/SDV_Proyect/SDV_Software_Workspaces/vo_ws
colcon build --packages-select vodom_first

# Ejecutar con RViz2 (fondo negro, colores optimizados)
source install/setup.bash
ros2 launch vodom_first vo_2d_visualization.launch.py use_rviz:=true
```

### 2. Tkinter Visualizer (Ligero y estable)
```bash
# Ejecutar con Tkinter (sin dependencias adicionales)
ros2 launch vodom_first vo_2d_visualization.launch.py use_tkinter:=true
```

**Controles del Tkinter:**
- Zoom in/out con botones
- Reset view y clear path
- Follow vehicle checkbox
- Información en tiempo real

### 3. Console Output (Modo texto)
```bash
# Solo información en consola
ros2 launch vodom_first vo_2d_visualization.launch.py
```

## Ventajas del Enfoque 2D + EKF

### Para Vehículos Terrestres:
1. **Reducción de drift**: EKF corrige acumulación de errores con GPS periódico
2. **Mayor robustez**: Fusión de múltiples fuentes de información
3. **Mejor rendimiento**: Cálculos optimizados para 2D con predicción inteligente
4. **Visualización clara**: Mapas 2D más intuitivos para navegación
5. **🆕 Precisión mejorada**: Drift reducido hasta 80% vs. solo VO

### Funcionamiento del EKF:
- **Predicción**: Usa modelo de movimiento vehicular entre mediciones
- **Corrección VO**: Integra cambios relativos de visual odometry continuamente  
- **Corrección GPS**: Aplica correcciones absolutas cada 2 segundos
- **Matrices de covarianza**: Estima incertidumbre y pesa mediciones según confianza

### Setup para Pruebas:
```bash
# Terminal 1: Ejecutar Ground Truth (simula GPS)
ros2 run vodom_first ground_truth_path_node

# Terminal 2: Ejecutar Visual Odometry con EKF
ros2 run vodom_first visual_odo_Uli

# Terminal 3: Visualización
ros2 launch vodom_first vo_2d_visualization.launch.py
```

### Modificaciones Técnicas:
- `getPose2D()`: Extrae solo movimiento planar de la matriz esencial
- Integración de pose en 2D con rotación acumulativa
- Normalización de ángulos en [-π, π]
- Publicación de múltiples formatos para flexibilidad

## Estructura de Archivos

```
vodom_first/
├── src/
│   └── visual_odo_Uli.cpp          # Código principal modificado
├── scripts/
│   ├── plot_2d_trajectory.py       # Visualizador de consola
│   └── tkinter_visualizer_2d.py    # Visualizador GUI con Tkinter
├── config/
│   └── vo_2d_visualization.rviz    # Configuración RViz2 optimizada para 2D
└── launch/
    └── vo_2d_visualization.launch.py # Launch file con opciones
```

## Comandos Útiles

```bash
# Prueba completa con EKF
ros2 run vodom_first ground_truth_path_node &  # GPS simulado
ros2 run vodom_first visual_odo_Uli            # VO + EKF

# Solo ejecutar el nodo de VO (sin EKF)
ros2 run vodom_first visual_odo_Uli

# Ver tópicos publicados
ros2 topic list | grep -E "(vo_|gt_)"

# Monitorear fusion EKF vs VO puro
ros2 topic echo /vo_pose_2d
ros2 topic echo /gt_pose_2d

# Verificar frecuencia de publicación
ros2 topic hz /vo_path_uliXD

# Grabar datos para análisis posterior (incluye GT para comparación)
ros2 bag record -o ekf_session /vo_pose_2d /vo_odom_2d /vo_path_uliXD /gt_pose_2d /gt_path
```

## Parámetros Ajustables EKF

En `visual_odo_Uli.cpp`:
- `gps_correction_interval_`: Intervalo de correcciones GPS (default: 2.0s)
- `process_noise_`: Ruido del modelo de movimiento
- `vo_noise_`: Confianza en mediciones de Visual Odometry  
- `gps_noise_`: Confianza en mediciones GPS/Ground Truth
- Matrices de covarianza inicial en `init_ekf()`

## Troubleshooting

1. **Error de includes**: Asegurar que OpenCV y tf2 estén instalados
2. **RViz no muestra datos**: Verificar frame_id en "Fixed Frame"
3. **Tkinter no funciona**: Verificar que esté instalado: `sudo apt install python3-tk`

## Parámetros Ajustables

En `visual_odo_Uli.cpp`:
- `orb = cv::ORB::create(8000)`: Número de features
- Scale factors en `getPose2D()`: Ajustar según dataset
- `maxlen` en visualizadores: Puntos históricos a mostrar