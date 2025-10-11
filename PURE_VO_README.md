# Pure Visual Odometry with Rosbag Support

Este ejecutable implementa odometría visual pura usando ORB features + FLANN matcher + RANSAC para estimar el movimiento de la cámara desde un rosbag.

## Archivos Creados

1. **visual_odo_Uli.cpp** - Nodo principal de odometría visual pura
2. **config/camera_calib.txt** - Parámetros de calibración de tu cámara
3. **launch/pure_vo_rosbag.launch.py** - Launch file para ejecutar con rosbag
4. **config/vo_visualization.rviz** - Configuración de RViz para visualización

## Parámetros de Cámara Configurados

```
fx = 727.023491125100
fy = 726.651025866800
cx = 317.938293261900
cy = 230.951386153000
```

## Uso

### 1. Compilar el workspace
```bash
cd /home/jorgelop/Documents/VantTec_SDV_SWARM/SDV_Proyect/SDV_Software_Workspaces/vo_ws
colcon build --packages-select vodom_first
source install/setup.bash
```

### 2. Ejecutar el nodo de odometría visual
```bash
ros2 run vodom_first visual_odo_Uli
```

### 3. En otra terminal, reproducir el rosbag
```bash
ros2 bag play your_rosbag.db3 --topics /camera/image_color
```

### 4. (Opcional) Visualizar en RViz
```bash
ros2 run rviz2 rviz2 -d src/vodom_first/config/vo_visualization.rviz
```

### 5. (Alternativa) Usar el launch file
```bash
ros2 launch vodom_first pure_vo_rosbag.launch.py rosbag_path:=your_rosbag.db3 image_topic:=/camera/image_color
```

## Características Implementadas

- ✅ **Suscripción a rosbag**: Procesa imágenes en color desde rosbag .db3
- ✅ **Calibración personalizada**: Usa tus parámetros de cámara específicos  
- ✅ **Odometría visual pura**: Sin fusión EKF, solo ORB + FLANN + RANSAC
- ✅ **Publicación solo de path**: Output en `/vo_path` para visualización
- ✅ **Conversión a escala de grises**: Automática para procesamiento ORB
- ✅ **Ejecutable independiente**: No depende de ground truth

## Tópicos ROS2

### Entrada
- `/camera/image_color` (sensor_msgs/Image) - Imágenes del rosbag

### Salida  
- `/vo_path` (nav_msgs/Path) - Trayectoria de odometría visual

## Algoritmo

1. **Recepción de imagen**: Convierte de BGR a escala de grises
2. **Detección ORB**: 8000 features por frame
3. **Matching FLANN**: LSH index con ratio test 0.7
4. **Essential Matrix**: RANSAC para estimación robusta
5. **Pose Recovery**: Extrae movimiento 2D (dx, dz, dyaw)
6. **Actualización global**: Transforma a coordenadas del mapa
7. **Publicación**: Path incremental en frame "map"

## Notas Importantes

- El nodo espera imágenes en `/camera/image_color`
- La calibración debe estar en formato 3x4 matrix
- El scale factor puede necesitar ajuste según tu aplicación
- Para mejores resultados, asegúrate de que las imágenes tengan suficientes features

## Troubleshooting

- Si no hay matches: Verifica la calidad de las imágenes
- Si el path se ve muy ruidoso: Ajusta el scale_factor en el código
- Si no recibe imágenes: Verifica el nombre del tópico con `ros2 topic list`
