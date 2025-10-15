# 🚗 Visual Odometry + EKF + GPS Fusion System

Sistema integrado de odometría visual con fusión EKF y **referencia de escala GPS** para navegación robótica.

## 📋 Componentes

### 1. 🎥 Visual Odometry (`visual_odo_simple`)
- **Pipeline**: KITTI visual odometry (ORB features + Essential Matrix)
- **Input**: `/camera/image_raw` (sensor_msgs/Image)
- **Outputs**: 
  - `/odometry/visual` (nav_msgs/Odometry) → Para EKF (forma de trayectoria)
  - `/vo_path` (nav_msgs/Path) → Para visualización

# 🚗 Visual Odometry + EKF Fusion System

Sistema integrado de odometría visual con fusión EKF para navegación robótica.

## 📋 Componentes

### 1. 🎥 Visual Odometry (`visual_odo_simple`)
- **Pipeline**: KITTI visual odometry (ORB features + Essential Matrix)
- **Input**: `/camera/image_raw` (sensor_msgs/Image)
- **Outputs**: 
  - `/odometry/visual` (nav_msgs/Odometry) → Para EKF
  - `/vo_path` (nav_msgs/Path) → Para visualización

### 2. 🔗 EKF Fusion (`ekf_node`)
- **Package**: `robot_localization`
- **Inputs**: 
  - `/odometry/visual` (Visual Odometry)
  - `/vectornav/imu` (IMU data)
- **Output**: `/odometry/filtered` (nav_msgs/Odometry)

### 3. 🛤️ Path Converter (`odometry_to_path`)
- **Input**: `/odometry/filtered` (nav_msgs/Odometry)
- **Output**: `/ekf_fused_path` (nav_msgs/Path) → Para RViz

### 3. �🔗 EKF Fusion (`ekf_node`)
- **Package**: `robot_localization`
- **Inputs**: 
  - `/odometry/visual` (VO - forma y orientación)
  - `/gps/pose` (GPS - **escala absoluta** 🎯)
  - `/vectornav/imu` (IMU - estabilidad angular)
- **Output**: `/odometry/filtered` (nav_msgs/Odometry)

### 4. 🛤️ Path Converter (`odometry_to_path`)
- **Input**: `/odometry/filtered` (nav_msgs/Odometry)
- **Output**: `/ekf_fused_path` (nav_msgs/Path) → Para RViz

## 🚀 Uso

### Verificar sistema:
```bash
./system_check.sh
```

### Ejecutar sistema completo:
```bash
./test_complete_system.sh
```

### En otra terminal - Rosbag:
```bash
ros2 bag play tu_rosbag.bag
```

### RViz:
```bash
rviz2
```
- Fixed Frame: `odom`
- Add Path: `/vo_path` (verde - VO puro)
- Add Path: `/ekf_fused_path` (azul - EKF fusionado)

## 📊 Topics

| Topic | Tipo | Descripción |
|-------|------|-------------|
| `/camera/image_raw` | sensor_msgs/Image | Input de cámara |
| `/vectornav/imu` | sensor_msgs/Imu | Input de IMU |
| `/odometry/visual` | nav_msgs/Odometry | VO para EKF |
| `/odometry/filtered` | nav_msgs/Odometry | EKF fusionado |
| `/vo_path` | nav_msgs/Path | Path VO (visualización) |
| `/ekf_fused_path` | nav_msgs/Path | Path EKF (visualización) |

## ⚙️ Configuración

### Calibración de cámara:
- Ubicación: `visual_odo_simple.cpp` → `loadCameraCalibration()`
- Valores actuales: fx=1092.84, fy=1091.40, cx=631.80, cy=349.32

### EKF:
- Archivo: `config/ekf_config.yaml`
- VO: Alta confianza en posición y orientación
- IMU: Velocidades angulares y estabilización

## 🔧 Compilación

```bash
colcon build --packages-select vodom_first
source install/setup.bash
```

## 📝 Notas

- Sistema optimizado para el pipeline KITTI que funcionó bien
- EKF configurado para dar prioridad a VO sobre IMU en orientación
- Transform broadcaster incluido para visualización en RViz
- Scale fijo (0.1) para estabilidad

---

## 🚗 Historia del desarrollo

### Versiones anteriores (Docker)
Este workspace inicialmente usaba Docker. Para referencia histórica:

```bash
# Construcción de imagen Docker (deprecado)
sudo docker build -t vodom_ws_humble .
sudo docker run -it vodom_ws_humble
```

### Nodos obsoletos (removidos)
- `ground_truth_path_node` - Para comparación con ground truth KITTI
- `visual_odo_Uli` - Versión anterior del pipeline VO
- Launch files múltiples - Ahora todo integrado en `simple_vo_ekf.launch.py`