# Guía para correr el workspace de ROS2 con Docker

## 1. Clona el repositorio en tu directorio home

```bash
cd ~
git clone https://github.com/GorgeLop07/vodom_ws.git
```

## 2. Entra al workspace

```bash
cd vodom_ws
```

## 3. Construye la imagen de Docker

```bash
sudo docker build -t vodom_ws_humble .
```

## 4. Corre el contenedor de Docker

```bash
sudo docker run -it vodom_ws_humble
```

---

## Opciones para correr dentro del Docker

### 1. Ejecutar el nodo de ground truth path

```bash
ros2 run vodom_first ground_truth_path_node
```

### 2. Ejecutar el nodo de visual odometry

```bash
ros2 run vodom_first visual_odo_Uli
```

### 3. Visualizar los resultados en RViz

- Abre una terminal nueva (fuera del Docker).
- Fuentea tu instalación de ROS2 Humble:
	```bash
	source /opt/ros/humble/setup.bash
	```
- Ejecuta RViz:
	```bash
	rviz2
	```
- Suscríbete a los tópicos publicados por los nodos (`/gt_path`, `/vo_path_uliXD`, etc.) para visualizar los resultados.

---

> **Nota:**  
> Si necesitas pasar parámetros personalizados (por ejemplo, la ruta de los archivos de datos), puedes hacerlo usando:
> ```bash
> ros2 run vodom_first ground_truth_path_node --ros-args -p poses_file:=src/vodom_first/KITTI_sequence_2/poses.txt
> ```
