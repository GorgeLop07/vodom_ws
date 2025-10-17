#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
import cv2
import os 
import numpy as np
import glob
from scipy.spatial.transform import Rotation
import torch
from transformers import AutoModelForDepthEstimation
from cv_bridge import CvBridge

class VisualOdometryNode(Node):
    def __init__(self):

        super().__init__('visual_odometry_node') #Constructor de la clase

        #Leer rosbag
        self.bridge=CvBridge()
        self.image_sub=self.create_subscription(
            Image, 
            '/camera/image_raw',
            self.image_callback,
            10
        )
        #Declarar y obtener parametros
        # ELIMINAR - self.declare_parameter('image_folder', '/home/fernanda/Downloads/vo_ws/src/vodom_first/KITTI_sequence_2/image_l')
        self.declare_parameter('frame_id', 'map')
        #ELIMINAR - self.image_folder_=self.get_parameter('image_folder').get_parameter_value().string_value
        self.frame_id_=self.get_parameter('frame_id').get_parameter_value().string_value

        #publicador para la trayectoria
        self.path_pub_=self.create_publisher(Path, '/Depth_vo_path', 10)

        #mensaje de la trayectoria y configurar cabecera
        self.path_msg_=Path()
        self.path_msg_.header.frame_id=self.frame_id_

        #cargar rutas de imagenes
        #ELIMINAR - self.image_paths_=[]
        #ELIMINAR - self.load_image_paths()
        self.get_logger().info(f'VisualOdometryNode started. Waiting for images on topic "/camera/image_raw"')

        #Inicializar variables de estado
        self.prev_img_=None
        self.prev_img_color_=None
        self.prev_R_=np.eye(3)
        self.prev_t_=np.zeros((3,1))

        #Parametros intrinsecos de la camara - default de KITTI
        self.focal_=718.856
        self.pp_=(607.1928, 185.2157)

        #Inicializar detector ORB
        self.orb_=cv2.ORB_create()
        self.bf_matcher_=cv2.BFMatcher(cv2.NORM_HAMMING)

        #Temporizador, procesar imagen cada 100 ms - 10Hz
        # ELIMINAR - self.timer_=self.create_timer(0.1,self.process_next_image)

        #cargar modelo
        self.get_logger().info('Loading depth estimation model...')

        #determinar si va a usar gpu o cpu
        self.device="cuda" if torch.cuda.is_available() else "cpu"
        self.get_logger().info(f'Using device: {self.device}')
        
        # Usar modelo DPT que es más compatible
        model_name="Intel/dpt-large"
        try:
            self.depth_model=AutoModelForDepthEstimation.from_pretrained(model_name).to(self.device)
            self.get_logger().info("DPT depth model loaded successfully.")
            
            # Parámetros de preprocessing manual para DPT
            self.input_size = (384, 384)  # Tamaño de entrada para DPT
            self.mean = [0.485, 0.456, 0.406]  # ImageNet mean
            self.std = [0.229, 0.224, 0.225]   # ImageNet std
            
        except Exception as e:
            self.get_logger().error(f'Failed to load DPT model: {e}')
            self.get_logger().info('Fallback: Using simple depth estimation based on stereo geometry')
            self.depth_model = None


    """def load_image_paths(self): #Carga y ordena rutas
        self.image_paths_=sorted(glob.glob(os.path.join(self.image_folder_,'*.png')))""" #ELIMINAR
    

    """def process_next_image(self): #Callback del temporizador
        
        #si no hay imagenes detiene el temporizador
        if self.current_idx_>=len(self.image_paths_):
            self.get_logger().info('All images processed')
            self.timer_.cancel()
            return
        
        image_path=self.image_paths_[self.current_idx_]
        img=cv2.imread(image_path,cv2.IMREAD_GRAYSCALE)

        if img is None:
            self.get_logger().warn(f'Could not read image: {image_path}')
            self.current_idx_+=1
            return
        
        if self.current_idx_==0:
            self.prev_img_=img
            # img a color para Midas
            self.prev_img_color_=cv2.imread(self.image_paths_[self.current_idx_])
            self.current_idx_+=1
            return""" #ELIMINAR
    
    def image_callback(self,msg):
        self.get_logger().info('Recibida una nueva imagen del rosbag')
        try:
            #imagen a color para prediccion de profundidad
            current_color_img=self.bridge.imgmsg_to_cv2(msg, "bgr8")
            #imagen a color para ORB
            current_gray_img=cv2.cvtColor(current_color_img, cv2.COLOR_BGR2GRAY)
        except Exception as e:
            self.get_logger().error(f'Error al convertir la imagen con CvBridge: {e}')
            return
        
        #Logica primera imagen
        if self.prev_img_ is None:
            self.get_logger().info('Primera imagen recibida. Usando como referencia inicial')
            self.prev_img_=current_gray_img
            self.prev_img_color_=current_color_img
            return
        

        #Prediccion de profundidad
        if self.depth_model is not None:
            try:
                #Preparar imagenes para el modelo
                input_tensor_prev = self.preprocess_image_for_depth(self.prev_img_color_).to(self.device)
                input_tensor_curr = self.preprocess_image_for_depth(current_color_img).to(self.device)
                
                with torch.no_grad():
                    # Procesar imagen previa
                    outputs_prev = self.depth_model(pixel_values=input_tensor_prev)
                    predicted_depth_prev = outputs_prev.predicted_depth
                    
                    # Procesar imagen actual
                    outputs_curr = self.depth_model(pixel_values=input_tensor_curr)
                    predicted_depth_curr = outputs_curr.predicted_depth

                #Interpolar profundidad al tamaño de la imagen original
                depth_pred_prev = torch.nn.functional.interpolate(
                    predicted_depth_prev.unsqueeze(1), 
                    size=(current_gray_img.shape[0], current_gray_img.shape[1]),
                    mode="bicubic",
                    align_corners=False,
                )
                
                depth_pred_curr = torch.nn.functional.interpolate(
                    predicted_depth_curr.unsqueeze(1), 
                    size=(current_gray_img.shape[0], current_gray_img.shape[1]),
                    mode="bicubic",
                    align_corners=False,
                )

                #Extraer componentes de profundidad y convertir a Numpy
                depth_map1 = depth_pred_prev.squeeze().cpu().numpy()
                depth_map2 = depth_pred_curr.squeeze().cpu().numpy()
                
            except Exception as e:
                self.get_logger().error(f'Error en predicción de profundidad: {e}')
                # Fallback: usar profundidad estimada simple
                depth_map1 = np.ones((current_gray_img.shape[0], current_gray_img.shape[1])) * 5.0
                depth_map2 = np.ones((current_gray_img.shape[0], current_gray_img.shape[1])) * 5.0
        else:
            # Fallback: usar profundidad estimada simple
            self.get_logger().warn('Usando profundidad estimada constante (fallback)')
            depth_map1 = np.ones((current_gray_img.shape[0], current_gray_img.shape[1])) * 5.0
            depth_map2 = np.ones((current_gray_img.shape[0], current_gray_img.shape[1])) * 5.0

        
        #feature detection and matching
        kp1, des1=self.orb_.detectAndCompute(self.prev_img_,None)
        kp2, des2=self.orb_.detectAndCompute(current_gray_img,None)

        matches=[]
        if des1 is not None and des2 is not None:
            matches=self.bf_matcher_.match(des1,des2)


        #Ordenar matches por distancia
        matches=sorted(matches, key=lambda x:x.distance)

        #extraer puntos correspondientes, solo se usan los mejores 100 matches
        pts1=np.float32([kp1[m.queryIdx].pt for m in matches[:100]]).reshape(-1,2)
        pts2=np.float32([kp2[m.trainIdx].pt for m in matches[:100]]).reshape(-1,2)

        #Estimar pose si hay suficientes puntos
        if pts1.shape[0]>=8:
            #elevar puntos 3d a 3d
            points_3d_1=self.unproject_points_to_3d(pts1, depth_map1)
            points_3d_2=self.unproject_points_to_3d(pts2, depth_map2)
            
            #Asegurar que tenemos la misma cantidad de puntos 3D validos
            if points_3d_1.shape[0]!=points_3d_2.shape[0] or points_3d_1.shape[0]<8:
                self.get_logger().warn("No hay suficientes puntos validos para estimar la escala")
                #Actualizar imagenes y continuar la sig interacion
                self.prev_img_=current_gray_img
                self.prev_img_color_=current_color_img
                return

            #Calcular escala real
            translations=np.linalg.norm(points_3d_2 - points_3d_1, axis=1)
            true_scale=np.median(translations)

            #Evitar escalas anomalas
            if true_scale<0.01 or true_scale>10:
                self.get_logger().warn(f'Escala anomala detectada: {true_scale}. Ignorando frame')
                self.prev_img_=current_gray_img
                self.prev_img_color_=current_color_img
                return
            #Aplicar escala
            #Calcular matriz esencial
            E, mask=cv2.findEssentialMat(pts2, pts1, self.focal_, self.pp_, cv2.RANSAC, 0.999, 1.0)
            if E is not None:
                #Recuperar rotacion y traslacion
                _, R,t,_=cv2.recoverPose(E, pts2, pts1, focal=self.focal_, pp=self.pp_, mask=mask)
                #Escalar vector de traslacion
                t_scaled=t*true_scale
                #Actualizar pose global
                #La traslacion es relativa, se acumula en el sistema de coords del mundo
                self.prev_t_=self.prev_t_+self.prev_R_ @t_scaled
                #La rotacion es relativa, se compone
                self.prev_R_=R @ self.prev_R_

                #Crear y publicar msj de la trayectoria
                pose=PoseStamped()
                pose.header.stamp=self.get_clock().now().to_msg()
                pose.header.frame_id=self.frame_id_

                #Posicion
                pose.pose.position.x=self.prev_t_[0,0]
                pose.pose.position.y=self.prev_t_[1,0]
                pose.pose.position.z=self.prev_t_[2,0]

                #Orientacion - convertir matriz de rotacion a cuaternion
                r=Rotation.from_matrix(self.prev_R_)
                quat=r.as_quat() # formato (x,y,z,w)
                pose.pose.orientation.x=quat[0]
                pose.pose.orientation.y=quat[1]
                pose.pose.orientation.z=quat[2]
                pose.pose.orientation.w=quat[3]

                self.path_msg_.poses.append(pose)
                self.path_msg_.header.stamp=self.get_clock().now().to_msg()

                self.path_pub_.publish(self.path_msg_)
                self.get_logger().info(f'Published path with {len(self.path_msg_.poses)} poses')

        #Acualizar img previa y el idx para la sig interaccion
        self.prev_img_=current_gray_img
        self.prev_img_color_=current_color_img

    def unproject_points_to_3d(self, points_2d, depth_map):
        points_3d=[]
        cx,cy=self.pp_
        fx=fy=self.focal_
        for point in points_2d:
            u,v=int(round(point[0])), int(round(point[1]))

            if 0<=v<depth_map.shape[0] and 0<=u<depth_map.shape[1]:
                depth=depth_map[v,u]
                if depth>0:
                    z=depth
                    x=(u-cx)*z/fx
                    y=(v-cy)*z/fy
                    points_3d.append([x,y,z])
        return np.array(points_3d)

    def preprocess_image_for_depth(self, image):
        """Manual preprocessing for depth estimation model"""
        # Convertir BGR a RGB
        if len(image.shape) == 3:
            image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        else:
            image_rgb = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)
        
        # Redimensionar
        image_resized = cv2.resize(image_rgb, self.input_size)
        
        # Normalizar
        image_normalized = image_resized.astype(np.float32) / 255.0
        
        # Aplicar mean y std
        for i in range(3):
            image_normalized[:, :, i] = (image_normalized[:, :, i] - self.mean[i]) / self.std[i]
        
        # Convertir a tensor y agregar batch dimension
        image_tensor = torch.from_numpy(image_normalized.transpose(2, 0, 1)).unsqueeze(0)
        return image_tensor

def main(args=None):
    rclpy.init(args=args)
    node=VisualOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':
    main()
