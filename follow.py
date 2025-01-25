#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# --- Librerías necesarias ---
import os  # Para operaciones con el sistema de archivos
import rospy  # Librería principal de ROS para Python
import cv2  # OpenCV para procesamiento de imágenes
import numpy as np  # Operaciones matemáticas y manejo de arrays
from geometry_msgs.msg import Twist  # Mensaje de control de velocidad del robot
from sensor_msgs.msg import Image, LaserScan  # Mensajes para imágenes y escaneo láser
from std_msgs.msg import Bool  # Mensaje tipo Booleano
from cv_bridge import CvBridge  # Conversión entre imágenes ROS y OpenCV
from subprocess import Popen, CalledProcessError, check_output  # Para lanzar y gestionar procesos del sistema

class TurtlebotFollower:
    def __init__(self):
        """
        Constructor de la clase. Inicializa el nodo y configura sus publicaciones, suscripciones y procesos.
        """
        rospy.init_node('turtlebot3_follower')  # Inicializa el nodo con nombre 'turtlebot3_follower'

        # --- Publicadores y suscriptores ---
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        # Publicador de comandos de velocidad al robot

        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
        # Suscriptor al tópico de imágenes de la cámara

        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        # Suscriptor al escaneo láser para evitar obstáculos

        self.following_pub = rospy.Publisher('/following_pink', Bool, queue_size=10)
        # Publicador para indicar si el robot está siguiendo el objeto rosa

        self.following_sub = rospy.Subscriber('/following_pink', Bool, self.following_callback)
        # Suscriptor para recibir comandos para activar/desactivar el seguimiento

        # --- Configuración inicial ---
        self.bridge = CvBridge()  # Interfaz para convertir imágenes ROS a OpenCV
        self.twist = Twist()  # Objeto Twist para comandos de velocidad

        # Flags y parámetros de control
        self.target_found = False  # Bandera para indicar si se detectó el objetivo
        self.following_enabled = False  # Bandera para habilitar/deshabilitar el seguimiento
        self.target_center_x = None  # Coordenada x del centro del objeto detectado

        self.desired_distance = 0.5  # Distancia deseada al objeto (en metros)
        self.fov_camera = 1.047  # Campo de visión horizontal de la cámara (en radianes)

        # Rango HSV para el color rosa
        self.lower_pink = np.array([145, 150, 150])  # Límite inferior
        self.upper_pink = np.array([155, 255, 255])  # Límite superior

        # --- Configuración para Gazebo y navegación 
        self.map_file = os.path.expanduser(
            "/home/jp/Escritorio/Robots_moviles/maps/map_house.yaml"
        )
        self.gazebo_process = None  # Proceso de Gazebo
        self.navigation_process = None  # Proceso de navegación

    def check_and_launch_navigation(self):
        """
        Comprueba si el stack de navegación está corriendo. Si no, lo lanza con el mapa.
        """
        if not self.is_node_running('/move_base'):
            rospy.loginfo("No se detecta /move_base. Lanzando el stack de navegación...")
            self.navigation_process = Popen([
                "roslaunch",
                "turtlebot3_navigation",
                "turtlebot3_navigation.launch",
                f"map_file:={self.map_file}"
            ])  # Lanza el stack de navegación con el archivo de mapa definido
        else:
            rospy.loginfo("move_base ya está en ejecución. No se lanzará de nuevo.")

    def is_node_running(self, node_name):
        """
        Verifica si un nodo específico está corriendo.
        :param node_name: Nombre del nodo a verificar (ejemplo: '/gazebo', '/move_base')
        :return: True si el nodo está activo, False en caso contrario.
        """
        try:
            output = check_output(["rosnode", "list"]).decode("utf-8").split()
            return (node_name in output)  # Retorna True si el nodo está en la lista
        except CalledProcessError:
            return False  # Si hay un error, asume que el nodo no está corriendo

    def following_callback(self, msg):
        """
        Callback para habilitar/deshabilitar el seguimiento.
        :param msg: Mensaje Bool indicando si se debe activar/desactivar el seguimiento.
        """
        self.following_enabled = msg.data
        rospy.loginfo(f"Seguimiento {'activado' if self.following_enabled else 'desactivado'}")
        if not self.following_enabled:
            self.stop_robot()

    def image_callback(self, data):
        """
        Callback para procesar las imágenes de la cámara y detectar el color rosa.
        :param data: Mensaje Image con la imagen de la cámara.
        """
        try:
            # Convierte la imagen de ROS a formato OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')

            # Convierte la imagen al espacio de color HSV
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            # Genera una máscara para detectar el color rosa
            mask = cv2.inRange(hsv, self.lower_pink, self.upper_pink)

            # Muestra la máscara en una ventana
            cv2.imshow("Pink Mask", mask)
            cv2.waitKey(1)

            if self.following_enabled:
                # Encuentra contornos en la máscara
                contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                if contours:
                    # Selecciona el contorno más grande
                    c = max(contours, key=cv2.contourArea)
                    (x, y), radius = cv2.minEnclosingCircle(c)
                    center = (int(x), int(y))

                    if radius > 10:  # Si el contorno es suficientemente grande
                        self.target_found = True
                        self.target_center_x = center[0]  # Guarda la posición en x
                        image_center_x = cv_image.shape[1] // 2  # Centro de la imagen
                        self.follow_object(center, image_center_x)  # Sigue el objeto
                    else:
                        self.target_found = False
                        self.target_center_x = None
                else:
                    self.target_found = False
                    self.target_center_x = None
        except Exception as e:
            rospy.logerr(f"Error procesando la imagen: {e}")

    def follow_object(self, center, image_center):
        """
        Genera comandos para seguir el objeto detectado.
        :param center: Coordenadas (x, y) del centro del objeto detectado.
        :param image_center: Centro de la imagen en el eje x.
        """
        error = center[0] - image_center  # Calcula el error de posición en el eje x
        self.twist.linear.x = 0.2  # Velocidad lineal constante
        self.twist.angular.z = -float(error) / 100  # Ajuste angular proporcional al error
        self.cmd_vel_pub.publish(self.twist)  # Publica el comando de velocidad

    def laser_callback(self, data):
        """
        Callback para evitar obstáculos usando el escáner láser.
        :param data: Mensaje LaserScan con las mediciones del láser.
        """
        if not self.following_enabled or not self.target_found:
            return

        # Calcula el ángulo del objeto en función de su posición en la imagen
        if self.target_center_x is not None:
            image_width = 640  # Ancho de la imagen (ajustar según tu cámara)
            angle = self.pixel_to_angle(self.target_center_x, image_width, self.fov_camera)

            # Convierte el ángulo en un índice de rangos del láser
            angle_min = data.angle_min
            angle_increment = data.angle_increment
            idx = int((angle - angle_min) / angle_increment)
            idx = max(0, min(idx, len(data.ranges) - 1))

            # Obtiene la distancia al objeto
            distancia_objeto = data.ranges[idx]

            if distancia_objeto <= self.desired_distance:
                rospy.loginfo("A ~0.5 metros del objeto rosa. Deteniendo seguimiento.")
                self.following_pub.publish(Bool(data=False))  # Desactiva el seguimiento
                self.stop_robot()
                return

            # Evitar obstáculos cercanos
            min_distance = min(data.ranges)
            if min_distance < 0.3:  # Si un obstáculo está demasiado cerca
                rospy.loginfo("Obstáculo cercano. Ajustando trayectoria...")
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.5
                self.cmd_vel_pub.publish(self.twist)

    def pixel_to_angle(self, center_x, image_width, fov=1.047):
        """
        Convierte la posición del objeto en la imagen a un ángulo en radianes.
        :param center_x: Coordenada x del objeto detectado.
        :param image_width: Ancho de la imagen.
        :param fov: Campo de visión horizontal de la cámara.
        :return: Ángulo en radianes.
        """
        offset_x = center_x - (image_width / 2.0)
        normalized = offset_x / (image_width / 2.0)  # Normaliza a [-1, 1]
        angle = normalized * (fov / 2.0)  # Escala al rango de [-fov/2, fov/2]
        return angle

    def stop_robot(self):
        """
        Detiene el movimiento del robot.
        """
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist)

    def shutdown(self):
        """
        Se ejecuta al cerrar el nodo. Termina procesos si los lanzamos.
        """
        rospy.loginfo("Cerrando nodo y procesos.")
        self.stop_robot()  # Detiene el robot
        if self.gazebo_process is not None:
            self.gazebo_process.terminate()  # Cierra Gazebo
        if self.navigation_process is not None:
            self.navigation_process.terminate()  # Cierra navegación

    def run(self):
        """
        Mantiene el nodo activo y registra el shutdown.
        """
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("TurtleBot Follower listo. Esperando señal para seguir...")
        rospy.spin()  # Mantiene el nodo en ejecución

# --- Punto de entrada ---
if __name__ == '__main__':
    try:
        follower = TurtlebotFollower()  # Instancia el nodo
        follower.run()  # Inicia la ejecución
    except rospy.ROSInterruptException:
        pass
