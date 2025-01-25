#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Importación de librerías principales y utilidades para ROS y Python
import rospy  # Biblioteca principal de ROS para implementar nodos en Python
import actionlib  # Para manejar acciones en ROS, como las usadas en move_base
import os  # Para manipular rutas y variables del sistema operativo
import math  # Para realizar cálculos matemáticos (no usado en este código pero importado por si es necesario)
from subprocess import Popen, CalledProcessError, check_output  # Para ejecutar y controlar procesos externos

# Importación de mensajes necesarios de ROS
from std_msgs.msg import Empty  # Mensajes vacíos usados como señal de eventos
from geometry_msgs.msg import PoseWithCovarianceStamped  # Mensajes con pose y covarianza, típicamente usados en AMCL
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal  # Mensajes necesarios para interactuar con move_base

class HomeBaseNode:
    def __init__(self):
        """
        Constructor de la clase HomeBaseNode.
        Configura suscriptores, publicadores, y prepara el cliente de acción move_base.
        """
        rospy.init_node("home_base_node", anonymous=True)  # Inicialización del nodo con un nombre único

        # --- Suscriptores ---
        # Suscriptor al tópico AMCL para obtener la pose estimada actual del robot
        self.amcl_sub = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.amcl_callback)

        # Suscriptor para recibir comandos que establecen una nueva base
        self.nueva_base_sub = rospy.Subscriber("nueva_base", Empty, self.nueva_base_callback)

        # Suscriptor para recibir comandos que ordenan al robot volver a la base
        self.vuelve_a_casa_sub = rospy.Subscriber("vuelve_a_casa", Empty, self.vuelve_a_casa_callback)

        # Publicador para indicar que el robot ha vuelto a la base
        self.vuelto = rospy.Publisher("vuelto", Empty, queue_size=10)

        # --- Variables internas ---
        # Variable para almacenar la pose actual del robot
        self.current_pose = None  
        # Variable para almacenar la pose designada como base
        self.home_pose = None  

        # --- Configuración del cliente de acción move_base ---
        # Se usa para enviar metas de navegación al stack de navegación de ROS
        self.move_base_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        # Espera a que el servidor de move_base esté disponible
        rospy.loginfo("Esperando a move_base ...")
        self.move_base_client.wait_for_server()
        rospy.loginfo("move_base listo!")

        # Log de inicialización del nodo
        rospy.loginfo("HomeBaseNode inicializado. Esperando comandos...")

        # Configuración de un método de limpieza para finalizar procesos
        rospy.on_shutdown(self.shutdown)

        # Mantiene el nodo activo y procesando mensajes
        rospy.spin()

    def amcl_callback(self, msg):
        """
        Callback ejecutado cuando se recibe un mensaje en /amcl_pose.
        Actualiza la pose estimada actual del robot.
        :param msg: Mensaje de tipo PoseWithCovarianceStamped.
        """
        self.current_pose = msg.pose.pose  # Solo guarda la pose (ignora la covarianza)

    def nueva_base_callback(self, msg):
        """
        Callback ejecutado cuando se recibe un mensaje en 'nueva_base'.
        Establece la posición actual del robot como nueva base.
        :param msg: Mensaje vacío (sin contenido útil).
        """
        if self.current_pose is None:
            rospy.logwarn("No se ha recibido /amcl_pose todavía. No puedo fijar nueva base.")
            return
        self.home_pose = self.current_pose  # Guarda la pose actual como nueva base
        rospy.loginfo("Nueva base establecida en la pose actual:\n%s", self.home_pose)

    def vuelve_a_casa_callback(self, msg):
        """
        Callback ejecutado cuando se recibe un mensaje en 'vuelve_a_casa'.
        Ordena al robot que regrese a la base previamente definida.
        :param msg: Mensaje vacío (sin contenido útil).
        """
        if self.home_pose is None:
            rospy.logwarn("No hay base definida. Publica en 'nueva_base' antes de usar 'vuelve_a_casa'.")
            return

        # Crea un mensaje MoveBaseGoal para enviar al cliente move_base
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"  # Define el marco de referencia como el mapa
        goal.target_pose.header.stamp = rospy.Time.now()  # Tiempo actual
        goal.target_pose.pose = self.home_pose  # Define la pose objetivo como la base

        rospy.loginfo("Enviando goal para volver a la base...")
        self.move_base_client.send_goal(goal)  # Envía la meta al servidor
        self.move_base_client.wait_for_result()  # Espera el resultado de la acción

        # Verifica el estado del resultado (3: éxito)
        result_state = self.move_base_client.get_state()
        if result_state == 3:
            rospy.loginfo("¡Llegamos a la base con éxito!")
            self.vuelto.publish()  # Publica una señal indicando éxito
        else:
            rospy.logwarn("El robot no pudo alcanzar la base. Estado: %d", result_state)
            self.vuelto.publish()  # Publica incluso en caso de fallo

    def shutdown(self):
        """
        Método que se ejecuta al apagar el nodo.
        Limpia procesos lanzados y recursos utilizados.
        """
        rospy.loginfo("Cerrando nodo y procesos.")
        if hasattr(self, 'gazebo_process'):  # Cierra Gazebo si fue lanzado
            self.gazebo_process.terminate()
        if hasattr(self, 'navigation_process'):  # Cierra navegación si fue lanzada
            self.navigation_process.terminate()

# --- Punto de entrada principal ---
if __name__ == '__main__':
    try:
        HomeBaseNode()  # Instancia y ejecuta el nodo principal
    except rospy.ROSInterruptException:
        pass  # Maneja la interrupción del programa (Ctrl+C)
