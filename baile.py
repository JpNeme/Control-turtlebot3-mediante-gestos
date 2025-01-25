#! /usr/bin/env python

# Código tomado de Robot Ignite Academy y disponible en múltiples fuentes en línea.
# Este código mueve el brazo robótico desde una posición inicial a otras posiciones predefinidas 
# y regresa a la posición inicial cuando se detiene.

# Este código parece tener algunos errores. Se reportan algunos mensajes "INFO" durante la operación. 
# El autor no ha podido resolver estos problemas.

# Importación de librerías necesarias para la operación del brazo robótico
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String

# Clase personalizada para manejar excepciones específicas durante la operación
class FalseException(Exception):
   pass

# Clase principal para manejar los movimientos del brazo y la lógica de "bailar"
class Baila():
    def __init__(self):
        # Inicialización de la variable que controla si el brazo debe moverse
        self.baila = False
        
        # Suscripción al tópico "/BAILA" para recibir comandos y controlar el estado de movimiento
        rospy.Subscriber("/BAILA", String, self.BAILA_detected_callback)

        # Inicialización de MoveIt Commander y el nodo ROS
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_execute_trajectory', anonymous=True)

        # Creación de objetos MoveIt necesarios para el control del robot
        robot = moveit_commander.RobotCommander()  # Controlador del robot
        scene = moveit_commander.PlanningSceneInterface()  # Interfaz para manejar el entorno
        arm_group = moveit_commander.MoveGroupCommander("arm")  # Controlador del grupo de movimiento (brazo)
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', 
                                                       moveit_msgs.msg.DisplayTrajectory, queue_size=20)

        # Limpieza de objetivos previos y obtención de los valores actuales de las juntas del brazo
        arm_group.clear_pose_targets()
        arm_group_variable_values = arm_group.get_current_joint_values()
        print("Current Joint Values: %s" % arm_group_variable_values)  # Mostrar valores iniciales de las juntas

    # Callback que se activa al recibir mensajes en el tópico "/BAILA"
    def BAILA_detected_callback(self, data):
        print(data)  # Imprime el mensaje recibido
        # Actualiza la variable "baila" según el comando recibido
        if data.data == "baila_joe":
            self.baila = True  # Activa el movimiento del brazo
        elif data.data == "para_joe":
            self.baila = False  # Detiene el movimiento del brazo

    # Método principal que define la lógica de los movimientos del brazo
    def Bailar(self):
        # Repetición de inicialización (innecesaria si ya se hace en __init__)
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_execute_trajectory', anonymous=True)

        # Inicialización de objetos necesarios para el control del brazo
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        arm_group = moveit_commander.MoveGroupCommander("arm")
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', 
                                                       moveit_msgs.msg.DisplayTrajectory, queue_size=20)

        arm_group.clear_pose_targets()  # Limpia los objetivos previos
        arm_group_variable_values = arm_group.get_current_joint_values()  # Obtiene los valores actuales de las juntas
        print("Current Joint Values: %s" % arm_group_variable_values)  # Imprime los valores iniciales
        
        rate = rospy.Rate(10)  # Establece la frecuencia del bucle (10 Hz)
        
        while not rospy.is_shutdown():  # Bucle principal hasta que se detenga ROS
            
            while not self.baila:  # Espera a que el estado "baila" sea activado
                rate.sleep()

            try:
                # Movimiento 1: Posición inicial
                arm_group_variable_values[0] = 0.00
                arm_group_variable_values[1] = 0.00
                arm_group_variable_values[2] = 0.00
                arm_group_variable_values[3] = 0.00
                arm_group.set_joint_value_target(arm_group_variable_values)  # Establece objetivo
                plan2 = arm_group.plan()  # Planifica el movimiento
                arm_group.go(wait=True)  # Ejecuta el movimiento
                rospy.sleep(0.2)  # Pequeña pausa

                if not self.baila:
                    raise FalseException()  # Lanza una excepción para regresar a posición inicial

                # Movimiento 2: Nueva posición (adelante y abajo)
                arm_group_variable_values[0] = 0.50
                arm_group_variable_values[1] = -0.50
                arm_group_variable_values[2] = 0.50
                arm_group_variable_values[3] = -0.50
                arm_group.set_joint_value_target(arm_group_variable_values)
                plan2 = arm_group.plan()
                arm_group.go(wait=True)
                rospy.sleep(0.2)
                if not self.baila:
                    raise FalseException()

                # Movimiento 3: Posición intermedia (mitad adelante/abajo)
                arm_group_variable_values[0] = 0.25
                arm_group_variable_values[1] = -0.25
                arm_group_variable_values[2] = 0.25
                arm_group_variable_values[3] = -0.25
                arm_group.set_joint_value_target(arm_group_variable_values)
                plan2 = arm_group.plan()
                arm_group.go(wait=True)
                rospy.sleep(0.1)
                if not self.baila:
                    raise FalseException()

                # Movimiento 4: Regreso a la posición previa (0.50)
                arm_group_variable_values[0] = 0.50
                arm_group_variable_values[1] = -0.50
                arm_group_variable_values[2] = 0.50
                arm_group_variable_values[3] = -0.50
                arm_group.set_joint_value_target(arm_group_variable_values)
                plan2 = arm_group.plan()
                arm_group.go(wait=True)
                rospy.sleep(0.2)
                if not self.baila:
                    raise FalseException()

                # Movimiento 5: Regreso a la posición inicial
                arm_group_variable_values[0] = 0.00
                arm_group_variable_values[1] = 0.00
                arm_group_variable_values[2] = 0.00
                arm_group_variable_values[3] = 0.00
                arm_group.set_joint_value_target(arm_group_variable_values)
                plan2 = arm_group.plan()
                arm_group.go(wait=True)
                rospy.sleep(0.1)

                # Movimiento 6: Movimiento hacia atrás (arriba/atrás)
                arm_group_variable_values[0] = -0.50
                arm_group_variable_values[1] = -0.50
                arm_group_variable_values[2] = 0.50
                arm_group_variable_values[3] = -0.50
                arm_group.set_joint_value_target(arm_group_variable_values)
                plan2 = arm_group.plan()
                arm_group.go(wait=True)
                rospy.sleep(0.2)
                if not self.baila:
                    raise FalseException()

                # Movimiento 7: Posición intermedia hacia atrás
                arm_group_variable_values[0] = -0.25
                arm_group_variable_values[1] = -0.25
                arm_group_variable_values[2] = 0.25
                arm_group_variable_values[3] = -0.25
                arm_group.set_joint_value_target(arm_group_variable_values)
                plan2 = arm_group.plan()
                arm_group.go(wait=True)
                rospy.sleep(0.1)
                if not self.baila:
                    raise FalseException()

                # Movimiento 8: Regreso a posición previa (-0.50)
                arm_group_variable_values[0] = -0.50
                arm_group_variable_values[1] = -0.50
                arm_group_variable_values[2] = 0.50
                arm_group_variable_values[3] = -0.50
                arm_group.set_joint_value_target(arm_group_variable_values)
                plan2 = arm_group.plan()
                arm_group.go(wait=True)
                rospy.sleep(0.2)

            except FalseException as e:  # Manejo de excepción para detener el movimiento
                # Movimiento HOME: Regresar a la posición inicial
                print("Voy a home")
                arm_group_variable_values[0] = 0.00
                arm_group_variable_values[1] = 0.00
                arm_group_variable_values[2] = 0.00
                arm_group_variable_values[3] = 0.00
                arm_group.set_joint_value_target(arm_group_variable_values)
                plan2 = arm_group.plan()
                arm_group.go(wait=True)  # Ejecuta el movimiento HOME
                continue  # Continúa esperando nuevos comandos

# Punto de entrada principal del script
if __name__ == '__main__':
    Objeto_baila = Baila()  # Crea una instancia de la clase Baila
    Objeto_baila.Bailar()  # Inicia el proceso de movimiento del brazo
