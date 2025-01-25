# -*- coding: utf-8 -*-
# from __future__ import print_function

import rospy
import actionlib
import smach_ros
from smach import State, StateMachine
from std_msgs.msg import Int32, String, Bool, Empty

# Clase para el estado de reposo y detección de opciones
definir opciones
class Reposo(State):
    def __init__(self):
        # Inicializa el estado con sus posibles resultados
        State.__init__(self, outcomes=['bailar', 'seguir', 'home', 'sethome'])
        self.option_detected = False
        self.option = 0
    
    def execute(self, userdata):
        # Suscribirse al tópico "/opcion" para recibir datos
        self.subOption = rospy.Subscriber("/opcion", Int32, self.option_detected_callback)
        rate = rospy.Rate(10)

        # Espera hasta detectar una opción
        while not self.option_detected and self.option == 0:
            rate.sleep()

        # Una vez detectada la opción, desuscribirse
        self.option_detected = False
        self.subOption.unregister()

        # Retorna el resultado según la opción detectada
        if self.option == 1:
            self.option = 0
            return 'bailar'

        if self.option == 2:
            self.option = 0
            return 'seguir'
        
        if self.option == 3:
            self.option = 0
            return 'home'
        
        if self.option == 4:
            self.option = 0
            return 'sethome'
        
    def option_detected_callback(self, data):
        # Callback para procesar el dato recibido
        self.option_detected = True
        self.option = data.data

class Bailar(State):
    def __init__(self):
        State.__init__(self, outcomes=['completed']) # Estado que indica que la tarea se ha completado
        self.Bailapub = rospy.Publisher('/BAILA', String , queue_size=10)
        self.option = 0

    def execute(self,userdata):
        # Publicar el comando para iniciar el baile
        self.Bailapub.publish("baila_joe")
        self.subOption = rospy.Subscriber("/opcion", Int32, self.option_detected_callback)
        rate = rospy.Rate(10)

        # Esperar hasta que se reciba la opción de detener el baile
        while not self.option == 1:
            rate.sleep()

        # Publicar el comando para detener el baile
        self.Bailapub.publish("para_joe")
        self.subOption.unregister()
        self.option = 0
        return "completed"

    def option_detected_callback(self, data):
        self.option_detected = True
        self.option = data.data

class Seguir(State):
    def __init__(self):
        State.__init__(self, outcomes=['completed']) # Estado que indica que la tarea se ha completado
        self.FollowingPink = rospy.Publisher('/following_pink', Bool , queue_size=10)
        self.option = 0

    def execute(self,userdata):
        # Publicar el comando para comenzar a seguir
        self.FollowingPink.publish(True)
        self.subOption = rospy.Subscriber("/opcion", Int32, self.option_detected_callback)

        rate = rospy.Rate(10)
        # Esperar hasta recibir la opción para detener el seguimiento
        while not self.option == 2:
            rate.sleep()

        # Publicar el comando para detener el seguimiento
        self.FollowingPink.publish(False) 
        self.subOption.unregister()
        self.option = 0
        return "completed"
    
    def option_detected_callback(self, data):
        self.option_detected = True
        self.option = data.data

class Home(State):
    def __init__(self):
        State.__init__(self, outcomes=['completed']) # Estado que indica que la tarea se ha completado
        self.Vuelve = rospy.Publisher('/vuelve_a_casa', Empty , queue_size=10)
        self.vuelto_detected = False

    def execute(self,userdata):
        # Publicar el comando para volver a casa
        self.Vuelve.publish()
        self.Vuelto = rospy.Subscriber("/vuelto", Empty, self.vuelto_detected_callback)

        rate = rospy.Rate(10)
        # Esperar hasta recibir confirmación de haber llegado a casa
        while not self.vuelto_detected:
            rate.sleep()

        self.Vuelve.publish() 
        self.Vuelto.unregister()
        return "completed"
    
    def vuelto_detected_callback(self, data):
        self.vuelto_detected = True

class SetHome(State):
    def __init__(self):
        State.__init__(self, outcomes=['completed']) # Estado que indica que la tarea se ha completado
        self.nueva_base = rospy.Publisher('/nueva_base', Empty , queue_size=10)

    def execute(self,userdata):
        # Publicar el comando para establecer una nueva base
        self.nueva_base.publish()
        return "completed"

if __name__ == '__main__':
    rospy.init_node("Proyecto") # Inicializamos el nodo principal de ROS
    sm = StateMachine(outcomes=['end']) # Creamos la máquina de estados
    
    with sm:
        # Configuramos los estados y las transiciones
        StateMachine.add('Reposo', Reposo(), 
           transitions={ 
               'bailar':'Bailar',
               'seguir':'Seguir',
               'home':'Home',
               'sethome':'SetHome'}) 
        StateMachine.add('Bailar', Bailar(), 
           transitions={
               'completed': 'Reposo'})
        StateMachine.add('Seguir', Seguir(), 
           transitions={
               'completed': 'Reposo'}) 
        StateMachine.add('Home', Home(), 
           transitions={
               'completed': 'Reposo'}) 
        StateMachine.add('SetHome', SetHome(), 
           transitions={
               'completed': 'Reposo'}) 
    
    # Inicializamos el servidor de introspección para visualizar la máquina de estados
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Ejecutamos la máquina de estados
    sm.execute()
    rospy.spin()
    