#!/usr/bin/env python

# --------------------------------------------------------------------------------------------------------
# Imports
# --------------------------------------------------------------------------------------------------------

import rospy
import smach
import time
from smach import State, StateMachine
from smach_ros import IntrospectionServer

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback
from tf.transformations import quaternion_from_euler
from collections import OrderedDict
from mesa_msg.srv import Mesa, MesaResponse

# --------------------------------------------------------------------------------------------------------


# --------------------------------------------------------------------------------------------------------
# Puntos de la casa
# --------------------------------------------------------------------------------------------------------
waypoints = [
    ['mesa1', (4.28, 3.24, 0.0), (0.0, 0.0, 1.0, 0.0)],
    ['mesa2', (2.35, 3.25, 0.0), (0.0, 0.0, 1.0, 0.0)],
    ['mesa3',(3.83, 4.65, 0.0), (0.0, 0.0, 1.0, 0.0)],
    ['mesa4', (2.37, 4.70, 0.0), (0.0, 0.0, 1.0, 0.0)],
    ['mesa5', (0.92, 4.70, 0.0), (0.0, 0.0, 1.0, 0.0)],
    ['cocina', (4.63, 1.64, 0.0), (0.0, 0.0, 1.0, 0.0)],
]
# --------------------------------------------------------------------------------------------------------


# --------------------------------------------------------------------------------------------------------
# Descripcion de los estados
# --------------------------------------------------------------------------------------------------------

class PowerOnRobot(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded','aborted'])

    def execute(self, userdata):
        rospy.loginfo("Encendiendo el Robot")
        time.sleep(2)
        return 'succeeded'


class WaitingOrder(State):
    def __init__(self):
        State.__init__(self, outcomes=['mesa1', 'mesa2', 'mesa3', 'mesa4', 'cocina','aborted'], input_keys=['input'], output_keys=[''])

    def execute(self, userdata):
        if userdata.input == 1:
            return 'mesa1'
        elif userdata.input == 2:
            return 'mesa2'
        elif userdata.input == 3:
            return 'mesa3'
        elif userdata.input == 4:
            return 'mesa4'
        elif userdata.input == 0:
            return 'cocina' 
        else:
            return 'aborted'


class Navigate(State):
    def __init__(self, position, orientation, place):
        State.__init__(self, outcomes=['succeeded', 'aborted'], input_keys=[''], output_keys=[''])
        self._position = position
        self._orientation = orientation
        self._place = place
        self._move_base = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        rospy.loginfo("Activando el cliente de navegacion..")
        self._move_base.wait_for_server(rospy.Duration(15))


    def execute(self, userdata):
        time.sleep(2)
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        rospy.loginfo(self._position)
        goal.target_pose.pose.position.x = self._position[0]
        goal.target_pose.pose.position.y = self._position[1]
        goal.target_pose.pose.position.z = self._position[2]
        goal.target_pose.pose.orientation.x = self._orientation[0]
        goal.target_pose.pose.orientation.y = self._orientation[1]
        goal.target_pose.pose.orientation.z = self._orientation[2]
        goal.target_pose.pose.orientation.w = self._orientation[3]

        rospy.loginfo("ROBOT %s" %(self._place))
        
        # sends the goal
        self._move_base.send_goal(goal)
        self._move_base.wait_for_result()
        
        # Comprobamos el estado de la navegacion
        nav_state = self._move_base.get_state()
        rospy.loginfo("[Result] State: %d" % (nav_state))
        #nav_state = 3

        if nav_state == 3:
            return 'succeeded'
        else:
            return 'aborted'


class Charge(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded','aborted'], input_keys=['input'], output_keys=[''])

    def execute(self, userdata):
        print("Revisando la carga de la bateria...")
        if userdata.input == 1:
            print("Robot cargado")
            return 'succeeded'
        else:
            print("Robot sin carga")
            return 'aborted'


class main():
    
    # ordenARealizar = 0
    # intro_server = None
    # maquinaEstadosNavegacion = None
    
    def __init__(self,ordenARealizar):
        
        self.ordenARealizar = ordenARealizar

        maquinaEstadosNavegacion = StateMachine(outcomes=['succeeded','aborted'])
        # maquinaEstadosNavegacion.userdata.chargeInput = 1
        maquinaEstadosNavegacion.userdata.ordenARealizar = self.ordenARealizar
        
        # global ordenARealizar
        
        with maquinaEstadosNavegacion:
            # Estado power_on
            StateMachine.add('POWER_ON', PowerOnRobot(), 
                             transitions={'succeeded':'WAITING_ORDER', 'aborted':'aborted'})
            # Estado esperar orden
            StateMachine.add('WAITING_ORDER', WaitingOrder(), 
                             transitions={'mesa1':waypoints[0][0], 'mesa2':waypoints[1][0], 'mesa3':waypoints[2][0], 'mesa4':waypoints[3][0],'cocina':waypoints[4][0], 'aborted':'WAITING_ORDER'},
                             remapping={'input':'ordenARealizar', 'output':''})
            
            # Estado mesas
            # MESA 1
            StateMachine.add(waypoints[0][0], Navigate( waypoints[0][1], waypoints[0][2], waypoints[0][0]), 
                             transitions={'succeeded':'succeeded','aborted':waypoints[4][0]})
            # MESA 2
            StateMachine.add(waypoints[1][0], Navigate( waypoints[1][1], waypoints[1][2], waypoints[1][0]), 
                             transitions={'succeeded':'succeeded','aborted':waypoints[4][0]})
            # MESA 3
            StateMachine.add(waypoints[2][0], Navigate( waypoints[2][1], waypoints[2][2], waypoints[2][0]), 
                             transitions={'succeeded':'succeeded','aborted':waypoints[4][0]})
            # MESA 4
            StateMachine.add(waypoints[3][0], Navigate( waypoints[3][1], waypoints[3][2], waypoints[3][0]), 
                             transitions={'succeeded':'succeeded','aborted':waypoints[4][0]})
            # Cocina
            StateMachine.add(waypoints[4][0], Navigate( waypoints[4][1], waypoints[4][2], waypoints[4][0]), 
                             transitions={'succeeded':'succeeded','aborted':'WAITING_ORDER'})
            
            # # Estado carga
            # StateMachine.add('CHARGE', Charge(), transitions={'succeeded': 'succeeded', 'aborted': 'aborted'},
            #                  remapping={'input':'chargeInput', 'output':''})
            
        intro_server = IntrospectionServer('Coockbot',maquinaEstadosNavegacion, '/SM_ROOT')
        intro_server.start() # iniciamos el servidor
        
        maquinaEstados_ejecucion = maquinaEstadosNavegacion.execute() # ejecutamos la maquina de estados
        intro_server.stop()


        def shutdown(self):
            rospy.loginf("Parando la ejecucion...")
            rospy.sleep(1)
        

# --------------------------------------------------------------------------------------------------------

# --------------------------------------------------------------------------------------------------------
# Servicio
# --------------------------------------------------------------------------------------------------------

# --------------------------------------------------------------------------------------------------------
# Callbak servicio
# --------------------------------------------------------------------------------------------------------
def callbackServicio(data):
    rospy.loginfo("Se ha llamado al servicio /navegacion_autonoma_servicio")
    numeroMesaSaleccionada = data.numeroMesa
    rospy.loginfo(numeroMesaSaleccionada)
    
    controlador = main(numeroMesaSaleccionada)
    
    response = MesaResponse()
    response.success = True
    
    return response
    
    


# Servicio
rospy.init_node('navegacion_autonoma', anonymous=False)
servicio = rospy.Service('/navegacion_autonoma_servicio',Mesa,callbackServicio)
rospy.Rate(1)
rospy.loginfo("El servicio esta listo")
rospy.spin()



# if __name__=='__main__':
#     try:
#         main()
#     except rospy.ROSInterruptException:
#         rospy.loginfo(" Testeo Robot Mayordomo finalizado")
