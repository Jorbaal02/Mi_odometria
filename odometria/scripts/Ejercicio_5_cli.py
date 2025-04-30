#!/usr/bin/env python
import rospy
import actionlib
from odometria.msg import MovimientoRobotAction, MovimientoRobotGoal

def feedback_cb(feedback):
    rospy.loginfo(f"Progreso: {feedback.progreso:.2f}")

if __name__ == '__main__':
    rospy.init_node('cliente_movimiento')

    client = actionlib.SimpleActionClient('movimiento_robot', MovimientoRobotAction)
    rospy.loginfo("Esperando al servidor de acciones...")
    client.wait_for_server()

    tipo = int(input("Introduce tipo de movimiento (0: lineal, 1: rotacional): "))
    valor = float(input("Introduce valor (metros o radianes): "))

    goal = MovimientoRobotGoal()
    goal.tipo_movimiento = tipo
    goal.valor = valor

    client.send_goal(goal, feedback_cb=feedback_cb)
    client.wait_for_result()

    result = client.get_result()
    if result.exito:
        rospy.loginfo("Movimiento completado con éxito.")
    else:
        rospy.logwarn("Movimiento interrumpido o fallido.")
