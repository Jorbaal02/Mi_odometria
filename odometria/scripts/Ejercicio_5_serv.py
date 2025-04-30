#!/usr/bin/env python
import rospy
import actionlib
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

from odometria.msg import MovimientoRobotAction, MovimientoRobotFeedback, MovimientoRobotResult

# Variables globales
pos_x = pos_y = yaw = 0.0

def get_pos(msg):
    global pos_x, pos_y, yaw
    pos_x = msg.pose.pose.position.x
    pos_y = msg.pose.pose.position.y
    orientation_q = msg.pose.pose.orientation
    (_, _, yaw) = euler_from_quaternion(
        [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    )

class MovimientoAction:
    def __init__(self, name):
        self._as = actionlib.SimpleActionServer(
            name,
            MovimientoRobotAction,
            execute_cb=self.execute_cb,
            auto_start=False
        )
        self._as.start()
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/odom', Odometry, get_pos)
        rospy.wait_for_message('/odom', Odometry)

    def execute_cb(self, goal):
        r = rospy.Rate(10)
        feedback = MovimientoRobotFeedback()
        result = MovimientoRobotResult()
        vel = Twist()

        kp = 0.5
        success = True

        if goal.tipo_movimiento == 0:  # Movimiento lineal
            x0, y0 = pos_x, pos_y
            while not rospy.is_shutdown():
                distancia = math.sqrt((pos_x - x0)**2 + (pos_y - y0)**2)
                error = goal.valor - distancia
                if error <= 0.01:
                    break
                vel.linear.x = kp * error
                vel.angular.z = 0.0
                self.pub.publish(vel)
                feedback.progreso = distancia
                self._as.publish_feedback(feedback)
                if self._as.is_preempt_requested():
                    self.pub.publish(Twist())  # parar
                    self._as.set_preempted()
                    success = False
                    return
                r.sleep()
            vel.linear.x = 0.0

        elif goal.tipo_movimiento == 1:  # Rotación
            yaw0 = yaw
            objetivo = yaw0 + goal.valor
            while not rospy.is_shutdown():
                error_ang = self.normalizar_angulo(objetivo - yaw)
                if abs(error_ang) < 0.01:
                    break
                vel.linear.x = 0.0
                vel.angular.z = kp * error_ang
                self.pub.publish(vel)
                feedback.progreso = yaw
                self._as.publish_feedback(feedback)
                if self._as.is_preempt_requested():
                    self.pub.publish(Twist())  # parar
                    self._as.set_preempted()
                    success = False
                    return
                r.sleep()
            vel.angular.z = 0.0

        self.pub.publish(vel)
        rospy.sleep(1.0)

        result.exito = success
        if success:
            self._as.set_succeeded(result)

    def normalizar_angulo(self, angulo):
        while angulo > math.pi:
            angulo -= 2 * math.pi
        while angulo < -math.pi:
            angulo += 2 * math.pi
        return angulo

if __name__ == '__main__':
    rospy.init_node('servidor_movimiento')
    server = MovimientoAction('movimiento_robot')
    rospy.spin()
