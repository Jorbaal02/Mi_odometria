#!/usr/bin/env python
import sys
import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

# Variables globales
pos_x = pos_y = yaw = 0.0
kp = 0.5

# Callback para obtener orientación y posición
def get_pos(msg):
    global yaw, pos_x, pos_y
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (_, _, yaw) = euler_from_quaternion(orientation_list)
    pos_x = msg.pose.pose.position.x
    pos_y = msg.pose.pose.position.y

def normalizar_angulo(angulo):
    while angulo > math.pi:
        angulo -= 2 * math.pi
    while angulo < -math.pi:
        angulo += 2 * math.pi
    return angulo

def main():
    rospy.init_node('poligono', anonymous=True)
    rospy.Subscriber('/odom', Odometry, get_pos)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rospy.wait_for_message('/odom', Odometry)
    r = rospy.Rate(10)
    vel = Twist()

    if len(sys.argv) < 3:
        print("Uso: rosrun odometria Ejercicio_7.py <nº_lados> <lado_en_metros>")
        sys.exit(1)

    N = int(sys.argv[1])
    L = float(sys.argv[2])

    # Ángulo interno del giro (en radianes)
    angulo_giro = (2 * math.pi) / N
    yaw_objetivo = yaw  # acumulador del ángulo deseado

    for i in range(N):
        # ---------- MOVIMIENTO LINEAL ----------
        x_inicial, y_inicial = pos_x, pos_y

        while not rospy.is_shutdown():
            distancia = math.sqrt((pos_x - x_inicial) ** 2 + (pos_y - y_inicial) ** 2)
            if distancia >= (L - 0.01):
                break
            vel.linear.x = kp * (L - distancia)
            vel.angular.z = 0.0
            pub.publish(vel)
            r.sleep()

        vel.linear.x = 0.0
        pub.publish(vel)
        rospy.sleep(1)

        # ---------- GIRO ----------
        yaw_objetivo += angulo_giro
        yaw_objetivo = normalizar_angulo(yaw_objetivo)

        while not rospy.is_shutdown():
            error_ang = normalizar_angulo(yaw_objetivo - yaw)
            if abs(error_ang) < 0.01:
                break
            vel.linear.x = 0.0
            vel.angular.z = kp * error_ang
            pub.publish(vel)
            r.sleep()

        vel.angular.z = 0.0
        pub.publish(vel)
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


