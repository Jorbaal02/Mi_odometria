#!/usr/bin/env python
import sys
import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

# Variables globales
L = float(sys.argv[1])
pos_x = pos_y = yaw = 0.0
kp = 0.5
angulo_giro = 2*math.pi/3  # 120 grados

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
    rospy.init_node('triangulo', anonymous=True)
    rospy.Subscriber('/odom', Odometry, get_pos)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rospy.wait_for_message('/odom', Odometry)
    r = rospy.Rate(10)
    vel = Twist()

    if len(sys.argv) < 2:
        print("Uso: rosrun odometria Ejercicio_6.py <lado_en_metros>")
        sys.exit(1)

    for i in range(3):
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
        yaw_inicial = yaw
        objetivo_yaw = yaw_inicial + angulo_giro

        while not rospy.is_shutdown():
            error_ang = normalizar_angulo(objetivo_yaw - yaw)
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

