#!/usr/bin/env python
import rospy
import math
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

# Parámetros de control y seguridad
distancia_seguridad = 0.2
velocidad_lineal = 0.2
kp = 0.5  
angulo_giro = math.pi / 2  
yaw = 0
estado = "avanzar"


def get_pos(msg):
    global yaw
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (_, _, yaw) = euler_from_quaternion(orientation_list)


def realizar_giro(pub, direccion):
    global yaw
    vel = Twist()
    yaw_inicial = yaw
    objetivo = yaw_inicial + direccion * angulo_giro
    rate = rospy.Rate(10)
    while abs(objetivo - yaw) > 0.05 and not rospy.is_shutdown():
        vel.angular.z = kp * (objetivo - yaw)
        vel.linear.x = 0.0
        pub.publish(vel)
        rate.sleep()
    vel.angular.z = 0.0
    pub.publish(vel)

def filtrar_lecturas(valores):
    return [r if r > 0.14 else 400.0 for r in valores]


def lidar_callback(scan_data):
    global estado
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    vel_msg = Twist()

    # Sector frontal del LIDAR (promedio)
    sector_centro = filtrar_lecturas(scan_data.ranges[-10:] + scan_data.ranges[0:10])  
    sector_izquierda = filtrar_lecturas(scan_data.ranges[60:80])  

    promedio_frontal = sum(sector_centro)/len(sector_centro)
    promedio_izquierda = sum(sector_izquierda)/len(sector_izquierda)

    rospy.loginfo("Distancia frontal: %.2f, izquierda: %.2f", promedio_frontal, promedio_izquierda)

    if estado == "avanzar":
        if promedio_frontal < distancia_seguridad:
            rospy.loginfo("Obstáculo detectado. Iniciando maniobra de rodeo.")
            vel_msg.linear.x = 0.0
            pub.publish(vel_msg)
            realizar_giro(pub, 1)  
            estado = "rodear"
        else:
            vel_msg.linear.x = velocidad_lineal
            pub.publish(vel_msg)

    elif estado == "rodear":
        if promedio_izquierda < distancia_seguridad:
            vel_msg.linear.x = velocidad_lineal
            pub.publish(vel_msg)
        else:
            vel_msg.linear.x = 0.0
            pub.publish(vel_msg)
            realizar_giro(pub, -1)  
            estado = "avanzar"


def main():
    rospy.init_node('robot_avoid_obstacle', anonymous=True)
    rospy.Subscriber('/scan', LaserScan, lidar_callback)
    rospy.Subscriber('/odom', Odometry, get_pos)
    rospy.wait_for_message('/odom',Odometry)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

