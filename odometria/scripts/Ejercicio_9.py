#!/usr/bin/env python
import rospy
import math
import sys
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

# Parámetros de control y seguridad
distancia_seguridad = 0.2
velocidad_lineal = 0.2
kp = 0.5  
yaw = 0
pos = 0
estado = "avanzar"
time = sys.argv[1]
largo = sys.argv[0]


def get_pos(msg):
    global yaw
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (_, _, yaw) = euler_from_quaternion(orientation_list)
    pos = msg.pose.pose.position.x


def realizar_giro(pub, direccion, angulo_giro):
    global yaw
    vel = Twist()
    yaw_inicial = yaw
    objetivo = yaw_inicial + direccion*angulo_giro
    rate = rospy.Rate(10)
    while abs(objetivo - yaw) > 0.05 and not rospy.is_shutdown():
        vel.angular.z = kp * (objetivo - yaw)
        pub.publish(vel)
        rate.sleep()

def realizar_mov(pub, direccion, pos_obj):
    global pos
    vel = Twist()
    pos_inicial = pos
    objetivo = pos_inicial + direccion*pos_obj
    rate = rospy.Rate(10)
    while abs(objetivo - pos) > 0.05 and not rospy.is_shutdown():
        vel.linear.x = kp * (objetivo - pos)
        pub.publish(vel)
        rate.sleep()

def parar(pub):
    vel = Twist()
    vel.linear.x = 0
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
    rate = rospy.Rate(10)

    if pos < largo:
        if estado == "avanzar":
            if promedio_izquierda > distancia_seguridad:
                rospy.loginfo("Aparcamiento detectado. Iniciando maniobra de aparcamiento.")
                vel_msg.linear.x = 0.0
                pub.publish(vel_msg)
                realizar_giro(pub, -1, math.pi/2)  
                estado = "aparcar"
            else:
                vel_msg.linear.x = velocidad_lineal
                pub.publish(vel_msg)
        elif estado == "aparcar":
            realizar_mov(pub, 1 ,promedio_frontal - 0.2)
            time_ini = rospy.Time.now().to_sec()
            while (rospy.Time.now().to_sec() - time_ini) < time:
                rospy.loginfo("APARCAO. Tiempo de espera: %d", rospy.Time.now().to_sec() - time_ini)
            realizar_mov(pub, -1 ,promedio_frontal - 0.2)
            realizar_giro(pub, 1, math.pi/2) 
            estado = "avanzar"
    else:
        parar(pub)

def main():
    rospy.init_node('buscando aparcamiento', anonymous=True)
    rospy.Subscriber('/scan', LaserScan, lidar_callback)
    rospy.Subscriber('/odom', Odometry, get_pos)
    rospy.wait_for_message('/odom',Odometry)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass