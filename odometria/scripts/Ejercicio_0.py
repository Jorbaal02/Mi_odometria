#!/usr/bin/env python
import sys
import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler

yaw = 0

def get_pos (msg): 
    global roll, pitch, yaw
    orientation_q = msg.pose.pose.orientation 
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w] 
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list) 
    
if __name__ == '__main__': 
    #Nodos y tópicos
    rospy.init_node('robot_move') 
    sub = rospy.Subscriber ('/odom', Odometry, get_pos) 
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
    r = rospy.Rate(10)

    #Control
    yaw_inicial = yaw
    kp = 0.5
    rospy.wait_for_message('/odom',Odometry)
    vel = Twist()

    if len(sys.argv) != 2:
        print("Uso: rosrun odometria Ejercicio0.py [valor]")
        sys.exit(1)
    
    while not rospy.is_shutdown():
        target = yaw_inicial + int(sys.argv[1])*math.pi/180
        vel.angular.z = kp * (target - yaw)
        pub.publish(vel)
        r.sleep()