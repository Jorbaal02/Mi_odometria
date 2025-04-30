#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

x = 0
kp = 0.4

def get_position(msg):
    global x
    x = msg.pose.pose.position.x
    #rospy.loginfo('Posicion: ' + str(x))

if __name__ == '__main__':
    rospy.init_node('rotate_robot')
    sub = rospy.Subscriber('/odom', Odometry, get_position)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
    r = rospy.Rate(10)
    vel = Twist()
    rospy.wait_for_message('/odom',Odometry)
    x_target = x + 1
    
    while not rospy.is_shutdown():
        vel.linear.x = kp * (x_target - x)
        pub.publish(vel)
        r.sleep()
        rospy.loginfo(f'target: {x_target}, current: {x}')