#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

vel_target = 0.05 #en m/s
time = 20 #en s

if __name__ == '__main__':
    rospy.init_node('rotate_robot')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
    r = rospy.Rate(10)
    vel = Twist()
    time_ini = rospy.Time.now().to_sec()
    
    while not rospy.is_shutdown():
        while (rospy.Time.now().to_sec() - time_ini) < time:
            vel.linear.x = vel_target
            pub.publish(vel)
            r.sleep()
        vel.linear.x = 0
        pub.publish(vel)
        r.sleep()
        rospy.loginfo('El robot ha recorrido 1 metro')
