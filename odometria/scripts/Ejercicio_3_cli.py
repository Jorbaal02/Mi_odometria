#!/usr/bin/env python
import sys
import rospy
from odometria.srv import moveT

def move_robot_client(distance):
    rospy.wait_for_service('move_robot')
    try:
        move_service = rospy.ServiceProxy('move_robot', moveT)
        response = move_service(distance)
        return response.success
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)
        return False

def usage():
    return "%s [distance]" % sys.argv[0]

if __name__ == "__main__":
    rospy.init_node('move_robot_client')  

    if len(sys.argv) == 2:
        distance = float(sys.argv[1])
    else:
        print(usage())
        sys.exit(1)
    
    print("Requesting move of %.2f meters..." % distance)
    result = move_robot_client(distance)
    print("Success!" if result else "The robot did not move.")

