from six.moves import input
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))

from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

def all_close(goal, actual, tolerance):
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True

class MoveGroupPythonInterfaceTutorial(object):
    def __init__(self):
        super(MoveGroupPythonInterfaceTutorial, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("robot_sediento", anonymous=True)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group_name = "arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )
        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)
        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)
        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())
        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
        self.gripper_group = moveit_commander.MoveGroupCommander("gripper")

    

    def go_to_joint_state(self, joint_positions):
        move_group = self.move_group
        move_group.go(joint_positions, wait=True)
        move_group.stop()

        current_joints = move_group.get_current_joint_values()
        return all_close(joint_positions, current_joints, 0.01)
    
    def move_to_xyz(self, target_joints):
        group = moveit_commander.MoveGroupCommander("arm")  
        group.set_pose_target(target_joints)      
        group.go(wait=True)
        group.stop()
        group.clear_pose_targets()

    
    def move_down_z(self, distancia):
        move_group = self.move_group  

        waypoints = []  
        wpose = move_group.get_current_pose().pose  

        wpose.position.z -= distancia  
        waypoints.append(copy.deepcopy(wpose))  

        
        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01  
        )

        move_group.execute(plan, wait=True)  
        return fraction

    def pinza(self, estado):
        joint_goal = self.gripper_group.get_current_joint_values()
        joint_goal[0] = (0.7 if estado else 0.0)
        joint_goal[1] = (0.7 if estado else 0.0)
        self.gripper_group.go(joint_goal, wait=True)
        self.gripper_group.stop()

    def wait_for_state_update(
        self, box_is_known=False, box_is_attached=False, timeout=4
    ):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL wait_for_scene_update
        ##
        ## Ensuring Collision Updates Are Received
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## If the Python node was just created (https://github.com/ros/ros_comm/issues/176),
        ## or dies before actually publishing the scene update message, the message
        ## could get lost and the box will not appear. To ensure that the updates are
        ## made, we wait until we see the changes reflected in the
        ## `get_attached_objects() and get_known_object_names() lists.
        ## For the purpose of this tutorial, we call this function after adding,
        ## removing, attaching or detaching an object in the planning scene. We then wait
        ## until the updates have been made or `timeout seconds have passed.
        ## To avoid waiting for scene updates like this at all, initialize the
        ## planning scene interface with  `synchronous = True.
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = box_name in scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False
        ## END_SUB_TUTORIAL
    
    def add_cylinder(self, name, x, y, z, radius, height):
        cylinder_pose = geometry_msgs.msg.PoseStamped()
        cylinder_pose.header.frame_id = self.planning_frame
        cylinder_pose.pose.orientation.w = 1.0
        cylinder_pose.pose.position.x = x
        cylinder_pose.pose.position.y = y
        cylinder_pose.pose.position.z = z #+ height / 2.0  # Centrar en la base
        self.scene.add_cylinder(name, cylinder_pose, height, radius)
        self.cylinder_name = name

    def attach_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        cylinder_name = self.cylinder_name
        robot = self.robot
        scene = self.scene
        eef_link = self.eef_link
        group_names = self.group_names

        ## BEGIN_SUB_TUTORIAL attach_object
        ##
        ## Attaching Objects to the Robot
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## Next, we will attach the box to the Gen2 wrist. Manipulating objects requires the
        ## robot be able to touch them without the planning scene reporting the contact as a
        ## collision. By adding link names to the `touch_links array, we are telling the
        ## planning scene to ignore collisions between those links and the box. For the Gen2
        ## robot, we set `grasping_group = 'Gen2_hand'. If you are using a different robot,
        ## you should change this value to the name of your end effector group name.
        grasping_group = "gripper"
        touch_links = robot.get_link_names(group=grasping_group)
        scene.attach_box(eef_link, cylinder_name, touch_links=touch_links)
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(
            box_is_attached=True, box_is_known=False, timeout=timeout
        )

    def detach_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        cylinder_name = self.cylinder_name
        scene = self.scene
        eef_link = self.eef_link

        ## BEGIN_SUB_TUTORIAL detach_object
        ##
        ## Detaching Objects from the Robot
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## We can also detach and remove the object from the planning scene:
        scene.remove_attached_object(eef_link, name = cylinder_name)
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(
            box_is_known=True, box_is_attached=False, timeout=timeout
        )
    def home_position(self):
        move_group = self.move_group
        return move_group.get_current_joint_values()


def main():
    try:
        print("")
        print("----------------------------------------------------------")
        print("Movimiento de traslacion de una lata de Fanta")
        print("----------------------------------------------------------")

        input(
            "============ Presiona Enter para empezar el movimiento ..."
        )
        fanta = MoveGroupPythonInterfaceTutorial()
        #fanta.pinza(False) #abrimos pinza
        home_position = fanta.home_position() #obtenemos la posicion de home
        pos_fanta_x = 0.4
        pos_fanta_y = 0
        pos_fanta_z = 0
        rad_fanta = 0.07/2
        heigh_fanta = 0.12
        fanta.add_cylinder("lata_de_fanta", pos_fanta_x, pos_fanta_y, pos_fanta_z, rad_fanta, heigh_fanta)

        input(
            "============ Presiona Enter para acercar la pinza a la lata ..."
        )
        #pos1 = [0, 2.679, 0, 4.579, 0, 1.595, 0]
        #fanta.go_to_joint_state(pos1) #Me muevo a la coordenada (0.4 0 0.3) aprox
        fanta.move_to_xyz([pos_fanta_x, pos_fanta_y,pos_fanta_z + 0.3, 0, 1, 0, 0])

        input(
            "============ Presiona Enter para coger la lata ..."
        )
        fanta.move_down_z(0.25)
        #fanta.pinza(True)
        fanta.attach_box()

        input(
            "============ Presiona Enter para mover la lata ..."
        )
        fanta.move_down_z(-0.25)

        #pos2 = [-pi/2, 2.679, 0, 4.579, 0, 1.595, 0]
        #fanta.go_to_joint_state(pos2) #Me muevo a la coordenada (0 0.4 0.3) aprox
        fanta.move_to_xyz([0, pos_fanta_x,pos_fanta_z + 0.3, 0, 1, 0, 0])

        fanta.move_down_z(0.25)
        #fanta.pinza(False)
        fanta.detach_box()
        fanta.move_down_z(-0.25)

        input(
            "============ Presiona Enter para terminar ..."
        )
        fanta.go_to_joint_state(home_position) 
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == "__main__":
    main()
    
