#!/usr/bin/env python3
 
import rospy
import numpy as np
import moveit_commander
import geometry_msgs.msg
 
# Inicializar ROS y MoveIt
rospy.init_node("kinova_joint_mover", anonymous=True)
moveit_commander.roscpp_initialize([])
 
# Inicializar interfaces de MoveIt
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("arm")  # Nombre del grupo cinemático de Kinova
 
pose_target = geometry_msgs.msg.Pose()
pose_target.position.x = 0.4
pose_target.position.y = 0.0
pose_target.position.z = 0.3
pose_target.orientation.w = 1.0

group.set_pose_target(pose_target)
plan = group.plan()
print(plan.joint_trajectory.points[-1].positions)
