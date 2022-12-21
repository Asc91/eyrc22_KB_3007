#!/usr/bin/env python3

'''
*****************************************************************************************
*
*               ===============================================
*                   Krishi Bot (KB) Theme (eYRC 2022-23)
*               ===============================================
*
*  This script is to implement Task 2.1 of Krishi Bot (KB) Theme (eYRC 2022-23).
*  
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or 
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''
# Team ID:          KB_3007
# Author List:      Dishie, Komal, Lakshaya, Mahesh
# Filename:         maniStack.py

####################### IMPORT MODULES #######################
import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import math
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from sensor_msgs.msg import Image
from std_msgs.msg import String
import numpy as np
import re
##############################################################


class ARM:

    # Constructor for arm
    def __init__(self):

        rospy.init_node('manipulator', anonymous=False)

        self._planning_group = "arm"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self._exectute_trajectory_client = actionlib.SimpleActionClient(
            'execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        self._box_name = ''

        # Current State of the Robot is needed to add box to planning scene
        self._curr_state = self._robot.get_current_state()

        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

    # function which moves end effector to given position
    def go_to_pose(self, arg_pose):
        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        self._group.set_pose_target(arg_pose)
        flag_plan = self._group.go(wait=True)  # wait=False for Async Move

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        if (flag_plan == True):
            rospy.loginfo(
                '\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found. "+ str(arg_pose) + '\033[0m')

        return flag_plan

    # function which moves end effector to predefined position
    def go_to_predefined_pose(self, arg_pose_name):
        rospy.loginfo('\033[94m' + "Going to Pose: {}".format(arg_pose_name) + '\033[0m')
        self._group.set_named_target(arg_pose_name)
        plan = self._group.plan()
        goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
        try:
            goal.trajectory = plan[1]
        except:
            goal.trajectory = plan
        self._exectute_trajectory_client.send_goal(goal)
        self._exectute_trajectory_client.wait_for_result()
        rospy.loginfo('\033[94m' + "Now at Pose: {}".format(arg_pose_name) + '\033[0m')

    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')


class GRIPPER:
    # Constructor for gripper
    def __init__(self):

        rospy.init_node('manipulator', anonymous=False)

        self._planning_group = "gripper"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self._exectute_trajectory_client = actionlib.SimpleActionClient(
            'execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        self._box_name = ''

        # Current State of the Robot is needed to add box to planning scene
        self._curr_state = self._robot.get_current_state()

        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

    def go_to_pose(self, arg_pose):

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        self._group.set_pose_target(arg_pose)
        flag_plan = self._group.go(wait=True)  # wait=False for Async Move
 
        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)
 
        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)
 
        if (flag_plan == True):
            rospy.loginfo(
                '\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found. " +'\033[0m')
 
        return flag_plan
 
    def go_to_predefined_pose(self, arg_pose_name):
        rospy.loginfo('\033[94m' + "Going to Pose: {}".format(arg_pose_name) + '\033[0m')
        self._group.set_named_target(arg_pose_name)
        plan = self._group.plan()
        goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
        try:
            goal.trajectory = plan[1]
        except:
            goal.trajectory = plan
        self._exectute_trajectory_client.send_goal(goal)
        self._exectute_trajectory_client.wait_for_result()
        rospy.loginfo('\033[94m' + "Now at Pose: {}".format(arg_pose_name) + '\033[0m')
 
    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')
 
 
 
def main():
    
    # initialising arm and gripper objects
    arm = ARM()
    gripper = GRIPPER()
 
    
 
    ######  mid1 can only see yellow from this point #######
    mid1 = geometry_msgs.msg.Pose()
    mid1.position.x = -0.01
    mid1.position.y = 0.0
    mid1.position.z = 0.82
    (m1x, m1y, m1z, m1w) = quaternion_from_euler(-3.138522, 0.0, -2.875968)
    mid1.orientation.x = m1x
    mid1.orientation.y = m1y
    mid1.orientation.z = m1z
    mid1.orientation.w = m1w
    
    arm.go_to_pose(mid1)
    
    ###### listening to broadcast ######
    yellow_listener = tf.TransformListener()
    yellow_listener.waitForTransform("ebot_base","fruit_yellow",rospy.Time(0), rospy.Duration(100.0))
    (trans_yellow,rot_yellow) = yellow_listener.lookupTransform('ebot_base', 'fruit_yellow', rospy.Time(0))

    y_px = round(trans_yellow[0], 2)
    y_py = round(trans_yellow[1], 2)
    y_pz = round(trans_yellow[2], 2)

    ###### intermidiate pose ######
    mid2 = geometry_msgs.msg.Pose()
    mid2.position.x = y_px
    mid2.position.y = y_py - 0.30
    mid2.position.z = y_pz + 0.1
    (m2x, m2y, m2z, m2w) = quaternion_from_euler(-3.138023, 0.000711, -2.682474)
    mid2.orientation.x = rot_yellow[0]
    mid2.orientation.y = rot_yellow[1]
    mid2.orientation.z = rot_yellow[2]
    mid2.orientation.w = rot_yellow[3]
    
    arm.go_to_pose(mid2)

    ###### Yellow Co-ordinates ######
    yellow = geometry_msgs.msg.Pose()
    yellow.position.x = y_px 
    yellow.position.y = y_py - 0.25
    yellow.position.z = y_pz - 0.01
    yellow.orientation.x = rot_yellow[0]
    yellow.orientation.y = rot_yellow[1]
    yellow.orientation.z = rot_yellow[2]
    yellow.orientation.w = rot_yellow[3]
    
    arm.go_to_pose(yellow)
    gripper.go_to_predefined_pose("close")

    ###### Final position for Yellow ######
    rest1 = geometry_msgs.msg.Pose()
    rest1.position.x = 0.25
    rest1.position.y = 0.10
    rest1.position.z = 0.86
    (q1x , q1y, q1z, q1w) = quaternion_from_euler(-2.688511, 0.066299,  1.773315)
    rest1.orientation.x = q1x
    rest1.orientation.y = q1y
    rest1.orientation.z = q1z
    rest1.orientation.w = q1w

    arm.go_to_predefined_pose("rest")
    arm.go_to_pose(rest1)
    gripper.go_to_predefined_pose("open")

    arm.go_to_predefined_pose("rest")

    ###### intermidiate position ######
    initial_pos = geometry_msgs.msg.Pose()
    initial_pos.position.x = 0.044949
    initial_pos.position.y = 0.100448
    initial_pos.position.z = 1.209144
    (ix, iy, iz, iw)= quaternion_from_euler(3.040491, -0.000100, -3.105546)
    initial_pos.orientation.x = ix
    initial_pos.orientation.y = iy
    initial_pos.orientation.z = iz
    initial_pos.orientation.w = iw

    arm.go_to_pose(initial_pos)    

    ###### Listening to broadcast ######
    red_listener = tf.TransformListener()
    red_listener.waitForTransform("ebot_base","fruit_red",rospy.Time(0), rospy.Duration(100.0))
    (trans_red,rot_red) = red_listener.lookupTransform('ebot_base', 'fruit_red', rospy.Time(0))

    r_px = round(trans_red[0], 2)
    r_py = round(trans_red[1], 2)
    r_pz = round(trans_red[2], 2)

    ###### Intermidiate Position for red ######
    mid4 = geometry_msgs.msg.Pose()
    mid4.position.x = r_px 
    mid4.position.y = r_py - 0.33
    mid4.position.z = r_pz - 0.06
    mid4.orientation.x = rot_red[0]
    mid4.orientation.y = rot_red[1]
    mid4.orientation.z = rot_red[2]
    mid4.orientation.w = rot_red[3]

    arm.go_to_pose(mid4)

    ####### 3d Co-ordinate for red ######
    red = geometry_msgs.msg.Pose()
    red.position.x = r_px 
    red.position.y = r_py - 0.25
    red.position.z = r_pz - 0.04
    red.orientation.x = rot_red[0]
    red.orientation.y = rot_red[1]
    red.orientation.z = rot_red[2]
    red.orientation.w = rot_red[3]

    arm.go_to_pose(red)
    gripper.go_to_predefined_pose("close")

    ###### Final position for red ######
    rest2 = geometry_msgs.msg.Pose()
    rest2.position.x = 0.296050
    rest2.position.y = 0.042699
    rest2.position.z = 0.825046
    (q2x , q2y, q2z, q2w) = quaternion_from_euler(3.151551, -0.000025,  1.002960)
    rest2.orientation.x = q2x
    rest2.orientation.y = q2y
    rest2.orientation.z = q2z
    rest2.orientation.w = q2w

    arm.go_to_predefined_pose("rest")
    arm.go_to_pose(rest2)
    gripper.go_to_predefined_pose("open")



if __name__ == '__main__':
    main()
