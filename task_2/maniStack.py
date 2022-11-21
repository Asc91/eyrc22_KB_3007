#! /usr/bin/env python


'''
*****************************************************************************************
*
*        		===============================================
*           		Krishi Bot (KB) Theme (eYRC 2022-23)
*        		===============================================
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
# Team ID:			KB_3007
# Author List:		Mahesh Tupe
# Filename:			maniStack.py


# importing required libraries
import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler

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

    # intermidiate, fruit and final positions for end effector
    mid1 = geometry_msgs.msg.Pose()
    mid1.position.x = 0.0
    mid1.position.y = 0.35
    mid1.position.z = 0.78
    (qx, qy, qz, qw) = quaternion_from_euler(2.959619, 0.000456, 3.122683)
    mid1.orientation.x = qx
    mid1.orientation.y = qy
    mid1.orientation.z = qz
    mid1.orientation.w = qw

    y_pepper = geometry_msgs.msg.Pose()
    y_pepper.position.x = 0.0
    y_pepper.position.y = 0.45
    y_pepper.position.z = 0.78
    y_pepper.orientation.x = qx
    y_pepper.orientation.y = qy
    y_pepper.orientation.z = qz
    y_pepper.orientation.w = qw
    
    rest1 = geometry_msgs.msg.Pose()
    rest1.position.x = 0.25
    rest1.position.y = 0.10
    rest1.position.z = 0.86
    (q1x , q1y, q1z, q1w) = quaternion_from_euler(-2.688511, 0.066299,  1.773315)
    rest1.orientation.x = q1x
    rest1.orientation.y = q1y
    rest1.orientation.z = q1z
    rest1.orientation.w = q1w
    
    (q3x, q3y, q3z, q3w) = quaternion_from_euler(3.060027, 0.024887, 2.877658)
    mid2 = geometry_msgs.msg.Pose()
    mid2.position.x = 0.181191
    mid2.position.y = 0.377814
    mid2.position.z = 1.242333
    mid2.orientation.x = q3x
    mid2.orientation.y = q3y
    mid2.orientation.z = q3z
    mid2.orientation.w = q3w

    (q3x, q3y, q3z, q3w) = quaternion_from_euler(2.697585, 0.078679, 2.915555)
    r_pepper = geometry_msgs.msg.Pose()
    r_pepper.position.x = 0.240755
    r_pepper.position.y = 0.504363
    r_pepper.position.z = 1.184131
    r_pepper.orientation.x = q3x
    r_pepper.orientation.y = q3y
    r_pepper.orientation.z = q3z
    r_pepper.orientation.w = q3w

    rest2 = geometry_msgs.msg.Pose()
    rest2.position.x = 0.296050
    rest2.position.y = 0.042699
    rest2.position.z = 0.825046
    (q2x , q2y, q2z, q2w) = quaternion_from_euler(3.151551, -0.000025,  1.002960)
    rest2.orientation.x = q2x
    rest2.orientation.y = q2y
    rest2.orientation.z = q2z
    rest2.orientation.w = q2w

    while not rospy.is_shutdown():
        
        # making sure gripper is open
        gripper.go_to_predefined_pose("open")

        #picking yellow pepper
        arm.go_to_pose(mid1)
        arm.go_to_pose(y_pepper)
        gripper.go_to_predefined_pose("close")

        #placing yellow pepper in basket
        arm.go_to_pose(mid1)
        arm.go_to_pose(rest1)
        gripper.go_to_predefined_pose("open")

        # picking red pepper
        arm.go_to_pose(mid2)
        arm.go_to_pose(r_pepper)
        gripper.go_to_predefined_pose("close")
       
        # placing red papper in basket
        arm.go_to_pose(mid2)
        arm.go_to_predefined_pose("rest")
        arm.go_to_pose(rest2)
        gripper.go_to_predefined_pose("open")
    
        exit()
    del arm
    del gripper


if __name__ == '__main__':
    main()
