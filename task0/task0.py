#!/usr/bin/env python3

'''
*****************************************************************************************
*
*        		===============================================
*           		Krishi Bot (KB) Theme (eYRC 2022-23)
*        		===============================================
*
*  This script should be used to implement Task 0 of Krishi Bot (KB) Theme (eYRC 2022-23).
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
# Filename:			task_0.py
# Functions:		callback(data), main() 
# Nodes:		    node_myMsg_listener


####################### IMPORT MODULES #######################
import rospy
import sys
import traceback


from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
##############################################################

################# ADD GLOBAL VARIABLES HERE #################
PI = 3.1415926535897

y = 0
angle = 0
x_vel = 0
angle_vel = 0
y_top = 0
##############################################################

################# ADD UTILITY FUNCTIONS HERE #################




##############################################################

def callback(data):

    """
	Purpose:
	---
	This function should be used as a callback. Refer Example #1: Pub-Sub with Custom Message in the Learning Resources Section of the Learning Resources.
    You can write your logic here.
    NOTE: Radius value should be 1. Refer expected output in document and make sure that the turtle traces "same" path.

	Input Arguments:
	---
        `data`  : []
            data received by the call back function

	Returns:
	---
        May vary depending on your logic.
	
	Example call:
	---
        Depends on the usage of the function.
    """
    global y
    global y_top
    global angle
    global x_vel
    global angle_vel

    #storing data from turtle1/pose in y and angle
    y = data.y

    if data.theta >= 0:
        angle = data.theta
    else:
        angle = 2*PI + data.theta
    
    print(data.theta)

    #following semicircle of radius 1 
    if angle < PI:
            angle_vel = 1
            x_vel = 1
            print("My turtleBot is moving in circle")

    #rotating 90 degrees   
    if angle >= PI and angle < 3*PI/2:
        y_top = y
        angle_vel = 1
        x_vel = 0
        print("My turtleBot is rotating")

    #Going straight to initial position
    if angle >= 3*PI/2 and y_top - y < 2:
        x_vel = 1
        angle_vel = 0
        print("My turtleBot is moving straight")

    #stopping at initial position     
    if y_top - y >= 2:
        x_vel = 0
        angle_vel = 0
        rospy.spin()
    
    
    
def main():
    """
	Purpose:
	---
	This function will be called by the default main function given below.
    You can write your logic here.

	Input Arguments:
	---
        None

	Returns:
	---
        None
	
	Example call:
	---
        main()
    """    
    #initialising node
    rospy.init_node('move', anonymous=True)
    
    #getting ready to publish to cmd_vel
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

    #subscribing to pose
    sub = rospy.Subscriber('/turtle1/pose', Pose, callback)

    rate = rospy.Rate(100) #update rate in Hz
    
    #publishing to cmd_vel
    while not rospy.is_shutdown():
        vel_msg = Twist()
        vel_msg.angular.z = angle_vel
        vel_msg.angular.x = 0  
        vel_msg.angular.y = 0


        vel_msg.linear.x = x_vel
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        
        pub.publish(vel_msg)
        rate.sleep()
        
        
######### YOU ARE NOT ALLOWED TO MAKE CHANGES TO THIS PART #########    
if __name__ == "__main__":
    try:
        print("------------------------------------------")
        print("         Python Script Started!!          ")
        print("------------------------------------------")
        main()
        
    except:
        print("------------------------------------------")
        traceback.print_exc(file=sys.stdout)
        print("------------------------------------------")
        sys.exit()

    finally:
        print("------------------------------------------")
        print("    Python Script Executed Successfully   ")
        print("------------------------------------------")
