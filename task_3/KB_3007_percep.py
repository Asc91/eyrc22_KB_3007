#!/usr/bin/env python3

'''
*****************************************************************************************
*
*        		===============================================
*           		Krishi Bot (KB) Theme (eYRC 2022-23)
*        		===============================================
*
*  This script is to implement Task 2.2 of Krishi Bot (KB) Theme (eYRC 2022-23).
*  
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or 
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:			KB#3007
# Author List:		Dishie, Komal, Lakshaya, Mahesh
# Filename:			percepStack.py
# Functions:		
# 					pose_clbck, img_clbck, depth_clbck, image_processing, main


####################### IMPORT MODULES #######################
import cv2 
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String
import numpy as np
import re
import tf
import geometry_msgs.msg
from std_msgs.msg import Header
from geometry_msgs.msg import Point
import math
from geometry_msgs.msg import PointStamped
##############################################################



################# ADD UTILITY FUNCTIONS HERE #################

# Topics on which initial data will be published for further calculation
pub_yellow = rospy.Publisher('/center_yellow', String, queue_size = 1)
pub_red = rospy.Publisher('/center_red', String, queue_size = 1)
pub_rgb = rospy.Publisher('/center_rgb', String, queue_size = 1)
pub_depth = rospy.Publisher('/center_depth', String, queue_size = 1)


# callback function for above topics
red_position = ""
def red_pose_clbck(pose_msg):
    global red_position
    red_position = pose_msg.data

yellow_position = ""
def yellow_pose_clbck(pose_msg):
    global yellow_position
    yellow_position = pose_msg.data

position = ""
def pose_clbck(pose_msg):
    global position
    position = pose_msg.data

##############################################################



def img_clbck(img_msg):
    '''
    Callback Function for RGB image topic

    Purpose:
    -----
    Convert the image in a cv2 format and then pass it 
    to image_processing function by saving to the 
    'image' variable.

    Input Args:
    -----
    img_msg: Callback message.
    '''
    global pub_rgb #, add global variable if any
    
    ############################### Add your code here #######################################

    bridge = CvBridge()
    image= bridge.imgmsg_to_cv2(img_msg, "bgr8")
    status3 = cv2.imwrite('/home/greyless/Pictures/rgb.png',image)
   
    ##########################################################################################
    pose = image_processing(image)
    pub_rgb.publish(str(pose))
   

def depth_clbck(depth_msg):
    '''
    Callback Function for Depth image topic

    Purpose:
	--- 
    1. Find the depth value of the centroid pixel returned by the
    image_processing() function.status3 = cv2.imwrite('/home/greyless/Pictures/rgb.png',image)
    2. Publish the depth value to the topic '/center_depth'


    NOTE: the shape of depth and rgb image is different. 
    
    Input Args:
    -----
    depth_msg: Callback message.
    '''
    depth_val = []
    ############################### Add your code here #######################################

    # tf listener
    red_listener = tf.TransformListener()
    red_listener.waitForTransform("camera_rgb_frame2","ebot_base",rospy.Time(0), rospy.Duration(2.0))
    yellow_listener = tf.TransformListener()
    yellow_listener.waitForTransform("camera_rgb_frame2","ebot_base",rospy.Time(0), rospy.Duration(2.0))

    # subscribing to initial data containing 2d center coordinates
    pose = rospy.Subscriber("/center_rgb", String, pose_clbck)
    pose_yellow = rospy.Subscriber("/center_yellow", String, yellow_pose_clbck)
    pose_red = rospy.Subscriber("/center_red", String, red_pose_clbck)

    regex = '\d+'
   
    yellow_pos_list = re.findall(regex,yellow_position)
    yellow_pos_list = [yellow_pos_list[i:i+2] for i in range(0, len(yellow_pos_list), 2)]

    red_pos_list = re.findall(regex,red_position)
    red_pos_list = [red_pos_list[i:i+2] for i in range(0, len(red_pos_list), 2)]

    # converting ros image to cv image
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(depth_msg, "32FC1")

    # focal length of camera
    fx = 554.3827128226441
    fy = 554.3827128226441
    # center of camera sensor
    camx = 320.5
    camy = 240.5
    
    # finding depth for given center of bell pepper
    for i in red_pos_list:
        x = int(i[0])
        y = int(i[1])
        
        depth_a = (image[y,x])

        X = depth_a * ((x - camx)/fx) 
        Y = depth_a * ((y - camy)/fy)
        Z = math.sqrt(depth_a*depth_a - X*X - Y*Y) 

        my_header = Header(stamp=rospy.Time(0), frame_id='camera_rgb_frame2')
        my_point = Point(X, Y, Z)
        my_point_stamped = PointStamped(header=my_header, point=my_point)
        red_coords = red_listener.transformPoint('ebot_base',my_point_stamped)
        
        # broadcasting tf only if depth is less than 1 meter
        if depth_a < 1:
            red = tf.TransformBroadcaster()
            red.sendTransform((red_coords.point.x, red_coords.point.y, red_coords.point.z),(0.0, 0.0, 0.0, 1.0),rospy.Time.now(),"fruit_red","ebot_base")

    # finding depth for given center of bell pepper
    for i in yellow_pos_list:
        x = int(i[0])
        y = int(i[1])
        
        depth_a = (image[y,x])

        X = depth_a * ((x - camx)/fx) 
        Y = depth_a * ((y - camy)/fy)
        Z = math.sqrt(depth_a*depth_a - X*X - Y*Y) 
      
        my_header = Header(stamp=rospy.Time(0), frame_id='camera_rgb_frame2')
        my_point = Point(X, Y, Z)
        my_point_stamped = PointStamped(header=my_header, point=my_point)
        yellow_coords = yellow_listener.transformPoint('ebot_base',my_point_stamped)

        yellow = tf.TransformBroadcaster()
        yellow.sendTransform((yellow_coords.point.x, yellow_coords.point.y, yellow_coords.point.z),(0.0, 0.0, 0.0, 1.0),rospy.Time.now(),"fruit_yellow","ebot_base")
    ##########################################################################################
    pub_depth.publish(str(depth_val))


def image_processing(image):
    '''
    NOTE: Do not modify the function name and return value.
          Only do the changes in the specified portion for this
          function.
          Use cv2.imshow() for debugging but make sure to REMOVE it before submitting.
    
    1. Find the centroid of the bell pepper(s).
    2. Add the x and y values of the centroid to a list.  
    3. Then append this list to the pose variable.
    3. If multiple fruits are found then append to pose variable multiple times.

    Input Args:
    ------
    image: Converted image in cv2 format.

    Example:
    ----
    pose = [[x1, y1] , [x2, y2] ...... ]
    '''
    pose = []
    ############### Write Your code to find centroid of the bell peppers #####################
    
    gray=cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    hsv=cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    lower_yellow=np.array([7, 152, 195])#lower HSV threshold for red colour
    upper_yellow=np.array([17, 255, 255])#upper HSV threshold for red colour
    lower_red=np.array([0, 70, 50])#lower HSV threshold for yellow colour
    upper_red=np.array([10, 255, 150])#upper HSV threshold for yellow colour

    mask1=cv2.inRange(hsv, lower_yellow, upper_yellow)#binarizing image to detect yellow colour-hue value is varied
    mask2=cv2.inRange(hsv, lower_red, upper_red)#binarizing image to detect red colour-hue value is varied
    
    c1, h1=cv2.findContours(mask1, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)#detecting contours in yellow masked image
    for i in c1:
        M=cv2.moments(i)
        if M['m00']!=0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            if cv2.contourArea(i)>100.0 and cv2.contourArea(i)<50000.0:
                pub_yellow.publish(str([cx,cy]))
                pose.append([cx,cy])
                

    c2, h2=cv2.findContours(mask2, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)#detecting contours for red masked image
    for i in c2:
        M=cv2.moments(i)
        if M['m00']!=0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            if cv2.contourArea(i)>700.0 and cv2.contourArea(i)<50000.0:
                pub_red.publish(str([cx,cy]))
                pose.append([cx,cy])
                

    ##########################################################################################
    return pose



def main():
    '''
    MAIN FUNCTION

    Purpose:
    -----
    Initialize ROS node and do the publish and subscription of data.

    NOTE: We have done the subscription only for one image, you have to iterate over 
    three images in the same script and publish the centroid and depth in the 
    same script for three images, calling the same callback function.

    '''

    #### EDIT YOUR CODE HERE FOR SUBSCRIBING TO OTHER TOPICS AND TO APPLY YOUR ALGORITHM TO PUBLISH #####
    rospy.init_node("percepStack", anonymous=True)
        
    sub_image_color_1 = rospy.Subscriber("/camera/color/image_raw2", Image, img_clbck)
    sub_image_depth_1 = rospy.Subscriber("/camera/depth/image_raw2", Image, depth_clbck)


    ####################################################################################################
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print("Error:", str(e))
    finally:
        print("Executed Perception Stack Script")