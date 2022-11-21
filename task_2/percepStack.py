#! /usr/bin/env python3

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
# You can add more if required
##############################################################


# Initialize Global variables


################# ADD UTILITY FUNCTIONS HERE #################
pub_rgb = rospy.Publisher('/center_rgb', String, queue_size = 1)
pub_depth = rospy.Publisher('/center_depth', String, queue_size = 1)
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
    #status3 = cv2.imwrite('/home/greyless/Pictures/rgb.png',image)
    # cv2.imshow("rgb", image)
    # cv2.waitKey(0)
    ##########################################################################################
    pose = image_processing(image)
    pub_rgb.publish(str(pose))

def depth_clbck(depth_msg):
    '''
    Callback Function for Depth image topic

    Purpose:
	--- 
    1. Find the depth value of the centroid pixel returned by the
    image_processing() function.
    2. Publish the depth value to the topic '/center_depth'


    NOTE: the shape of depth and rgb image is different. 
    
    Input Args:
    -----
    depth_msg: Callback message.
    '''
    depth_val = []
    ############################### Add your code here #######################################
    pose = rospy.Subscriber("/center_rgb", String, pose_clbck)
    #print(position) 
    regex = '\d+'
    pos_list = re.findall(regex,position)
    pos_list = [pos_list[i:i+2] for i in range(0, len(pos_list), 2)]
    # print(pos_list)
    #print(pos_list)
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(depth_msg, "mono16")
    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(image, alpha=0.03), cv2.COLORMAP_JET)
    #status = cv2.imwrite('/home/greyless/Pictures/depth.png',depth_colormap)
    pts1 = np.float32([[234,197],[440,254],[308,201],[354,265]])
    pts2 = np.float32([[211,283],[662,391],[375,283],[456,445]])
    M = cv2.getPerspectiveTransform(pts1,pts2)
    new_depth = cv2.warpPerspective(image,M,(1280,720))
    for i in pos_list:
        x = int(i[0])
        y = int(i[1])
        depth_a = (new_depth[x,y])/1000
        depth_val.append(depth_a)

    # new_depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(new_depth, alpha=0.03), cv2.COLORMAP_JET)
    # cv2.imshow("depth", depth_colormap)
    # cv2.waitKey(0)
    # depth_image = cv2.resize(depth_colormap, (1280,720), interpolation = cv2.INTER_CUBIC)
    # status2 = cv2.imwrite('/home/greyless/Pictures/new_depth.png',new_depth_colormap)
    #depth_array = np.array(depth_image, dtype=np.float32)
    #print(depth_array.shape)
    #print('center depth:', depth_array[[271], [596]])
    #cv2.imshow("contours", image)
    #cv2.waitKey(0)
    # img = cv2.imread(image,-1)
    # print(depth_val)
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
    thresh=cv2.adaptiveThreshold(gray, 255,cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 11, 5)
    #cv2.imshow("Thresholded", thresh)
    # threshold, thresh= cv.threshold(gray, 180, 255, cv.THRESH_BINARY)
    # cv.imshow("thresh", thresh)
    mask=cv2.inRange(hsv, (0, 125, 100), (255,255,255))
    #cv2.imshow("mask", mask)
    contours, hierarchy= cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    for i in contours:
        M=cv2.moments(i)
        if M['m00']!=0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            if cv2.contourArea(i)>5000.0 and cv2.contourArea(i)<50000.0:
                cv2.drawContours(image, [i], 0, (0, 255, 0), 1)
                cv2.circle(image, (cx, cy), 7, (0, 0, 255), -1)
                cv2.putText(image, "center", (cx - 20, cy - 20),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
                # cv.drawContours(img, contours, -1, (0,0,255), 3)
                # c= max(contours, key=cv.contourArea)
                # x, y, w, h=cv.boundingRect(c)
                # cv.rectangle(img, (x, y), (x+w, y+h), (0,255,0))
                # cv2.imshow("contour",image)
                # cv2.waitKey(0)
                #print(cx,cy)
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
    for i in range(3):
        rgb_topic = "/device_0/sensor_1/Color_0/image/data_" + str(i+1)
        depth_topic = "/device_0/sensor_0/Depth_0/image/data_" + str(i+1)
        sub_image_color_1 = rospy.Subscriber(rgb_topic, Image, img_clbck)
        sub_image_depth_1 = rospy.Subscriber(depth_topic, Image, depth_clbck)
    
    #pub_rgb = rospy.Publisher('/center_rgb', String, queue_size = 1)
    #pub_depth = rospy.Publisher('/center_depth', String, queue_size = 1)

    ####################################################################################################
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print("Error:", str(e))
    finally:
        print("Executed Perception Stack Script")