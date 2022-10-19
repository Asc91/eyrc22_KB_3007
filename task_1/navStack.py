#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


# dividing lidar readings in useful regions
regions = {
        "bright": 0,
        "fright": 0,
        "front": 0,
        "fleft": 0,
        "bleft": 0,
    }

# max range of lidar
range_max = 0


# callback function which returns updated distance of nearest obstacle in given region
def laser_callback(msg):
    global range_max 
    range_max= msg.range_max
    
    global regions
    regions = {
        "bright":  min(min(msg.ranges[0:7]), range_max),
        "fright":  min(min(msg.ranges[8:339]), range_max),
        "front":   min(min(msg.ranges[340:379]), range_max),
        "fleft":   min(min(msg.ranges[380:711]), range_max),
        "bleft":   min(min(msg.ranges[712:719]), range_max),
    }


# main control loop
def control_loop():

    # initialize node
    rospy.init_node('ebot_controller')
    
    # publish velocity to cmd_vel
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # subscribe to laser scan topic
    rospy.Subscriber('/ebot/laser/scan', LaserScan, laser_callback)
    
    rate = rospy.Rate(10) 
    velocity_msg = Twist()
    velocity_msg.linear.x = 0
    velocity_msg.angular.z = 0
    pub.publish(velocity_msg)

    # status of bot
    aligned_to_lane = 1
    lane_counter = 0

    # variables used for PD controller
    
    prev_error = 0
    c_error = 0
    while not rospy.is_shutdown():
        
        # bot is aligned with lane if lot of space in front region
        if regions["front"] > 5:
            prev_status = aligned_to_lane
            aligned_to_lane = 1

            # counting lane if status changed
            if prev_status + aligned_to_lane == 1:
                lane_counter = lane_counter + 1

        # consider bot to be unaligned if distance in front region is less
        if regions["front"] < 2.5:
            aligned_to_lane = 0
        print("lane counter = ", lane_counter)

        # error for middle lane and other lanes
        if (regions["bleft"] < 1 and regions["bright"] < 1):
            error = regions["bleft"] - regions["bright"]

        elif regions["front"]<2.5 and lane_counter >= 1:
            error = 0.65 - regions["bright"]

        # Derivative error
        d_error = (error - prev_error)/0.1
        
        # Cumulative error
        c_error = c_error + error
        # values of kp and kd constants
        Kd = 0.07
        Kp = 1.6
        Ki = 0

        # Condition for going straight straight 
        if aligned_to_lane == 1:
            linear_vel = 1

            # condition for skipping middle lane
            if regions["bright"] < 4:
                if error < 0:
                    angular_vel = max(error*Kp + d_error*Kd + c_error*Ki, -0.6)
                elif error > 0:
                    angular_vel = min(error*Kp + d_error*Kd + c_error*Ki, 0.6)
                elif error == 0:
                    angular_vel = 0
            else:
                angular_vel = 0

        # condition for turning left
        if aligned_to_lane != 1 :
            linear_vel = 0.3
            if error < 0:
                angular_vel = max(error*Kp + d_error*Kd + c_error*Ki, -1)
            elif error > 0:
                angular_vel = min(error*Kp + d_error*Kd + c_error*Ki, 1)
            elif error == 0:
                angular_vel = 0

        # condition for stopping
        if regions["front"] < 2 and regions["bright"] > 7 and lane_counter > 2:
            linear_vel= 0 
            angular_vel = 0

        # chanig prev_error after all calculations
        prev_error = error

        print(linear_vel, angular_vel)
        # publishing velocity msg
        velocity_msg.linear.x = linear_vel
        velocity_msg.angular.z = angular_vel
        pub.publish(velocity_msg)
        print("Controller message pushed at {}".format(rospy.get_time()))
        rate.sleep()

    



if __name__ == '__main__':
    try:
        control_loop()
    except rospy.ROSInterruptException:
        pass