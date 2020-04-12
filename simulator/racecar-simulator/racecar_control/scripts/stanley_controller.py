#!/usr/bin/env python

import rospy
from racecar_control.msg import drive_param
from nav_msgs.msg import Odometry
import math
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import os 

#############
# CONSTANTS #
#############

VELOCITY = 0.15 # m/s

###########
# GLOBALS #
###########

# waypoints - hard-coded for now

path_points = [(float(-49), float(0.2625), float(0.05)), (float(-48), float(0.2625), float(0.05)), (float(-47), float(0.2625), float(0.05)), (float(-46), float(0.2625), float(0.05)), (float(-45), float(0.2625), float(0.05)), (float(-44), float(0.2625), float(0.05)), (float(-43), float(0.2625), float(0.05)), (float(-42), float(0.2625), float(0.05)), (float(-41), float(0.2625), float(0.05)), (float(-40), float(0.2625), float(0.05)), (float(-39), float(0.2625), float(0.05)), (float(-38), float(0.2625), float(0.05)), (float(-37), float(0.2625), float(0.05)), (float(-36), float(0.2625), float(0.05)), (float(-35), float(0.2625), float(0.05)), (float(-34), float(0.2625), float(0.05)), (float(-33), float(0.2625), float(0.05))]
       
# Publisher for 'drive_parameters' (speed and steering angle)
pub = rospy.Publisher('/drive_parameters', drive_param, queue_size=1)


#############
# FUNCTIONS #
#############
    
# Input data is Odometry message from topic /vesc/odom
# Runs stanley controller and publishes velocity and steering angle
def callback(data):

    # Find the path point closest to the vehicle
    min_idx       = 0
    min_dist      = float("inf")

    for i in range(len(path_points)):
        dist = np.linalg.norm(np.array([
                path_points[i][0] - data.pose.pose.position.x,
                path_points[i][1] - data.pose.pose.position.y]))
        if dist < min_dist:
            min_dist = dist
            min_idx = i

    if min_idx == len(path_points)-1:
        min_idx = min_idx - 1           
    

    # determine the current yawof the vehicle
    orientation_q = data.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)

    # calculate the heading
    psi = -yaw + np.arctan2((path_points[min_idx+1][1]-path_points[min_idx][1]),(path_points[min_idx+1][0]-path_points[min_idx][0]))

    # calculate an angle proportional to the cross-track error
    phi1 = np.arctan2((path_points[min_idx][1] - data.pose.pose.position.y), path_points[min_idx][0] - data.pose.pose.position.x)
    phi2 = np.arctan2((path_points[min_idx+1][1] - path_points[min_idx][1]), (path_points[min_idx+1][0] - path_points[min_idx][0]))

    phi = (np.mod(phi1 + np.pi, 2 * np.pi) - np.pi) - (np.mod(phi2 + np.pi, 2 * np.pi) - np.pi)
    phi = np.mod(phi + np.pi, 2 * np.pi) - np.pi

    # calculate the re2uired steering angle
    k = 0.005
    ks = 0.1
    angle    = (k/(ks+VELOCITY))*phi + psi
    
    angle = np.clip(angle, -0.4189, 0.4189) # 0.4189 radians = 24 degrees because car can only turn 24 degrees max

    # publish the drive_param msg
    msg = drive_param()
    msg.velocity = VELOCITY
    msg.angle = angle
    pub.publish(msg)
    
if __name__ == '__main__':
    rospy.init_node('stanley_controller')
    rospy.Subscriber('/vesc/odom', Odometry, callback, queue_size=1)
    rospy.spin()
