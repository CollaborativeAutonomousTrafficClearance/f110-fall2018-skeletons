#!/usr/bin/env python

import rospy
from racecar_control.msg import drive_param
from nav_msgs.msg import Odometry
import math
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import os 
#from uncertainties import ufloat
#from uncertainties.umath import * 
#from gazebo_msgs.srv import GetModelState, GetModelStateRequest 
from std_msgs.msg import Header

#############
# CONSTANTS #
#############

VELOCITY = 0.4 # m/s

###########
# GLOBALS #
###########

awlBool = 0
tanyBool = 0
nosBool = 0


# waypoints - hard-coded for now

#path_points = [(float(-50), float(-0.2625), float(0.05)),
# (float(-49.5), float(-0.2625), float(0.05)),
 # (float(-49), float(-0.2625), float(0.05)),
  #(float(-48), float(-0.5), float(0.05)),
   #(float(-48), float(-0.75), float(0.05))]

#path_points = [ (float(-49), float(-0.2625), float(0.05)), 
 #               (float(-49), float(0.2625), float(0.05)),  
#                (float(-48.5), float(0.2625), float(0.05))]


path_points = [ (float(-50), float(0.2625), float(0.05)), 
                (float(-49.5), float(0.2625), float(0.05)),  
                (float(-49), float(0.2625), float(0.05))]


#path_points = [(float(-40), float(-0.7875), float(0.05)),
#               (float(-40+0.5), float(-0.7875), float(0.05)),
#               (float(-40+1), float(-0.7875), float(0.05))]
            

       
# Publisher for 'drive_parameters' (speed and steering angle)
pub = rospy.Publisher('/drive_parameters', drive_param, queue_size=1)

pub_time_start = rospy.Publisher('/myTimeS', Odometry, queue_size=1)
pub_time_mid = rospy.Publisher('/myTimeM', Odometry, queue_size=1)
pub_time_finish = rospy.Publisher('/myTimeF', Odometry, queue_size=1)




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

    # calculate the required steering angle
    if (abs((path_points[min_idx][1] - data.pose.pose.position.y)) >= 0.2):
        k = 0.1
    else:
        k = 0.005

    ks = 0.15

    angle    = (k/(ks+VELOCITY))*phi + psi
    
    angle = np.clip(angle, -0.4189, 0.4189) # 0.4189 radians = 24 degrees because car can only turn 24 degrees max

    # publish the drive_param msg
    msg = drive_param()
    msg.velocity = VELOCITY
    msg.angle = angle
    pub.publish(msg)

    checkTIME(data,yaw)



def checkTimeOlddddddddddddddddddddddd(data,yaw):

    InitandFinPts = [(path_points[0]) , (path_points[2])]
    uncertainity_error = 0.1
    if (InitandFinPts[0][0] == ufloat(data.pose.pose.position.x ,uncertainity_error ) and InitandFinPts[0][1] == ufloat(data.pose.pose.position.y, uncertainity_error) ): # initial point 
        ## start time 
        #t_i = time.localtime()
        #start_time = time.strftime("%H:%M:%S", t_i)
        #print(start_time)
        pub_time_start.publish(data)


    if (InitandFinPts[1][0] == ufloat(data.pose.pose.position.x,uncertainity_error) and InitandFinPts[1][1] == ufloat(data.pose.pose.position.y,uncertainity_error) and yaw <=1e-03 ): # final point 
        ## end time
        #t_f = time.localtime()
        #end_time = time.strftime("%H:%M:%S", t_f)
        #print(end_time)
        pub_time_start.publish(data)


def checkTIME(data,yaw):

    InitandFinPts = [(path_points[0]) , (path_points[2])]

    global awlBool
    global tanyBool
    global nosBool

    if (data.pose.pose.position.x > InitandFinPts[0][0]):
        if(awlBool == 0):
            pub_time_start.publish(data)
            awlBool = 1
    
    if (data.pose.pose.position.y >= 0):
        if(nosBool == 0):
            pub_time_mid.publish(data)
            nosBool = 1

    if ((data.pose.pose.position.x > InitandFinPts[1][0]) and (abs(yaw) <= 10e-02)):
        if(tanyBool == 0):
            pub_time_finish.publish(data)
            tanyBool = 1



    
if __name__ == '__main__':
    rospy.init_node('stanley_controller')
    rospy.Subscriber('/vesc/odom', Odometry, callback, queue_size=1)
    rospy.spin()
