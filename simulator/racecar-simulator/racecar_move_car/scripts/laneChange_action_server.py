#!/usr/bin/env python

import rospy
import actionlib
import math
import numpy as np
import os
from racecar_move_car.msg import  MoveCarAction, MoveCarGoal, MoveCarFeedback, MoveCarResult
from racecar_control.msg import drive_param
from racecar_communication.msg import IDsCombined
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

##------------------------------------------------------------------------------------------##
##------------------------------------------------------------------------------------------##
##------------------------------------------------------------------------------------------##
##                           L A N E    C H A N G E    C L A S S                            ##
##------------------------------------------------------------------------------------------##
##------------------------------------------------------------------------------------------##
##------------------------------------------------------------------------------------------##
class LaneChange:

    def __init__(self):


        self.direction = 0 # int representing whether lane change is requested and what is the required direction (0:= no lane change requested, 1:= left lane change requested, 2:= right lane change requested)
        self.feasibility = -1 # int representing whether lane change is feasible (-1:= initialization value 0:= lane change is infeasible, 1:= lane change is feasible)
        self.finished = -1 # int representing whether lane change is finished (-1:= initialization value 0:= lane change still being executed, 1:= lane change finished)
        
        self.lc_default_vel = 0.1 # ego vehicle's default lane change velocity 
        self.lc_vel = 0 # lane change velocity
        self.lc_time = 0 # expected time needed to perform lane change
        self.lc_halfTime = 0 # expected half time needed to perform lane change
        self.lc_vdist = 0 # vertical distance expected to be covered during lane change
        self.lc_halfvdist = 0 # half vertical distance expected to be covered during lane change
        self.lc_waypoints = [] # the waypoints to follow using Stanley controller in order to perform lane change

        '''          
        ---A--------B---> x of the lane the ego vehicle seeks 
        ---C--*-----D---> x of ego vehicle's current lane 

        *:= the ego vehicle
        A:= the vehicle directly behind the ego vehicle when it reaches the lane it seeks 
        B:= the vehicle directly in front of the ego vehicle when it reaches the lane it seeks 
        C:= the vehicle directly behind the ego vehicle in its current lane 
        D:= the vehicle directly in front the ego vehicle in its current lane 
        '''
        self.isAlone = 1 # bool representing whether there are other vehicles around our ego vehicle (1:= no other vehicles, 0:= there are other vehicles)
        self.vehicle_ids = IDsCombined() # a variable regularly updated with the communicated array of vehicle ids
        self.my_vehicle_ids = IDsCombined() # the array of vehicle ids currently used to investigate feasibility
        self.vehicle_A = -1 # int representing the index of vehicle A in self.my_vehicle_ids (-1:= there is no vehicle A)
        self.vehicle_B = -1 # int representing the index of vehicle B in self.my_vehicle_ids (-1:= there is no vehicle B)
        self.vehicle_D = -1 # int representing the index of vehicle D in self.my_vehicle_ids (-1:= there is no vehicle D)

        '''  
        Lane number convention in the "threeLanes" world:
            y^  
            |--------------> x      lane number 0 
            |--------------> x      lane number 1
            |--------------> x      lane number 2 
        '''
        self.curr_lane = 0 # ego vehicle current lane number
        self.to_go_lane = 0 # ego vehicle to go lane number
        self.my_x_pos = 0 # ego vehicle current x position (in global frame)
        self.my_y_pos = 0 # ego vehicle current y position (in global frame)
        self.my_yaw = 0 # ego vehicle current yaw

        ''' Variables needed by Krauss model '''
        self.t_r = 1 # estimated driver reaction time ~ 1 sec
        self.max_vel = 0.5 # maximum ego vehicle velocity allowed ################## rospy.get_param('')
        self.max_acc = 0.1 # maximum ego vehicle acceleration allowed ############## rospy.get_param('')
        self.max_dec = -0.1 # maximum ego vehicle deceleration allowed ############## rospy.get_param('')
        self.vehicle_length = 0.58 # length of vehicles - please adjust this value if another vehicle model is used ################## rospy.get_param('')
        self.del_time = 1 # chosen delta time between iterations ################## rospy.get_param('')

        self.pub = rospy.Publisher('drive_parameters', drive_param, queue_size=1) # publisher for 'drive_parameters' (speed and steering angle)


    # Calculates which lane the ego vehicle is in and to which it needs to go, assuming we are in the threeLanes world
    # Returns false if the lane the ego vehicle needs to go is outside the world
    def whichLanes(self):

        # determining current lane
        if (self.my_y_pos >= 0):
            self.curr_lane = 0
        elif (self.my_y_pos < -0.525):
            self.curr_lane = 2;   
        else:
            self.curr_lane = 1

        # determining to-go lane
        if (self.direction == 1):
            self.to_go_lane = self.curr_lane - 1
        elif (self.direction == 2):
            self.to_go_lane = self.curr_lane + 1

        if ((self.to_go_lane < 0) or (self.to_go_lane > 2)):
            return False
        else:
            return True


    # Determines the required time and vertical distance needed for lane change (based on the vehicle's speed)
    def setLCTimeAndVdist(self):

        if (abs(self.lc_vel - 0.1) <= 0.05):
            self.lc_time = 15
            self.lc_halfTime = 7
            self.lc_vdist = 1.367
            self.lc_halfvdist = 0.681
            return 

        if (abs(self.lc_vel - 0.2) <= 0.05):
            self.lc_time = 9
            self.lc_halfTime = 4
            self.lc_vdist = 1.631
            self.lc_halfvdist = 0.819
            return

        if (abs(self.lc_vel - 0.3) <= 0.05):
            self.lc_time = 6
            self.lc_halfTime = 3
            self.lc_vdist = 1.754
            self.lc_halfvdist = 0.984
            return

        if (abs(self.lc_vel - 0.4) <= 0.05):
            self.lc_time = 4
            self.lc_halfTime = 2
            self.lc_vdist = 1.855
            self.lc_halfvdist = 1.108
            return
        
        if (abs(self.lc_vel - 0.5) <= 0.05):
            self.lc_time = 4
            self.lc_halfTime = 2
            self.lc_vdist = 1.976
            self.lc_halfvdist = 1.23
            return
        

    # Calculates the maximum velocity the rear vehicle could have in order to keep a safe distance between it and the leading vehicle
    def calcMaxSafeVel(self, rear_x_pos, rear_vel, lead_x_pos, lead_vel):

        lead_gap = lead_x_pos - rear_x_pos - self.vehicle_length

        safe_vel = lead_vel + ((lead_gap - (lead_vel * self.t_r))/(self.t_r + ((lead_vel + rear_vel)/(2*self.max_acc))))

        #max_possible_safe_vel = min(self.max_vel, (rear_vel + self.max_acc*self.del_time), safe_vel)

        return safe_vel


    # Checks the feasibility of the required lane change
    def isLCFeasible(self):

        # if there are no other vehicles around the ego vehicle, then indicate that lane change is feasible
        if (self.isAlone):
            return True

        # determine which vehicles are in the position of the A, B or D vehicles (if any)
        # returns false if there is a vehicle in a critical position such that a safe lane change is not guaranteed 
        if (not self.getVehiclesABD()):
            return False

        # check if it is safe to lane change given the presence of a vehicle in position D (if any)
        if (self.isSafeWRTD()):
            # check if it is safe to lane change given the presence of a vehicle in position A (if any)
            if (self.isSafeWRTA()):
            # check if it is safe to lane change given the presence of a vehicle in position B (if any)
                if (self.isSafeWRTB()):
                    # indicate that lane change is feasible and return
                    return True

        # indicate that lane change is infeasible and return
        return False


    # Determines which vehicles are in the position of the A, B or D vehicles (if any)
    def getVehiclesABD(self):
        
        # initialize indices of vehicles A,B and D with -1 (to be updated below)
        tempA = -1
        tempB = -1
        tempD = -1
        
        # loop on vehicles in my_vehicle_ids list
        for i in range(0,len(self.my_vehicle_ids.ids),1):

            # check for vehicle D (leading in ego vehicle current lane) 
            if ((self.my_vehicle_ids.ids[i].lane_num == self.curr_lane) and (self.my_vehicle_ids.ids[i].x_position >= self.my_x_pos)):
                if ((tempD == -1) or (self.my_vehicle_ids.ids[i].x_position < self.my_vehicle_ids.ids[tempD].x_position)):
                    tempD = i
            
            # check for vehicles A or B (in ego vehicle to go lane) 
            elif (self.my_vehicle_ids.ids[i].lane_num == self.to_go_lane):
                result = self.isAOrB(i)
                if (result == 0):
                    return False
                elif (result == 1):
                    if ((tempA == -1) or (self.my_vehicle_ids.ids[i].x_position > self.my_vehicle_ids.ids[tempA].x_position)):
                        tempA = i
                elif (result == 2):
                    if ((tempB == -1) or (self.my_vehicle_ids.ids[i].x_position < self.my_vehicle_ids.ids[tempB].x_position)):
                        tempB = i

        self.vehicle_A = tempA
        self.vehicle_B = tempB
        self.vehicle_D = tempD
        rospy.loginfo("Found vehicle A at index: %d",tempA)
        rospy.loginfo("Found vehicle B at index: %d",tempB)
        rospy.loginfo("Found vehicle D at index: %d",tempD)

        return True


    # Determines whether a certain vehicle is in the position of A or B
    # Return value: 0:= infeasible 1:= vehicle A 2:= vehicle B
    def isAOrB(self, vehicleIndex): 

        # calculate center line of lane change trajectory
        lc_centerline = self.lc_halfvdist + self.my_x_pos

        # calculate number of timesteps taken by ego vehicle to reach centerline 
        LCnoOfTS_half = float(self.lc_halfTime) / float(self.del_time)
        LCnoOfTS_half = math.ceil(LCnoOfTS_half)
        LCnoOfTS_half = int(LCnoOfTS_half)

        # get the initial positon and velocity for the input vehicle
        givenVehicle_initial_POS = self.my_vehicle_ids.ids[vehicleIndex].x_position
        givenVehicle_initial_VEL = self.my_vehicle_ids.ids[vehicleIndex].velocity

        # test if the input vehicle is vehicle B #

        # assume vehicle applies maximum deceleration during all timesteps the ego vehicle takes to reach centerline. (Before the center line)
        hypoth_VEL = givenVehicle_initial_VEL 
        hypoth_POS = givenVehicle_initial_POS 

        for i in range(0,LCnoOfTS_half,1):
            temp_VEL = hypoth_VEL + (self.max_dec * self.del_time)
            if(temp_VEL >= 0):
                hypoth_POS = hypoth_POS + (hypoth_VEL * self.del_time) + 0.5*self.max_dec*self.del_time*self.del_time
                hypoth_VEL = temp_VEL
            else:
                hypoth_POS = hypoth_POS + (0.5 * (hypoth_VEL + 0) * self.del_time)
                hypoth_VEL = 0   

        # check if input vehicle is SURELY B
        if (hypoth_POS > lc_centerline):
            return 2
        
        # test if the input vehicle is vehicle A #

        # assume vehicle applies maximum acceleration during all timesteps the ego vehicle takes to reach centerline. (Before the center line)
        hypoth_VEL = givenVehicle_initial_VEL 
        hypoth_POS = givenVehicle_initial_POS

        for i in range(0,LCnoOfTS_half,1):
            temp_VEL = hypoth_VEL + (self.max_acc * self.del_time)
            if (temp_VEL <= self.max_vel):
                hypoth_POS = hypoth_POS + (hypoth_VEL * self.del_time) + 0.5*self.max_acc*self.del_time*self.del_time
                hypoth_VEL = temp_VEL
            else:
                hypoth_POS = hypoth_POS +  (0.5 * (hypoth_VEL + self.max_vel)*self.del_time)
                hypoth_VEL = self.max_vel

        # check if input vehicle is SURELY A
        if (hypoth_POS < lc_centerline):
            return 1
        # not sure whether it is SURELY A or B ( it is complicated :( )
        else:
            return 0


    # Checks if it is safe to lane change given the presence of a vehicle in position A (if any)
    def isSafeWRTA(self): 
        
        # return true if there is no vehicle A
        if (self.vehicle_A == -1):
            return True

        # calculate center line of lane change trajectory
        lc_centerline = self.lc_halfvdist + self.my_x_pos
        
        # get the initial positon and velocity for the vehicle A
        vehicle_A_initial_VEL = self.my_vehicle_ids.ids[self.vehicle_A].velocity  
        vehicle_A_initial_POS = self.my_vehicle_ids.ids[self.vehicle_A].x_position  

        # calculate number of timesteps taken by ego vehicle to reach centerline 
        LCnoOfTS_half = float(self.lc_halfTime) / float(self.del_time)
        LCnoOfTS_half = math.ceil(LCnoOfTS_half)
        LCnoOfTS_half = int(LCnoOfTS_half)

        # assume vehicle applies maximum acceleration during all timesteps the ego vehicle takes to reach centerline. (Before the center line)
        hypoth_VEL = vehicle_A_initial_VEL 
        hypoth_POS = vehicle_A_initial_POS 

        for i in range(0,LCnoOfTS_half,1):
            temp_VEL = hypoth_VEL + (self.max_acc * self.del_time)
            if (temp_VEL <= self.max_vel):
                hypoth_POS = hypoth_POS + (hypoth_VEL * self.del_time) + 0.5*self.max_acc*self.del_time*self.del_time
                hypoth_VEL = temp_VEL
            else:
                hypoth_POS = hypoth_POS + (0.5 * (hypoth_VEL + self.max_vel)*self.del_time)
                hypoth_VEL = self.max_vel

        # check whether the relative velocity between ego vehicle and vehicle A is safe (using krauss model)
        max_possible_safe_VEL = self.calcMaxSafeVel(hypoth_POS, hypoth_VEL,lc_centerline, self.lc_vel)
        if (hypoth_VEL > max_possible_safe_VEL):
            return False
        else:
            return True


    # Checks if it is safe to lane change given the presence of a vehicle in position B (if any)
    def isSafeWRTB(self):
        
        # return true if there is no vehicle B
        if (self.vehicle_B == -1):
            return True
        
        # calculate center line of lane change trajectory
        lc_centerline = self.lc_halfvdist + self.my_x_pos
        
        # calculate right line of lane change trajectory
        lc_rightline = self.lc_vdist + self.my_x_pos

        # get the initial positon and velocity for the vehicle B
        vehicle_B_initial_VEL = self.my_vehicle_ids.ids[self.vehicle_B].velocity  
        vehicle_B_initial_POS = self.my_vehicle_ids.ids[self.vehicle_B].x_position 
        
        # calculate number of timesteps taken by ego vehicle to reach centerline
        LCnoOfTS_half = float(self.lc_halfTime) / float(self.del_time)
        LCnoOfTS_half = math.ceil(LCnoOfTS_half)
        LCnoOfTS_half = int(LCnoOfTS_half)
        
        # assume vehicle applies maximum deceleration during all timesteps the ego vehicle takes to reach centerline. (Before the center line)
        hypoth_VEL = vehicle_B_initial_VEL 
        hypoth_POS = vehicle_B_initial_POS 

        for i in range(0,LCnoOfTS_half,1):
            temp_VEL = hypoth_VEL + (self.max_dec * self.del_time)
            if(temp_VEL >= 0):
                hypoth_POS = hypoth_POS + (hypoth_VEL * self.del_time) + 0.5*self.max_dec*self.del_time*self.del_time
                hypoth_VEL = temp_VEL
            else:
                hypoth_POS = hypoth_POS + (0.5 * (hypoth_VEL + 0) * self.del_time)
                hypoth_VEL = 0

        # calculate number of timesteps taken by ego vehicle to reach rightline (starting from centerline)
        LCnoOfTS_half = float(self.lc_time - self.lc_halfTime) / float(self.del_time)
        LCnoOfTS_half = math.ceil(LCnoOfTS_half)
        LCnoOfTS_half = int(LCnoOfTS_half)

        # get the final positon and velocity for the ego vehicle
        vehicle_ego_final_VEL = self.lc_vel
        vehicle_ego_final_DIST =  self.lc_vdist
        vehicle_ego_final_POS = self.lc_vdist + self.my_x_pos

        # assume vehicle applies maximum deceleration during all timesteps the ego vehicle takes to reach rightline (starting from centerline)
        vehicle_B_final_VEL = hypoth_VEL 
        vehicle_B_final_POS = hypoth_POS 

        for i in range(0,LCnoOfTS_half,1):
            temp_VEL = vehicle_B_final_VEL + (self.max_dec * self.del_time)
            if (temp_VEL >= 0):
                vehicle_B_final_POS = vehicle_B_final_POS + (vehicle_B_final_VEL * self.del_time) + 0.5*self.max_dec*self.del_time*self.del_time
                vehicle_B_final_VEL = temp_VEL
            else:
                vehicle_B_final_POS = vehicle_B_final_POS + (0.5 * (vehicle_B_final_VEL + 0) * self.del_time)
                vehicle_B_final_VEL = 0
        
        # check for possible collision
        if (vehicle_B_final_POS <= lc_rightline):
            return False 
        else:
            # check whether the relative velocity between ego vehicle and vehicle B is safe (using krauss model)
            max_possible_safe_VEL = self.calcMaxSafeVel(vehicle_ego_final_POS, vehicle_ego_final_VEL, vehicle_B_final_POS, vehicle_B_final_VEL)
            if (vehicle_ego_final_VEL > max_possible_safe_VEL):
                return False
            else:
                return True    

    
    # Checks if it is safe to lane change given the presence of a vehicle in position D (if any)
    def isSafeWRTD(self):

        # return true if there is no vehicle D
        if (self.vehicle_D == -1):
            return True
        
        # calculate center line of lane change trajectory
        lc_centerline = self.lc_halfvdist + self.my_x_pos

        # get the initial positon and velocity for the vehicle D
        vehicle_D_initial_VEL = self.my_vehicle_ids.ids[self.vehicle_D].velocity  
        vehicle_D_initial_POS = self.my_vehicle_ids.ids[self.vehicle_D].x_position

        # calculate number of timesteps taken by ego vehicle to reach centerline 
        LCnoOfTS_half = float(self.lc_halfTime) / float(self.del_time)
        LCnoOfTS_half = math.ceil(LCnoOfTS_half)
        LCnoOfTS_half = int(LCnoOfTS_half)

        # assume vehicle applies maximum deceleration during all timesteps the ego vehicle takes to reach centerline. (Before the center line)
        hypoth_VEL = vehicle_D_initial_VEL
        hypoth_POS = vehicle_D_initial_POS

        for i in range(0,LCnoOfTS_half,1):
            temp_VEL = hypoth_VEL + (self.max_dec * self.del_time)
            if (temp_VEL >= 0):
                hypoth_POS = hypoth_POS + (hypoth_VEL * self.del_time) + 0.5*self.max_dec*self.del_time*self.del_time
                hypoth_VEL = temp_VEL
            else:
                hypoth_POS = hypoth_POS + (0.5 * (hypoth_VEL + 0) * self.del_time)
                hypoth_VEL = 0 

        # check for possible collision        
        if (hypoth_POS <= lc_centerline):
            return False
        else:
            return True 


    # Generates the waypoints to be used by Stanley controller to perform the lane change
    def generateLCWaypoints(self):

        if (self.to_go_lane == 0):
            self.lc_waypoints = [(float(self.my_x_pos), float(0.2625), float(0.05)),
                                (float(self.my_x_pos+0.5), float(0.2625), float(0.05)),
                                (float(self.my_x_pos+1), float(0.2625), float(0.05))]
            return
        
        if (self.to_go_lane == 1):
            self.lc_waypoints = [(float(self.my_x_pos), float(-0.2625), float(0.05)),
                                (float(self.my_x_pos+0.5), float(-0.2625), float(0.05)),
                                (float(self.my_x_pos+1), float(-0.2625), float(0.05))]
            return

        if (self.to_go_lane == 2):
            self.lc_waypoints = [(float(self.my_x_pos), float(-0.7875), float(0.05)),
                                (float(self.my_x_pos+0.5), float(-0.7875), float(0.05)),
                                (float(self.my_x_pos+1), float(-0.7875), float(0.05))]
            
            return 

    # Uses Stanley controller in executing lane change
    # Returns true if lane change has finished 
    def stanleyController(self):

        if ((self.my_x_pos > self.lc_waypoints[2][0]) and (abs(self.my_yaw) <= 10e-02)):
            return True
        
        # Find the path point closest to the vehicle
        min_idx = 0
        min_dist      = float("inf")

        for i in range(len(self.lc_waypoints)):
            dist = np.linalg.norm(np.array([
                    self.lc_waypoints[i][0] - self.my_x_pos,
                    self.lc_waypoints[i][1] - self.my_y_pos]))
            if dist < min_dist:
                min_dist = dist
                min_idx = i

        if min_idx == len(self.lc_waypoints)-1:
            min_idx = min_idx - 1           
    

        # calculate the heading
        psi = -self.my_yaw + np.arctan2((self.lc_waypoints[min_idx+1][1]-self.lc_waypoints[min_idx][1]),(self.lc_waypoints[min_idx+1][0]-self.lc_waypoints[min_idx][0]))

        # calculate an angle proportional to the cross-track error
        phi1 = np.arctan2((self.lc_waypoints[min_idx][1] - self.my_y_pos), self.lc_waypoints[min_idx][0] - self.my_x_pos)
        phi2 = np.arctan2((self.lc_waypoints[min_idx+1][1] - self.lc_waypoints[min_idx][1]), (self.lc_waypoints[min_idx+1][0] - self.lc_waypoints[min_idx][0]))

        phi = (np.mod(phi1 + np.pi, 2 * np.pi) - np.pi) - (np.mod(phi2 + np.pi, 2 * np.pi) - np.pi)
        phi = np.mod(phi + np.pi, 2 * np.pi) - np.pi

        # calculate the required steering angle
        if (abs((self.lc_waypoints[min_idx][1] - self.my_y_pos)) >= 0.2):
            k = 0.1
        else:
            k = 0.005

        ks = 0.15

        angle    = (k/(ks+self.lc_vel))*phi + psi
        angle = np.clip(angle, -0.4189, 0.4189) # 0.4189 radians = 24 degrees because car can only turn 24 degrees max

        # publish the drive_param msg
        msg = drive_param()
        msg.velocity = self.lc_vel
        msg.angle = angle
        self.pub.publish(msg)
        return False


    # Input data (IDsCombined message from topic ids_combined) used to update the vehicle_ids variable
    def idsCallback(self, idsMsg):

        # if the array of vehicle ids is not empty
        for i in range(len(idsMsg.ids)):
            if (idsMsg.ids[i] != []):
                # indicate the ego vehicle is not alone
                self.isAlone = 0
                # update the vehicle_ids variable
                self.vehicle_ids = idsMsg
                return

        # if the array of vehicle ids not empty, indicate the ego vehicle is alone
        self.isAlone = 1
        return


    # Input data is Odometry message from topic /vesc/odom
    def odomCallback(self, odomMsg):

        """
        # update ego vehicle's last non-zero velocity 
        if (odomMsg.twist.twist.linear.x >= 0.1):
            self.last_non_zero_vel = odomMsg.twist.twist.linear.x
        """
                
        # if lane change is required    
        if (self.direction != 0):

            rospy.loginfo("Desired direction :%d",self.direction) 

            # set vehicle's odometry information
            self.my_x_pos = odomMsg.pose.pose.position.x
            self.my_y_pos = odomMsg.pose.pose.position.y

            orientation_q = odomMsg.pose.pose.orientation
            orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
            (roll, pitch, self.my_yaw) = euler_from_quaternion (orientation_list)

            # if we still did not check the feasibility of the lane change and hence did not start executing it 
            if (self.finished == -1):
                # determine vehicle's current and to-go lane numbers
                # if the to-go lane is outside the world, declare that lane change is infeasible and return
                if (not self.whichLanes()):
                    self.feasibility = 0
                    rospy.loginfo("Lane change is: Infeasible") 
                    self.direction = 0
                    return
                
                # determine the required time and vertical distance needed for lane change (based on the vehicle's speed)
                if (odomMsg.twist.twist.linear.x >= 0.1):
                    self.lc_vel = odomMsg.twist.twist.linear.x
                else:
                    self.lc_vel = self.lc_default_vel

                self.setLCTimeAndVdist()


                # check the feasibility of lane change (based on the communicated array of vehicle ids)
                self.my_vehicle_ids = self.vehicle_ids
                self.feasibility = self.isLCFeasible()
	
        	# if lane change is infeasible, return
                if (self.feasibility == 0):
                    rospy.loginfo("Lane change is: Infeasible") 
                    self.direction = 0
                    return

                rospy.loginfo("Lane change is: Feasible") 
                # generate the waypoints to be used in performing the lane change
                self.generateLCWaypoints()
                
                # Indicate that lane change will start being executed 
                self.finished = 0
                
            # start/continue performing lane change
            # if lane change has been successfully executed, indicate that
            if (self.stanleyController()): 
                self.direction = 0
                rospy.loginfo("Lane change has finished") 
                self.finished = 1



##------------------------------------------------------------------------------------------##
##------------------------------------------------------------------------------------------##
##------------------------------------------------------------------------------------------##
##             L A N E    C H A N G E    A C T I O N    S E R V E R    C L A S S            ##
##------------------------------------------------------------------------------------------##
##------------------------------------------------------------------------------------------##
##------------------------------------------------------------------------------------------##

class LCActionServer():

    def __init__(self):

        self.a_server = actionlib.SimpleActionServer("move_car/laneChange_action_server", MoveCarAction, self.execute_cb, False)
        self.a_server.start()
        self.feedback = MoveCarFeedback()
        self.result = MoveCarResult()

        self.lc = LaneChange() # object of class LaneChange


    # Goal callback function; goal is a MoveCarAction message
    def execute_cb(self, goal):

        rospy.loginfo("Received goal in lane change action server")

        # extra check that lane change is requested 
        if (goal.mcGoal.control_action != 0):

            self.lc.feasibility = -1
            self.lc.finished = -1
            self.lc.direction = goal.mcGoal.control_action

	    # wait if action is still being tested for feasibility
            while (self.lc.feasibility == -1):
               continue

            # if action is infeasible
            if (self.lc.feasibility == 0):
                self.feedback.mcFeedback = 0
                self.result.mcResult = 0
                rospy.loginfo("Infeasible goal in lane change action server")
                self.a_server.publish_feedback(self.feedback)
                self.a_server.set_aborted(self.result)

            # if action is feasible
	    elif (self.lc.feasibility == 1):
                self.feedback.mcFeedback = 1
                self.a_server.publish_feedback(self.feedback)
                rospy.loginfo("Lane change is feasible and is being executed now")

  	        # wait till action gets executed
                while (self.lc.finished != 1):
                   continue

                self.result.mcResult = 1
                rospy.loginfo("Finished lane change in lane change action server")
                self.a_server.set_succeeded(self.result)



if __name__ == "__main__":
    rospy.init_node("laneChange_action_server")

    # create object of class LCActionServer
    lcas = LCActionServer()

    # subscribe to '/vesc/odom'
    rospy.Subscriber('/vesc/odom', Odometry, lcas.lc.odomCallback, queue_size=2)
    # subscribe to 'ids_combined'
    rospy.Subscriber('ids_combined', IDsCombined, lcas.lc.idsCallback, queue_size=2)

    rospy.spin()
