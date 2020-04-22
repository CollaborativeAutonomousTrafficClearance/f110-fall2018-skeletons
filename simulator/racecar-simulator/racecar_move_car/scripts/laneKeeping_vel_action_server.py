#!/usr/bin/env python

import rospy
import actionlib
from racecar_move_car.msg import  MoveCarAction, MoveCarGoal, MoveCarResult
from racecar_communication.msg import IDsCombined
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

##------------------------------------------------------------------------------------------##
##------------------------------------------------------------------------------------------##
##------------------------------------------------------------------------------------------##
##                           K R A U S S    M O D E L    C L A S S                          ##
##------------------------------------------------------------------------------------------##
##------------------------------------------------------------------------------------------##
##------------------------------------------------------------------------------------------##
class KraussModel:

    def __init__(self):

        self.t_r = 1 # estimated driver reaction time ~ 1 sec
        self.max_vel = 0.5 # maximum ego vehicle velocity allowed
        self.max_acc = 0.1 # maximum ego vehicle acceleration allowed
        self.vehicle_length = 0.58 # length of vehicles - please adjust this value if another vehicle model is used

        self.vel = 0 # ego vehicle current velocity
        self.x_pos = 0 # ego vehicle current x position (in global frame)
        self.y_pos = 0 # ego vehicle current y position (in global frame)
        self.lane_num = 0 # ego vehicle current lane number

        self.found_lead = 0 # boolean representing whether there is a leading vehicle infront of the ego vehicle
        self.lead_vel = 0 # leading vehicle's current velocity
        self.lead_x_pos = 0 # leading vehicle's current x position
        self.lead_gap = 0 # gap between leading and ego vehicles

        self.safe_vel = 0 # calculated safe velocity for ego vehicle based on Krauss model
        self.req_nav_vel = 0 # velocity requested by navigaion module
        self.req_RL_acc = 0 # velocity requested by RL policy

        self.last_time = 0 # time of last iteration in seconds
        self.del_time = 1 # chosen delta time between iterations
        self.active = 0 # int representing whether the lane keeping velocity needs to be calculated (0:= not needed, 1:= needed by navigaion module, 2:= needed by RL policy, -1:= needed by RL policy but infeasible)

        self.pub = rospy.Publisher('move_car/desired_lk_vel', Float32, queue_size=10) # publisher for the desired lane keeping velocity

    # Calculates which lane the ego vehicle is in, assuming we are in the threeLanes world
    def whichLane(self):

        if (self.y_pos >= 0):
            self.lane_num = 0;
        elif (self.y_pos < -0.525):
            self.lane_num = 2;   
        else:
            self.lane_num = 1; 


    # Checks if a leading vehicle exists and updates its information
    def idsCallback(self, idsMsg):

        self.whichLane()

        self.found_lead = 0
        
        for i in range(len(idsMsg.ids)):

            if (idsMsg.ids[i] != []):

                if ((idsMsg.ids[i].lane_num == self.lane_num) and (idsMsg.ids[i].x_position >= self.x_pos)):

                    if ((self.found_lead == 0) or (idsMsg.ids[i].x_position < self.lead_x_pos)):
                        self.found_lead = 1
                        self.lead_vel = idsMsg.ids[i].velocity
                        self.lead_x_pos = idsMsg.ids[i].x_position


        if (self.found_lead == 1):
            self.calcGap();


    # Calculates gap between leading and ego vehicles
    def calcGap(self):

        self.lead_gap = self.lead_x_pos - self.x_pos - self.vehicle_length


    # Calculates safe velocity for ego vehicle based on Krauss model
    def calcSafeVel(self):

        self.safe_vel = self.lead_vel + ((self.lead_gap - (self.lead_vel * self.t_r))/(self.t_r + ((self.lead_vel + self.vel)/(2*self.max_acc))));
        

    # Input data is Twist message from topic /cmd_vel
    def velCallback(self, velMsg):
        self.req_nav_vel = velMsg.linear.x


    # Input data is Odometry message from topic /vesc/odom
    def odomCallback(self, odomMsg):

        # calculate current delta time
        curr_del_time = rospy.Time.now().secs - self.last_time      

        if ((self.active == 2) or ((self.active == 1) and (curr_del_time >= self.del_time))):

		# update ego vehicle information
		self.vel = odomMsg.twist.twist.linear.x;
		self.x_pos = odomMsg.pose.pose.position.x;
		self.y_pos = odomMsg.pose.pose.position.y;

		# calculate safe velocity if a leaing vehicle exists, and accordingly calculate the desired velocity
		if (self.found_lead == 1):
		    self.calcSafeVel();
		    self.ceil_vel = min(self.max_vel, (self.vel + self.max_acc*self.del_time), self.safe_vel)
		else:
		    self.ceil_vel = min(self.max_vel, (self.vel + self.max_acc*self.del_time))


	        # if velocity is requested by navigation module
                if (self.active == 1):
                    lk_vel = min(self.ceil_vel, self.req_nav_vel)

                    # publish the calculated lane keeping velocity
		    self.pub.publish(lk_vel)
                    rospy.loginfo("Velocity published by vel server for navigation:")
		    rospy.loginfo(lk_vel)

	        # if acceleraion is requested by RL policy
                elif (self.active == 2):
                    lk_vel = (self.vel + self.req_RL_acc*self.del_time)
                    if ((lk_vel > self.ceil_vel) or (self.req_RL_acc > self.max_acc)):
                        # set active to -1 indicating an infeasible action
                        self.active = -1
                    else: 
   	                # publish the calculated lane keeping velocity
		        self.pub.publish(lk_vel)
                        rospy.loginfo("Velocity published by vel server to follow RL policy:")
		        rospy.loginfo(lk_vel)

	                # switch to inactive
	                self.active = 0


		# update time
		self.last_time = rospy.Time.now().secs
                rospy.loginfo("Curr time:")
		rospy.loginfo(self.last_time)


##------------------------------------------------------------------------------------------##
##------------------------------------------------------------------------------------------##
##------------------------------------------------------------------------------------------##
##                     V E L    A C T I O N    S E R V E R    C L A S S                     ##
##------------------------------------------------------------------------------------------##
##------------------------------------------------------------------------------------------##
##------------------------------------------------------------------------------------------##

class VelActionServer():

    def __init__(self):

        self.a_server = actionlib.SimpleActionServer("move_car/laneKeeping_vel_action_server", MoveCarAction, self.execute_cb, False)
        self.a_server.start()
        self.result = MoveCarResult()

        self.km = KraussModel() # object of class KraussModel


    # Goal callback function; goal is a MoveCarAction message
    def execute_cb(self, goal):

        rospy.loginfo("Received goal in lane keeping velocity action server")

	# if action is requested by navigation module
        if (goal.mcGoal.action_source == 0):

	    self.km.active = 1
            rate = rospy.Rate(1)

            rospy.loginfo("Executing goal in lane keeping velocity action server")

            while(1):
                # check that preempt has not been requested by the client
                if self.a_server.is_preempt_requested():
		    self.km.active = 0
                    rospy.loginfo("Goal preempted in lane keeping velocity action server")
                    self.a_server.set_preempted() #########send result?
                    break

                rate.sleep()

	# if action is requested by RL policy
        elif (goal.mcGoal.action_source == 1):

            self.km.req_RL_acc = goal.mcGoal.acc
            self.km.active = 2

	    # wait if action is still being tested for feasibility or being executed
            while ((self.km.active != -1) and (self.km.active != 0)):
                continue

	    # if action was successfully executed
            if (self.km.active == 0):
                self.result.result = 1
                rospy.loginfo("Reached goal in lane keeping velocity action server")
                self.a_server.set_succeeded(self.result)

	    # if action was found to be infeasible
            elif (self.km.active == -1):
                self.result.result = 0
                rospy.loginfo("Infeasible goal in lane keeping velocity action server")
                self.a_server.set_aborted(self.result)



if __name__ == "__main__":
    rospy.init_node("laneKeeping_vel_action_server")

    # create object of class VelActionServer
    vas = VelActionServer()

    # subscribe to '/vesc/odom'
    rospy.Subscriber('/vesc/odom', Odometry, vas.km.odomCallback, queue_size=2)
    # subscribe to 'ids_combined'
    rospy.Subscriber('ids_combined', IDsCombined, vas.km.idsCallback, queue_size=2)
    # subscribe to 'cmd_vel'
    rospy.Subscriber('cmd_vel', Twist, vas.km.velCallback, queue_size=2)

    rospy.spin()
