#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from racecar_communication.msg import IDsCombined

class KraussModel:
    def __init__(self):
        
        self.t_r = 1 # estimated driver reaction time ~ 1 sec
        self.max_vel = rospy.get_param('max_vel') # maximum ego vehicle velocity allowed
        self.max_acc = rospy.get_param('max_acc') # maximum ego vehicle acceleration allowed
        self.vehicle_length = 0.58 # length of vehicles - please adjust this value if another vehicle model is used - value could be increased than actual to enforce a greater minimal gap between vehicles

        self.vel = 0 # ego vehicle current velocity
        self.x_pos = 0 # ego vehicle current x position (in global frame)
        self.y_pos = 0 # ego vehicle current y position (in global frame)
        self.lane_num = 0 # ego vehicle current lane number

        self.found_lead = 0 # boolean representing whether there is a leading vehicle infront of the ego vehicle
        self.lead_vel = 0 # leading vehicle's current velocity
        self.lead_x_pos = 0 # leading vehicle's current x position
        self.lead_gap = 0 # gap between leading and ego vehicles

        self.safe_vel = 0 # calculated safe velocity for ego vehicle based on Krauss model

        self.last_time = 0 # time of last iteration in seconds
        self.del_time = 1 # chosen delta time between iterations

        self.pub = rospy.Publisher('/desired_vel', Float32, queue_size=50) # publisher for the desired ego velocity

    # Calculates which lane the ego vehicle is in, assuming we are in the threeLanes world
    def whichLane(self):
        '''  
         Lane number convention in the "threeLanes" world:
             y^  
              |--------------> x      lane number 0 
              |--------------> x      lane number 1
              |--------------> x      lane number 2 
        '''

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
        

    # Input data is Odometry message from topic /vesc/odom and IDsCombined message from topic /ids_combined
    def odomCallback(self, odomMsg):

        # calculate current delta time
        curr_del_time = rospy.Time.now().secs - self.last_time      

	# to perform update every 1 second
        if (curr_del_time >= self.del_time):

		# update ego vehicle information
		self.vel = odomMsg.twist.twist.linear.x;
		self.x_pos = odomMsg.pose.pose.position.x;
		self.y_pos = odomMsg.pose.pose.position.y;

		# calculate safe velocity if a leaing vehicle exists, and accordingly calculate the desired velocity
		if (self.found_lead == 1):
		    self.calcSafeVel();
		    des_vel = min(self.max_vel, (self.vel + self.max_acc*curr_del_time), self.safe_vel)
		else:
		    des_vel = min(self.max_vel, (self.vel + self.max_acc*curr_del_time))

		# publish the calculated desired velocity
		self.pub.publish(des_vel)
		rospy.loginfo("Desired velocity:")
		rospy.loginfo(des_vel)

		# update time
		self.last_time = rospy.Time.now().secs
                rospy.loginfo("Curr time:")
		rospy.loginfo(self.last_time)


if __name__ == '__main__':
    rospy.init_node('krauss_model')

    # create object of class KraussModel
    km = KraussModel()

    # subscribe to '/vesc/odom'
    rospy.Subscriber('/vesc/odom', Odometry, km.odomCallback, queue_size=50)
    # subscribe to '/ids_combined'
    rospy.Subscriber('/ids_combined', IDsCombined, km.idsCallback, queue_size=50)

    rospy.spin()
