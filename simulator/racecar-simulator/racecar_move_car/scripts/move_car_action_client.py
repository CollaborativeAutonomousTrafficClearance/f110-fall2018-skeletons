#!/usr/bin/env python
import rospy
import actionlib
import message_filters

from std_msgs.msg import Bool
from racecar_navigation.msg import BoolWithHeader, NavAction
from racecar_move_car.msg import Goal, MoveCarAction, MoveCarGoal, MoveCarFeedback, MoveCarResult

class baseClient:

    def __init__(self, ActionTopicName):
        rospy.loginfo("Inside the baseclient")

        self.client = actionlib.SimpleActionClient(ActionTopicName, MoveCarAction) # action client of MoveCarAction
        #self.client.wait_for_server(0) # block till server connects

        self.current_goal = MoveCarGoal()

        self.current_feedback = MoveCarFeedback()   # in case of lane change, this should be updated; very critical variable
	self.current_feedback.mcFeedback = -2 # uninitialized

        self.current_result = MoveCarResult()
	self.current_result.mcResult = -2 # uninitialized

        rospy.loginfo("Outside the baseclient")


    def cancel_all_goals(self):
        self.client.cancel_all_goals()
        rospy.loginfo("Move car action client is cancelling all old goals.")

    def cancel_this_goal(self):
        self.client.cancel_goal()
        rospy.loginfo("Move car action client is cancelling the current goal.")
    
    def cancel_goals_before(self, time):
        self.client.cancel_goals_at_and_before_time(time)
        rospy.loginfo("Move car action client is cancelling goals before the specifie time.")

    def destroy_last_goal_handle(self):
        self.client.stop_tracking_goal()    # stops tracking the state of the current goal. Unregisters this goal's callbacks.

    def send_new_goal(self, goal):   # send a new goal to server
        self.current_goal = goal
	self.current_feedback.mcFeedback = -1 # initialize feedback with -1
	self.current_result.mcResult = -1 # initialize result with -1
        self.client.send_goal(self.current_goal, self.done_cb, self.active_cb, self.feedback_cb) # specify appropriate callback functions

    def feedback_cb(self, feedback): # callback function to update feedback
        self.current_feedback = feedback
        rospy.loginfo("Retuned feedback for this goal.")
    
    def active_cb(self): # active callback function
        rospy.loginfo("Goal is currently active.")
    
    def done_cb(self, state, result): # callback function to update result
        self.current_result = result
        rospy.loginfo("Retuned result for this goal.")


class laneKeepingClient(baseClient):

    def __init__(self):
        rospy.loginfo("Inside the lk client")

        actionName = "move_car/laneKeeping_action_server"
        baseClient.__init__(self, actionName)

        #self.current_goal.mcGoal.control_action = 0  # 0:= Lane Keeping
        #self.current_goal.mcGoal.acc = -100  # Undefined for this goal

        rospy.loginfo("Outside the lk client")

class velocityClient(baseClient):

    def __init__(self):
        rospy.loginfo("Inside the vel client")

        self.actionName = 'move_car/laneKeeping_vel_action_server'
        baseClient.__init__(self, self.actionName)

        #self.current_goal.mcGoal.control_action = 0  # 0:= Lane Keeping
        #self.last_vel = 0.0
        #self.vel = 0.0
        #self.last_acc = 0.0
        #self.acc = 0.0

        rospy.loginfo("Outside the vel client")
    
#    def update_vel(self, new_vel):
#        self.vel = new_vel
    
#    def update_acc(self, new_acc):
#        self.acc = new_acc
    
#    def update_vel_acc(self, new_vel, new_acc):
#        self.update_vel(new_vel)
#        self.update_acc(new_acc)

class laneChangeClient(baseClient):

    def __init__(self):
        rospy.loginfo("Inside the lc client")

        self.actionName = 'move_car/laneChange_action_server'
        baseClient.__init__(self, self.actionName)

        #self.direction = 0 #####################
        #self.last_direction = 0 ####################
        #self.current_goal.mcGoal.acc = -100  #Undefined for this goal

        rospy.loginfo("Outside the lc client")

class action_source:

    def __init__(self):

        self.lk_client = laneKeepingClient() # lane keeping action client
        self.vel_client = velocityClient() # lane keeping velocity action client
        self.lc_client = laneChangeClient() # lane change action client
    
    # cancel all goals from this action source (navigation)
    def muteControlSource(self):
        # cancel lk and lk vel goals
        self.lk_client.cancel_all_goals()
        self.lk_client.destroy_last_goal_handle()
        self.vel_client.cancel_all_goals()
        self.vel_client.destroy_last_goal_handle()

        # wait for lane change if it is not finished
        while (self.lc_client.current_result.mcResult == -1):
            rospy.loginfo("Lane change is still unfinished.")

        # cancel lc goals
        self.lc_client.cancel_all_goals()
        self.lc_client.destroy_last_goal_handle()

    # check whether lane change is still executing
    def checkLaneChange(self):

        if self.lc_client.current_result.mcResult == -1:
            rospy.loginfo("Lane change is still unfinished.")
            return False
        else:
            return True


class nav_master(action_source):

    def __init__(self):
        rospy.loginfo("Inside the nav master")
        action_source.__init__(self)

        self.last_control_action = -1   # Initially
        self.current_control_action = -1 # Initially
        #self.goalReached = -1 # Initially

        #self.vel_client.current_goal.mcGoal.action_source = 0
        #self.lk_client.current_goal.mcGoal.action_source = 0
        #self.lc_client.current_goal.mcGoal.action_source = 0
        #self.lc_client.current_feedback = -1   #To enter the first loop in call server
        #self.lc_client.current_result = -1  #To enter the first loop in call server

        rospy.loginfo("Outside the nav master")
    
    def update_action_server_goals(self, chosen_action_in, goalReached_in):

        rospy.loginfo("Inside update 1")
	
	#if (!goalReached_in.Bool)
        self.current_control_action = chosen_action_in.control_action
        rospy.loginfo("Received nav action in move_car action client >>>>>>>>>>>>>>>>>>>>")
        rospy.loginfo(self.current_control_action)

        #self.chosen_action.header.stamp = rospy.Time.now()
        #self.goalReached.header.stamp = rospy.Time.now()

        self.call_servers()

        rospy.loginfo("Inside update")

    def call_servers(self):

        rospy.loginfo("Inside call server")

        # if lane change is still executing, do not send any new goals
	if (self.checkLaneChange() == 0):
            rospy.loginfo("Cannot execute a new action; lane change is still unfinished.")

	else:
      
            # if both last and current actions are lane keeping, do not send any new goals
	    if (self.current_control_action == 0 and self.last_control_action == 0):
		rospy.loginfo("Keep lane keeping.")

		
	    else:
                 # create a new goal object
    	         new_goal = MoveCarGoal()
	         new_goal.mcGoal.header.stamp = rospy.Time.now()
                 new_goal.mcGoal.action_source = 0
                 new_goal.mcGoal.control_action = self.current_control_action
                 new_goal.mcGoal.acc = -100	 

                 # if current action is lane keeping, send goals to lk and lk vel servers
		 if (self.current_control_action == 0):
		     # Cancel any current goals
                     self.muteControlSource()
                     self.vel_client.send_new_goal(new_goal) 
                     self.lk_client.send_new_goal(new_goal) 

		 else:
                     # if current action is lane change, send goal to lc server
                     self.lc_client.send_new_goal(new_goal) 

                     # wait to check if lane change is feasible
		     while(self.lc_client.current_feedback.mcFeedback == -1):
			rospy.loginfo("Checking feasibility of lane change.")

                     # if lane change is infeasible, lane keep instead
		     if (self.lc_client.current_feedback.mcFeedback == 0):
			rospy.loginfo("Lane change was infeasible; lane keeping instead")

			# If vehicle was not lane keeping already, let it lane keep
			if (self.last_control_action != 0):

    	                    new_goal.mcGoal.header.stamp = rospy.Time.now()

			    self.current_control_action = 0
			    new_goal.mcGoal.control_action = self.current_control_action

		  	    # Cancel any current goals
          	            self.muteControlSource()
                 	    self.vel_client.send_new_goal(new_goal) 
                    	    self.lk_client.send_new_goal(new_goal)

                     # If lane change is feasible, cancel lane keeping and velocity goals
		     elif (self.lc_client.current_feedback.mcFeedback == 1):

          	        self.lk_client.cancel_all_goals()
                        self.lk_client.destroy_last_goal_handle()
                        self.vel_client.cancel_all_goals()
                        self.vel_client.destroy_last_goal_handle()

                 # update last control action
                 self.last_control_action = self.current_control_action  


class qLearning_ambulance_master:
    def __init__(self):
        action_source.__init__(self)
        self.deltaTime = 1  


def listener():
    rospy.loginfo("Inside move_car_action_client listener")

    navigation = nav_master()

    # subscribe to receive navigation actions
    chosen_nav_action = message_filters.Subscriber("move_car/nav/chosen_action", NavAction)
    # subscribe to know if goal is reached
    goalReached  = message_filters.Subscriber("move_car/nav/goalReached", BoolWithHeader)

    ts = message_filters.TimeSynchronizer([chosen_nav_action, goalReached], 50)
    rospy.loginfo("Before synchronized msg")
    ts.registerCallback(navigation.update_action_server_goals)
    rospy.loginfo("After synchronized msg")

    rospy.spin()

if __name__ == '__main__':
    try:
        rospy.init_node('move_car_action_client', anonymous=True)
        listener()

    except rospy.ROSInterruptException as e:
        print 'Error in move_car_client: ', e
