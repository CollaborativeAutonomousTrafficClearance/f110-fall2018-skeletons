#!/usr/bin/env python
import rospy
import actionlib
import message_filters

from std_msgs.msg import Bool
from racecar_navigation.msg import BoolWithHeader, NavAction
from racecar_move_car.msg import Goal, MoveCarAction, MoveCarGoal, MoveCarFeedback, MoveCarResult
from racecar_qlearning.msg import RLPolicyAction

class baseClient:

    def __init__(self, ActionTopicName):
 

        self.client = actionlib.SimpleActionClient(ActionTopicName, MoveCarAction) # action client of MoveCarAction
        #self.client.wait_for_server(0) # block till server connects

        #initializing the action goal to be sent 
        self.current_goal = MoveCarGoal()

        #initializing the action feedback
        self.current_feedback = MoveCarFeedback()   # in case of lane change, this should be updated; very critical variable
        self.current_feedback.mcFeedback = -2 # uninitialized

        #initializing the action result 
        self.current_result = MoveCarResult()
        self.current_result.mcResult = -2 # uninitialized

        #indicating the sent goal, feedback and result timing
        self.goal_sent_time = -1 
        self.feedback_rec_time = -1
        self.result_rec_time = -1

        #to indicate if all goal timings are for the same goal 
        self.goal_timings_completed = False

    def cancel_all_goals(self):  
        #cancelling all sent goals (pending or active) including goals from other clients 
        self.client.cancel_all_goals()
        rospy.loginfo("Move car action client is cancelling all old goals.")

    def cancel_this_goal(self):
        #cancelling the goal that the server is currently pursuing 
        self.client.cancel_goal()
        rospy.loginfo("Move car action client is cancelling the current goal.")
    
    def cancel_goals_before(self, time):
        #cancelling all goals from all clients before a certain time 
        self.client.cancel_goals_at_and_before_time(time)
        rospy.loginfo("Move car action client is cancelling goals before the specifie time.")

    def destroy_last_goal_handle(self):
        #stops tracking the state of the current goal. Unregisters this goal's callbacks.
        self.client.stop_tracking_goal()    

    def update_current_goal(self, goal):
        #for updating the current goal attribute
        self.current_goal = goal
    
    def send_new_goal(self):   # send a new goal to server
        self.current_feedback.mcFeedback = -1 # initialize feedback with -1
        self.current_result.mcResult = -1 # initialize result with -1
        
        #set the header just before sending the goal
        self.current_goal.mcGoal.header = rospy.time.now()

        #send the class' current goal
        self.client.send_goal(self.current_goal, self.done_cb, self.active_cb, self.feedback_cb) # specify appropriate callback functions

    def feedback_cb(self, feedback): 
        #callback function to update feedback

        #setting the recieved feedback timing
        self.feedback_rec_time = rospy.time.now()    
        self.current_feedback = feedback
        rospy.loginfo("Retuned feedback for this goal.")
    
    def active_cb(self):
        #active callback function
    
        #indicating the goal is accepted by the server, starts executing 
        self.goal_sent_time = rospy.time.now()
        self.goal_timings_completed = False   #new goal is activated 
        
        rospy.loginfo("Goal is currently active.")
    
    def done_cb(self, state, result): 
        #callback function to update result

        #indicating the result is sent 
        self.result_rec_time = rospy.time.now()
        
        self.goal_timings_completed = True   #goal is sent and done
        
        self.current_result = result
        rospy.loginfo("Retuned result for this goal.")
    
        
    def set_action_source(self, action_source_in):
        #Update the action source 
        self.current_goal.mcGoal.action_source = action_source_in   


class laneKeepingClient(baseClient):

    def __init__(self):
        
        actionName = "move_car/laneKeeping_action_server"
        baseClient.__init__(self, actionName)
        
        self.current_goal.mcGoal.control_action = 0  # 0:= Lane Keeping
        self.current_goal.mcGoal.acc = 0 #Lane keeping in this module doesn't require acceleration

class velocityClient(baseClient):

    def __init__(self):

        self.actionName = "move_car/laneKeeping_vel_action_server"
        baseClient.__init__(self, self.actionName)

        self.current_goal.mcGoal.control_action = 0  # 0:= Lane Keeping
        self.current_goal.mcGoal.acc = self.acc   #Set the current goal's acceleration
        
        #To store information about the car and know the next acceleration value
        self.last_vel = 0.0
        self.vel = 0.0
        self.last_acc = 0.0
        self.acc = 0.0
    
    def update_acc(self, delta_acc):   #argument is the change in acceleration required (RL)
        self.last_acc = self.acc
        self.acc = self.acc + delta_acc
    
    def get_current_car_acc(self):
        return self.acc

class laneChangeClient(baseClient):

    def __init__(self):

        self.actionName = "move_car/laneChange_action_server"
        baseClient.__init__(self, self.actionName)

        self.current_goal.mcGoal.acc = 0 #lane change doesn't have acceleration in this module

    def set_direction(self, dir):  #1 --> Left Lane Change, 2 --> Right Lane Change
        self.current_goal.mcGoal.control_action = dir

class action_source:

    def __init__(self):

        self.lk_client = laneKeepingClient() # lane keeping action client
        self.vel_client = velocityClient() # lane keeping velocity action client
        self.lc_client = laneChangeClient() # lane change action client
    
    # cancel all goals sent from all clients 
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

        #Set the action source in the move car goal to be zero, indicating navigation
        self.vel_client.current_goal.mcGoal.action_source = 0
        self.lk_client.current_goal.mcGoal.action_source = 0
        self.lc_client.current_goal.mcGoal.action_source = 0

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
        pass

"""
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
			rospy.loginfo("Checing feasibility of lane change.")

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
"""

class qLearning_ambulance_master(action_source):
    def __init__(self):
        action_source.__init__(self)

        self.last_RL_control_action = -1   # Initially (0 --> Lane keeping, 1--> left lane change, 2--> right lane change)
        self.RL_control_action = -1 # Initially (0 --> Lane keeping, 1--> left lane change, 2--> right lane change)
        self.delta_acc = 0.0 #Initially
        
        #Set the action source in the move car goal to be 1, indicating RL policy
        self.lc_client.set_action_source(1)
        self.lk_client.set_action_source(1)
        self.vel_client.set_action_source(1)

    def update_action_server_goals(self, chosen_action_in):

        #update the current chosen action 
        self.RL_control_action = chosen_action_in.control_action
        self.delta_acc = chosen_action_in.acc

        #call servers to send the goal
        self.call_servers()

    def call_servers(self):
        
        #wait for the lane change to finish
        while self.checkLaneChange() == False:    
            rospy.loginfo("Cannot execute a new action from RL policy; lane change is still unfinished.")
        
        else:
            if self.RL_control_action == 0:   #If the control action is lane keeping
                self.vel_client.update_acc(self.delta_acc)  #Update the acceleration
                
            elif self.RL_control_action == 1:
                self.lc_client.set_direction(1)   #Set the direction same as control action, left lane change
            
            elif self.RL_control_action == 2:
                self.lc_client.set_direction(2)    #Set the direction same as control action, right lane change


class clients_master:
    def __init__(self):
        self.navigation = nav_master()
        self.RL = qLearning_ambulance_master()
        self.isRLactivated = False    #False --> navigation is mastering, True --> RL is mastering
    
    def nav_listener(self):    ## listener for navigation and RL policy chosen goals
        rospy.loginfo("Inside move_car_action_client listener")
        
        if self.isRLactivated == False:    #This means RL is not activated, so subscribers should continue
            
            ######## Navigation Subscribtions ###########
            # subscribe to receive navigation actions
            chosen_nav_action = message_filters.Subscriber("move_car/nav/chosen_action", NavAction)
            # subscribe to know if goal is reached
            goalReached  = message_filters.Subscriber("move_car/nav/goalReached", BoolWithHeader)

            ts = message_filters.TimeSynchronizer([chosen_nav_action, goalReached], 50)
            ts.registerCallback(navigation.update_action_server_goals)
        
    
    def RL_listener(self):
        
        ########## RL Policy Actions Subscribtions #############
        # subscribe to recieve RL actions
        rospy.Subscriber("move_car/RL_chosen_action", RLPolicyAction, self.RL_activate)

    
    def RL_activate(self, chosen_action):
        
        # There has to be a message type written here to indicate that RL is deactivated
        self.isRLactivated = True
        self.navigation.muteControlSource()

        


if __name__ == '__main__':
    try:
        rospy.init_node('move_car_action_client', anonymous=True)
        cm = clients_master()
        cm.RL_listener()
        cm.nav_listener()
        rospy.spin()

    except rospy.ROSInterruptException as e:
        print 'Error in move_car_client: ', e
