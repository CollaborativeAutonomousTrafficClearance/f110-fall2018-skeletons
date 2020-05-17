#!/usr/bin/env python
import rospy
import actionlib
import message_filters

from std_msgs.msg import Bool
from racecar_navigation.msg import BoolWithHeader, NavAction
from racecar_move_car.msg import Goal, MoveCarAction, MoveCarGoal, MoveCarFeedback, MoveCarResult
#from racecar_qlearning.msg import RLPolicyRequest
from racecar_qlearning.srv import RLPolicyActionService, RLPolicyActionServiceRequest, RLPolicyActionServiceResponse
import threading

#RL_lock = threading.Lock()
#RL_lock.acquire()

#for lane change feedback check
Nav_lc_lock_feedback = threading.Lock()
Nav_lc_lock_feedback.acquire()

#For synchronizing result and feedback, as if done_cb got executed, feedback_cb is unregistered
lc_fb_dn_syn_lock = threading.Lock()
lc_fb_dn_syn_lock.acquire()


class baseClient:

    def __init__(self, ActionTopicName):
 

        self.client = actionlib.SimpleActionClient(ActionTopicName, MoveCarAction) # action client of MoveCarAction
        #self.client.wait_for_server(0) # block till server connects

        #initializing the action goal to be sent, current and last  
        self.current_goal = MoveCarGoal()
        self.last_goal = MoveCarGoal()   

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
        self.goal_timings_completed = rospy.Duration(6,0)

        #to store how long should the client wait for a result
        #self.wait_for_result_duration = rospy.Duration()

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

    def update_last_goal(self,goal):
        #for updating the last goal attribute
        self.last_goal = goal
    
    def send_new_goal(self):   # send a new goal to server
        self.current_feedback.mcFeedback = -1 # initialize feedback with -1
        self.current_result.mcResult = -1 # initialize result with -1
        
        #set the header just before sending the goal
        self.current_goal.mcGoal.header.stamp = rospy.Time.now()

        #send the class' current goal
        self.client.send_goal(self.current_goal, self.done_cb, self.active_cb, self.feedback_cb) # specify appropriate callback functions
    
    def send_goal_and_wait(self, delta_time):
        #function for sending a goal and waiting delta time to be executed
        #delta_time must be rospy.duration()
        self.state = self.client.send_goal_and_wait(self.current_goal, execute_timeout = delta_time)
    
    def feedback_cb(self, feedback): 
        #callback function to update feedback

        #setting the recieved feedback timing
        self.feedback_rec_time = rospy.Time.now()    
        self.current_feedback = feedback
        rospy.loginfo("Retuned feedback for this goal.")

        if self.current_goal.mcGoal.control_action > 0:   #If the action is lane change
            Nav_lc_lock_feedback.release()
        #After all the logic, releasing the lock in order for the done_cb to finish execution 
            lc_fb_dn_syn_lock.release()
        
    
    def get_feedback(self, feedback):
        return self.current_feedback.mcFeedback 
        
    def active_cb(self):
        #active callback function
    
        #indicating the goal is accepted by the server, starts executing 
        self.goal_sent_time = rospy.Time.now()
        self.goal_timings_completed = False   #new goal is activated 
        
        rospy.loginfo("Goal is currently active.")
    
    def done_cb(self, state, result): 
        #callback function to update result

        #indicating the result is sent 
        self.result_rec_time = rospy.Time.now()
        
        self.goal_timings_completed = True   #goal is sent and done
        
        self.current_result = result
        rospy.loginfo("Retuned result for this goal.")

        if self.current_goal.mcGoal.action_source == 1:   #If the action source is RL
            pass
        #releasing the lock, for the RL policy action server response to be sent
            #RL_lock.release()

        #If the action is lane change
        if self.current_goal.mcGoal.control_action > 0:   
            #Waiting for the feedback_cb to register if it is late
            lc_fb_dn_syn_lock.acquire()

    def block_till_result(self):
        self.client.wait_for_result()
    
    def get_result(self):
        return self.current_result.mcResult

    def set_action_source(self, action_source_in):
        #Update the action source in the move car goal
        self.current_goal.mcGoal.action_source = action_source_in  

    def set_acc(self, acc_in):
        #Update the acceleration in the move car goal
        self.current_goal.mcGoal.acc = acc_in
    
    def set_control_action(self, control_action_in):
        #Update the control action in the move car goal
        self.current_goal.mcGoal.control_action = control_action_in



class laneKeepingClient(baseClient):

    def __init__(self):
        
        actionName = "move_car/laneKeeping_action_server"
        baseClient.__init__(self, actionName)
        
        "No other parameters to be set except the action source"

        self.set_control_action(0)  # 0:= Lane Keeping
        self.set_acc(0) #Lane keeping in this module doesn't require acceleration

        

class velocityClient(baseClient):

    def __init__(self):

        self.actionName = "move_car/laneKeeping_vel_action_server"
        baseClient.__init__(self, self.actionName)

        "No other parameters to be set except the action source and the acceleration"
        self.set_control_action(0) # 0:= Lane Keeping
    


class laneChangeClient(baseClient):

    def __init__(self):

        self.actionName = "move_car/laneChange_action_server"
        baseClient.__init__(self, self.actionName)

        "No other parameters to be set except the action source and the direction(Control Action)"
        
        self.set_acc(0) #lane change doesn't have acceleration in this module

    def set_direction(self, dir):  #1 --> Left Lane Change, 2 --> Right Lane Change
        self.set_control_action(dir)

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
        #Check whether the lane change is finished or not according to it's result
        if self.lc_client.get_result() == -1:
            rospy.loginfo("Lane change is still unfinished.")
            return False
        else:
            return True
    
    def cancel_goals_lk_vel_clients(self):
        self.lk_client.cancel_all_goals()
        self.lk_client.destroy_last_goal_handle()
        self.vel_client.cancel_all_goals()
        self.vel_client.destroy_last_goal_handle()



class nav_master(action_source):

    def __init__(self):
        rospy.loginfo("Inside the nav master")
        action_source.__init__(self)
        
        #Initializing the last and current control actions required in the navigation module
        self.last_nav_control_action = -1   # Initially
        self.current_nav_control_action = -1 # Initially
        #self.goalReached = -1 # Initially
        

        #Set the action source in the move car goal to be zero, indicating navigation
        self.vel_client.set_action_source(0)
        self.lk_client.set_action_source(0)
        self.lc_client.set_action_source(0)

        #Set the acceleration in velocity to be -100 as a flag, as 
        #velocity client takes it's acceleration from cmd_vel from Nav Stack
        self.vel_client.set_acc(-100) 

    
    def update_action_server_goals(self, chosen_action_in):
	
	    #if (!goalReached_in.Bool)
        self.current_nav_control_action = chosen_action_in.control_action
        
        rospy.loginfo("Received nav action in move_car action client >>>>>>>>>>>>>>>>>>>>")
        rospy.loginfo(self.current_nav_control_action)

        #If the action is lane change, update it's direction
        if chosen_action_in.control_action > 0 : 
            self.lc_client.set_control_action(chosen_action_in.control_action)

        #Calling the server to send goals 
        self.call_servers()



    def call_servers(self):
        rospy.loginfo("Inside call server")

        # if both last and current actions are lane keeping, do not send any new goals
        if (self.current_nav_control_action == 0 and self.last_nav_control_action == 0):
            rospy.loginfo("Keep lane keeping.")
            
        else:
            # if current action is lane keeping, send goals to lk and lk vel servers
            if (self.current_nav_control_action == 0):
                    
                # Cancel any current goals from all clients 
                self.muteControlSource()

                #Send goals in for velocity and lane keeping clients 
                self.vel_client.send_new_goal()
                self.lk_client.send_new_goal() 

            else:
                # if current action is lane change, send goal to lc server
                self.lc_client.send_new_goal() 
                    
                #Blocking the function till lane change feasibility check is finished (Feedback and result changes)
                rospy.loginfo("Waiting for lane change feasibility check to end")
                Nav_lc_lock_feedback.acquire()
                rospy.loginfo("lane change feasibility check ended")

                # if lane change is infeasible, lane keep instead
                #if (self.lc_client.get_feedback == 0 or self.lc_client.get_result == 0):
                if (self.lc_client.get_feedback == 0):
                    #rospy.loginfo("Lane change was infeasible; lane keeping instead")

                    # If vehicle was not lane keeping already, let it lane keep
                    if (self.last_nav_control_action != 0):

                        self.current_nav_control_action = 0
                        self.lk_client.set_control_action(self.current_nav_control_action)
                        self.vel_client.set_control_action(self.current_nav_control_action)
                            
                        # Cancel any current goals
                        self.muteControlSource()
                            
                        #Send the new lane keeping goal
                        self.vel_client.send_new_goal() 
                        self.lk_client.send_new_goal()

                # If lane change is feasible, cancel lane keeping and velocity goals
                elif (self.lc_client.get_feedback() == 1):
                    self.cancel_goals_lk_vel_clients()
                        
                    #wait for lane change to finish
                    self.lc_client.block_till_result()

            # update last control action
            self.last_nav_control_action = self.current_nav_control_action  
                

class qLearning_ambulance_master(action_source):
    def __init__(self):
        action_source.__init__(self)

        self.last_RL_control_action = -1   # Initially (0 --> Lane keeping, 1--> left lane change, 2--> right lane change)
        self.RL_control_action = -1 # Initially (0 --> Lane keeping, 1--> left lane change, 2--> right lane change)
        self.RL_acc = 0   #Initially 

        #Set the action source in the move car goal to be 1, indicating RL policy
        self.lc_client.set_action_source(1)
        self.lk_client.set_action_source(1)
        self.vel_client.set_action_source(1)

        #delta time
        self.delta_lk_time = rospy.Duration(4,0)

        #storing if the action is finished or infeasible, to be sent to the client 
        # 0 --> Infeasible action, 1--> Finished Action
        self.RLActionResult = False
    
    def set_RL_acc(self, acc_in):
        self.RL_acc = acc_in
    
    def set_RL_control_action(self, action_in):
        self.RL_control_action = action_in

    def call_servers(self):            
            if self.RL_control_action == 0:   #If the control action is lane keeping
                
                self.vel_client.set_acc(self.RL_acc)  #Update the acceleration
                "Lane keeping client should not begin without the velocity client"
                self.lk_client.send_new_goal()  #start lane keeping

                self.vel_client.send_goal_and_wait(self.delta_lk_time) #keep waiting for the goal for the required delta time

                            
                #blocking the function until a result is sent 
                #RL_lock.acquire()
                self.lk_client.cancel_this_goal()    #Cancel the lane keeping goal
                
                #setting the result to be sent to the service client again
                if self.vel_client.current_result == 0:  #infeasible goal
                    self.RLActionResult = False 
                elif self.vel_client.current_result == 1:  #accomplished goal
                    self.RLActionResult = True
        
            
            else:
                if self.RL_control_action == 1:
                    self.lc_client.set_direction(1)   #Set the direction same as control action, left lane change
                    self.lc_client.send_new_goal()
            
                elif self.RL_control_action == 2:
                    self.lc_client.set_direction(2)    #Set the direction same as control action, right lane change
                    self.lc_client.send_new_goal()






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
            #chosen_nav_action = message_filters.Subscriber("move_car/nav/chosen_action", NavAction)
            # subscribe to know if goal is reached
            #goalReached  = message_filters.Subscriber("move_car/nav/goalReached", BoolWithHeader)

            rospy.Subscriber("move_car/nav/chosen_action", NavAction, self.navigation.update_action_server_goals)
            
            #ts = message_filters.TimeSynchronizer([chosen_nav_action, goalReached], 50)
            #ts.registerCallback(self.navigation.update_action_server_goals)
        
    
    def RL_activate(self):
        #function to indicate that RL should be activated, it waits for navigation to end
        self.isRLactivated = True
        self.navigation.muteControlSource()

    
    def handle_RLPolicyAction_service(self, chosen_action_in):
        #handler for the RL policy action service when the RL requests an action to be performed 
        
        #if the RL action policy service request's flag is false, this means the car is still
        #in the ambulance's window
        if chosen_action_in.RLChosenAction.doneRLflag == False :

            #update the current chosen action 
            self.RL.set_RL_acc(chosen_action_in.RLChosenAction.acc)
            self.RL.set_RL_control_action(chosen_action_in.RLChosenAction.control_action)

            #waiting for the action to be processed 
            self.RL.call_servers()

            #return the response when done, in order for the RL action generator to send a new action 
            return self.RL.RLActionResult
        
        else:
            #If the car is outside the ambulance window
            self.isRLactivated = False
            return True
    
    def RLPolicyAction_server(self):
        #the service connected to the RL policy action generator, when the action is finished, it informs the 
        #action generator
        self.RLPolicyActionService = rospy.Service('move_car/RL/RLPolicyActionService', RLPolicyActionService, self.handle_RLPolicyAction_service)
        
        #engage RL and mute any goal sent by navigation, this function is called once 
        self.RL_activate()


        


if __name__ == '__main__':
    try:
        rospy.init_node('move_car_action_client', anonymous=True)
        #Initializing master client object
        cm = clients_master()

        #calling the RL policy action server, to execute the sent actions generated by RL policy
        #cm.RLPolicyAction_server()

        #calling the navigation listener, to execute goals sent by navigation stack
        cm.nav_listener()
        
        #wait for any callbacks in navigation listener or RL action policy server 
        rospy.spin()

    except rospy.ROSInterruptException as e:
        print 'Error in move_car_client: ', e




#rosrun image_view image_view image:=/racecar/camera1/image_raw