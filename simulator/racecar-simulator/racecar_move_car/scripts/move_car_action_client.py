#!/usr/bin/env python
import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
import message_filters

from std_msgs.msg import Bool
from racecar_navigation.msg import BoolWithHeader, NavAction
from racecar_move_car.msg import Goal, MoveCarAction, MoveCarGoal, MoveCarFeedback, MoveCarResult
#from racecar_qlearning.msg import RLPolicyRequest
from racecar_qlearning.srv import RLPolicyActionService, RLPolicyActionServiceRequest, RLPolicyActionServiceResponse
import threading



#RL_lock = threading.Lock()
#RL_lock.acquire()

# @Locks For Lane Change

#for lane change feedback check
lc_lock_feedback = threading.Lock()
lc_lock_feedback.acquire()


class baseClient:

    def __init__(self, ActionTopicName):
        #initializing action client and blocking till it connects 
        self.client = actionlib.SimpleActionClient(ActionTopicName, MoveCarAction) 
        self.client.wait_for_server() 

        #initializing the action goal to be sent, current and last  
        self.current_goal = MoveCarGoal() 

        #initializing the action feedback
        self.current_feedback = MoveCarFeedback()   
        self.current_feedback.mcFeedback = -2 # uninitialized

        #initializing the action result 
        self.current_result = MoveCarResult()
        self.current_result.mcResult = -2 # uninitialized

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
        self.current_goal.mcGoal.header.stamp = rospy.Time.now()

        #send the class' current goal
        self.client.send_goal(self.current_goal, self.done_cb, self.active_cb, self.feedback_cb) # specify appropriate callback functions
    
    def send_goal_and_wait(self, delta_time):
        #function for sending a goal and waiting delta time to be executed
        self.state = self.client.send_goal_and_wait(self.current_goal, execute_timeout = delta_time)
    
    def feedback_cb(self, feedback): 
        #callback function to update feedback
        self.current_feedback = feedback

        rospy.loginfo("Retuned feedback for this goal, feedback is %i.", self.current_feedback.mcFeedback)
        
    def get_feedback(self):
        return self.current_feedback.mcFeedback 
        
    def active_cb(self):
        #active callback function
        rospy.loginfo("Goal is currently active.")
    
    def done_cb(self, state, result): 
        #callback function to update result
        self.current_result = result
        rospy.loginfo("Retuned result for this goal.")
    
    def get_state(self):
        #Pending:0, Active:1, Preempted:2, Succeeded:3, Aborted:4, Rejected:5, Preempting:6, Recalling:7, 
        #Recalled:8, lost:9
        return self.client.get_state()

    def block_till_result(self):
        #Blocking the call thread till result is registered 
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

        #Defining timings
        self.lc_goal_activated_time = rospy.Time()   #Initially, 0
        self.lc_goal_duration = rospy.Time()   #Initially, 0

    def set_direction(self, dir):  #1 --> Left Lane Change, 2 --> Right Lane Change
        self.set_control_action(dir)

    def reset_timing(self):
        #Initialize the timings again
        self.lc_goal_activated_time = rospy.Time()
        self.lc_goal_duration = rospy.Time()

    # @ redfining the lane change callback as is it uses locks

    def send_new_goal(self):   # send a new goal to server
        self.current_feedback.mcFeedback = -1 # initialize feedback with -1
        self.current_result.mcResult = -1 # initialize result with -1
        
        #set the header just before sending the goal
        self.current_goal.mcGoal.header.stamp = rospy.Time.now()

        #send the class' current goal
        self.client.send_goal(self.current_goal, self.done_cb_lc, self.active_cb_lc, self.feedback_cb_lc) # specify appropriate callback functions
    
    
    def active_cb_lc(self):
        #active callback function
        self.lc_goal_activated_time = rospy.get_time()
        rospy.loginfo("Goal is currently active.")
    
    def feedback_cb_lc(self, feedback): 
        
        #callback function to update feedback  
        self.current_feedback = feedback
        rospy.loginfo("Retuned feedback for this goal, lane change feasibility is %i.", self.current_feedback.mcFeedback) 
        
        #Releasing this lock to indicate that feasibility is done checking 
        lc_lock_feedback.release()




    def done_cb_lc(self, state, result): 
        #callback function to update result
        if result.mcResult != 0:   #If the lane change was feasible
            self.lc_goal_duration = rospy.get_time() - self.lc_goal_activated_time
        else:
            self.lc_goal_duration = 0

        #rospy.loginfo("Lane change lasted for %f", self.lc_goal_duration)
        self.current_result = result
        rospy.loginfo("Retuned result for this goal, result is %i", self.current_result.mcResult)

        #If the feedback is not released when the result is called, release feedback
        if lc_lock_feedback.locked():
            lc_lock_feedback.release()

        #lc_lock_result.release()


    

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

    # check whether lane change is still executing
    def checkLaneChange(self):
        #Check whether the lane change is finished or not according to it's result
        if self.lc_client.get_result() == -1:
            rospy.loginfo("Lane change is still unfinished.")
            return False
        else:
            return True
    
    def cancel_goals_lk_vel_clients(self):
        #Cancel lane keeping and lane change goals 
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

        #Set the action source in the move car goal to be zero, indicating navigation
        self.vel_client.set_action_source(0)
        self.lk_client.set_action_source(0)
        self.lc_client.set_action_source(0)

        #Set the acceleration in velocity to be -100 as a flag, as 
        #velocity client takes it's acceleration from cmd_vel from Nav Stack
        self.vel_client.set_acc(-100) 

        self.deactivate = False   #nav is activated

    
    def update_action_server_goals(self, chosen_action_in, goalReached_in):
        #self.goalReached = goalReached_in
        self.current_nav_control_action = chosen_action_in.control_action
        
        #rospy.loginfo("Received nav action in move_car action client, action is %i",self.current_nav_control_action )

        #If the action is lane change, update it's direction
        if chosen_action_in.control_action > 0 : 
            self.lc_client.set_control_action(chosen_action_in.control_action)

        #Calling the server to send goals
        if not self.deactivate and goalReached_in: 
            self.call_servers()


    def call_servers(self):

        # if both last and current actions are lane keeping, do not send any new goals
        if (self.current_nav_control_action == 0 and self.last_nav_control_action == 0):
            rospy.loginfo("Keep lane keeping.")
            
        else:
            # if current action is lane keeping, send goals to lk and lk vel servers
            if (self.current_nav_control_action == 0):
                    
                # Cancel any current goals from all clients 
                self.muteControlSource()

                #Send goals in for velocity and lane keeping clients 
                rospy.loginfo("Sening lane keeping goals in navigation module")
                self.vel_client.send_new_goal()
                self.lk_client.send_new_goal() 

            else:
                # if current action is lane change, send goal to lc server
                self.lc_client.send_new_goal() 
                    
                #Blocking the function till lane change feasibility check is finished (Feedback changes)
                rospy.loginfo("Waiting for lane change feasibility check to end")
                lc_lock_feedback.acquire()
                rospy.loginfo("lane change feasibility check ended")

                # if lane change is infeasible, lane keep instead
                if (self.lc_client.get_feedback() == 0 or self.lc_client.get_result() == 0):
                    rospy.loginfo("Lane change was infeasible; lane keeping instead")

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

        #Storing control actions sent to RL, and acceleration values for RL module
        self.last_RL_control_action = -1   # Initially (0 --> Lane keeping, 1--> left lane change, 2--> right lane change)
        self.RL_control_action = -1 # Initially (0 --> Lane keeping, 1--> left lane change, 2--> right lane change)
        self.RL_acc = 0   #Initially 

        #Set the action source in the move car goal in clients to be 1, indicating RL policy
        self.lc_client.set_action_source(1)
        self.lk_client.set_action_source(1)
        self.vel_client.set_action_source(1)

        #delta time for lane keeping goal, lane change takes its desired time
        self.delta_lk_time = 6
        self.delta_lk_duration = rospy.Duration(self.delta_lk_time,0)

        #The response RL action time
        self.RL_resp_time = 0  #Initially zero

        #storing if the action is finished or infeasible, to be sent to the client 
        # 0 --> Infeasible action, 1--> Finished Action
        self.RLActionFeasibility = False

    def set_RL_goal_params(self, action_in, acc_in):
        self.RL_control_action = action_in

        if action_in == 0:   #If the action is lane keeping
            self.RL_acc = acc_in
            self.vel_client.set_acc(self.RL_acc)  #No server other that vel server uses acceleration
        else:   #If the action is lane change
            self.lc_client.set_direction(action_in)
    
    def get_RL_response(self):
        return self.RLActionFeasibility, self.RL_resp_time


    #Set and reset the delta time used in RL policy action
    def set_RL_resp_time(self, time):
        self.RL_resp_time = time
    
    def reset_RL_resp_time(self):
        self.RL_resp_time = 0

    def call_servers(self):   
        rospy.loginfo("Will Execute RL goal")
        if self.RL_control_action == 0:   #If the control action is lane keeping
            rospy.loginfo("Sending lane keeping goal")
            #start lane keeping
            self.lk_client.send_new_goal()  
            #keep waiting for the goal for the required delta time
            rospy.loginfo("Sending velocity goal for duration: %i", self.delta_lk_time)
            before = rospy.get_time()
            self.vel_client.send_goal_and_wait(self.delta_lk_duration) 
            duration = rospy.get_time() - before

            rospy.loginfo("Velocity goal is ended after duration: %f", duration)

            #Cancel the lane keeping goal
            rospy.loginfo("Cancelling lane keeping goal")
            self.lk_client.cancel_this_goal()    
                
            #setting the result to be sent to the service client again
            if self.vel_client.get_state()== 4:  #infeasible goal, ABORTED
                rospy.loginfo("lane keeping with acceleration: %f was infeasible", self.RL_acc)
                self.RLActionFeasibility = False 

                #Reset the time
                self.reset_RL_resp_time()   
                
            elif self.vel_client.get_state()== 3:  #accomplished goal, SUCCEEDED
                rospy.loginfo("lane keeping with acceleration: %f was feasible", self.RL_acc)
                self.RLActionFeasibility = True

                #Set the RL action time as the lane keeping action defined duration
                self.set_RL_resp_time(self.delta_lk_time)  
            else:
                rospy.loginfo("Undefined feasibility check, status is %i", self.vel_client.get_state())
        
        #If the action is lane change
        else:     
            self.lc_client.send_new_goal()  
            #Blocking the function till lane change feasibility check is finished (Feedback changes)
            rospy.loginfo("Waiting for lane change feasibility check to end")
            lc_lock_feedback.acquire()
            rospy.loginfo("lane change feasibility check ended")

            # if lane change is infeasible, send back to RL master to chose another action
            if (self.lc_client.get_feedback() == 0 or self.lc_client.get_result() == 0):
                rospy.loginfo("Lane change was infeasible; returning to RL master")
                    
                #Setting the return parameters
                self.RLActionFeasibility = False       
                self.reset_RL_resp_time()

            # If lane change is feasible, wait for execution
            elif (self.lc_client.get_feedback() == 1):
                rospy.loginfo("Lane change was feasible with direction: %i; waiting for the goal to be executed", self.RL_control_action)
                self.lc_client.block_till_result()
                rospy.loginfo("Lane change finished executed") 
                #Send the action results to master 
                self.RLActionFeasibility = True
                self.set_RL_resp_time(self.lc_client.lc_goal_duration) 
 


class clients_master:
    def __init__(self):
        self.navigation = nav_master()
        self.RL = qLearning_ambulance_master()
        self.isRLactivated = False    #False --> navigation is mastering, True --> RL is mastering

    ##listener for navigation chosen goals   
    def nav_listener(self):    

        chosen_action = message_filters.Subscriber("move_car/nav/chosen_action", NavAction)
        goalReached  = message_filters.Subscriber('move_car/nav/goalReached', BoolWithHeader)
        ts = message_filters.TimeSynchronizer([chosen_action, goalReached], 50)
        ts.registerCallback(self.navigation.update_action_server_goals)
        
        #rospy.Subscriber("move_car/nav/chosen_action", NavAction, self.navigation.update_action_server_goals)

    
    def RL_activate(self):
        #function to indicate that RL should be activated, it waits for navigation to end
        rospy.loginfo("RL is activated")
        self.isRLactivated = True
        self.navigation.__init__()   #initializing navigation variables
        self.navigation.deactivate = True
        self.navigation.muteControlSource()

    
    def handle_RLPolicyAction_service(self, chosen_action_in):
        if self.isRLactivated == False: 
            self.RL_activate()

        #handler for the RL policy action service when the RL requests an action to be performed 
        rospy.loginfo("Move Car Client recieved a request from RL master")
        rospy.loginfo("The control action requested is %i", chosen_action_in.control_action)
        rospy.loginfo("The acceleration requested is %f", chosen_action_in.acc)
        #if the RL action policy service request's flag is false, this means the car is still
        #in the ambulance's window
        if chosen_action_in.doneRLflag == False :

            #update the current chosen action parameters 
            self.RL.set_RL_goal_params(chosen_action_in.control_action, chosen_action_in.acc)

            #waiting for the action to be processed 
            self.RL.call_servers()

            feasibility_Res, action_duration = self.RL.get_RL_response()

            #Reset RL, lane change duration values
            self.RL.reset_RL_resp_time()
            self.RL.lc_client.reset_timing()

            rospy.loginfo("Returning Response to RL master")
            #return the response when done, in order for the RL action generator to send a new action 
            resp = RLPolicyActionServiceResponse()
            resp.RLActionresult = feasibility_Res
            #resp.RLActionTime = action_duration
            resp.RLActionTime = action_duration
            return resp
        
        else:
            #If the car is outside the ambulance window
            self.isRLactivated = False
            self.navigation.deactivate = False #activate again
            RL_finished = True

            rospy.loginfo("Returning Response to RL master")
            resp = RLPolicyActionServiceResponse()
            resp.RLActionresult = RL_finished
            resp.RLActionTime = 0
            return resp
    
    def RLPolicyAction_server(self):
        #the service connected to the RL policy action generator, when the action is finished, it informs the 
        #action generator
        self.RLPolicyActionService = rospy.Service('move_car/RL/RLPolicyActionService', RLPolicyActionService, self.handle_RLPolicyAction_service)
        
        rospy.loginfo("Estabilishing a service server client link between RL master and move car client master")
        
        #rospy.sleep(2)


        


if __name__ == '__main__':
    try:
        rospy.init_node('move_car_action_client', anonymous=True)
        #Initializing master client object
        cm = clients_master()

        #calling the RL policy action server, to execute the sent actions generated by RL policy
        cm.RLPolicyAction_server()

        #calling the navigation listener, to execute goals sent by navigation stack
        cm.nav_listener()
        
        #wait for any callbacks in navigation listener or RL action policy server 
        rospy.spin()

    except rospy.ROSInterruptException as e:
        print 'Error in move_car_client: ', e

