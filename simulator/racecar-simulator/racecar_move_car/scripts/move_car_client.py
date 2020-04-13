#!/usr/bin/env python
import rospy
import actionlib
from racecar_move_car.msg import Move_CarAction, Move_CarGoal, MoveCarGoal, Move_CarFeedback, Move_CarResult
from std_msgs.msg import Bool
import message_filters
from racecar_navigation.msg import BoolWithHeader, Nav_Action
import roslib
roslib.load_manifest('racecar_move_car')
####from racecar_qlearning.msg #import RL_Policy_Action 

class baseClient:
    def __init__(self, ActionTopicName):
        self.current_goal = MoveCarGoal()
        self.current_feedback = Move_CarFeedback()   #In case of lane change, this should be updated, very critical variable
        self.current_result = Move_CarResult()    #Check if initializing actions before sending is possible, for the result variable to be initially false
        self.client = actionlib.SimpleActionClient(ActionTopicName, Move_CarAction)
        self.client.wait_for_server()

    def cancel_all_goals(self, client_name):
        self.client.cancel_all_goals()
        rospy.loginfo("move car %s client goal cancels all old goals", client_name)

    def cancel_this_goal(self, client_name):
        self.client.cancel_goal()
        rospy.loginfo("move car %s client goal cancels this goal", client_name)
    
    def cancel_goals_before(self, time):
        self.client.cancel_goals_at_and_before_time(time)

    def send_new_goal(self, goal):   #If preemption is a must, cancel the old goal first
        self.current_goal = goal
        self.client.send_goal(self.current_goal, self.done_cb, self.active_cb, self.feedback_cb)

    def feedback_cb(self, feedback, client_name):
        self.current_feedback = feedback
        rospy.loginfo("move car %s client just recieved feedback", client_name)
    
    def active_cb(self, client_name):
        rospy.loginfo("move car %s client goal just sent", client_name)
    
    def done_cb(self,result, client_name):
        self.current_result = result
        rospy.loginfo("move car %s client goal just recieved result", client_name)
            
    def destroy_last_goal_handle(self):
        self.client_lane_keeping.stop_tracking_goal()    #brief Stops tracking the state of the current goal. Unregisters this goal's callbacks


class laneKeepingClient(baseClient):
    def __init__(self):
        self.actionName = 'move_car/laneKeeping_action_server'
        baseClient.__init__(self, self.actionName)
        self.current_goal.speed = -1  #Undefined for this goal
        self.current_goal.acc = -100  #Undefined for this goal
        self.current_goal.direction = -1 #Undefined for this goal
    

class velocityClient(baseClient):
    def __init__(self):
        self.actionName = 'move_car/velocity_action_server'
        baseClient.__init__(self, self.actionName)
        self.last_vel = 0.0
        self.vel = 0.0
        self.last_acc = 0.0
        self.acc = 0.0
    
    def update_vel(self, new_vel):
        self.vel = new_vel
    
    def update_acc(self, new_acc):
        self.acc = new_acc
    
    def update_vel_acc(self, new_vel, new_acc):
        self.update_vel(new_vel)
        self.update_acc(new_acc)

class laneChangeClient(baseClient):
    def __init__(self):
        self.actionName = 'move_car/laneChange_action_server'
        baseClient.__init__(self, self.actionName)
        self.direction = 0
        self.last_direction = 0
        self.current_goal.speed = -1  #Undefined for this goal
        self.current_goal.acc = -100  #Undefined for this goal

class action_source:
    def __init__(self):
        self.lk_client = laneKeepingClient()
        self.vel_client = velocityClient()
        self.lc_client = laneChangeClient()
    
    def mute_ControlSource(self):
        self.lk_client.cancel_all_goals(self, "lane keeping")
        self.lk_client.destroy_last_goal_handle()
        self.vel_client.cancel_all_goals(self, "velocity")
        self.vel_client.destroy_last_goal_handle()

        while self.lc_client.current_result == False:
            rospy.loginfo("Lane change is still unfinished")

        self.lc_client.cancel_all_goals(self, "lane change")
        self.vel_client.destroy_last_goal_handle()

    def check_laneChange(self):
        if self.lc_client.current_result == False:
            rospy.loginfo("Lane change is still unfinished")
            return False

        return True


class nav_master(action_source):
    def __init__(self):
        action_source.__init__(self)
        self.last_chosen_action.control_action = -100   #Initially
        self.chosen_action = Nav_Action()
        self.goalReached = BoolWithHeader()
        self.vel_client.current_goal.action_source = 0
        self.lk_client.current_goal.action_source = 0
        self.lc_client.current_goal.action_source = 0
    
    def update_actions_goals(self, chosen_action, goalReached):
        self.chosen_action = chosen_action
        self.goalReached = goalReached
        self.call_servers()

    def call_servers(self):
        while self.check_laneChange == False:
            rospy.loginfo("Cannot call a new service, Lane change is still unfinished")

        if self.last_chosen_action.control_action != self.chosen_action.control_action:
            if self.chosen_action == 0:
                #Setting lane keeping client goal
                self.lk_client.current_goal.header.stamp = rospy.Time.now()
                self.lk_client.send_new_goal(self.lk_client.current_goal)
            
                #Setting velocity client goal
                self.vel_client.current_goal.header.stamp = rospy.Time.now()
                self.vel_client.current_goal.speed = -1  #Undefined for this action source
                self.vel_client.current_goal.acc = -100  #Undefined for this action source
                self.vel_client.current_goal.direction = -1 #Undefined for this action source
                self.vel_client.send_new_goal(self.vel_client.current_goal)
            else:
                self.lc_client.current_goal.header.stamp = rospy.Time.now()

                if self.lk_client.chosen_action.control_action == 1:
                    self.lc_client.current_goal.direction = 1 
                    self.lc_client.send_new_goal(self.lc_client.current_goal)
                
                elif self.lk_client.chosen_action.control_action == 2:
                    self.lc_client.current_goal.direction = 2
                    self.lc_client.send_new_goal(self.lc_client.current_goal)
                
                #If the lane change is feasible, cancel lane keeping and velocity clients goals
                if self.lc_client.current_feedback == True:
                    self.lk_client.cancel_all_goals()
                    self.lk_client.destroy_last_goal_handle()
                    self.vel_client.cancel_all_goals()
                    self.vel_client.destroy_last_goal_handle()


class qLearning_ambulance_master:
    def __init__(self):
        action_source.__init__(self)
        self.deltaTime = 1  


def listener():
    navigation = nav_master()
    chosen_nav_action = message_filters.Subscriber("move_car/nav/chosen_action", Nav_Action)
    goalReached  = message_filters.Subscriber("move_car/nav/goalReached", BoolWithHeader)
    ts = message_filters.TimeSynchronizer([chosen_nav_action, goalReached], 50)
    ts.registerCallback(navigation.update_actions_goals)

    #rospy.Subscriber("move_car/RL/chosen_action", RL_Policy_Action, c2.)
    rospy.spin()

if __name__ == '__main__':
    try:
        rospy.init_node('action_client_LaneKeeping')
        listener()

    except rospy.ROSInterruptException as e:
        print 'Error in action client lanekeeping: ', e



