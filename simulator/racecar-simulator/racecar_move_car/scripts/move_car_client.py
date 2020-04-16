#!/usr/bin/env python
import rospy
import actionlib
from racecar_move_car.msg import Move_CarAction, Move_CarGoal, MoveCarGoal, Move_CarFeedback, Move_CarResult
from std_msgs.msg import Bool
import message_filters
from racecar_navigation.msg import BoolWithHeader, Nav_Action

class baseClient:
    def __init__(self, ActionTopicName):
        rospy.loginfo("inside the baseclient")
        self.current_goal = Move_CarGoal()
        self.current_feedback = Move_CarFeedback()   #In case of lane change, this should be updated, very critical variable
        self.current_result = Move_CarResult()    #Check if initializing actions before sending is possible, for the result variable to be initially false
        self.client = actionlib.SimpleActionClient(ActionTopicName, Move_CarAction)
        #self.client.wait_for_server()
        rospy.loginfo("outside the baseclient")

    def cancel_all_goals(self):
        self.client.cancel_all_goals()
        rospy.loginfo("move car client goal cancels all old goals")

    def cancel_this_goal(self):
        self.client.cancel_goal()
        rospy.loginfo("move car client goal cancels all old goals")
    
    def cancel_goals_before(self, time):
        self.client.cancel_goals_at_and_before_time(time)

    def send_new_goal(self, goal):   #If preemption is a must, cancel the old goal first
        self.current_goal = goal
        self.client.send_goal(self.current_goal, self.done_cb, self.active_cb, self.feedback_cb)

    def feedback_cb(self, feedback):
        self.current_feedback = feedback
        rospy.loginfo("move car client goal cancels all old goals")
    
    def active_cb(self):
        rospy.loginfo("move car client goal cancels all old goals")
    
    def done_cb(self,result):
        self.current_result = result
        rospy.loginfo("move car client goal cancels all old goals")
            
    def destroy_last_goal_handle(self):
        self.client_lane_keeping.stop_tracking_goal()    #brief Stops tracking the state of the current goal. Unregisters this goal's callbacks


class laneKeepingClient(baseClient):
    def __init__(self):
        rospy.loginfo("inside the lk")
        actionName = "move_car/laneKeeping_action_server"
        baseClient.__init__(self, actionName)
        self.current_goal.mcGoal.speed = -1  #Undefined for this goal
        self.current_goal.mcGoal.acc = -100  #Undefined for this goal
        self.current_goal.mcGoal.direction = -1 #Undefined for this goal
        rospy.loginfo("outside the lk")

class velocityClient(baseClient):
    def __init__(self):
        rospy.loginfo("inside the vc")
        self.actionName = 'move_car/velocity_action_server'
        baseClient.__init__(self, self.actionName)
        self.last_vel = 0.0
        self.vel = 0.0
        self.last_acc = 0.0
        self.acc = 0.0
        rospy.loginfo("outside the vc")
    
    def update_vel(self, new_vel):
        self.vel = new_vel
    
    def update_acc(self, new_acc):
        self.acc = new_acc
    
    def update_vel_acc(self, new_vel, new_acc):
        self.update_vel(new_vel)
        self.update_acc(new_acc)

class laneChangeClient(baseClient):
    def __init__(self):
        rospy.loginfo("inside the lc")
        self.actionName = 'move_car/laneChange_action_server'
        baseClient.__init__(self, self.actionName)
        self.direction = 0
        self.last_direction = 0
        self.current_goal.mcGoal.speed = -1  #Undefined for this goal
        self.current_goal.mcGoal.acc = -100  #Undefined for this goal
        rospy.loginfo("outside the lc")

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

        while self.lc_client.current_result == 0:
            rospy.loginfo("Lane change is still unfinished")

        self.lc_client.cancel_all_goals(self, "lane change")
        self.vel_client.destroy_last_goal_handle()

    def check_laneChange(self):
        if self.lc_client.current_result == 0:
            rospy.loginfo("Lane change is still unfinished")
            return False
        elif self.lc_client.current_result == -1:
            return True
        else:
            return True


class nav_master(action_source):
    def __init__(self):
        rospy.loginfo("inside the nav master")
        action_source.__init__(self)
        self.last_chosen_action = 100   #Initially
        self.chosen_action = Nav_Action()
        self.goalReached = BoolWithHeader()
        self.vel_client.current_goal.mcGoal.action_source = 0
        self.lk_client.current_goal.mcGoal.action_source = 0
        self.lc_client.current_goal.mcGoal.action_source = 0
        self.lc_client.current_feedback = -1   #To enter the first loop in call server
        self.lc_client.current_result = -1  #To enter the first loop in call server
        rospy.loginfo("outside the nav master")
    
    def update_actions_goals(self, chosen_action_in, goalReached_in):
        self.last_chosen_action = self.chosen_action.control_action
        rospy.loginfo("Inside update 1 ")
        self.chosen_action.control_action = chosen_action_in.control_action
        self.goalReached.Bool = goalReached_in.Bool
        rospy.loginfo(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
        rospy.loginfo(self.chosen_action.control_action)
        self.chosen_action.header.stamp = rospy.Time.now()
        self.goalReached.header.stamp = rospy.Time.now()
        self.call_servers()
        rospy.loginfo("Inside update ")

    def call_servers(self):
        while self.check_laneChange() == 0:
            rospy.loginfo("Cannot call a new service, Lane change is still unfinished")
        
        rospy.loginfo("Inside call server ")
        if self.last_chosen_action != self.chosen_action.control_action or self.last_chosen_action == self.chosen_action.control_action  :
            rospy.loginfo("Inside call server, if condition")
            if self.chosen_action.control_action == 0:
                #Setting lane keeping client goal
                self.lk_client.current_goal.mcGoal.header.stamp = rospy.Time.now()
                self.lk_client.send_new_goal(self.lk_client.current_goal)
                rospy.loginfo("goal sent")
                #Setting velocity client goal
                self.vel_client.current_goal.mcGoal.header.stamp = rospy.Time.now()
                self.vel_client.current_goal.mcGoal.speed = -1  #Undefined for this action source
                self.vel_client.current_goal.mcGoal.acc = -100  #Undefined for this action source
                self.vel_client.current_goal.mcGoal.direction = -1 #Undefined for this action source
                self.vel_client.send_new_goal(self.vel_client.current_goal)
            else:
                self.lc_client.current_goal.header.stamp = rospy.Time.now()

                if self.chosen_action.control_action == 1:
                    self.current_goal.direction = 1 
                    self.lc_client.send_new_goal(self.lc_client.current_goal)
                
                elif self.chosen_action.control_action == 2:
                    self.current_goal.direction = 2
                    self.lc_client.send_new_goal(self.lc_client.current_goal)
                
                #If the lane change is feasible, cancel lane keeping and velocity clients goals
                if self.lc_client.current_feedback == 1:
                    self.lk_client.cancel_all_goals()
                    self.lk_client.destroy_last_goal_handle()
                    self.vel_client.cancel_all_goals()
                    self.vel_client.destroy_last_goal_handle()
        

class qLearning_ambulance_master:
    def __init__(self):
        action_source.__init__(self)
        self.deltaTime = 1  


def listener():
    rospy.loginfo("Inside the listener in the client")
    navigation = nav_master()
    chosen_nav_action = message_filters.Subscriber("move_car/nav/chosen_action", Nav_Action)
    goalReached  = message_filters.Subscriber("move_car/nav/goalReached", BoolWithHeader)
    ts = message_filters.TimeSynchronizer([chosen_nav_action, goalReached], 50)
    rospy.loginfo("before synchronized msg")
    ts.registerCallback(navigation.update_actions_goals)
    rospy.loginfo("after synchronized msg")
    rospy.spin()

if __name__ == '__main__':
    try:
        rospy.init_node('move_car_client', anonymous=True)
        listener()

    except rospy.ROSInterruptException as e:
        print 'Error in action client lanekeeping: ', e



