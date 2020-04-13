#!/usr/bin/env python
import rospy
import actionlib
from racecar_navigation.msg import  LaneKeepAction, LaneKeepGoal
from std_msgs.msg import Bool
import message_filters
from racecar_navigation.msg import BoolWithHeader, Nav_Action

class ActionClient:
    def __init__(self):
        self.client = actionlib.SimpleActionClient("Lane_Keep_as", LaneKeepAction)
        self.client.wait_for_server()
        self.last_chosen_action = True 
    
    def call_server(self, chosen_action, goalReached):
        if goalReached.Bool == True:
            self.goal = LaneKeepGoal(False)
            self.client.send_goal(self.goal)
        else:
            if self.last_chosen_action.control_action != chosen_action.control_action:
                self.goal = LaneKeepGoal(chosen_action.control_action)
                self.client.send_goal(self.goal)
                self.last_chosen_action = chosen_action
            


    

def listener():
    c = ActionClient()
    chosen_action = message_filters.Subscriber("move_car/nav/chosen_action", Nav_Action)
    goalReached  = message_filters.Subscriber("move_car/nav/goalReached", BoolWithHeader)
    ts = message_filters.TimeSynchronizer([chosen_action, goalReached], 50)
    ts.registerCallback(c.call_server)
    rospy.spin()

if __name__ == '__main__':
    try:
        rospy.init_node('action_client_LaneKeeping')
        listener()

    except rospy.ROSInterruptException as e:
        print 'Error in action client lanekeeping: ', e



