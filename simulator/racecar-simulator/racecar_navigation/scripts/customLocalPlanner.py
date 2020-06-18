#!/usr/bin/env python
import rospy
import tf.transformations
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from racecar_navigation.msg import Lanes_Info, NavAction, BoolStamped
from racecar_navigation.srv import CustomLocalPlannerFeedback, CustomLocalPlannerFeedbackRequest, CustomLocalPlannerFeedbackResponse
import message_filters

pub = rospy.Publisher('move_car/nav/chosen_action', NavAction, queue_size=20)

class CustomLocalPlanner:
    def __init__(self):
        self.final_action = NavAction()
        self.chosenAction_header = self.final_action.header
        self.oneLaneChangeExecuted = False
        self.newGoal = False

    def chosenAction(self,future_LaneInfo, current_LaneInfo):
        "Lanes: "
        "y: 0.5| Lane 0  |0| Lane 1 |-0.5| Lane 2 |-1 "
        #rospy.loginfo("\is one change executed? : %i\n", self.oneLaneChangeExecuted) #TODO
        #only send a new goal if lane change is not being executed 
        if self.oneLaneChangeExecuted == False:
            if ((future_LaneInfo.map_array[1]  < current_LaneInfo.map_array[1])):
                self.final_action.control_action = 1  #left lane change
        
            elif ((future_LaneInfo.map_array[1]  > current_LaneInfo.map_array[1])):
                self.final_action.control_action = 2  #right lane change

            else:
                self.final_action.control_action = 0 #lane keep

        else:
            self.final_action.control_action = 0 #lane keep
        
        self.chosenAction_header.stamp = rospy.Time.now()
        rospy.loginfo("\nthe goal to be sent to the move car action client is %i\n", self.final_action.control_action)
        pub.publish(self.final_action)

    def listener_current_future_lanes(self):
        # listens to where the car is (current lane) and where the car wants to go (future lane)
        FutureLaneInfoSub = message_filters.Subscriber("move_car/nav/future_laneInfo", Lanes_Info)
        CurrentLaneInfoSub  = message_filters.Subscriber("move_car/nav/current_laneInfo", Lanes_Info)
        ts = message_filters.TimeSynchronizer([FutureLaneInfoSub, CurrentLaneInfoSub], 20)
        ts.registerCallback(self.chosenAction)
    
    def handle_customLocalPlannerFeedbackSrv(self, feedback):
        # it sets the flag indicating lane change is active or not
        self.oneLaneChangeExecuted = feedback.laneChangeActive
        
        return True
    
    def feedback_server(self):
        # feedback for the navigation module to stop sending lane change goals
        feedback = rospy.Service('move_car/Nav/CustomLocalPlannerFeedbackSrv', CustomLocalPlannerFeedback, self.handle_customLocalPlannerFeedbackSrv)
        
        rospy.loginfo("Estabilishing a service server client link between Custom Local Planner and Move Car Action Client")

    def alarm_new_goal(self, msg):
        self.newGoal = True
        rospy.loginfo("A new goal is sent")

    def listener_new_goals(self):
        rospy.Subscriber("move_base_simple/goal", PoseStamped, self.alarm_new_goal)    


if __name__ == '__main__':
    rospy.init_node('customLocalPlanner', anonymous=True)
    clp = CustomLocalPlanner()
    clp.listener_current_future_lanes()
    clp.feedback_server()
    clp.listener_new_goals()
    rospy.spin()
