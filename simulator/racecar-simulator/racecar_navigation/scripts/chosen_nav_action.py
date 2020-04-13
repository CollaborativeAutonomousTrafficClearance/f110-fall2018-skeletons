#!/usr/bin/env python
import rospy
import tf.transformations
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from racecar_navigation.msg import Lanes_Info
from racecar_navigation.msg import BoolWithHeader, Nav_Action
import message_filters

pub = rospy.Publisher('move_car/nav/chosen_action', Nav_Action, queue_size=20)

def chosenAction(future_LaneInfo, current_LaneInfo):
    final_action = Nav_Action()
    h = final_action.header
    h.stamp = rospy.Time.now()
    if future_LaneInfo.map_array[1] == current_LaneInfo.map_array[1]:
        final_action.control_action = 0
        pub.publish(final_action)
    elif future_LaneInfo.map_array[1]  > current_LaneInfo.map_array[1]:
        final_action.control_action = 1
        pub.publish(final_action)
    else:
        final_action.control_action = 2
        pub.publish(final_action)



def listener():
    rospy.init_node('chosen_nav_action', anonymous=True)
    FutureLaneInfoSub = message_filters.Subscriber("move_car/nav/future_laneInfo", Lanes_Info)
    CurrentLaneInfoSub  = message_filters.Subscriber("move_car/nav/current_laneInfo", Lanes_Info)
    ts = message_filters.TimeSynchronizer([FutureLaneInfoSub, CurrentLaneInfoSub], 20)
    ts.registerCallback(chosenAction)
    rospy.spin()

if __name__ == '__main__':
    listener()