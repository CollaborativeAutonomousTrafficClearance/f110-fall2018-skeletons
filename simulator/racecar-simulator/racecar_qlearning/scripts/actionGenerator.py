#!/usr/bin/env python
import rospy
import random
from racecar_qlearning.msg import RLPolicyAction


#This node randomly publish actions, simulation for RL

pub = rospy.Publisher('move_car/RL/chosen_action', RLPolicyAction, queue_size=10)

def genActions():
    chosen_action = RLPolicyAction()
    chosen_action.header.stamp = rospy.Time.now()
    
    random_action = random.randint(1,5)
    if random_action == 1:
        chosen_action.control_action = 1   #left lane change
    elif random_action == 2:
        chosen_action.control_action = 2   #right lane change
    elif random_action == 3:
        chosen_action.control_action = 0   #keep constant velocity
        chosen_action.acc = 0.0
    elif random_action == 4:
        chosen_action.control_action = 0   #accelerate by 1
        chosen_action.acc = 0.1
    elif random_action == 5:
        chosen_action.control_action = 0   #deccelerate by 1
        chosen_action.acc = -0.1

    pub.publish(chosen_action)


if __name__ == '__main__':
    
    rospy.init_node("actionGenerator", anonymous=True)
    genActions()
    rospy.spin()

    