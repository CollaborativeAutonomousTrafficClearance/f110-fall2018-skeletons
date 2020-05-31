#!/usr/bin/env python
import rospy
from racecar_rl_environments.msg import areWeDone


def callback(data):
    rospy.loginfo("the activasion is %d and done state is %d" % (data.is_activated, data.is_episode_done))
    
def listener():
    rospy.init_node('sub', anonymous=True)
    rospy.Subscriber("sub", areWeDone, callback)
   
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
   
if __name__ == '__main__':
    listener()