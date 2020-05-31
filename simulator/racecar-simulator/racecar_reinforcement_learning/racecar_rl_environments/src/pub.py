#!/usr/bin/env python
import rospy
from racecar_rl_environments.msg import areWeDone
from std_msgs.msg import Bool

def talker():

    pub = rospy.Publisher('test_topic', areWeDone)
    rospy.init_node('pub', anonymous=True)
    r = rospy.Rate(10) #10hz
    msg = areWeDone()
    msg.is_activated = 0
    msg.is_episode_done = 2

   
    while not rospy.is_shutdown():

        rospy.loginfo(msg)
        pub.publish(msg)
        r.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass