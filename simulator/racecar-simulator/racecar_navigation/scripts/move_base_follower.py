#!/usr/bin/env python

"""
This node subscribes to the control commands sent by move_base and publishes
 AckermannDriveStamped messages. The commands sent by move_base are of type
 geometry_msgs/Twist
"""

import rospy
from geometry_msgs.msg import Twist
from ackermann_publisher import AckermannPublisher
import math

class MoveBaseFollower(AckermannPublisher):

    def __init__(self, node_name):
        super(MoveBaseFollower, self).__init__(node_name)
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_callback, queue_size=5)

    def cmd_callback(self, msg):
        speed = math.sqrt(msg.linear.x*msg.linear.x + msg.linear.y*msg.linear.y)
        steering_angle = msg.angular.z

        self.publish_ackermann(steering_angle, speed)


if __name__ == '__main__':
    MoveBaseFollower("move_base_follower")
    rospy.spin()