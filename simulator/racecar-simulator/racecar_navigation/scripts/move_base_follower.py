#!/usr/bin/env python

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
import math
from geometry_msgs.msg import Twist

ack_publisher = rospy.Publisher('/ackermann_cmd_mux/input/teleop', AckermannDriveStamped, queue_size=1)

def ackermanCallback(msg):
    ack_msg = AckermannDriveStamped()
    speed = math.sqrt(msg.linear.x*msg.linear.x + msg.linear.y*msg.linear.y)
    steering_angle = msg.angular.z
    ack_msg.drive.steering_angle = steering_angle
    ack_msg.drive.speed = speed

    ack_publisher.publish(ack_msg)


if __name__ == '__main__':
    rospy.init_node('custom_move_base_follower',anonymous=True)  #When I removed queue size, it worked
    rospy.Subscriber("/cmd_vel", Twist, ackermanCallback)
    
    while not rospy.is_shutdown():
        rospy.spin()
