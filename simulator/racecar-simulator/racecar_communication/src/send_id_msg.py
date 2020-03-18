#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PolygonStamped
from racecar_communication.msg import IDStamped

class SendIDMsg:
    def __init__(self):
        
        self.velocity = 0
        self.x_position = 0
        self.y_position = 0
        self.robot_num = rospy.get_param('robot_num')

        self.pub = rospy.Publisher('/id_msgs', IDStamped, queue_size=10) # publisher for 'IDStamped' message

    # Input data is Odometry message from topic /vesc/odom
    def odometryCallback(self, odomMsg):

        self.velocity = odomMsg.twist.twist.linear.x;
        self.x_position = odomMsg.pose.pose.position.x;
        self.y_position = odomMsg.pose.pose.position.y;


    # Input data is PolygonStamped message from topic /move_base/local_costmap/footprint
    def footprintCallback(self, footprintMsg):

        idMsg = IDStamped()
        idMsg.header.stamp = rospy.Time.now();
        idMsg.header.frame_id = "map";

        idMsg.robot_num = self.robot_num
        idMsg.footprint = footprintMsg.polygon

        idMsg.velocity = self.velocity
        idMsg.x_position = self.x_position
        idMsg.y_position = self.y_position

        self.pub.publish(idMsg)


if __name__ == '__main__':
    rospy.init_node('send_id_msg')

    # create object from class SendIDMsg
    sim = SendIDMsg()

    # subscribe to '/vesc/odom'
    rospy.Subscriber('/vesc/odom', Odometry, sim.odometryCallback, queue_size=1)
    # subscribe to '/move_base/local_costmap/footprint'
    rospy.Subscriber('/move_base/local_costmap/footprint', PolygonStamped, sim.footprintCallback)

    rospy.spin()
