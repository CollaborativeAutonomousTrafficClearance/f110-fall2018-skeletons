#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PolygonStamped, Polygon
from racecar_communication.msg import IDStamped

class SendIDMsg:
    def __init__(self):
        
        self.velocity = 0
        self.x_position = 0
        self.y_position = 0
        self.lane_num = 0
        self.footprint = Polygon()

        self.robot_num = rospy.get_param('robot_num')
        self.r_type = rospy.get_param('r_type')
        self.max_vel = rospy.get_param('max_vel')
        self.max_acc = rospy.get_param('max_acc')

        self.pub = rospy.Publisher('/id_msgs', IDStamped, queue_size=50) # publisher for 'IDStamped' message

    # Calculates which lane the vehicle is in, assuming we are in the threeLanes world
    def whichLane(self):

        if (self.y_position >= 0):
            self.lane_num = 0;
        elif (self.y_position < -0.525):
            self.lane_num = 2;   
        else:
            self.lane_num = 1; 


    # Input data is Odometry message from topic /vesc/odom
    def odometryCallback(self, odomMsg):

        self.velocity = odomMsg.twist.twist.linear.x;
        self.x_position = odomMsg.pose.pose.position.x;
        self.y_position = odomMsg.pose.pose.position.y;
   
        self.whichLane();

        idMsg = IDStamped()
        idMsg.header.stamp = rospy.Time.now();
        idMsg.header.frame_id = "map";

        idMsg.robot_num = self.robot_num

        idMsg.id.type.data = self.r_type
        idMsg.id.x_position = self.x_position
        idMsg.id.y_position = self.y_position
        idMsg.id.lane_num = self.lane_num
        idMsg.id.footprint = self.footprint

        idMsg.id.velocity = self.velocity
        idMsg.id.max_vel = self.max_vel
        idMsg.id.max_acc = self.max_acc

        self.pub.publish(idMsg)


    # Input data is PolygonStamped message from topic /move_base/local_costmap/footprint
    def footprintCallback(self, footprintMsg):

        self.footprint = footprintMsg.polygon


if __name__ == '__main__':
    rospy.init_node('send_id_msg')

    # create object from class SendIDMsg
    sim = SendIDMsg()

    # subscribe to '/vesc/odom'
    rospy.Subscriber('/vesc/odom', Odometry, sim.odometryCallback, queue_size=50)
    # subscribe to '/move_base/local_costmap/footprint'
    rospy.Subscriber('/move_base/local_costmap/footprint', PolygonStamped, sim.footprintCallback, queue_size=50)

    rospy.spin()
