#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Polygon
from racecar_communication.msg import IDStamped, FootprintsCombined

class RecIDMsgs:
    def __init__(self):
        
        self.robot_num = rospy.get_param('robot_num')
        self.last_received_footprints = FootprintsCombined()

        self.pub = rospy.Publisher('/footprints_combined', FootprintsCombined, queue_size=10) # test publisher for 'FootprintsCombined' message

    # Input data is IDStamped message from topic /id_msgs
    def idMsgsCallback(self, idMsgs):
        if (idMsgs.robot_num != self.robot_num):

                self.last_received_footprints.header.stamp = rospy.Time.now();
                self.last_received_footprints.header.frame_id = "map";

                self.last_received_footprints.robot_num = self.robot_num

                self.last_received_footprints.footprints[idMsgs.robot_num - 1] = idMsgs.footprint

                self.pub.publish(self.last_received_footprints)


if __name__ == '__main__':
    rospy.init_node('rec_id_msg')

    # create object from class RecIDMsgs
    rim = RecIDMsgs()

    # subscribe to '/id_msgs'
    rospy.Subscriber('/id_msgs', IDStamped, rim.idMsgsCallback, queue_size=1)
    rospy.spin()
