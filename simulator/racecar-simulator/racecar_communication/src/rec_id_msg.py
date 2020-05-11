#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Polygon
from racecar_communication.msg import IDStamped, FootprintsCombined, ID, IDsCombined

class RecIDMsgs:
    def __init__(self):
        
        self.robot_num = rospy.get_param('robot_num')
        self.last_received_footprints = FootprintsCombined()
        self.last_received_ids = self.initiIdsCombined()

        self.pub1 = rospy.Publisher('/ids_combined', IDsCombined, queue_size=50) # publisher for 'IDsCombined' message

        self.pub2 = rospy.Publisher('/footprints_combined', FootprintsCombined, queue_size=50) # publisher for 'FootprintsCombined' message

    
    # Initialize IDsCombined_msg 
    def initiIdsCombined(self):
        idsCombined_msg = IDsCombined()

	# Set lane number to -1 as a flag
        for i in range(0,len(idsCombined_msg.ids),1):
            idsCombined_msg.ids[i].lane_num = -1
        return idsCombined_msg

    
    def idMsgsCallback(self, idMsgs):
        if (idMsgs.robot_num != self.robot_num):

                # Publishing IDsCombined message
                self.last_received_ids.header.stamp = rospy.Time.now();
                self.last_received_ids.header.frame_id = "map";
                self.last_received_ids.robot_num = self.robot_num

                self.last_received_ids.ids[idMsgs.robot_num - 1] = idMsgs.id

                self.pub1.publish(self.last_received_ids)


                # Publishing FootprintsCombined message
                self.last_received_footprints.header.stamp = rospy.Time.now();
                self.last_received_footprints.header.frame_id = "map";
                self.last_received_footprints.robot_num = self.robot_num

                self.last_received_footprints.footprints[idMsgs.robot_num - 1] = idMsgs.id.footprint

                if (idMsgs.id.footprint.points != []):
	                self.pub2.publish(self.last_received_footprints)


if __name__ == '__main__':
    rospy.init_node('rec_id_msg')

    # create object from class RecIDMsgs
    rim = RecIDMsgs()

    # subscribe to '/id_msgs'
    rospy.Subscriber('/id_msgs', IDStamped, rim.idMsgsCallback, queue_size=50)
    rospy.spin()
