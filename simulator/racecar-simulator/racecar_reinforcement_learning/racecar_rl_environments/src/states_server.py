#!/usr/bin/env python
import rospy
from racecar_rl_environments.srv import states, statesRequest, statesResponse
from racecar_communication.msg import IDStamped, FootprintsCombined, ID, IDsCombined
from nav_msgs.msg import Odometry




class clear_ev_route_basic_env:
    
    def __init__(self):

        self.robot_num = 0
        self.last_received_ids = self.initIdsCombined()

        self.savedIDs = [10,20,30,40,50,60]  # dummy initilization 

        self.agent_vel = 0
        self.agent_lane = -1
        self.amb_vel = 0
        self.amb_lane = -1
        self.rel_amb_y = 0

        self.agent_x_pos = 0
        self.agent_y_pos = 0

    # Initialize IDsCombined message 
    def initIdsCombined(self):
        idsCombined_msg = IDsCombined()

	    # Set the initialization value of lane number to -1 (flag)
        for i in range(0,len(idsCombined_msg.ids),1):
            idsCombined_msg.ids[i].lane_num = -1

        return idsCombined_msg


    def serverCallback(self,req):

        self.agent_vel = self.savedIDs[req.robot_num].id.velocity
        self.agent_lane = self.savedIDs[req.robot_num].id.lane_num

        self.agent_x_pos = self.savedIDs[req.robot_num].id.x_position
        self.agent_y_pos = self.savedIDs[req.robot_num].id.y_position


        resp = statesResponse()
        resp.agent_vel = self.agent_vel
        resp.agent_lane = self.agent_lane
        resp.amb_vel = self.amb_vel
        resp.amb_lane = self.agent_lane
        resp.rel_amb_y = self.rel_amb_y
        rospy.loginfo(self.savedIDs)
        
        return resp

    def statesServer(self):
        server = rospy.Service('states', states, self.serverCallback)

    
    def idMsgsCallback(self, idMsgs):
        
        self.robot_num = idMsgs.robot_num

        self.last_received_ids.header.stamp = rospy.Time.now()
        self.last_received_ids.header.frame_id = "map"
        self.last_received_ids.robot_num = self.robot_num

        self.savedIDs[idMsgs.robot_num] = idMsgs
        

    def odomSub(self):

        # subscribe to '/id_msgs'
        rospy.Subscriber('/id_msgs', IDStamped, self.idMsgsCallback, queue_size=50)


    


        
if __name__ == '__main__':
    rospy.init_node("states_server")

    cEVrbe = clear_ev_route_basic_env()
    cEVrbe.statesServer()
    #rospy.loginfo("after states server")
    cEVrbe.odomSub()
    #rospy.loginfo("after odomsub")
    rospy.loginfo(cEVrbe.savedIDs)
    #rospy.Subscriber('/id_msgs', IDStamped, cEVrbe.idMsgsCallback, queue_size=2)

    rospy.spin()
