#!/usr/bin/env python
import rospy
from racecar_rl_environments.srv import states, statesRequest, statesResponse
from racecar_communication.msg import IDStamped, FootprintsCombined, ID, IDsCombined
#from std_msgs.msg import float32



def statesClient():
    rospy.wait_for_service('states')
    try:
        client = rospy.ServiceProxy('states', states)
        resp1 = client(2) #TODO id = 2 l7ad ma nshof eh da wla hangebo mnen 
        rospy.loginfo("Client, sending id  = 3")
        return resp1
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
    



if __name__ == '__main__':
    rospy.init_node("states_client", anonymous=True)
    while not rospy.is_shutdown():
        y = statesClient()
        rospy.loginfo("client recieved")
        rospy.loginfo(y)
        """
        rospy.loginfo("Agent vel: %d",y.agent_vel)
        
        """
        rospy.sleep(3.)
    