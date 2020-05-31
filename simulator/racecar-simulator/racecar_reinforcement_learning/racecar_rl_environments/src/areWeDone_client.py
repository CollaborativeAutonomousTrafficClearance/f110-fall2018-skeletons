#!/usr/bin/env python
"""
import rospy
from racecar_rl_environments.srv import reward, rewardRequest, rewardResponse



def rewardClient():
    rospy.wait_for_service('areWeDone')
    try:
        client = rospy.ServiceProxy('areWeDone', areWeDone)
        resp1 = client(0.1,0.5)

        rospy.loginfo("Client, sending vels  ---------- ")
        return resp1.reward
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
    



if __name__ == '__main__':
    rospy.init_node("areWeDone_client", anonymous=True)
    while not rospy.is_shutdown():
        y = areWeDoneClient()
        # debug # rospy.loginfo("client recieved")
        # debug # rospy.loginfo(y)
        
        rospy.loginfo("reward from client: %f",y)
        
    
        rospy.sleep(3.)


        
"""