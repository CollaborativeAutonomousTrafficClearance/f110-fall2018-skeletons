#!/usr/bin/env python
import rospy
import random
from racecar_qlearning.msg import RLPolicyRequest
from racecar_qlearning.srv import RLPolicyActionService, RLPolicyActionServiceRequest, RLPolicyActionServiceResponse
from std_msgs.msg import Header

#This node randomly publish actions, simulation for RL

class genActionsClient:
    def __init__(self):

        #waiting for the service to connect
        #rospy.wait_for_service('move_car/RL/RLPolicyActionService')

        #defining the chosen action by the RL, its parameters are acceleration, control action and a header
        #self.chosen_action = RLPolicyRequest()
        
        #defining the service request and response 
        self.RLaction_request = RLPolicyActionServiceRequest()
        self.RLaction_response = RLPolicyActionServiceResponse() 

        #initializing the service proxy, a function for calling the server
        #self.sendAction = rospy.ServiceProxy('move_car/RL/RLPolicyActionService',RLPolicyActionService)

        self.acc = 0
        self.control_action = 0
        self.doneRLflag = False
        self.header = Header()

    def RandomActions(self):
        #gettinf random action, to be replaced by RL policy chosen action
        random_action = random.randint(1,3)
        if random_action == 1:
            self.acc = 0
            self.control_action = 0 #keep constant velocity
        elif random_action == 2:
            self.acc = 0.1          #accelerate by 0.1
            self.control_action = 0 
        elif random_action == 3:
            self.acc = -0.1 #deccelerate by 1
            self.control_action = 0 #keep constant velocity
        
            #self.RLaction_request.RLChosenAction.header.stamp = rospy.Time.now()
            self.doneRLflag = False
        


    def sendActionClient(self):
        #getting random action
        self.RandomActions()
        self.header.stamp = rospy.Time.now()
        
        #self.RLaction_request.RLChosenAction = self.chosen_action
        self.RLaction_request = RLPolicyActionServiceRequest(self.header,self.control_action,self.acc,self.doneRLflag)
        rospy.loginfo("sending")
        #sending the action and getting the response 
        #rospy.wait_for_service('move_car/RL/RLPolicyActionService')
        try:
            sendAction = rospy.ServiceProxy('move_car/RL/RLPolicyActionService',RLPolicyActionService, persistent=True)
            self.RLaction_response = sendAction(self.RLaction_request )
        except rospy.ServiceException, e:
            #rospy.wait_for_service('move_car/RL/RLPolicyActionService')
            print "Service call failed: %s"%e


if __name__ == '__main__':
    
    rospy.init_node("actionGenerator", anonymous=True)
    Action = genActionsClient()

    while not rospy.is_shutdown():
        Action.sendActionClient()
        #rospy.sleep(4.0)

    