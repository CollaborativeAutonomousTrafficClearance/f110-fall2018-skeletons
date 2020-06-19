#!/usr/bin/env python
import rospy
import random
from std_msgs.msg import Header
from racecar_clear_ev_route.srv import RLPolicyActionService, RLPolicyActionServiceRequest, RLPolicyActionServiceResponse

#This node randomly publish actions, simulation for RL
random.seed(0)

#3 #6 #9
class genActionsClient:
    def __init__(self):
        #waiting for the service to connect
        #rospy.wait_for_service('move_car/RL/RLPolicyActionService')
        
        #defining the service response 
        self.resp_feasibility = False
        self.resp_time = rospy.Time()
        self.resp = -1

        self.acc = 0
        self.control_action = 0
        self.doneRLflag = False
        self.header = Header()


        self.send = 0

    def RandomActions(self, exclude, count):
        #gettinf random action, to be replaced by RL policy chosen action
        random_arr = [1,4,5]
        random_action = random.choice(random_arr)
        
        if random_action == 1:
            self.acc = 0
            self.control_action = 0 #keep constant velocity
            if self.control_action == exclude and count>0:
                self.RandomActions(exclude, count-1)
        elif random_action == 2:
            self.acc = 0.023          #accelerate by 0.1
            self.control_action = 0 
            if self.control_action == exclude and count>0:
                self.RandomActions(exclude, count-1)
        elif random_action == 3:
            self.acc = -0.023 #deccelerate by 1
            self.control_action = 0 #keep constant velocity
            if self.control_action == exclude and count>0:
                self.RandomActions(exclude, count-1)
        elif random_action == 4:
            self.acc = 0
            self.control_action = 1
            if self.control_action == exclude and count>0:
                self.RandomActions(exclude, count-1)
        elif random_action == 5:
            self.acc = 0
            self.control_action = 2
            if self.control_action == exclude and count>0:
                self.RandomActions(exclude, count-1)
        
    def testrandom(self):
        max = 6
        exclude = -1
        itr = 0
        while not rospy.is_shutdown():
            itr = itr + 1
            rospy.loginfo("\n\n\nItr is %i", itr)
            if itr > max:
                break
            self.RandomActions(exclude, 2)
            rospy.loginfo("control action is %d, and acc is %f", self.control_action, self.acc)


    def sendActionClient(self):
        #rospy.sleep(2)
        exclude = -1
        max = 6
        itr = 0
        while not rospy.is_shutdown():
            itr = itr + 1
            rospy.loginfo("\n\n\nItr is %i", itr)
            #getting random action
            if itr > max:
                itr = 0
                break
            elif itr == max:
                self.doneRLflag = True
            else:
                self.doneRLflag = False

            self.RandomActions(exclude, 2)
            self.header.stamp = rospy.Time.now()
            

            rospy.loginfo("control actions is %i", self.control_action)
            rospy.loginfo("Acceleration is %f", self.acc)
            try:
                rospy.wait_for_service('move_car/RL/RLPolicyActionService') #TODO
                sendAction = rospy.ServiceProxy('move_car/RL/RLPolicyActionService',RLPolicyActionService)
                resp = sendAction(self.header, self.control_action, self.acc, self.doneRLflag)
                self.resp_feasibility = resp.RLActionresult
                self.resp_time = resp.RLActionTime
                rospy.loginfo("resp is %i", self.resp_feasibility)
                rospy.loginfo("time is %f", self.resp_time)
    
            except rospy.ServiceException, e:
                rospy.wait_for_service('move_car/RL/RLPolicyActionService')
                print "Service call failed: %s"%e
        
            if self.resp_feasibility == False:
                exclude = self.control_action
            else:
                exclude = 0
            


        


if __name__ == '__main__':

    rospy.init_node("actionGenerator", anonymous=True)
    c = genActionsClient()
    c.sendActionClient()
    #rospy.loginfo("hi")
    #c.testrandom()




    