#!/usr/bin/env python
import rospy
from racecar_qlearning.srv import RLPolicyActionService, RLPolicyActionServiceRequest, RLPolicyActionServiceResponse


def handle_RLPolicyAction_service(req):
    return RLPolicyActionServiceResponse(True)


def server():
    s = rospy.Service('move_car/RL/RLPolicyActionService', RLPolicyActionService, handle_RLPolicyAction_service)
        



if __name__ == '__main__':
    rospy.init_node('server')
    server()
    rospy.spin()