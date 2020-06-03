#!/usr/bin/env python
import rospy
from racecar_rl_environments.srv import states, statesRequest, statesResponse, reward, rewardRequest, rewardResponse, startSim, startSimRequest, startSimResponse, resetSim, resetSimRequest, resetSimResponse

class BasicEnvComm:

    def __init__(self):

        rospy.wait_for_service('states')
        rospy.wait_for_service('reward')
        rospy.wait_for_service('startSim')
        rospy.wait_for_service('resetSim')

        self.states_client = rospy.ServiceProxy('states', states)
        self.reward_client = rospy.ServiceProxy('reward', reward)
        self.startSim_client = rospy.ServiceProxy('startSim', startSim)
        self.resetSim_client = rospy.ServiceProxy('resetSim', resetSim)


    def getState(self, robot_num):
        try:
            resp = self.states_client(robot_num)
            return resp
        except rospy.ServiceException as e:
            print("States service call failed: %s"%e)

    def calcReward(self, amb_last_velocity, execution_time):
        try:
            resp = reward_client(amb_last_velocity, execution_time)
            return resp
        except rospy.ServiceException as e:
            print("Reward service call failed: %s"%e)

    def startSim(self, num_of_agents, num_of_EVs):
        try:
            resp = startSim_client(num_of_agents, num_of_EVs)
            return resp
        except rospy.ServiceException as e:
            print("Start Simulation service call failed: %s"%e)

    def resetSim(self):
        try:
            resp = resetSim_client(True)
            return resp
        except rospy.ServiceException as e:
            print("Reset Simulation service call failed: %s"%e)
