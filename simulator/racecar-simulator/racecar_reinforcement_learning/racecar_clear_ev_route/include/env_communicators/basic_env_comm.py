#!/usr/bin/env python
import rospy
from racecar_rl_environments.srv import states, statesRequest, statesResponse, reward, rewardRequest, rewardResponse, startSim, startSimRequest, startSimResponse, resetSim, resetSimRequest, resetSimResponse

class BasicEnvComm:

    def __init__(self):

        rospy.loginfo("Initializing Basic Environment Communicator.")

        self.states_client = rospy.ServiceProxy('/states', states)
        self.reward_client = rospy.ServiceProxy('/reward', reward)
        self.startSim_client = rospy.ServiceProxy('/startSim', startSim)
        self.resetSim_client = rospy.ServiceProxy('/resetSim', resetSim)

        rospy.loginfo("Finished initializing Basic Environment Communicator.")

    def getWindowParams(self):

        rospy.loginfo("Getting window parameters Basic Environment Communicator.")

        ## Communication range ##
        if rospy.has_param('/communicaion_range'):
            comm_range = rospy.get_param('/communicaion_range')
        else:
            comm_range = 24

        ## Ambulance parameters ##
        if rospy.has_param('/ambulance/amb_max_vel'):
            amb_vel_max = rospy.get_param('/ambulance/amb_max_vel')
        else:
            amb_vel_max = 1 # ambulance max vel
        ####
        if rospy.has_param('/agent/amb_min_vel'):
            amb_vel_min = rospy.get_param('/agent/amb_min_vel')
        else:
            amb_vel_min = 0 # ambulance min vel
        ####
        if rospy.has_param('/ambulance/amb_max_acc'):
            emer_max_accel = rospy.get_param('/ambulance/amb_max_acc')
        else:
            emer_max_accel = 0.0333 # ambulance max acc
        ####
        if rospy.has_param('/ambulance/rel_amb_y_min'):
            if (rospy.get_param('/ambulance/rel_amb_y_min') < 0):
                rel_amb_y_min = - min(comm_range, abs(rospy.get_param('/ambulance/rel_amb_y_min')))
            else:
                rel_amb_y_min = min(comm_range, abs(rospy.get_param('/ambulance/rel_amb_y_min')))
        else:
            rel_amb_y_min = - min(comm_range, 24) # rear limit of window around agent 
        ####
        if rospy.has_param('/ambulance/rel_amb_y_max'):
            if (rospy.get_param('/ambulance/rel_amb_y_max') < 0):
                rel_amb_y_max = - min(comm_range, abs(rospy.get_param('/ambulance/rel_amb_y_max')))
            else:
                rel_amb_y_max = min(comm_range, abs(rospy.get_param('/ambulance/rel_amb_y_max')))
        else:
            rel_amb_y_max = min(comm_range, 9) # front limit of window around agent 

        ## Agent parameters ##
        if rospy.has_param('/agent/agent_max_vel'):
            agent_vel_max = rospy.get_param('/agent/agent_max_vel')
        else:
            agent_vel_max = 0.5 # agent max vel
        ####
        if rospy.has_param('/agent/agent_min_vel'):
            agent_vel_min = rospy.get_param('/agent/agent_min_vel')
        else:
            agent_vel_min = 0 # agent min vel
        ####
        if rospy.has_param('/agent/agent_max_acc'):
            agent_max_acc = rospy.get_param('/agent/agent_max_acc')
        else:
            agent_max_acc = 0.0167 # agent max acc

        window_params = dict()
        window_params['rel_amb_y_min'] = rel_amb_y_min
        window_params['rel_amb_y_max'] = rel_amb_y_max
        window_params['amb_vel_min'] = amb_vel_min
        window_params['amb_vel_max'] = amb_vel_max
        window_params['agent_vel_min'] = agent_vel_min
        window_params['agent_vel_max'] = agent_vel_max
        window_params['amb_acc'] = emer_max_accel
        window_params['agent_acc'] = agent_max_acc

        return window_params

    def getState(self, robot_num):
        rospy.wait_for_service('/states')
        try:
            resp = self.states_client(robot_num)
            return resp
        except rospy.ServiceException as e:
            print("States service call failed: %s"%e)
            rospy.wait_for_service('/states')

    def calcReward(self, amb_last_velocity, execution_time):
        rospy.wait_for_service('/reward')
        try:
            resp = self.reward_client(amb_last_velocity, execution_time)
            return resp
        except rospy.ServiceException as e:
            print("Reward service call failed: %s"%e)
            rospy.wait_for_service('/reward')

    def startSim(self, num_of_agents, num_of_EVs):
        rospy.wait_for_service('/startSim')
        try:
            resp = self.startSim_client(num_of_agents, num_of_EVs)
            return resp
        except rospy.ServiceException as e:
            print("Start Simulation service call failed: %s"%e)
            rospy.wait_for_service('/startSim')

    def resetSim(self):
        rospy.wait_for_service('/resetSim')
        try:
            resp = self.resetSim_client(True)
            return resp
        except rospy.ServiceException as e:
            print("Reset Simulation service call failed: %s"%e)
            rospy.wait_for_service('/resetSim')
