#!/usr/bin/env python
import rospy
from racecar_rl_environments.srv import states, statesRequest, statesResponse, reward, rewardRequest, rewardResponse, startSim, startSimRequest, startSimResponse, resetORcloseSim, resetORcloseSimRequest, resetORcloseSimResponse

class BasicEnvComm:

    def __init__(self):

        rospy.loginfo("Initializing Basic Environment Communicator.")

        self.states_client = rospy.ServiceProxy('/states', states)
        self.reward_client = rospy.ServiceProxy('/reward', reward)
        self.startSim_client = rospy.ServiceProxy('/startSim', startSim)
        self.resetSim_client = rospy.ServiceProxy('/resetSim', resetORcloseSim)
        self.closeSim_client = rospy.ServiceProxy('/closeSim', resetORcloseSim)

        rospy.loginfo("Finished initializing Basic Environment Communicator.")

    def getWindowParams(self):

        rospy.loginfo("Getting window parameters in Basic Environment Communicator.")

        ## Communication range ##
        if rospy.has_param('/communicaion_range'):
            comm_range = rospy.get_param('/communicaion_range')
            rospy.loginfo("Reading communicaion_range parameter: %f.", comm_range)
        else:
            comm_range = 24
            rospy.loginfo("Setting communicaion_range parameter to default: %f.", comm_range)

        ## Ambulance parameters ##
        if rospy.has_param('/ambulance/amb_max_vel'):
            amb_vel_max = rospy.get_param('/ambulance/amb_max_vel')
            rospy.loginfo("Reading amb_max_vel parameter: %f.", amb_vel_max)
        else:
            amb_vel_max = 1 # ambulance max vel
            rospy.loginfo("Setting amb_max_vel parameter to default: %f.", amb_vel_max)
        ####
        if rospy.has_param('/ambulance/amb_min_vel'):
            amb_vel_min = rospy.get_param('/agent/amb_min_vel')
            rospy.loginfo("Reading amb_min_vel parameter: %f.", amb_vel_min)
        else:
            amb_vel_min = 0 # ambulance min vel
            rospy.loginfo("Setting amb_min_vel parameter to default: %f.", amb_vel_min)
        ####
        if rospy.has_param('/ambulance/amb_max_acc'):
            emer_max_accel = rospy.get_param('/ambulance/amb_max_acc')
            rospy.loginfo("Reading amb_max_acc parameter: %f.", emer_max_accel)
        else:
            emer_max_accel = 0.0333 # ambulance max acc
            rospy.loginfo("Setting amb_max_acc parameter to default: %f.", emer_max_accel)
        ####
        if rospy.has_param('/ambulance/rel_amb_y_min'):
            if (rospy.get_param('/ambulance/rel_amb_y_min') < 0):
                rel_amb_y_min = - min(comm_range, abs(rospy.get_param('/ambulance/rel_amb_y_min')))
            else:
                rel_amb_y_min = min(comm_range, abs(rospy.get_param('/ambulance/rel_amb_y_min')))
            rospy.loginfo("Reading rel_amb_y_min parameter and setting to: %f.", rel_amb_y_min)
        else:
            rel_amb_y_min = - min(comm_range, 24) # rear limit of window around agent 
            rospy.loginfo("Setting rel_amb_y_min parameter to: %f.", rel_amb_y_min)
        ####
        if rospy.has_param('/ambulance/rel_amb_y_max'):
            if (rospy.get_param('/ambulance/rel_amb_y_max') < 0):
                rel_amb_y_max = - min(comm_range, abs(rospy.get_param('/ambulance/rel_amb_y_max')))
            else:
                rel_amb_y_max = min(comm_range, abs(rospy.get_param('/ambulance/rel_amb_y_max')))
            rospy.loginfo("Reading rel_amb_y_max parameter and setting to: %f.", rel_amb_y_max)
        else:
            rel_amb_y_max = min(comm_range, 9) # front limit of window around agent 
            rospy.loginfo("Setting rel_amb_y_max parameter to: %f.", rel_amb_y_max)

        ## Agent parameters ##
        if rospy.has_param('/agent/agent_max_vel'):
            agent_vel_max = rospy.get_param('/agent/agent_max_vel')
            rospy.loginfo("Reading agent_max_vel parameter: %f.", agent_vel_max)
        else:
            agent_vel_max = 0.5 # agent max vel
            rospy.loginfo("Setting agent_max_vel parameter to default: %f.", agent_vel_max)
        ####
        if rospy.has_param('/agent/agent_min_vel'):
            agent_vel_min = rospy.get_param('/agent/agent_min_vel')
            rospy.loginfo("Reading agent_min_vel parameter: %f.", agent_vel_min)
        else:
            agent_vel_min = 0 # agent min vel
            rospy.loginfo("Setting agent_min_vel parameter to default: %f.", agent_vel_min)
        ####
        if rospy.has_param('/agent/agent_max_acc'):
            agent_max_acc = rospy.get_param('/agent/agent_max_acc')
            rospy.loginfo("Reading agent_max_acc parameter: %f.", agent_max_acc)
        else:
            agent_max_acc = 0.0167 # agent max acc
            rospy.loginfo("Setting agent_max_acc parameter to default: %f.", agent_max_acc)

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
            return resp.reward
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

    def closeSim(self):
        rospy.wait_for_service('/closeSim')
        try:
            resp = self.closeSim_client(True)
            return resp
        except rospy.ServiceException as e:
            print("Reset Simulation service call failed: %s"%e)
            rospy.wait_for_service('/closeSim')
