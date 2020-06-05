#!/usr/bin/env python
import threading
import pdb

import rospy
from racecar_rl_environments.srv import states, statesRequest, statesResponse
from racecar_communication.msg import IDStamped, FootprintsCombined, ID, IDsCombined
from nav_msgs.msg import Odometry

#reward service imports
from racecar_rl_environments.srv import reward, rewardRequest, rewardResponse
import numpy as np




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

        # reward services vars

        self.track_len = 100 # make sure from nadine
        self.emer_max_speed = 0.5
        self.emer_max_accel = 0.5

        # msh 3arf han3ml 7aga belly fo2 dol wla la2 ? ! .. amb_vel y3ni
        self.emer_spd = 0 #TODO get from state
        self.amb_last_velocity = 0 #TODO get from state 

        # Calculation for optimal time is kept in case the track_len is changed between episodes
        self.optimal_time = int(np.round(
            self.track_len / self.emer_max_speed))  # Optimal Number of time steps: number of time steps taken by ambulance at maximum speed
        self.max_steps = 20 * self.optimal_time

        self.isDone = 0 # bool
        self.number_of_steps = -1
        self.max_final_reward = 20
        self.min_final_reward = -20
        self.max_step_reward = 0
        self.min_step_reward = -1.25
        self.give_final_reward = 0 # bool
    

        self.reward = 0

        # are we done topic 
        

    


    # Initialize IDsCombined message 
    def initIdsCombined(self):
        idsCombined_msg = IDsCombined()

	    # Set the initialization value of lane number to -1 (flag)
        for i in range(0,len(idsCombined_msg.ids),1):
            idsCombined_msg.ids[i].lane_num = -1

        return idsCombined_msg


    def states_serverCallback(self,req):

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
        server = rospy.Service('states', states, self.states_serverCallback)

    
    def idMsgsCallback(self, idMsgs):
        
        self.robot_num = idMsgs.robot_num

        self.last_received_ids.header.stamp = rospy.Time.now()
        self.last_received_ids.header.frame_id = "map"
        self.last_received_ids.robot_num = self.robot_num

        self.savedIDs[idMsgs.robot_num] = idMsgs
        

    def odomSub(self):

        # subscribe to '/id_msgs'
        rospy.Subscriber('/id_msgs', IDStamped, self.idMsgsCallback, queue_size=50)


    def calc_reward(self, amb_last_velocity, isDone, number_of_steps, max_final_reward, min_final_reward, max_step_reward, min_step_reward):
        #Edit by sayed: 
        #pdb.set_trace()
        amb_last_velocity = self.amb_last_velocity
        isDone = self.isDone
        number_of_steps = self.number_of_steps
        max_final_reward = self.max_final_reward
        min_final_reward = self.min_final_reward
        max_step_reward = self.max_step_reward
        min_step_reward = self.min_step_reward
        # end edit by sayed

        # TODO: Fix final reward logic according last discussiion : if agent finishes first, assume the ambulance  will conitnue at its current
        #   velocity till the end.
        '''
        :logic: Calculate reward to agent from current state
        :param amb_last_velocity: float, previous velocity the ambulance (self.emer) had
        :param isDone: bool, whether this is the last step in the simulation or not (whether to calculate final reward or step rewrard)
        :param number_of_steps: number of steps in simulation so far. Used to calculate final reward but not step reward
        :param max_final_reward: reward for achieving end of simulation (done) with number_of_steps = self.optimal time
        :param min_final_reward: reward for achieving end of simulation (done) with number_of_steps = 20 * self.optimal time
        :param max_step_reward: reward for having an acceleration of value = self.emer_max_accel (=2) over last step
        :param min_step_reward: reward for having an acceleration of value = - self.emer_max_accel (=- 2) over last step
        :return: reward (either step reward or final reward)


        :Notes:
        #Simulation Time is not allowed to continue after 20*optimal_time (20* time steps with ambulance at its maximum speed)
        '''

        if(isDone and self.give_final_reward): #Calculate a final reward
            #Linear reward. y= mx +c. y: reward, x: ration between time achieved and optimal time. m: slope. c: y-intercept
            m = ( (max_final_reward - min_final_reward) *20 ) /19 #Slope for straight line equation to calculate final reward
            c = max_final_reward - 1*m #c is y-intercept for the reward function equation #max_final_reward is the y for x = 1
            reward = m * (self.optimal_time/number_of_steps) + c
            #debug#print(f'c: {c}, m: {m}, steps: {number_of_steps}, optimal_time: {self.optimal_time}')
            self.reward = reward
            return reward

        else: #Calcualate a step reward
            steps_needed_to_halt = 30
            ration_of_halt_steps_to_total_steps = steps_needed_to_halt/self.track_len

            #self.emer.getSpd()  # Make sure emergency vehicle's speed is up-to-date

            m = (max_step_reward - min_step_reward)/(2 * self.emer_max_accel)  # Slope for straight line equation to calculate step reward
            # debug # rospy.loginfo("M = : %f", m)
            #2 * self.emer.max_accel since: = self.emer.max_accel - * self.emer.max_decel
            c = max_step_reward - self.emer_max_accel * m  # c is y-intercept for the reward function equation #max_step_reward is the y for x = 2 (max acceleration)
            # debug # rospy.loginfo("C = : %f", c)
            reward = m * (self.emer_spd - amb_last_velocity) + c
            rospy.loginfo("Reward = : %f", reward)
            #debug#print(f'c: {c}, m: {m}, accel: {(self.emer.spd - amb_last_velocity)}')
            #rospy.loginfo("if condition = : %f", abs(amb_last_velocity-self.emer_max_speed))
            
            if ( abs(amb_last_velocity-self.emer_max_speed) <= 1e-10 ):
            #since ambulance had maximum speed and speed did not change that much; unless we applied the code below.. the acceleration
            #   will be wrongly assumed to be zero. Although the ambulance probably could have accelerated more, but this is its maximum velocity.
                reward = max_step_reward #same reward as maximum acceleration (+2),

            self.reward = reward
            rospy.loginfo("Reward after return = : %f", reward)

            return reward

    
    def reward_serverCallback(self,req):
        #pdb.set_trace()
        
        self.amb_last_velocity =  req.amb_vel_1
        self.emer_spd = req.amb_vel_2

        #anaDa = self.addAy7aga(self.amb_last_velocity)

        
        myreward = self.calc_reward(self.amb_last_velocity, self.isDone,self.number_of_steps,self.max_final_reward,self.min_final_reward,self.max_step_reward, self.min_step_reward)
        
        return rewardResponse(myreward)

    def rewardServer(self):
        server = rospy.Service('reward', reward, self.serverCallback)



        
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
