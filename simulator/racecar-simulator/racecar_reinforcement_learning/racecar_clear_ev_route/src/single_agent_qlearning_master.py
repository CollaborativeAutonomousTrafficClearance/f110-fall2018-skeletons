#!/usr/bin/env python
import rospy
import threading
import numpy as np
import os ####
import sys ####
import optparse ####
#from Config import *  # TODO TODO: Make sure this is the first one imported, to have random.seed() used before any actual randomness is assigned
#from std_msgs.msg import Bool ####
from rl_algorithms.single_agent_qlearning import *
from env_communicators.basic_env_comm import *

# Lock to engage and disengage following the RL model in the test mode
test_activity_lock = threading.Lock()
test_activity_lock.acquire()

# Lock to engage and disengage following the RL model in the train mode
train_activity_lock = threading.Lock()
train_activity_lock.acquire()

class SAQLMaster:
    def __init__(self):
        
        self.is_activated = False
        self.is_episode_done = 0
        self.episode_num = 0

        # History Variables:
        self.total_reward_per_episode = []  # list of doubles
        self.reward_history_per_episode = []  # list of lists

        self.getParams()
        
    def getParams(self):

        self.robot_num = rospy.get_param('robot_num')
        ####
        if rospy.has_param('test_mode_on'):
            self.test_mode_on = rospy.get_param('test_mode_on')
        else:
            self.test_mode_on = True
        ####
        self.max_time_steps_per_episode = rospy.get_param('max_time_steps_per_episode')
        self.max_num_episodes = rospy.get_param('max_num_episodes')
        self.every_n_episodes = rospy.get_param('every_n_episodes')
        self.every_n_steps = rospy.get_param('every_n_steps')
        self.print_reward_every_episode = rospy.get_param('print_reward_every_episode')
        ####
        if rospy.has_param('environment'):
            if (rospy.get_param('environment') != "basic_environment"):
                rospy.loginfo("Unknown environment is requested. Will use the basic_environment instead.")
            self.ENV_COMM = env(vehicles_list) #TODO TODO: env class - give it max time max_time_steps_per_episode
        else:
            self.ENV_COMM = env(vehicles_list) #TODO TODO: env class - give it max time max_time_steps_per_episode
            rospy.loginfo("Using the default environment: basic_environment.")
        ####
        if rospy.has_param('rl_algorithm'):
            if (rospy.get_param('rl_algorithm') != "single_agent_qlearning"):
                rospy.loginfo("Unknown RL algorithm is requested. Will use the single_agent_qlearning algorithm instead.")
            self.RL_ALGO = RLAlgorithm(self.ENV_COMM, algo_params = rospy.get_param('q_learning_params'), load_q_table = rospy.get_param('load_q_table'), self.test_mode_on) #TODO TODO: RLAlgorithm class, perhaps pass only the needed vars instead of ENV_COMM
        else:
            self.RL_ALGO = RLAlgorithm(self.ENV_COMM, algo_params = rospy.get_param('q_learning_params'), load_q_table = rospy.get_param('load_q_table'), self.test_mode_on) #TODO TODO: RLAlgorithm class, perhaps pass only the needed vars instead of ENV_COMM
            rospy.loginfo("Using the default RL algorithm: single_agent_qlearning.")
        ####

    # Input data is ####### message from topic TODO TODO: ####################
    def isActivatedOrDoneCallback(self, inputMsg):
        if (self.test_mode_on == True):
            if ((self.is_activated == False) and (inputMsg.is_activated == True)):
                test_activity_lock.release()
            self.is_activated = inputMsg.is_activated
        else:
            if (self.is_activated == False):
                if ((inputMsg.is_activated == True) || (inputMsg.is_episode_done != 0)):
                    train_activity_lock.release()
            self.is_activated = inputMsg.is_activated
            self.is_episode_done = inputMsg.is_episode_done


    def execute(self):

        if (self.test_mode_on == True):
            self.test_model();

        else:

            #TODO TODO: #######initialize new episode, reset env
            self.is_episode_done = 0

            episode_reward, episode_reward_list = self.episode()
            self.total_reward_per_episode.append(episode_reward)
            self.reward_history_per_episode.append(episode_reward_list)


            while(self.episode_num < self.max_num_episodes):

                #TODO TODO: #######initialize new episode, reset env
                self.episode_num += 1
                self.is_episode_done = 0

                episode_reward, episode_reward_list = self.episode()
                self.total_reward_per_episode.append(episode_reward)
                self.reward_history_per_episode.append(episode_reward_list)

        
            # Save Q-table after episodes ended:
            self.RL_ALGO.save_q_table()


    def test_model(self):

        step = 0

        # MAIN LOOP

        while (1):

            ########################
            if (self.is_activated == False):
                rospy.loginfo("Disengaging the RL model for agent %d because the EV is outside the agent's window.", self.robot_num)
                self.RL_ALGO.disengage()
                test_activity_lock.acquire()
            ########################                

            step += 1

            # 1: Store state before taking action
            agent_state_before = self.ENV_COMM.getState(self.robot_num)

            # ----------------------------------------------------------------- #
            #              2:    E X E C U T E      A C T I O N                 #
            # ----------------------------------------------------------------- #

            executed_action = self.RL_ALGO.take_action()

            # 3: Store state after taking action
            agent_state_after = self.ENV_COMM.getState(self.robot_num)

            # 4: Print step info
            if (step % self.every_n_steps == 0):
                rospy.loginfo("LastState: ")
                rospy.loginfo(agent_state_before) #TODO TODO: ####################

                rospy.loginfo("LastAction: %s", executed_action)

                rospy.loginfo("NewState: ")
                rospy.loginfo(agent_state_after) #TODO TODO: ####################

        return


    def episode(self):

        ########################
        # 1: inits
        #done = False  # are we done with the episode or not
        step = 0  # step number

        episode_reward = 0
        episode_reward_list = []

        ########################
        # measure initial state
        agent_state = self.ENV_COMM.getState(self.robot_num)

        ########################
        # 2: MAIN LOOP

        if (self.episode_num % self.every_n_episodes == 0):
            rospy.loginfo("Episode: %d. Epsilon: %f. State:", self.episode_num, self.RL_ALGO.epsilon)
            rospy.loginfo(agent_state) #TODO TODO: ####################

        while (1):

            ########################
            if (self.is_activated == False):
                rospy.loginfo("Disengaging the RL model for agent %d in episode %d because the EV is outside the agent's window.", self.robot_num, self.episode_num)
                self.RL_ALGO.disengage()
                train_activity_lock.acquire()

            if (self.is_episode_done): # DO NOT REMOVE THIS (IT BREAKS IF WE ARE DONE)
                if(self.episode_num % self.every_n_episodes == 0):
                    if (self.is_episode_done == 1):
                        episode_end_reason = "max time steps"
                    elif (self.is_episode_done == 2):
                        episode_end_reason = "ambulance goal"
                    elif (self.is_episode_done == 3):  # TODO: #TOFIX: What should be the state here?
                        episode_end_reason = "agent goal"
                    else:
                        episode_end_reason = "unknown"
                        rospy.loginfo("Episode: %d done is True = %d but reason not known!", self.episode_num, self.is_episode_done)

                    rospy.loginfo("Episode: %d. Step: %d. LastActionMethod: %s. LastAction: %s. Reward: %f. CumReward: %f. NewState:", self.episode_num, step, self.RL_ALGO.action_chosing_method, executed_action, reward, episode_reward)
                    rospy.loginfo(agent_state_after) #TODO TODO: ####################
                break
            ########################                

            # 3.1: Store state before taking action
            agent_state_before = self.ENV_COMM.getState(self.robot_num)

            # ----------------------------------------------------------------- #
            # 3.2:   M O V E      O N E      S I M U L A T I O N       S T E P  #
            # ----------------------------------------------------------------- #
            #                   E X E C U T E      A C T I O N                  #
            # ----------------------------------------------------------------- #

            executed_action = self.RL_ALGO.take_action()
            step += 1

            # 3.3: measurements and if we are done check
            agent_state_after = self.ENV_COMM.getState(self.robot_num)

            self.is_episode_done = self.ENV_COMM.are_we_done(full_state = Proudhon.full_state, step_number=step) #TODO TODO: ####################

            # 3.4: reward last step's chosen action
            reward = self.ENV_COMM.calc_reward(amb_last_velocity, self.is_episode_done, step) #TODO TODO: ####################
            episode_reward += reward  # for history
            episode_reward_list.append(reward)  # for history

            # 3.5: update q table using backward reward logic
            self.RL_ALGO.update_q_table(executed_action, reward, agent_state_after,
                                      agent_state_before)

            if (step % self.every_n_steps == 0 and self.episode_num % self.every_n_episodes == 0): # print step info
                rospy.loginfo("Episode: %d. Step: %d. LastActionMethod: %s. LastAction: %s. Reward: %f. CumReward: %f. NewState:", self.episode_num, step, self.RL_ALGO.action_chosing_method, executed_action, reward, episode_reward)
                rospy.loginfo(agent_state_after) #TODO: ####################

            if (self.is_episode_done): # DO NOT REMOVE THIS (IT BREAKS IF WE ARE DONE)
                if(self.episode_num % self.every_n_episodes == 0):
                    if (self.is_episode_done == 1):
                        episode_end_reason = "max time steps"
                    elif (self.is_episode_done == 2):
                        episode_end_reason = "ambulance goal"
                    elif (self.is_episode_done == 3):  # TODO: #TOFIX: What should be the state here?
                        episode_end_reason = "agent goal"
                    else:
                        episode_end_reason = "unknown"
                        rospy.loginfo("Episode: %d done is True = %d but reason not known!", self.episode_num, self.is_episode_done)

                    rospy.loginfo("Episode: %d. Step: %d. LastActionMethod: %s. LastAction: %s. Reward: %f. CumReward: %f. NewState:", self.episode_num, step, self.RL_ALGO.action_chosing_method, executed_action, reward, episode_reward)
                    rospy.loginfo(agent_state_after) #TODO: ####################
                break

        # Episode End
        # 4: Update Epsilon after episode is done
        old_epsilon = self.RL_ALGO.epsilon
        self.RL_ALGO.epsilon = self.RL_ALGO.min_epsilon + (self.RL_ALGO.max_epsilon - self.RL_ALGO.min_epsilon) * \
                             np.exp(-self.RL_ALGO.decay_rate * self.episode_num)

        if (self.episode_num % self.every_n_episodes == 0):
            rospy.loginfo("Episode: %d. FinalCumReward: %f. OldEpsilon: %f. NewEpsilon: %f.", self.episode_num, episode_reward, old_epsilon, self.RL_ALGO.epsilon) #TODO: print episode end reason 


        if(self.print_reward_every_episode and self.episode_num % self.every_n_episodes):
            rospy.loginfo("Episode: %d. FinalCumReward: %f.", self.episode_num, episode_reward)

        return episode_reward, episode_reward_list


if __name__ == "__main__":
    rospy.init_node('single_agent_qlearning_master')

    # create object from class SAQLMaster
    saqlm = SAQLMaster()

    saqlm.execute()

    # subscribe to 'TODO: ####################'
    rospy.Subscriber('/#############', SMTH, saqlm.isActivatedOrDoneCallback, queue_size=2)

    rospy.spin()
