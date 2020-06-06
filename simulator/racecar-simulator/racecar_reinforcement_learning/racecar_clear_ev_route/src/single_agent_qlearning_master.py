#!/usr/bin/env python
import rospy
import threading
import numpy as np
import os
import sys
#from Config import *  # TODO TODO: Make sure this is the first one imported, to have random.seed() used before any actual randomness is assigned
from std_msgs.msg import Bool
from racecar_rl_environments.msg import areWeDone
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
        
        rospy.loginfo("Initializing Single Agent Q-Learning Master.")
        self.is_activated = True
        self.is_episode_done = 0
        self.episode_num = 0

        # History Variables:
        self.total_reward_per_episode = []  # list of doubles
        self.reward_history_per_episode = []  # list of lists

        # subscribe to /RL/is_active_or_episode_done
        rospy.Subscriber('/RL/is_active_or_episode_done', areWeDone, self.isActivatedOrDoneCallback, queue_size = 2)

        self.getParams()
        rospy.loginfo("Finished initializing Single Agent Q-Learning Master.")
        
    def getParams(self):

        rospy.loginfo("Getting parameters in Single Agent Q-Learning Master.")

        self.robot_num = rospy.get_param('robot_num')
        ####
        if rospy.has_param('test_mode_on'):
            self.test_mode_on = rospy.get_param('test_mode_on')
        else:
            self.test_mode_on = False
        ####
        #self.max_time_steps_per_episode = rospy.get_param('max_time_steps_per_episode')
        self.max_num_episodes = rospy.get_param('max_num_episodes')
        self.every_n_episodes = rospy.get_param('vis_update_params/every_n_episodes')
        self.every_n_steps = rospy.get_param('vis_update_params/every_n_steps')
        self.print_reward_every_episode = rospy.get_param('vis_update_params/print_reward_every_episode')
        ####
        if rospy.has_param('environment'):
            if (rospy.get_param('environment') != "basic_environment"):
                rospy.loginfo("Unknown environment is requested. Will use the basic_environment instead.")
            self.ENV_COMM = BasicEnvComm()
        else:
            self.ENV_COMM = BasicEnvComm()
            rospy.loginfo("Using the default environment: basic_environment.")
        ####
        if rospy.has_param('rl_algorithm'):
            if (rospy.get_param('rl_algorithm') != "single_agent_qlearning"):
                rospy.loginfo("Unknown RL algorithm is requested. Will use the single_agent_qlearning algorithm instead.")
            self.RL_ALGO = SingleAgentQlearning(environment_init = self.ENV_COMM.getWindowParams(), algo_params = rospy.get_param('q_learning_params'), load_q_table = rospy.get_param('load_q_table'), test_mode_on = self.test_mode_on)
        else:
            self.RL_ALGO = SingleAgentQlearning(environment_init = self.ENV_COMM.getWindowParams(), algo_params = rospy.get_param('q_learning_params'), load_q_table = rospy.get_param('load_q_table'), test_mode_on = self.test_mode_on)
            rospy.loginfo("Using the default RL algorithm: single_agent_qlearning.")
        ####

    # Input data is areWeDone message from topic /RL/is_active_or_episode_done
    def isActivatedOrDoneCallback(self, inputMsg):
        if (self.test_mode_on == True):
            if ((self.is_activated == False) and (inputMsg.is_activated == True)):
                test_activity_lock.release()
            self.is_activated = inputMsg.is_activated
        else:
            if (self.is_activated == False):
                if ((inputMsg.is_activated == True) or (inputMsg.is_episode_done != 0)):
                    train_activity_lock.release()
            self.is_activated = inputMsg.is_activated
            self.is_episode_done = inputMsg.is_episode_done


    def execute(self):

        rospy.loginfo("Starting simulation.")
        resp = self.ENV_COMM.startSim(1, 1)
        if (resp.is_successful == False):
            raise Exception("Could not start simulation. Terminating.")
            return

        if (self.test_mode_on == True):
            self.test_model();

        else:
            self.is_episode_done = 0

            episode_reward, episode_reward_list = self.episode()
            rospy.loginfo("Episode Number: %d", self.episode_num)
            ##self.total_reward_per_episode.append(episode_reward)
            ##self.reward_history_per_episode.append(episode_reward_list)


            while(self.episode_num < self.max_num_episodes):

                rospy.loginfo("Resetting simulation.")
                ##resp = self.ENV_COMM.resetSim()
                ##if (resp.is_successful == False):
                ##    raise Exception("Could not reset simulation. Terminating.")
                ##    return

                self.episode_num += 1
                self.is_episode_done = 0

                episode_reward, episode_reward_list = self.episode()
                rospy.loginfo("Episode Number: %d", self.episode_num)
                ##self.total_reward_per_episode.append(episode_reward)
                ##self.reward_history_per_episode.append(episode_reward_list)

        
            # Save Q-table after episodes ended:
            rospy.loginfo("Saving Q-table")
            ##self.RL_ALGO.save_q_table() #TODO TODO: see when to call so that we have a constantly updated qtable saved


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

            executed_action, execution_time = self.RL_ALGO.take_action(agent_state_before)

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
        rospy.loginfo("Getting agent's state before taking action.")
        ##agent_state = self.ENV_COMM.getState(self.robot_num)

        ########################
        # 2: MAIN LOOP

        if (self.episode_num % self.every_n_episodes == 0):
            rospy.loginfo("Episode: %d. Epsilon: %f. State:", self.episode_num, self.RL_ALGO.epsilon)
            ##rospy.loginfo(agent_state) #TODO TODO: ####################

        while (1):

            rospy.loginfo("Step: %d.", step)

            ########################
            if (self.is_activated == False):
                rospy.loginfo("Disengaging the RL model for agent %d in episode %d because the EV is outside the agent's window.", self.robot_num, self.episode_num)
                ##self.RL_ALGO.disengage()
                train_activity_lock.acquire()

            if (self.is_episode_done): # DO NOT REMOVE THIS (IT BREAKS IF WE ARE DONE)
                if(self.episode_num % self.every_n_episodes == 0):
                    if (self.is_episode_done == 1):
                        episode_end_reason = "max time steps"
                    elif (self.is_episode_done == 2):
                        episode_end_reason = "ambulance goal"
                    elif (self.is_episode_done == 3):
                        episode_end_reason = "agent goal"
                    else:
                        episode_end_reason = "unknown"

                    rospy.loginfo("Episode: %d ended for the reason: %s", self.episode_num, episode_end_reason)
                    rospy.loginfo("Episode: %d. Step: %d. LastActionMethod: %s. LastAction: %s. Reward: %f. CumReward: %f. NewState:", self.episode_num, step, self.RL_ALGO.action_chosing_method, executed_action, reward, episode_reward)
                    ##rospy.loginfo(agent_state_after) #TODO TODO: ####################
                break
            ########################                

            # 3.1: Store state before taking action
            rospy.loginfo("Getting agent's state before taking action.")
            ##agent_state_before = self.ENV_COMM.getState(self.robot_num)

            # ----------------------------------------------------------------- #
            # 3.2:   M O V E      O N E      S I M U L A T I O N       S T E P  #
            # ----------------------------------------------------------------- #
            #                   E X E C U T E      A C T I O N                  #
            # ----------------------------------------------------------------- #

            rospy.loginfo("Taking action.")
            ##executed_action, execution_time = self.RL_ALGO.take_action(agent_state_before)
            step += 1

            # 3.3: measurements and if we are done check
            rospy.loginfo("Getting agent's state after taking action.")
            ##agent_state_after = self.ENV_COMM.getState(self.robot_num)

            # 3.4: reward last step's chosen action
            rospy.loginfo("Calculating reward.")
            ##reward = self.ENV_COMM.calc_reward(agent_state_before.amb_vel, execution_time)
            ##episode_reward += reward  # for history
            ##episode_reward_list.append(reward)  # for history

            # 3.5: update q table using backward reward logic
            rospy.loginfo("Updating qtable.")
            ##self.RL_ALGO.update_q_table(executed_action, reward, agent_state_after)

            ##if (step % self.every_n_steps == 0 and self.episode_num % self.every_n_episodes == 0): # print step info
                ##rospy.loginfo("Episode: %d. Step: %d. LastActionMethod: %s. LastAction: %s. Reward: %f. CumReward: %f. NewState:", self.episode_num, step, self.RL_ALGO.action_chosing_method, executed_action, reward, episode_reward)
                ##rospy.loginfo(agent_state_after) #TODO TODO: ####################

            if (self.is_episode_done): # DO NOT REMOVE THIS (IT BREAKS IF WE ARE DONE)
                if(self.episode_num % self.every_n_episodes == 0):
                    if (self.is_episode_done == 1):
                        episode_end_reason = "max time steps"
                    elif (self.is_episode_done == 2):
                        episode_end_reason = "ambulance goal"
                    elif (self.is_episode_done == 3):
                        episode_end_reason = "agent goal"
                    else:
                        episode_end_reason = "unknown"

                    rospy.loginfo("Episode: %d ended for the reason: %s", self.episode_num, episode_end_reason)
                    rospy.loginfo("Episode: %d. Step: %d. LastActionMethod: %s. LastAction: %s. Reward: %f. CumReward: %f. NewState:", self.episode_num, step, self.RL_ALGO.action_chosing_method, executed_action, reward, episode_reward)
                    ##rospy.loginfo(agent_state_after) #TODO TODO: ####################
                break

        # Episode End
        # 4: Update Epsilon after episode is done
        old_epsilon = self.RL_ALGO.epsilon
        self.RL_ALGO.epsilon = self.RL_ALGO.min_epsilon + (self.RL_ALGO.max_epsilon - self.RL_ALGO.min_epsilon) * \
                             np.exp(-self.RL_ALGO.decay_rate * self.episode_num)

        if (self.episode_num % self.every_n_episodes == 0):
            rospy.loginfo("Episode: %d. FinalCumReward: %f. OldEpsilon: %f. NewEpsilon: %f.", self.episode_num, episode_reward, old_epsilon, self.RL_ALGO.epsilon)


        if(self.print_reward_every_episode and self.episode_num % self.every_n_episodes):
            rospy.loginfo("Episode: %d. FinalCumReward: %f.", self.episode_num, episode_reward)

        return episode_reward, episode_reward_list


if __name__ == "__main__":
    rospy.init_node('single_agent_qlearning_master', disable_signals=True)

    # create object from class SAQLMaster
    saqlm = SAQLMaster()

    saqlm.execute()

    rospy.spin()
