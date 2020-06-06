#!/usr/bin/env python
import rospy
import numpy as np
import random
import os
from std_msgs.msg import Header
from racecar_clear_ev_route.srv import RLPolicyActionService, RLPolicyActionServiceRequest, RLPolicyActionServiceResponse
from racecar_rl_environments.srv import states, statesRequest, statesResponse


class observed_state:
    def __init__(self):
        #states
        self.agent_vel = -1
        self.agent_lane = -1
        self.amb_vel = -1
        self.amb_lane = -1
        self.rel_amb_y = -1
        
class SingleAgentQlearning:
    def __init__(self, environment_init, algo_params=dict(), load_q_table = False, test_mode_on = False):

        rospy.loginfo("Initializing Single Agent Qlearning.")

        self.test_mode_on = test_mode_on
        #Constants for the environment
        self.environment_init = environment_init
        
        #window fixed parameters
        self.rel_amb_y_min = self.environment_init['rel_amb_y_min']
        self.rel_amb_y_max = self.environment_init['rel_amb_y_max']
        self.amb_vel_min = self.environment_init['amb_vel_min']
        self.amb_vel_max = self.environment_init['amb_vel_max']
        self.agent_vel_min = self.environment_init['agent_vel_min']
        self.agent_vel_max = self.environment_init['agent_vel_max']
        self.amb_acc = self.environment_init['amb_acc']
        self.agent_acc = self.environment_init['agent_acc']

        #RL Actions
        self.Actions = ["change_left", "change_right", "acc", "no_acc", "dec"]
        self.action_string_to_index_dict = {
            "change_left": 0,
            "change_right": 1,
            "acc": 2,
            "no_acc": 3,
            "dec": 4
        }  # Must maintain order in Actions
        self.actions_indices = []   
        for act in self.Actions:
            self.actions_indices.append(self.action_string_to_index_dict[act])
        
        self.action_chosing_method = None  # To be asssigned: Exploration or Exploitation based on exp_exp_tradeoff and epsilon

        #Q table Initialization 
        if(load_q_table or self.test_mode_on): # load_q_table if we are testing or we want to load it
            self.q_table = self.load_q_table()
        else:
            #self.q_table = np.zeros((6, 3, 11, 3, 58, 5))
            self.q_table = np.zeros((6, 3, 11, 3, (abs(self.rel_amb_y_max) + 1 + abs(self.rel_amb_y_min)), 5))
            #Initialize all the Q table with -1000 as a flag for unvisited action
            self.q_table.fill(-1000)
            
            #Setting Algorithm parameters 
            self.exp_exp_tradeoff = algo_params['exp_exp_tradeoff']
            self.epsilon = algo_params['epsilon']
            self.gamma = algo_params['gamma']
            self.learning_rate = algo_params['learning_rate']
            self.max_epsilon = algo_params['max_epsilon']
            self.min_epsilon = algo_params['min_epsilon']
            self.decay_rate = algo_params['decay_rate']
        
        #new and last observed states parameters 
        #self.new_observed_state_for_this_agent = observed_state()
        #self.last_observed_state_for_this_agent = observed_state()
        self.new_observed_state_for_this_agent = statesResponse()
        self.last_observed_state_for_this_agent = statesResponse()

        #new and last obesrved states INDICES parameters
        #self.new_observed_state_INDEX_for_this_agent = observed_state()
        #self.last_observed_state_INDEX_for_this_agent = observed_state()
        self.new_observed_state_INDEX_for_this_agent = statesResponse()
        self.last_observed_state_INDEX_for_this_agent = statesResponse()
        
        #RL engagement and disengagement
        self.RLdisengage = False  

        #establishing a connection with the Action Execution Server (Move Car Action Client)
        ##rospy.wait_for_service('move_car/RL/RLPolicyActionService') #TODO: NEED TO MOVE IT SOMEWHERE ELSE BECAUSE InITIALLY IT IS NOT AVAILABLE

        rospy.loginfo("Finished initializing Single Agent Qlearning.")
    
    #updates the new observed state parameters  
    def update_new_observed_state_for_this_agent(self, new_observed_state_for_this_agent):
        #first, update last observed state for this agent
        self.last_observed_state_for_this_agent = self.new_observed_state_for_this_agent
        self.last_observed_state_INDEX_for_this_agent = self.new_observed_state_INDEX_for_this_agent

        #update the new observed state for this agent (states and indices)
        self.new_observed_state_for_this_agent.agent_vel = new_observed_state_for_this_agent[0]
        self.new_observed_state_for_this_agent.agent_lane = new_observed_state_for_this_agent[1]
        self.new_observed_state_for_this_agent.amb_vel = new_observed_state_for_this_agent[2]
        self.new_observed_state_for_this_agent.amb_lane = new_observed_state_for_this_agent[3]
        self.new_observed_state_for_this_agent.rel_amb_y = np.clip(new_observed_state_for_this_agent[4], self.rel_amb_y_min,
                            self.rel_amb_y_max)  # rel_amb_y  (16+1+41 = 58): [-41,-40,-39,.....,0,...13,14,15,16]

        "If it takes an array, uncomment the following (89) and remove from 81 to 86"
        #self.new_observed_state_for_this_agent = new_observed_state_for_this_agent

        #getting indices from states

        #velocity #Multiplied by 10 to convert it to indices
        self.new_observed_state_INDEX_for_this_agent.agent_vel = int(np.round(self.new_observed_state_for_this_agent.agent_vel*10))  # [0,1,2,3,4,5] 

        self.new_observed_state_INDEX_for_this_agent = self.new_observed_state_for_this_agent.agent_lane

        #velocity #Multiplied by 10 to convert it to indices
        self.new_observed_state_INDEX_for_this_agent.amb_vel = int(np.round(self.new_observed_state_for_this_agent.amb_vel*10))  # [0,1,2,3,4,5,6,7,8,9,10] 
        
        self.new_observed_state_INDEX_for_this_agent.amb_lane = self.new_observed_state_for_this_agent.amb_lane
        
        self.new_observed_state_INDEX_for_this_agent.rel_amb_y = int(np.round(self.new_observed_state_for_this_agent.rel_amb_y) + abs(self.rel_amb_y_min))

    #picks an action by exploitation or exploration 
    def pick_action(self, feasible_actions_indices):
        
        if (self.action_chosing_method == 'expLOIT'):  # test_mode_on will force the algorithm to choose exploitation.
            
            #max_value_index = np.argmax(self.q_table[self.new_agent_vel_index, self.new_agent_lane_index, self.new_amb_vel_index, self.new_amb_lane_index, self.new_rel_amb_y_index, feasible_actions_indices]) 
            max_value_index = np.argmax(self.q_table[self.new_observed_state_INDEX_for_this_agent.agent_vel, 
                    self.new_observed_state_INDEX_for_this_agent.agent_lane,
                    self.new_observed_state_INDEX_for_this_agent.amb_vel, 
                    self.new_observed_state_INDEX_for_this_agent.amb_lane, 
                    self.new_rel_amb_y_index, feasible_actions_indices]) 
            
            desired_action_string = self.actions_indices[max_value_index]
            self.Action = desired_action_string
        
        elif(self.action_chosing_method == 'expLORE'):
            
            action_index = random.choice(self.actions_indices)

            desired_action_string = self.actions_indices[action_index]
            #debug# print("I am the picked action for exploration ", desired_action_string)
            self.Action = desired_action_string
    
    #action execution, return its feasibility and taken time if feasible
    #also, it deactivates RL
    def execute_action(self):
        action_feasibility = False
        action_taken_time = 0
        
        #update request parameters 
        if (self.Action == 'change_left'):
            action_request_control_action = 1
            action_request_acc = 0
        elif(self.Action == 'change_right'):
            action_request_control_action = 2
            action_request_acc = 0
        elif(self.Action == 'acc'):
            action_request_control_action = 0
            action_request_acc = self.agent_acc
        elif(self.Action == 'no_acc'):
            action_request_control_action = 0
            action_request_acc = 0
        elif(self.Action == 'dec'):
            action_request_control_action = 0
            action_request_acc = -self.agent_acc
        
        action_request_header = rospy.Time.now()
        deactivateRL = False
        
        try:
            sendAction = rospy.ServiceProxy('move_car/RL/RLPolicyActionService',RLPolicyActionService, persistent=True)
            resp = sendAction(action_request_header, action_request_control_action, action_request_acc, deactivateRL)
            action_feasibility = resp.RLActionresult
            action_taken_time = resp.RLActionTime
            rospy.loginfo("resp is %i", self.resp_feasibility)
    
        except rospy.ServiceException, e:
                rospy.wait_for_service('move_car/RL/RLPolicyActionService')
                print "Service call failed: %s"%e
        
        return action_feasibility, action_taken_time

    
    #final method to be used by RL master client, it takes the new observed state and returns the
    #action with the time taken for the it to be executed 
    def take_action(self, new_observed_state_for_this_agent):
        
        #update the new observed state for the action
        self.update_new_observed_state_for_this_agent(new_observed_state_for_this_agent)
        #exploration or exploitation choice 
        self.exp_exp_tradeoff = random.uniform(0,1)
        
        if (self.exp_exp_tradeoff > self.epsilon or self.test_mode_on):
            self.action_chosing_method = 'expLOIT'
        else:
            self.action_chosing_method = 'expLORE'

        #initially all actions are feasible
        feasible_actions_indices = self.actions_indices
        found_feasible_action = False

        #keep trying to pick an action until it's feasible
        #remove the infeasible actions from feasible_actions_indices
        while(not found_feasible_action):
            
            self.pick_action(feasible_actions_indices)
            action_feasibility, action_taken_time = self.execute_action()
            
            if (action_feasibility == True and action_taken_time > 0):  #If action was feasible and executed 
                found_feasible_action = True
            else:
                found_feasible_action = False
                feasible_actions_indices.remove(self.action_string_to_index_dict[self.Action])

        return self.Action, action_taken_time

    def disengage(self):
        self.RLdisengage = True
        try:
            sendAction = rospy.ServiceProxy('move_car/RL/RLPolicyActionService',RLPolicyActionService, persistent=True)
            resp = sendAction(self.action_request_header, self.action_request_control_action, self.action_request_acc, self.RLdisengage)
    
        except rospy.ServiceException, e:
                rospy.wait_for_service('move_car/RL/RLPolicyActionService')
                print "Service call failed: %s"%e
    
    def update_q_table(self, chosen_action, reward, new_observed_state_for_this_agent):
        
        if(self.test_mode_on):  # do not update q_table if test_mode is on ! just use it.
            pass
        else:
            # Update Q(s,a):= Q(s,a) + lr [R(s,a) + gamma * max Q(s',a') - Q(s,a)]
            '''
            s: old state for which a was decided
            a: action taken in state s
            s': new state we are in because of action a (previous action)
            a': new action we are expected to choose if we exploit on s' (new state)
            '''
            action_index = self.action_string_to_index_dict[chosen_action]
            self.update_new_observed_state_for_this_agent(new_observed_state_for_this_agent)

            # OLD STATE VARIABLES:
            agent_vel_index = self.last_observed_state_INDEX_for_this_agent.agent_vel
            agent_lane_index = self.last_observed_state_INDEX_for_this_agent.agent_lane
            amb_vel_index = self.last_observed_state_INDEX_for_this_agent.amb_vel
            amb_lane_index = self.last_observed_state_INDEX_for_this_agent.amb_lane
            rel_amb_y_index = self.last_observed_state_INDEX_for_this_agent.rel_amb_y

            # NEW STATE VARIABLES:
            new_agent_vel_index = self.new_observed_state_INDEX_for_this_agent.agent_vel
            new_agent_lane_index = self.new_observed_state_INDEX_for_this_agent.agent_lane
            new_amb_vel_index = self.new_observed_state_INDEX_for_this_agent.amb_vel
            new_amb_lane_index = self.new_observed_state_INDEX_for_this_agent.amb_lane
            new_rel_amb_y_index = self.new_observed_state_INDEX_for_this_agent.rel_amb_y

            # Q(s,a):
            q_of_s_a_value = \
            self.q_table[agent_vel_index][agent_lane_index][amb_vel_index][amb_lane_index][rel_amb_y_index][action_index]

            #ROS edits in problem forumation 
            if (q_of_s_a_value == -1000):
                q_of_s_a_value = 0

            # max Q(s',a')
            max_q_of_s_value_new = np.max(self.q_table[
                                              new_agent_vel_index, new_agent_lane_index, new_amb_vel_index, new_amb_lane_index, new_rel_amb_y_index])

            # Actual Update:
            q_of_s_a_value = q_of_s_a_value + self.learning_rate * (reward + self.gamma * max_q_of_s_value_new - q_of_s_a_value)


            # actual update step:
            self.q_table[agent_vel_index][agent_lane_index][amb_vel_index][amb_lane_index][rel_amb_y_index][
                action_index] = q_of_s_a_value

    
    def save_q_table(self, variables_folder_path = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(__file__))),"saved_variables")):
        if(self.test_mode_on):
            pass  # do not save
        else:
            np.save(variables_folder_path+'/Q_TABLE.npy', self.q_table)


    def load_q_table(self, variables_folder_path = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(__file__))),"saved_variables")):
        #rospy.loginfo("Loaded Q_TABLE from {variables_folder_path + '/Q_TABLE.npy'}")
        return np.load(variables_folder_path+'/Q_TABLE.npy')

    '''
        Q                #Q_table. Multi-dimensional np.ndarray, each dimension: either state partial assignment or action (string action -> integer)
                          transformation is defined via action_to_string_dict
                         #Access order for the Q_table is [agent_vel][agent_lane][amb_vel][amb_lane][rel_amb_y][action]
                         #Values are kept as integers by rounding and casting as int. Values are clipped using np.clip() function
                         # agent_vel  (6): [0,1,2,3,4,5] #Clipped before applying velocity
                         # agent_lane (3): [0,1,2]
                         # amb_vel    (11): [0,1,2,3,4,5,6,7,8,9,10]
                         # amb_lane   (3): [0,1,2]
                         # rel_amb_y  (16+1+41 = 58): [-41,-40,-39,.....,0,...13,14,15,16]
                            #since window is designed to be:
                            #<--4 time steps *10 cells/step  = 40 steps  behind -- . agent . -- 3 steps * 5 cells/sec -->
                        # action     (5) : [Change left, Change right, acc +1, acc -1, acc 0]
                        Q_table size, is therefore = 6 * 3 * 11 *3 * 58 * 5 = 172260 ~ 170K .
                            Note that some Q(s,a) pairs will be infeasible and hence will not be trained/updated.
        '''
          




    
