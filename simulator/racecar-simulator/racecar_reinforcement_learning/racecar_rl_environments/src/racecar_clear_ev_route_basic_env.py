#!/usr/bin/env python
import rospy
import os
import numpy as np
import random
import roslaunch
import jinja2
import threading
import subprocess
import multiprocessing
from gazebo_msgs.srv import DeleteModel
from racecar_rl_environments.msg import areWeDone
from racecar_rl_environments.srv import states, statesRequest, statesResponse, reward, rewardRequest, rewardResponse, startSim, startSimRequest, startSimResponse, resetSim, resetSimRequest, resetSimResponse
from racecar_communication.msg import ID, IDsCombined, IDStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16, Bool
import pdb

# Locks to synchronize with main thread
toMainThread_lock = threading.Lock()
toMainThread_lock.acquire()

fromMainThread_lock = threading.Lock()
fromMainThread_lock.acquire()

class ClearEVRouteBasicEnv: # IMPORTANT NOTE: currently only handles a single agent + one ambulance --  could be extended

    def __init__(self):

        rospy.loginfo("Initializing Clear EV Route Basic Environment.")

        # set service callbacks
        rospy.Service('/states', states, self.statesServerCallback)
        rospy.Service('/reward', reward, self.rewardServerCallback)
        rospy.Service('/startSim', startSim, self.startSimCallback)
        rospy.Service('/resetSim', resetSim, self.resetSimCallback)

        # subscribe to /id_msgs to receive IDs of all vehicles
        rospy.Subscriber('/id_msgs', IDStamped, self.idMsgsCallback, queue_size = 50)

        # publisher on /RL/is_active_or_episode_done to indicate when the RL model should be deactivated or when the episode is done
        # NOTE: there should be a publisher for each agent in the environment -- edit if more agents are added
        self.pub = rospy.Publisher('/RL/is_active_or_episode_done', areWeDone, queue_size = 2)

        self.last_vehicle_ids = self.initIdsCombinedMsg() # last received array of vehicle IDs
        self.EV_index = -1 # index of the emergency vehicle in the IDs array

        # number of agents and emergency vehicles in the environment -- currently only supports 1 agent and 1 ambulance
        self.num_of_agents = -1
        self.num_of_EVs = -1

        self.is_episode_done = 0 # int: 0: episode not finished, 1: episode finished because reached max simulation time, 2: episode finished because ambulance reached goal, 3: episode finished because agent reached goal
        self.is_activated = False # bool: if RL model should be activated
       
        self.is_start_sim_requested = False
        self.is_reset_sim_requested = False

        self.is_gazebo_alive = False
        self.ping_gazebo_thread = threading.Thread(target=self.pingGazebo)

        self.episode_start_time = -1 # time at which episode starts
        self.optimal_time = 10000 # initialization of optimal episode time -- overwritten later
        self.max_time_steps = 20 * self.optimal_time # initialization of max simulation time -- overwritten later

        # get all parameters
        self.getParams()

        # initialize launch files variables
        self.initLaunchFiles()

        rospy.loginfo("Finished initializing Clear EV Route Basic Environment.")

        self.ping_gazebo_thread.start()
        self.waitForLaunchNodesRequests()

    # Initializes IDsCombined message 
    def initIdsCombinedMsg(self):

        idsCombined_msg = IDsCombined()

        # Set the initialization value of lane number to -1 (flag)
        for i in range(0,len(idsCombined_msg.ids),1):
            idsCombined_msg.ids[i].lane_num = -1

        return idsCombined_msg


    # Gets all parameters
    def getParams(self):

        rospy.loginfo("Getting parameters in Clear EV Route Basic Environment.")

        ## Name of vehicles ## 
        if rospy.has_param('r_name'):
            self.r_name = rospy.get_param('r_name')
        else:
            self.r_name = "racecar"

        ## Use gazebo gui ## 
        if rospy.has_param('gazebo_gui'):
            self.use_gazebo_gui = rospy.get_param('gazebo_gui')
        else:
            self.use_gazebo_gui = True

        ## Reward parameters ##
        if rospy.has_param('reward/max_final_reward'):
            self.max_final_reward = rospy.get_param('reward/max_final_reward')
        else:
            self.max_final_reward = 20 # reward for achieving end of simulation (done) with number_of_time_steps = self.optimal time
        ####
        if rospy.has_param('reward/min_final_reward'):
            self.min_final_reward = rospy.get_param('reward/min_final_reward')
        else:
            self.min_final_reward = -20 # reward for achieving end of simulation (done) with number_of_time_steps = 20 * self.optimal time
        ####
        if rospy.has_param('reward/max_step_reward'):
            self.max_step_reward = rospy.get_param('reward/max_step_reward')
        else:
            self.max_step_reward = 0 # reward for having an acceleration of value = self.emer_max_accel over last step
        ####
        if rospy.has_param('reward/min_step_reward'):
            self.min_step_reward = rospy.get_param('reward/min_step_reward')
        else:
            self.min_step_reward = -1.25 # reward for having an acceleration of value = - self.emer_max_accel over last step
        ####
        if rospy.has_param('reward/give_final_reward'):
            self.give_final_reward = rospy.get_param('reward/give_final_reward')
        else:
            self.give_final_reward = False # whether to give a final reward or not

        ## Communication range ##
        if rospy.has_param('communicaion_range'):
            self.comm_range = rospy.get_param('communicaion_range')
        else:
            self.comm_range = 24

        ## Ambulance parameters ##
        if rospy.has_param('ambulance/amb_start_x'):
            self.amb_start_x = rospy.get_param('ambulance/amb_start_x')
        else:
            self.amb_start_x = -50 # ambulance starting x coordinate
        ####
        if rospy.has_param('ambulance/amb_goal_x'):
            self.amb_goal_x = rospy.get_param('ambulance/amb_goal_x')
        else:
            self.amb_goal_x = 50 # ambulance goal x coordinate
        ####
        if rospy.has_param('ambulance/amb_max_vel'):
            self.emer_max_speed = rospy.get_param('ambulance/amb_max_vel')
        else:
            self.emer_max_speed = 1 # ambulance max vel
        ####
        if rospy.has_param('ambulance/amb_max_acc'):
            self.emer_max_accel = rospy.get_param('ambulance/amb_max_acc')
        else:
            self.emer_max_accel = 0.0333 # ambulance max acc
        ####
        if rospy.has_param('ambulance/rel_amb_y_min'):
            if (rospy.get_param('ambulance/rel_amb_y_min') < 0):
                self.rel_amb_y_min = - min(self.comm_range, abs(rospy.get_param('ambulance/rel_amb_y_min')))
            else:
                self.rel_amb_y_min = min(self.comm_range, abs(rospy.get_param('ambulance/rel_amb_y_min')))
        else:
            self.rel_amb_y_min = - min(self.comm_range, 24) # rear limit of window around agent 
        ####
        if rospy.has_param('ambulance/rel_amb_y_max'):
            if (rospy.get_param('ambulance/rel_amb_y_max') < 0):
                self.rel_amb_y_max = - min(self.comm_range, abs(rospy.get_param('ambulance/rel_amb_y_max')))
            else:
                self.rel_amb_y_max = min(self.comm_range, abs(rospy.get_param('ambulance/rel_amb_y_max')))
        else:
            self.rel_amb_y_max = min(self.comm_range, 9) # front limit of window around agent 

        ## Agent parameters ##
        if rospy.has_param('agent/agent_max_vel'):
            self.agent_max_vel = rospy.get_param('agent/agent_max_vel')
        else:
            self.agent_max_vel = 0.5 # agent max vel
        ####
        if rospy.has_param('agent/agent_max_acc'):
            self.agent_max_acc = rospy.get_param('agent/agent_max_acc')
        else:
            self.agent_max_acc = 0.0167 # agent max acc


    # Initializes launch files variables
    def initLaunchFiles(self):

        rospy.loginfo("Initializing launch files variables in Clear EV Route Basic Environment.")

        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.uuid)

        temp_folder = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(__file__))),"racecar_clear_ev_route/templates")
        temp_loader = jinja2.FileSystemLoader(searchpath = temp_folder)
        temp_env = jinja2.Environment(loader=temp_loader)

        ####
        empty_world_temp = temp_env.get_template("empty_world_temp.launch")

        self.empty_world = ['racecar_gazebo', 'empty_world.launch']
        self.empty_world = roslaunch.rlutil.resolve_launch_arguments(self.empty_world)
        empty_world_args = dict()

        empty_world_args['gui'] = self.use_gazebo_gui

        with open(self.empty_world[0], "w") as fp:
            fp.writelines(empty_world_temp.render(data=empty_world_args))

        ####
        self.rviz = ['racecar_rviz', 'view_two_navigation.launch']
        self.rviz = roslaunch.rlutil.resolve_launch_arguments(self.rviz)

        ####
        self.one_racecar_one_ambulance_temp = temp_env.get_template("one_racecar_one_ambulance_temp.launch")

        self.one_racecar_one_ambulance = ['racecar_clear_ev_route', 'one_racecar_one_ambulance.launch']
        self.one_racecar_one_ambulance = roslaunch.rlutil.resolve_launch_arguments(self.one_racecar_one_ambulance)


    def pingGazebo(self):

        while(1):
            ping_gazebo_server = subprocess.Popen(["rosnode", "ping", "-c", "1", "/gazebo"], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            gzs_output, gzs_error = ping_gazebo_server.communicate()

            ping_gazebo_client = subprocess.Popen(["rosnode", "ping", "-c", "1", "/gazebo_gui"], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            gzc_output, gzc_error = ping_gazebo_client.communicate()

            if (self.use_gazebo_gui == False):
                if (gzs_error != ""):
                    self.is_gazebo_alive = False
                else:
                    self.is_gazebo_alive = True
            else:
                if ((gzs_error != "") or (gzc_error != "")): #TODO: edit if do not want to quit upon death of gazebo gui
                    self.is_gazebo_alive = False
                else:
                    self.is_gazebo_alive = True

            rospy.sleep(1) #TODO: change rate as desired


    def deleteModel(self, vehicle_index):

        delete_model_client = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
        rospy.wait_for_service('gazebo/delete_model')

        delete_model_client(self.r_name + str(vehicle_index + 1))


    def relaunchGazeboIfDead(self,force_shutdown):

        while (self.is_gazebo_alive == False or force_shutdown == True):
            self.launch_empty_world.shutdown()
            os.system("killall -9 gazebo & killall -9 gzserver & killall -9 gzclient")
            rospy.loginfo("\n\nkilling gazebo\n\n")
            rospy.loginfo("\n\nkilling gazebo\n\n")
            rospy.loginfo("\n\nkilling gazebo\n\n")
            rospy.loginfo("\n\nkilling gazebo\n\n")
            rospy.sleep(60)
            rospy.loginfo("\n\nlaunching gazebo\n\n")
            rospy.loginfo("\n\nlaunching gazebo\n\n")
            rospy.loginfo("\n\nlaunching gazebo\n\n")
            rospy.loginfo("\n\nlaunching gazebo\n\n")
            self.launch_empty_world = roslaunch.parent.ROSLaunchParent(self.uuid, self.empty_world, force_screen=True, verbose=True)
            self.launch_empty_world.start()
            rospy.loginfo("\n\nlaunch empty world\n\n")
            rospy.loginfo("\n\nlaunch empty world\n\n")
            rospy.loginfo("\n\nlaunch empty world\n\n")
            rospy.loginfo("\n\nlaunch empty world\n\n")
            rospy.loginfo("\n\nlaunch empty world\n\n")
            rospy.loginfo("\n\nlaunch empty world\n\n")
            rospy.sleep(20)
            force_shutdown = False
        

    def waitForLaunchNodesRequests(self):

        while (1):
            toMainThread_lock.acquire()

            if (self.is_start_sim_requested == True):
                self.startSim()
                self.is_start_sim_requested = False
                fromMainThread_lock.release()
            elif (self.is_reset_sim_requested == True):
                self.resetSim()
                self.is_reset_sim_requested = False
                fromMainThread_lock.release()


    def genTemplateArgs(self):

        one_racecar_one_ambulance_args = dict()

        # agent args

        one_racecar_one_ambulance_args['agent_start_x'] = random.randint(-36, -36) #TODO: (LATER) let min and max position depend on EV's starting/ening position and vehicles' max vel/acc # current is -18.5 to 12 but rounded for simplicity
        one_racecar_one_ambulance_args['agent_start_y'] = self.genRandLanePos()


        one_racecar_one_ambulance_args['agent_max_vel'] = self.agent_max_vel
        one_racecar_one_ambulance_args['agent_max_acc'] = self.agent_max_acc

        one_racecar_one_ambulance_args['agent_goal_x'] = self.amb_goal_x
        one_racecar_one_ambulance_args['agent_goal_y'] = one_racecar_one_ambulance_args['agent_start_y']

        # EV args
        one_racecar_one_ambulance_args['EV_start_x'] = self.amb_start_x
        one_racecar_one_ambulance_args['EV_start_y'] = self.genRandLanePos()
        one_racecar_one_ambulance_args['EV_max_vel'] = self.emer_max_speed
        one_racecar_one_ambulance_args['EV_max_acc'] = self.emer_max_accel

        # communication range
        one_racecar_one_ambulance_args['comm_range'] = self.comm_range

        return one_racecar_one_ambulance_args


    def genRandLanePos(self):
        '''  
         Lane number convention in the "threeLanes" world:
             y^  
              |--------------> x      lane number 0 
              |--------------> x      lane number 1
              |--------------> x      lane number 2 
        '''

        lane_num = random.randint(0, 2)

        if (lane_num == 0):
           return 0.2625
        elif (lane_num == 1):
           return -0.2625
        if (lane_num == 2):
           return -0.7875


    def startSim(self):

        self.launch_empty_world = roslaunch.parent.ROSLaunchParent(self.uuid, self.empty_world, force_screen=True, verbose=True)
        self.launch_empty_world.start()
        rospy.sleep(20)

        force_shutdown = False
        self.relaunchGazeboIfDead(force_shutdown)

        ####

        one_racecar_one_ambulance_args = self.genTemplateArgs()

        with open(self.one_racecar_one_ambulance[0], "w") as fp:
            fp.writelines(self.one_racecar_one_ambulance_temp.render(data=one_racecar_one_ambulance_args))

        self.episode_start_time = rospy.Time.now()

        self.launch_one_racecar_one_ambulance = roslaunch.parent.ROSLaunchParent(self.uuid, self.one_racecar_one_ambulance, force_screen=True, verbose=True)
        self.launch_one_racecar_one_ambulance.start()
        #rospy.sleep(5)

        ####
        ##self.launch_rviz = roslaunch.parent.ROSLaunchParent(self.uuid, self.rviz, force_screen=True, verbose=True)
        ##self.launch_rviz.start()

        rospy.sleep(30)  #was 60

        self.episode_start_time = rospy.Time.now()

    
    def resetSim(self):

        if (self.is_gazebo_alive == False):
            ping_env = subprocess.Popen(["rosnode", "ping", "-c", "1", "/clear_ev_route_basic_env"], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            env_output, env_error = ping_env.communicate()
            if (env_error != ""):
                rospy.loginfo("\n\n\n\nNODE Ended\n\n\n\n")
                rospy.loginfo("\n\n\n\nNODE Ended\n\n\n\n")
                rospy.loginfo("\n\n\n\nNODE Ended\n\n\n\n")
                rospy.loginfo("\n\n\n\nNODE Ended\n\n\n\n")
                rospy.sleep(5)
            
            self.launch_one_racecar_one_ambulance.shutdown()
            ping_env = subprocess.Popen(["rosnode", "ping", "-c", "1", "/clear_ev_route_basic_env"], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            env_output, env_error = ping_env.communicate()
            if (env_error != ""):
                rospy.loginfo("\n\n\n\nNODE Ended222222222\n\n\n\n")
                rospy.loginfo("\n\n\n\nNODE Ended222222222\n\n\n\n")
                rospy.loginfo("\n\n\n\nNODE Ended222222222\n\n\n\n")
                rospy.loginfo("\n\n\n\nNODE Ended222222222\n\n\n\n")
                rospy.sleep(5)
            rospy.sleep(60)
        
            force_shutdown = True
            self.relaunchGazeboIfDead(force_shutdown)
            ping_env = subprocess.Popen(["rosnode", "ping", "-c", "1", "/clear_ev_route_basic_env"], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            env_output, env_error = ping_env.communicate()
            if (env_error != ""):
                rospy.loginfo("\n\n\n\nNODE Ended33333333333\n\n\n\n")
                rospy.loginfo("\n\n\n\nNODE Ended33333333333\n\n\n\n")
                rospy.loginfo("\n\n\n\nNODE Ended33333333333\n\n\n\n")
                rospy.loginfo("\n\n\n\nNODE Ended33333333333\n\n\n\n")
                rospy.sleep(5)
            rospy.sleep(60)
            
            ######case######
        
        else:
            
            is_gazebo_relaunch_needed = False

            for vehicle_index in range((self.num_of_agents + self.num_of_EVs)):
                del_model_process = multiprocessing.Process(target=self.deleteModel, args=(vehicle_index,))
                del_model_process.start()
                ping_env = subprocess.Popen(["rosnode", "ping", "-c", "1", "/clear_ev_route_basic_env"], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                env_output, env_error = ping_env.communicate()
                if (env_error != ""):
                    rospy.loginfo("\n\n\n\nNODE Ended555555555\n\n\n\n")
                    rospy.loginfo("\n\n\n\nNODE Ended555555555\n\n\n\n")
                    rospy.loginfo("\n\n\n\nNODE Ended555555555\n\n\n\n")
                    rospy.loginfo("\n\n\n\nNODE Ended555555555\n\n\n\n")
                del_model_process.join(7.0) #TODO: might consider increasing timeout
                ping_env = subprocess.Popen(["rosnode", "ping", "-c", "1", "/clear_ev_route_basic_env"], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                env_output, env_error = ping_env.communicate()
                if (env_error != ""):
                    rospy.loginfo("\n\n\n\nNODE Ended6666666\n\n\n\n")
                    rospy.loginfo("\n\n\n\nNODE Ended6666666\n\n\n\n")
                    rospy.loginfo("\n\n\n\nNODE Ended6666666\n\n\n\n")
                    rospy.loginfo("\n\n\n\nNODE Ended6666666\n\n\n\n")
                if (del_model_process.is_alive() == True):
                    rospy.loginfo("\n\nbefore terminate\n\n")
                    ping_env = subprocess.Popen(["rosnode", "ping", "-c", "1", "/clear_ev_route_basic_env"], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                    env_output, env_error = ping_env.communicate()
                    if (env_error != ""):
                        rospy.loginfo("\n\n\n\nNODE Ended777777\n\n\n\n")
                        rospy.loginfo("\n\n\n\nNODE Ended777777\n\n\n\n")
                        rospy.loginfo("\n\n\n\nNODE Ended777777\n\n\n\n")
                        rospy.loginfo("\n\n\n\nNODE Ended777777\n\n\n\n")
                    del_model_process.terminate()
                    rospy.loginfo("\n\after terminate\n\n")
                    ping_env = subprocess.Popen(["rosnode", "ping", "-c", "1", "/clear_ev_route_basic_env"], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                    env_output, env_error = ping_env.communicate()
                    if (env_error != ""):
                        rospy.loginfo("\n\n\n\nNODE Ended88888\n\n\n\n")
                        rospy.loginfo("\n\n\n\nNODE Ended88888\n\n\n\n")
                        rospy.loginfo("\n\n\n\nNODE Ended88888\n\n\n\n")
                        rospy.loginfo("\n\n\n\nNODE Ended88888\n\n\n\n")
                    is_gazebo_relaunch_needed = True
                    break
            ping_env = subprocess.Popen(["rosnode", "ping", "-c", "1", "/clear_ev_route_basic_env"], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            env_output, env_error = ping_env.communicate()
            if (env_error != ""):
                rospy.loginfo("\n\n\n\nNODE Ended90909090909090\n\n\n\n")
                rospy.loginfo("\n\n\n\nNODE Ended90909090909090\n\n\n\n")
                rospy.loginfo("\n\n\n\nNODE Ended90909090909090\n\n\n\n")
                rospy.loginfo("\n\n\n\nNODE Ended90909090909090\n\n\n\n")
                

            rospy.loginfo("\n\n\n\nbeforee sleep\n\n\n\n")
            rospy.loginfo("\n\n\n\nbeforee sleep\n\n\n\n")
            rospy.loginfo("\n\n\n\nbeforee sleep\n\n\n\n")
            rospy.loginfo("\n\n\n\nbeforee sleep\n\n\n\n")
            rospy.sleep(10)
            self.launch_one_racecar_one_ambulance.shutdown()
            ping_env = subprocess.Popen(["rosnode", "ping", "-c", "1", "/clear_ev_route_basic_env"], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            env_output, env_error = ping_env.communicate()
            if (env_error != ""):
                rospy.loginfo("\n\n\n\nNODE Ended9999999\n\n\n\n")
                rospy.loginfo("\n\n\n\nNODE Ended9999999\n\n\n\n")
                rospy.loginfo("\n\n\n\nNODE Ended9999999\n\n\n\n")
                rospy.loginfo("\n\n\n\nNODE Ended9999999\n\n\n\n")
            rospy.loginfo("\n\nshutdown one racecar one ambulance\n\n")
            rospy.sleep(60)

            if (is_gazebo_relaunch_needed == True):
                force_shutdown = True
                self.relaunchGazeboIfDead(force_shutdown)

        ping_env = subprocess.Popen(["rosnode", "ping", "-c", "1", "/clear_ev_route_basic_env"], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        env_output, env_error = ping_env.communicate()
        if (env_error != ""):
            rospy.loginfo("\n\n\n\nNODE Ended101010101010\n\n\n\n")
            rospy.loginfo("\n\n\n\nNODE Ended101010101010\n\n\n\n")
            rospy.loginfo("\n\n\n\nNODE Ended101010101010\n\n\n\n")
            rospy.loginfo("\n\n\n\nNODE Ended101010101010\n\n\n\n")
            rospy.loginfo("\n\n\n\nNODE Ended101010101010\n\n\n\n")
            rospy.loginfo("\n\n\n\nNODE Ended101010101010\n\n\n\n")


        ###
        #############case#########
        self.last_vehicle_ids = self.initIdsCombinedMsg()
        self.is_episode_done = 0
        self.is_activated = False

        ####
        ########case#######
        one_racecar_one_ambulance_args = self.genTemplateArgs()

        with open(self.one_racecar_one_ambulance[0], "w") as fp:
            fp.writelines(self.one_racecar_one_ambulance_temp.render(data=one_racecar_one_ambulance_args))

        self.episode_start_time = rospy.Time.now()
        
        self.launch_one_racecar_one_ambulance = roslaunch.parent.ROSLaunchParent(self.uuid, self.one_racecar_one_ambulance, force_screen=True, verbose=True)
        self.launch_one_racecar_one_ambulance.start()
        rospy.sleep(30)  #TODO: was 60

        self.episode_start_time = rospy.Time.now()


    def startSimCallback(self, req):

        resp = startSimResponse()

        if (req.num_of_agents != 1):
            raise Exception("Current version only supports ONE SINGLE AGENT. Terminating.")
            resp.is_successful = False
            return resp

        if (req.num_of_EVs != 1):
            raise Exception("Current version only supports ONE EMERGENCY VEHICLE. Terminating.")
            resp.is_successful = False
            return resp

        self.num_of_agents = req.num_of_agents
        self.num_of_EVs = req.num_of_EVs
        self.is_episode_done = 0
        self.is_activated = False

        self.is_start_sim_requested = True
        toMainThread_lock.release()
        fromMainThread_lock.acquire()

        resp.is_successful = True
        return resp


    def resetSimCallback(self, req):
        resp = resetSimResponse()

        if (req.reset_env == True):

            self.is_reset_sim_requested = True
            toMainThread_lock.release()
            fromMainThread_lock.acquire()

            resp.is_successful = True
            return resp

        else:
            resp.is_successful = False
            return resp


    def statesServerCallback(self, req):

        resp = statesResponse()

        resp.agent_vel = max(self.last_vehicle_ids.ids[req.robot_num - 1].velocity, 0)
        resp.agent_lane = self.last_vehicle_ids.ids[req.robot_num - 1].lane_num

        resp.amb_vel = max(self.last_vehicle_ids.ids[self.EV_index].velocity, 0)
        resp.amb_lane = self.last_vehicle_ids.ids[self.EV_index].lane_num


        agent_x_pos = self.last_vehicle_ids.ids[req.robot_num - 1].x_position
        amb_x_pos = self.last_vehicle_ids.ids[self.EV_index].x_position

        resp.rel_amb_y = amb_x_pos - agent_x_pos #TODO: (LATER) misleading name: change to rel_amb_x 
     
        return resp


    def rewardServerCallback(self, req):
        
        myreward = self.calc_reward(req.amb_last_velocity, req.execution_time)
        return rewardResponse(myreward)


    def calc_reward(self, amb_last_velocity, execution_time):

        '''
        :logic: Calculate reward to agent from current state
        :param amb_last_velocity: float, previous velocity the EV/ambulance had
        :return: reward (either step reward or final reward)


        :Notes:
        #Simulation Time is not allowed to continue after 20*optimal_time (20* time steps with ambulance at its maximum speed)
        '''

        if(self.is_episode_done and self.give_final_reward): #Calculate a final reward

            number_of_time_steps = rospy.Time.now() - self.episode_start_time #Time spent in episode so far

            #Linear reward. y= mx +c. y: reward, x: ration between time achieved and optimal time. m: slope. c: y-intercept
            m = ( (self.max_final_reward - self.min_final_reward) *20 ) /19 #Slope for straight line equation to calculate final reward
            c = self.max_final_reward - 1*m #c is y-intercept for the reward function equation #max_final_reward is the y for x = 1
            reward = m * (self.optimal_time/number_of_time_steps) + c
            #debug#print(f'c: {c}, m: {m}, steps: {number_of_time_steps}, optimal_time: {self.optimal_time}')
            return reward

        else: #Calcualate a step reward
            steps_needed_to_halt = 30
            ration_of_halt_steps_to_total_steps = steps_needed_to_halt/(self.amb_goal_x - self.amb_start_x)

            m = (self.max_step_reward - self.min_step_reward)/(2 * self.emer_max_accel)  # Slope for straight line equation to calculate step reward
            # debug # rospy.loginfo("M = : %f", m)
            #2 * self.emer.max_accel since: = self.emer.max_accel - * self.emer.max_decel
            c = self.max_step_reward - self.emer_max_accel * m  # c is y-intercept for the reward function equation #max_step_reward is the y for x = 2 (max acceleration)
            # debug # rospy.loginfo("C = : %f", c)
            emer_curr_spd = self.last_vehicle_ids.ids[self.EV_index].velocity
            reward = m * (emer_curr_spd - amb_last_velocity) + c
            reward = reward/execution_time
            #rospy.loginfo("Reward = : %f", reward)
            #debug#print(f'c: {c}, m: {m}, accel: {(self.emer.spd - amb_last_velocity)}')
            #rospy.loginfo("if condition = : %f", abs(amb_last_velocity-self.emer_max_speed))
            
            if ( abs(amb_last_velocity-self.emer_max_speed) <= 1e-10 ):
            #since ambulance had maximum speed and speed did not change that much; unless we applied the code below.. the acceleration
            #   will be wrongly assumed to be zero. Although the ambulance probably could have accelerated more, but this is its maximum velocity.
                reward = self.max_step_reward/execution_time #same reward as maximum acceleration

            rospy.loginfo("Reward: %f", reward)

            return reward


    def idMsgsCallback(self, idMsgs):
        rospy.loginfo("\n\nidMsgs logged\n\n")
        # extra check
        if (idMsgs.robot_num > (self.num_of_agents + self.num_of_EVs)):
            #raise Exception("NUMBER OF VEHICLES IN ENVIRONMENT IS GREATER THAN EXPECTED!") #TODO:
            pub = rospy.Publisher('/VehiclesException', Bool, queue_size=10)
            pub.publish(True)
            pub = rospy.Publisher('/VehiclesException/robotNum', Int16, queue_size=10)
            pub.publish(idMsgs.robot_num)
            pub = rospy.Publisher('/VehiclesException/numofagents', Int16, queue_size=10)
            pub.publish(self.num_of_agents)
            pub = rospy.Publisher('/VehiclesException/numofEvs', Int16, queue_size=10)
            pub.publish(self.num_of_EVs)
            

        self.last_vehicle_ids.header.stamp = rospy.Time.now()
        self.last_vehicle_ids.header.frame_id = "map"
        self.last_vehicle_ids.robot_num = idMsgs.robot_num

        self.last_vehicle_ids.ids[idMsgs.robot_num - 1] = idMsgs.id

        if (idMsgs.id.type.data == "ambulance"):
            if (self.EV_index == -1):
                self.EV_index = idMsgs.robot_num - 1
                self.emer_max_speed = idMsgs.id.max_vel
                self.emer_max_accel = idMsgs.id.max_acc
                self.optimal_time = int(np.round((self.amb_goal_x - self.amb_start_x) / self.emer_max_speed))  # Optimal number of time steps: number of time steps taken by ambulance at maximum speed
                self.max_time_steps = 20 * self.optimal_time
            # extra check
            elif (self.EV_index != (idMsgs.robot_num - 1)):
                #raise Exception("NUMBER OF EMERGENCY VEHICLES IN ENVIRONMENT IS GREATER THAN EXPECTED!") #TODO:
                pub = rospy.Publisher('/EmergencyException', Bool, queue_size=10)
                pub.publish(True)
                pub = rospy.Publisher('/EmergencyException/EVIndex', Int16, queue_size=10)
                pub.publish(self.EV_index)
                pub = rospy.Publisher('/EmergencyException/robot_num', Int16, queue_size=10)
                pub.publish(idMsgs.robot_num)

        self.are_we_done()
        self.is_RL_activated()

        msg = areWeDone()
        msg.is_activated = self.is_activated
        msg.is_episode_done = self.is_episode_done
        self.pub.publish(msg)


    def are_we_done(self):

        amb_abs_y = self.last_vehicle_ids.ids[self.EV_index].x_position #TODO: (LATER) misleading name: change to amb_abs_x 

        #1: steps == max_time_steps - 1
        time_step_number = rospy.Time.now() - self.episode_start_time
        if(time_step_number.secs >= self.max_time_steps-1):
            self.is_episode_done = 1
            return
        #2: goal reached
        #elif(amb_abs_y >= self.amb_goal_x): #TODO
        elif(amb_abs_y >= -35.5):
            # DONE: Change NET file to have total distance = 511. Then we can have the condition to compare with 500 directly.
            #return 2 #GOAL IS NOW 500-10-1 = 489 cells ahead. To avoid ambulance car eacaping
            self.is_episode_done = 2
            return 
        for agent_index in range((self.num_of_agents + self.num_of_EVs)):
            if (agent_index != self.EV_index):
                agent_abs_y = self.last_vehicle_ids.ids[agent_index].x_position #TODO: (LATER) misleading name: change to agent_abs_x

                #if (agent_abs_y > self.amb_goal_x): #TODO: changed for test
                if (agent_abs_y > -35):
                    self.is_episode_done = 3
                    return
        if(self.is_gazebo_alive == False):
            self.is_episode_done = 4
            return

        # 0: not done
        self.is_episode_done = 0
        return


    def is_RL_activated(self):

        for agent_index in range((self.num_of_agents + self.num_of_EVs)):
            if (agent_index != self.EV_index):

                agent_x_pos = self.last_vehicle_ids.ids[agent_index].x_position
                amb_x_pos = self.last_vehicle_ids.ids[self.EV_index].x_position
                rel_amb_x = amb_x_pos - agent_x_pos

                if ((rel_amb_x < self.rel_amb_y_min) or (rel_amb_x > self.rel_amb_y_max)): #TODO: (LATER) misleading name: change to rel_amb_x_min 
                    self.is_activated = False
                else:
                    self.is_activated = True
        return

    def close_launch_files(self):
        self.launch_empty_world.shutdown()
        self.launch_one_racecar_one_ambulance.shutdown()
        #self.launch_rviz.shutdown()

if __name__ == '__main__':
    rospy.init_node("clear_ev_route_basic_env")

    cEVrbe = ClearEVRouteBasicEnv()
    rospy.on_shutdown(cEVrbe.close_launch_files())

    rospy.spin()
