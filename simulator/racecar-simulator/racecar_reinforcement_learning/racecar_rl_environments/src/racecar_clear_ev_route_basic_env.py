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
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16, Bool
from racecar_rl_environments.msg import areWeDone
from racecar_rl_environments.srv import states, statesRequest, statesResponse, reward, rewardRequest, rewardResponse, startSim, startSimRequest, startSimResponse, resetORcloseSim, resetORcloseSimRequest, resetORcloseSimResponse
from racecar_communication.msg import ID, IDsCombined, IDStamped

# Locks to synchronize service callbacks with main thread
toMainThread_lock = threading.Lock()
toMainThread_lock.acquire()

fromMainThread_lock = threading.Lock()
fromMainThread_lock.acquire()

# Locks to synchronize with the simulation being ready
simReady_lock = threading.Lock()
simReady_lock.acquire()

class ClearEVRouteBasicEnv: # IMPORTANT NOTE #: this version currently only handles a single agent + one ambulance --  could be extended

    def __init__(self):

        rospy.loginfo("Initializing Clear EV Route Basic Environment.")

        # set service callbacks
        rospy.Service('/states', states, self.statesServerCallback)
        rospy.Service('/reward', reward, self.rewardServerCallback)
        rospy.Service('/startSim', startSim, self.startSimCallback)
        rospy.Service('/resetSim', resetORcloseSim, self.resetSimCallback)
        rospy.Service('/closeSim', resetORcloseSim, self.closeSimCallback)

        # subscribe to /id_msgs to receive IDs of all vehicles
        rospy.Subscriber('/id_msgs', IDStamped, self.idMsgsCallback, queue_size = 50)

        # publisher on /RL/is_active_or_episode_done to indicate when the RL model should be deactivated or when the episode is done
        # NOTE #: there should be a publisher for each agent in the environment -- edit if more agents are added
        self.pub = rospy.Publisher('/RL/is_active_or_episode_done', areWeDone, queue_size = 2)

        self.last_vehicle_ids = self.initIdsCombinedMsg() # last received array of vehicle IDs
        self.EV_index = -1 # index of the emergency vehicle in the IDs array

        # number of agents and emergency vehicles in the environment -- currently only supports 1 agent and 1 ambulance
        self.num_of_agents = -1
        self.num_of_EVs = -1

        self.is_episode_done = 0 # int: 0: episode not finished, 1: episode finished because reached max simulation time, 2: episode finished because ambulance reached goal, 3: episode finished because agent reached goal
        self.is_activated = False # bool: if RL model should be activated
       
        self.is_start_sim_requested = False # bool: if it is requested to start simulation
        self.is_reset_sim_requested = False # bool: if it is requested to reset simulation
        self.is_close_sim_requested = False # bool: if it is requested to close simulation

        self.is_gazebo_alive = False # bool: if gazebo server and/or client nodes are properly running
        self.ping_gazebo_thread = threading.Thread(target=self.pingGazebo) # a thread to frequently check the activity of gazebo nodes

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
            self.r_name = rospy.get_param('r_name') # vehicles' name
            rospy.loginfo("Reading vehicles' name (r_name) parameter: %s.", self.r_name)
        else:
            self.r_name = "racecar"
            rospy.loginfo("Setting vehicles' name (r_name) to default: %s.", self.r_name)

        ## Use gazebo gui ## 
        if rospy.has_param('gazebo_gui'):
            self.use_gazebo_gui = rospy.get_param('gazebo_gui') # whether to display gazebo gui
            rospy.loginfo("Reading gazebo_gui parameter: %d.", self.use_gazebo_gui)
        else:
            self.use_gazebo_gui = False
            rospy.loginfo("Setting gazebo_gui parameter to default: %d.", self.use_gazebo_gui)

        ## Reward parameters ##
        if rospy.has_param('reward/max_final_reward'): # reward for achieving end of simulation (done) with number_of_time_steps = self.optimal time
            self.max_final_reward = rospy.get_param('reward/max_final_reward')
            rospy.loginfo("Reading max_final_reward parameter: %f.", self.max_final_reward)
        else:
            self.max_final_reward = 20
            rospy.loginfo("Setting max_final_reward parameter to default: %f.", self.max_final_reward)
        ####
        if rospy.has_param('reward/min_final_reward'): # reward for achieving end of simulation (done) with number_of_time_steps = 20 * self.optimal time
            self.min_final_reward = rospy.get_param('reward/min_final_reward')
            rospy.loginfo("Reading min_final_reward parameter: %f.", self.min_final_reward)
        else:
            self.min_final_reward = -20
            rospy.loginfo("Setting min_final_reward parameter to default: %f.", self.min_final_reward)
        ####
        if rospy.has_param('reward/max_step_reward'): # reward for having an acceleration of value = self.emer_max_accel over last step
            self.max_step_reward = rospy.get_param('reward/max_step_reward')
            rospy.loginfo("Reading max_step_reward parameter: %f.", self.max_step_reward)
        else:
            self.max_step_reward = 0
            rospy.loginfo("Setting max_step_reward parameter to default: %f.", self.max_step_reward)
        ####
        if rospy.has_param('reward/min_step_reward'): # reward for having an acceleration of value = - self.emer_max_accel over last step
            self.min_step_reward = rospy.get_param('reward/min_step_reward')
            rospy.loginfo("Reading min_step_reward parameter: %f.", self.min_step_reward)
        else:
            self.min_step_reward = -1.25
            rospy.loginfo("Setting min_step_reward parameter to default: %f.", self.min_step_reward)
        ####
        if rospy.has_param('reward/give_final_reward'): # whether to give a final reward or not
            self.give_final_reward = rospy.get_param('reward/give_final_reward')
            rospy.loginfo("Reading give_final_reward parameter: %d.", self.give_final_reward)
        else:
            self.give_final_reward = False
            rospy.loginfo("Setting give_final_reward parameter to default: %d.", self.give_final_reward)

        ## Communication range ##
        if rospy.has_param('communicaion_range'): # vehciles' communication range
            self.comm_range = rospy.get_param('communicaion_range')
            rospy.loginfo("Reading communicaion_range parameter: %f.", self.comm_range)
        else:
            self.comm_range = 24
            rospy.loginfo("Setting communicaion_range parameter to default: %f.", self.comm_range)

        ## Ambulance parameters ##
        if rospy.has_param('ambulance/amb_start_x'): # ambulance starting x coordinate
            self.amb_start_x = rospy.get_param('ambulance/amb_start_x')
            rospy.loginfo("Reading amb_start_x parameter: %f.", self.amb_start_x)
        else:
            self.amb_start_x = -50
            rospy.loginfo("Setting amb_start_x parameter to default: %f.", self.amb_start_x)
        ####
        if rospy.has_param('ambulance/amb_goal_x'): # ambulance goal x coordinate
            self.amb_goal_x = rospy.get_param('ambulance/amb_goal_x')
            rospy.loginfo("Reading amb_goal_x parameter: %f.", self.amb_goal_x)
        else:
            self.amb_goal_x = 50
            rospy.loginfo("Setting amb_goal_x parameter to default: %f.", self.amb_goal_x)
        ####
        if rospy.has_param('ambulance/amb_max_vel'): # ambulance max vel
            self.emer_max_speed = rospy.get_param('ambulance/amb_max_vel')
            rospy.loginfo("Reading amb_max_vel parameter: %f.", self.emer_max_speed)
        else:
            self.emer_max_speed = 1
            rospy.loginfo("Setting amb_max_vel parameter to default: %f.", self.emer_max_speed)
        ####
        if rospy.has_param('ambulance/amb_max_acc'): # ambulance max acc
            self.emer_max_accel = rospy.get_param('ambulance/amb_max_acc')
            rospy.loginfo("Reading amb_max_acc parameter: %f.", self.emer_max_accel)
        else:
            self.emer_max_accel = 0.0333
            rospy.loginfo("Setting amb_max_acc parameter to default: %f.", self.emer_max_accel)
        ####
        if rospy.has_param('ambulance/rel_amb_y_min'): # rear limit of RL activity window around agent 
            if (rospy.get_param('ambulance/rel_amb_y_min') < 0):
                self.rel_amb_y_min = - min(self.comm_range, abs(rospy.get_param('ambulance/rel_amb_y_min')))
            else:
                self.rel_amb_y_min = min(self.comm_range, abs(rospy.get_param('ambulance/rel_amb_y_min')))
            rospy.loginfo("Reading rel_amb_y_min parameter and setting to: %f.", self.rel_amb_y_min)
        else:
            self.rel_amb_y_min = - min(self.comm_range, 24)
            rospy.loginfo("Setting rel_amb_y_min parameter to: %f.", self.rel_amb_y_min)
        ####
        if rospy.has_param('ambulance/rel_amb_y_max'): # front limit of RL activity window around agent
            if (rospy.get_param('ambulance/rel_amb_y_max') < 0):
                self.rel_amb_y_max = - min(self.comm_range, abs(rospy.get_param('ambulance/rel_amb_y_max')))
            else:
                self.rel_amb_y_max = min(self.comm_range, abs(rospy.get_param('ambulance/rel_amb_y_max')))
            rospy.loginfo("Reading rel_amb_y_max parameter and setting to: %f.", self.rel_amb_y_max)
        else:
            self.rel_amb_y_max = min(self.comm_range, 9) 
            rospy.loginfo("Setting rel_amb_y_max parameter to: %f.", self.rel_amb_y_max)

        ## Agent parameters ##
        if rospy.has_param('agent/agent_max_vel'): # agent max vel
            self.agent_max_vel = rospy.get_param('agent/agent_max_vel')
            rospy.loginfo("Reading agent_max_vel parameter: %f.", self.agent_max_vel)
        else:
            self.agent_max_vel = 0.5
            rospy.loginfo("Setting agent_max_vel parameter to default: %f.", self.agent_max_vel)
        ####
        if rospy.has_param('agent/agent_max_acc'): # agent max acc
            self.agent_max_acc = rospy.get_param('agent/agent_max_acc')
            rospy.loginfo("Reading agent_max_acc parameter: %f.", self.agent_max_acc)
        else:
            self.agent_max_acc = 0.0167
            rospy.loginfo("Setting agent_max_acc parameter to default: %f.", self.agent_max_acc)


    # Initializes launch files variables
    def initLaunchFiles(self):

        rospy.loginfo("Initializing launch files variables in Clear EV Route Basic Environment.")

        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        # for logging purposes
        roslaunch.configure_logging(self.uuid)

        # open the templates folder
        temp_folder = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(__file__))),"racecar_clear_ev_route/templates")
        temp_loader = jinja2.FileSystemLoader(searchpath = temp_folder)
        temp_env = jinja2.Environment(loader=temp_loader)

        ## Gazebo launch file ##

        # get template
        empty_world_temp = temp_env.get_template("empty_world_temp.launch")

        # get full file path from package and script name
        self.empty_world = ['racecar_gazebo', 'empty_world.launch']
        self.empty_world = roslaunch.rlutil.resolve_launch_arguments(self.empty_world)

        # set arguments using parameters
        empty_world_args = dict()
        empty_world_args['gui'] = self.use_gazebo_gui

        # render a version of the template using the set arguments
        with open(self.empty_world[0], "w") as fp:
            fp.writelines(empty_world_temp.render(data=empty_world_args))

        ## RViz launch file ##

        # get full file path from package and script name
        ##self.rviz = ['racecar_rviz', 'view_two_navigation.launch']
        ##self.rviz = roslaunch.rlutil.resolve_launch_arguments(self.rviz)

        ## Vehciles' launch file ##
        # NOTE #: this launch file should be changed if the environment is extended to handle more than 1 agent & 1 EV 

        # get template
        self.one_racecar_one_ambulance_temp = temp_env.get_template("one_racecar_one_ambulance_temp.launch")

        # get full file path from package and script name
        self.one_racecar_one_ambulance = ['racecar_clear_ev_route', 'one_racecar_one_ambulance.launch']
        self.one_racecar_one_ambulance = roslaunch.rlutil.resolve_launch_arguments(self.one_racecar_one_ambulance)


    # Frequently checks the activity of gazebo nodes
    def pingGazebo(self):

        while(1):
            # ping the gazebo server node and save the standard output and error of the command
            ping_gazebo_server = subprocess.Popen(["rosnode", "ping", "-c", "1", "/gazebo"], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            gzs_output, gzs_error = ping_gazebo_server.communicate()

            # ping the gazebo client node and save the standard output and error of the command
            ping_gazebo_client = subprocess.Popen(["rosnode", "ping", "-c", "1", "/gazebo_gui"], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            gzc_output, gzc_error = ping_gazebo_client.communicate()

            # check if any errors occured while trying to ping the node/s and set the variable is_gazebo_alive accordingly
            if (self.use_gazebo_gui == False):
                if (gzs_error != ""):
                    self.is_gazebo_alive = False
                else:
                    self.is_gazebo_alive = True
            else:
                if ((gzs_error != "") or (gzc_error != "")): # NOTE #: edit if you do not want to quit episode upon death of the gazebo gui
                    self.is_gazebo_alive = False
                else:
                    self.is_gazebo_alive = True

            rospy.sleep(1) # NOTE #: change rate as desired

    # Deletes a certain vehicle from the simulation
    def deleteModel(self, vehicle_index):

        try:
            # wait for the delete_model service provided by gazebo
            delete_model_client = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
            rospy.wait_for_service('gazebo/delete_model')

            # request deletion of the model
            rospy.loginfo("\n\nAttempting to delete the model: %s.\n\n", self.r_name + str(vehicle_index + 1))
            delete_model_client(self.r_name + str(vehicle_index + 1))

        except:
            rospy.loginfo("Failed to delete the model: %s.\n\n", self.r_name + str(vehicle_index + 1))

        else:
            rospy.loginfo("Successfully deleted the model: %s.\n\n", self.r_name + str(vehicle_index + 1))


    # Kills and relaunches gazebo if it is dead or if force shutown is requested
    def relaunchGazeboIfDead(self, force_shutdown):

        # if gazebo is dead if force shutown is true
        while (self.is_gazebo_alive == False or force_shutdown == True):

            rospy.loginfo("\n\nKilling gazebo.\n\n")
            self.launch_empty_world.shutdown()
            os.system("killall -9 gazebo & killall -9 gzserver & killall -9 gzclient")
            rospy.sleep(5)
            rospy.loginfo("\n\nDone killing gazebo.\n\n")
            
            rospy.loginfo("\n\nLaunching gazebo.\n\n")
            self.launch_empty_world = roslaunch.parent.ROSLaunchParent(self.uuid, self.empty_world, force_screen=True, verbose=True)
            self.launch_empty_world.start()
            rospy.sleep(20)
            rospy.loginfo("\n\nDone launching gazebo.\n\n")
            force_shutdown = False
        

    # Gets activated if starting/resetting/closing the simulation are requested and calls the appropriate functions
    def waitForLaunchNodesRequests(self):

        while (1):
            # blocks till either of the services is requested
            toMainThread_lock.acquire()

            # checks which service is requested
            if (self.is_start_sim_requested == True):
                # starts the simulation
                self.startSim()
                self.is_start_sim_requested = False
                fromMainThread_lock.release()
            elif (self.is_reset_sim_requested == True):
                # resets the simulation
                self.resetSim()
                self.is_reset_sim_requested = False
                fromMainThread_lock.release()
            elif (self.is_close_sim_requested == True):
                # closes the simulation
                self.closeSim()
                self.is_close_sim_requested = False
                fromMainThread_lock.release()


    # Generates arguments to render a version of the vehciles' launch file 
    # NOTE #: should be changed if the environment is extended to handle more than 1 agent & 1 EV 
    def genTemplateArgs(self):


        one_racecar_one_ambulance_args = dict()

        ## Agent args ##

        # set agent's start point to be such that interaction between it and the EV within the episode is guaranteed -- for training purposes
        self.agent_start_x = random.randint(-18, -18)
        one_racecar_one_ambulance_args['agent_start_x'] = self.agent_start_x #TODO: (LATER) let min and max position depend on EV's starting/ening position and vehicles' max vel/acc -- current values are calculated for default values and are -18.5 to 12 but rounded for simplicity # -18, 12
        # select a random start lane for the agent
        #one_racecar_one_ambulance_args['agent_start_y'] = self.genRandLanePos()
        one_racecar_one_ambulance_args['agent_start_y'] = -0.2625
        # set the agent's max vel/acc as specified by the input parameters
        one_racecar_one_ambulance_args['agent_max_vel'] = self.agent_max_vel
        one_racecar_one_ambulance_args['agent_max_acc'] = self.agent_max_acc

        # set the agent's goal to be the same as the EV's goal but in the same lane as the agent's start lane
        one_racecar_one_ambulance_args['agent_goal_x'] = self.amb_goal_x
        one_racecar_one_ambulance_args['agent_goal_y'] = one_racecar_one_ambulance_args['agent_start_y']

        ## EV args ##

        # select a random start lane for the agent
        #one_racecar_one_ambulance_args['EV_start_y'] = self.genRandLanePos()
        one_racecar_one_ambulance_args['EV_start_y'] = -0.2625
        
        # set the agent's start position and max vel/acc as specified by the input parameters
        one_racecar_one_ambulance_args['EV_start_x'] = self.amb_start_x
        one_racecar_one_ambulance_args['EV_max_vel'] = self.emer_max_speed
        one_racecar_one_ambulance_args['EV_max_acc'] = self.emer_max_accel

        ## Communication range ##

        # set the vehicles' communication range as specified by the input parameter
        one_racecar_one_ambulance_args['comm_range'] = self.comm_range

        return one_racecar_one_ambulance_args


    # Generates a random lane number (hence a y-coordinate)
    # NOTE #: works with the default threeLanes world -- should be adusted if another world is used
    def genRandLanePos(self):
        '''  
         Lane number convention in the "threeLanes" world:
             y^  
              |--------------> x      lane number 0 
              |--------------> x      lane number 1
              |--------------> x      lane number 2 
        '''

        # generates a random lane number
        lane_num = random.randint(0, 2)

        # maps lane number to world y-coordinate
        if (lane_num == 0):
           return 0.2625
        elif (lane_num == 1):
           return -0.2625
        if (lane_num == 2):
           return -0.7875


    # Starts the simulation
    def startSim(self):

        # init
        self.episode_start_time = -1

        ## Gazebo launch ##

        # launch gazebo
        rospy.loginfo("\n\nLaunching gazebo.\n\n")
        self.launch_empty_world = roslaunch.parent.ROSLaunchParent(self.uuid, self.empty_world, force_screen=True, verbose=True)
        self.launch_empty_world.start()
        rospy.sleep(20)
        rospy.loginfo("\n\nDone launching gazebo.\n\n")

        # check that gazebo is launched successfully and keeps relaunching if it's not
        force_shutdown = False
        self.relaunchGazeboIfDead(force_shutdown)


        ## RViz launch ##

        # launch rviz (uncomment if needed)
        ###rospy.loginfo("\n\nLaunching rviz.\n\n")
        ###self.launch_rviz = roslaunch.parent.ROSLaunchParent(self.uuid, self.rviz, force_screen=True, verbose=True)
        ###self.launch_rviz.start()
        ###rospy.sleep(20)
        ###rospy.loginfo("\n\nDone launching rviz.\n\n")


        ## Vehicle's launch ##

        # generate arguments to render a version of the vehciles' launch file
        one_racecar_one_ambulance_args = self.genTemplateArgs()

        # render a version of the vehciles' launch file from its template
        with open(self.one_racecar_one_ambulance[0], "w") as fp:
            fp.writelines(self.one_racecar_one_ambulance_temp.render(data=one_racecar_one_ambulance_args))


        # launch the vehicles' launch file
        rospy.loginfo("\n\nLaunching one_racecar_one_ambulance.\n\n")
        self.launch_one_racecar_one_ambulance = roslaunch.parent.ROSLaunchParent(self.uuid, self.one_racecar_one_ambulance, force_screen=True, verbose=True)
        self.launch_one_racecar_one_ambulance.start()
        rospy.sleep(30)
        rospy.loginfo("\n\nDone launching one_racecar_one_ambulance.\n\n")


        # block till nodes launch and EV reaches max velocity
        simReady_lock.acquire() #TODO: might consider setting a timeout and pinging critical nodes
        rospy.loginfo("\n\nSimulation ready and EV has maximum velocity.\n\n")
        rospy.loginfo("\n\nSimulation ready and EV has maximum velocity.\n\n")
        rospy.loginfo("\n\nSimulation ready and EV has maximum velocity.\n\n")
        rospy.loginfo("\n\nSimulation ready and EV has maximum velocity.\n\n")


        # reset the episode start time
        self.episode_start_time = rospy.Time.now().secs

    
    # Resets the simulation
    def resetSim(self):

        # init
        self.episode_start_time = -1

        ## Shutdown vehicles' nodes and relaunch gazebo if needed ##

        # if gazebo nodes are dead
        if (self.is_gazebo_alive == False):
            
            # kill vehicles' nodes
            rospy.loginfo("\n\nKilling one_racecar_one_ambulance.\n\n")
            self.launch_one_racecar_one_ambulance.shutdown()
            rospy.sleep(40)
            rospy.loginfo("\n\nDone killing one_racecar_one_ambulance.\n\n")
        
            # kill and relaunch gazebo
            force_shutdown = True
            self.relaunchGazeboIfDead(force_shutdown)

        # if gazebo is still properly running 
        else:
            
            is_gazebo_relaunch_needed = False

            # delete all vehicle models in gazebo
            for vehicle_index in range((self.num_of_agents + self.num_of_EVs)):
                # start a parallel process to handle the deletion
                del_model_process = multiprocessing.Process(target=self.deleteModel, args=(vehicle_index,))
                del_model_process.start()
                
                # block till deletion finishes or till the specified timeout is reached
                del_model_process.join(7.0) # NOTE #: change timeout as appropriate
                
                # check if timeout occured
                if (del_model_process.is_alive() == True):
                    rospy.loginfo("\n\nTIMEOUT: model deletion took too long.\n\n")
                    # gazebo will need to be relaunched
                    is_gazebo_relaunch_needed = True
                    break


            rospy.loginfo("\n\nKilling one_racecar_one_ambulance.\n\n")
            self.launch_one_racecar_one_ambulance.shutdown()
            rospy.sleep(40)
            rospy.loginfo("\n\nDone killing one_racecar_one_ambulance.\n\n")

            # if gazebo needs to be relaunched
            if (is_gazebo_relaunch_needed == True):
                # kill and relaunch gazebo
                force_shutdown = True
                self.relaunchGazeboIfDead(force_shutdown)


        ## Relaunch vehicles' nodes ##

        # re-initialize necessary variables
        self.last_vehicle_ids = self.initIdsCombinedMsg()
        self.is_episode_done = 0
        self.is_activated = False

        # generate arguments to render a version of the vehciles' launch file
        one_racecar_one_ambulance_args = self.genTemplateArgs()

        # render a version of the vehciles' launch file from its template
        with open(self.one_racecar_one_ambulance[0], "w") as fp:
            fp.writelines(self.one_racecar_one_ambulance_temp.render(data=one_racecar_one_ambulance_args))

        # launch the vehicles' launch file
        rospy.loginfo("\n\nLaunching one_racecar_one_ambulance.\n\n")
        self.launch_one_racecar_one_ambulance = roslaunch.parent.ROSLaunchParent(self.uuid, self.one_racecar_one_ambulance, force_screen=True, verbose=True)
        self.launch_one_racecar_one_ambulance.start()
        rospy.sleep(30)
        rospy.loginfo("\n\nDone launching one_racecar_one_ambulance.\n\n")
        

        # block till nodes launch and EV reaches max velocity
        simReady_lock.acquire() #TODO: might consider setting a timeout and pinging critical nodes
        rospy.loginfo("\n\nSimulation ready and EV has maximum velocity.\n\n")
        rospy.loginfo("\n\nSimulation ready and EV has maximum velocity.\n\n")
        rospy.loginfo("\n\nSimulation ready and EV has maximum velocity.\n\n")
        rospy.loginfo("\n\nSimulation ready and EV has maximum velocity.\n\n")
        rospy.loginfo("\n\nSimulation ready and EV has maximum velocity.\n\n")


        # reset the episode start time
        self.episode_start_time = rospy.Time.now().secs


    # Closes the simulation
    def closeSim(self):

        rospy.loginfo("\n\nKilling one_racecar_one_ambulance.\n\n")
        self.launch_one_racecar_one_ambulance.shutdown()
        rospy.loginfo("\n\nDone killing one_racecar_one_ambulance.\n\n")

        rospy.loginfo("\n\nKilling gazebo.\n\n")
        self.launch_empty_world.shutdown()
        os.system("killall -9 gazebo & killall -9 gzserver & killall -9 gzclient")
        rospy.loginfo("\n\nDone killing gazebo.\n\n")

        ###rospy.loginfo("\n\nKilling rviz.\n\n")
        ###self.launch_rviz.shutdown()
        ###rospy.loginfo("\n\nDone killing rviz.\n\n")


    # Start simulation service callback
    def startSimCallback(self, req):

        resp = startSimResponse()

        # check that the requested number of agents and EVs = 1 & 1 to match the current version of the environment
        # NOTE #: change if the environment is extended to support more vehicles
        if (req.num_of_agents != 1):
            raise Exception("Current version only supports ONE SINGLE AGENT. Terminating.")
            resp.is_successful = False
            return resp

        if (req.num_of_EVs != 1):
            raise Exception("Current version only supports ONE EMERGENCY VEHICLE. Terminating.")
            resp.is_successful = False
            return resp

        # set simulation initialization variables
        self.num_of_agents = req.num_of_agents
        self.num_of_EVs = req.num_of_EVs
        self.is_episode_done = 0
        self.is_activated = False

        # indicate that starting simulation is requested so it gets handled by the main thread
        self.is_start_sim_requested = True
        toMainThread_lock.release()
        # block till simulation is started
        fromMainThread_lock.acquire()

        resp.is_successful = True
        return resp


    # Restet simulation service callback
    def resetSimCallback(self, req):

        resp = resetORcloseSimResponse()

        if (req.reset_close_sim == True):

            # indicate that resetting simulation is requested so it gets handled by the main thread
            self.is_reset_sim_requested = True
            toMainThread_lock.release()
            # block till simulation is reset
            fromMainThread_lock.acquire()

            resp.is_successful = True
            return resp

        else:
            resp.is_successful = False
            return resp


    # Close simulation service callback
    def closeSimCallback(self, req):

        resp = resetORcloseSimResponse()

        if (req.reset_close_sim == True):

            # indicate that closing simulation is requested so it gets handled by the main thread
            self.is_close_sim_requested = True
            toMainThread_lock.release()
            # block till simulation is closed
            fromMainThread_lock.acquire()

            resp.is_successful = True
            return resp

        else:
            resp.is_successful = False
            return resp


    # Getting agent's state service callback
    def statesServerCallback(self, req):

        resp = statesResponse()

        # return the state's components for the requested agent number as per the last received vehcile IDs
        # the state's components are: agent's velocity, agent's lane number, EV's velocity, EV's lane number and the relative longitudinal distance between them

        resp.agent_vel = max(self.last_vehicle_ids.ids[req.robot_num - 1].velocity, 0)
        resp.agent_lane = self.last_vehicle_ids.ids[req.robot_num - 1].lane_num

        resp.amb_vel = max(self.last_vehicle_ids.ids[self.EV_index].velocity, 0)
        resp.amb_lane = self.last_vehicle_ids.ids[self.EV_index].lane_num


        agent_x_pos = self.last_vehicle_ids.ids[req.robot_num - 1].x_position
        amb_x_pos = self.last_vehicle_ids.ids[self.EV_index].x_position

        resp.rel_amb_y = amb_x_pos - agent_x_pos #TODO: (LATER) misleading name: change to rel_amb_x 
     
        return resp

    # Calculating agent's reward service callback
    def rewardServerCallback(self, req):
        
        myreward = self.calc_reward(req.amb_last_velocity, req.execution_time)
        return rewardResponse(myreward)


    # Calculates agent's reward
    def calc_reward(self, amb_last_velocity, execution_time):

        '''
        :logic: Calculate reward to agent from current state
        :param amb_last_velocity: float, previous velocity the EV/ambulance had
        :return: reward (either step reward or final reward)
        :Notes:
        #Simulation Time is not allowed to continue after 20*optimal_time (20* time steps with ambulance at its maximum speed)
        '''

        if(self.is_episode_done and self.give_final_reward): # calculate a final reward

            number_of_time_steps = rospy.Time.now().secs - self.episode_start_time # time spent in episode so far

            # linear reward. y= mx +c. y: reward, x: ration between time achieved and optimal time. m: slope. c: y-intercept
            m = ( (self.max_final_reward - self.min_final_reward) *20 ) /19 # slope for straight line equation to calculate final reward
            c = self.max_final_reward - 1*m # c is y-intercept for the reward function equation #max_final_reward is the y for x = 1
            reward = m * (self.optimal_time/number_of_time_steps) + c
            #debug#print(f'c: {c}, m: {m}, steps: {number_of_time_steps}, optimal_time: {self.optimal_time}')
            return reward

        else: # calcualate a step reward
            steps_needed_to_halt = 30
            ration_of_halt_steps_to_total_steps = steps_needed_to_halt/(self.amb_goal_x - self.amb_start_x)

            m = (self.max_step_reward - self.min_step_reward)/(2 * self.emer_max_accel)  # slope for straight line equation to calculate step reward
            # debug # rospy.loginfo("M = : %f", m)
            # 2 * self.emer.max_accel since: = self.emer.max_accel - * self.emer.max_decel
            c = self.max_step_reward - self.emer_max_accel * m  # c is y-intercept for the reward function equation #max_step_reward is the y for x = 2 (max acceleration)
            # debug # rospy.loginfo("C = : %f", c)
            emer_curr_spd = self.last_vehicle_ids.ids[self.EV_index].velocity
            reward = m * (emer_curr_spd - amb_last_velocity) + c
            # divide by action execution time for normalization purposes
            reward = reward/execution_time
            #rospy.loginfo("Reward = : %f", reward)
            #debug#print(f'c: {c}, m: {m}, accel: {(self.emer.spd - amb_last_velocity)}')
            #rospy.loginfo("if condition = : %f", abs(amb_last_velocity-self.emer_max_speed))
            
            if (abs(amb_last_velocity-self.emer_max_speed) <= 0.01 ) and (abs(emer_curr_spd-self.emer_max_speed) <= 0.01):
            # since ambulance had maximum speed and speed did not change that much; unless we applied the code below.. the acceleration
            # will be wrongly assumed to be zero. Although the ambulance probably could have accelerated more, but this is its maximum velocity.
                reward = self.max_step_reward/execution_time #same reward as maximum acceleration # divide by action execution time for normalization purposes

            rospy.loginfo("Reward: %f", reward)
            return reward


    # Updates list of vehicle IDs and publishes on /RL/is_active_or_episode_done
    def idMsgsCallback(self, idMsgs):

        # extra check
        if (idMsgs.robot_num > (self.num_of_agents + self.num_of_EVs)):
            raise Exception("NUMBER OF VEHICLES IN ENVIRONMENT IS GREATER THAN EXPECTED!")            

        # update list of vehicle IDs
        self.last_vehicle_ids.header.stamp = rospy.Time.now()
        self.last_vehicle_ids.header.frame_id = "map"
        self.last_vehicle_ids.robot_num = idMsgs.robot_num

        self.last_vehicle_ids.ids[idMsgs.robot_num - 1] = idMsgs.id

        # if the received ID is that of an ambulance
        if (idMsgs.id.type.data == "ambulance"):
            # if this is the first time to save the ambulance's ID
            if (self.EV_index == -1):
                self.EV_index = idMsgs.robot_num - 1
                self.emer_max_speed = idMsgs.id.max_vel
                self.emer_max_accel = idMsgs.id.max_acc
                self.optimal_time = int(np.round((self.amb_goal_x - self.amb_start_x) / self.emer_max_speed))  # optimal number of time steps: number of time steps taken by ambulance to reqch goal at maximum speed
                self.max_time_steps = 20 * self.optimal_time

            # extra check
            elif (self.EV_index != (idMsgs.robot_num - 1)):
                raise Exception("NUMBER OF EMERGENCY VEHICLES IN ENVIRONMENT IS GREATER THAN EXPECTED!")

            # indicate that simulation is ready if EV almost reached max vel
            if (idMsgs.id.velocity >= (self.emer_max_speed - self.emer_max_accel)):
                if (simReady_lock.locked() == True):
                    simReady_lock.release()
                    rospy.loginfo("\n\nEV has maximum velocity now (Released lock).\n\n")
                    rospy.loginfo("\n\nEV has maximum velocity now (Released lock).\n\n")
                    rospy.loginfo("\n\nEV has maximum velocity now (Released lock).\n\n")
                    rospy.loginfo("\n\nEV has maximum velocity now (Released lock).\n\n")



        # check if episode is done or RL should be activated for the agent 
        # NOTE #: edit as appropriate if environment is extended to work with more agents
        self.are_we_done()
        self.is_RL_activated()

        # publish the areWeDone message
        msg = areWeDone()
        msg.is_activated = self.is_activated
        msg.is_episode_done = self.is_episode_done
        self.pub.publish(msg)


    # Checks if the episode is done
    def are_we_done(self):

        amb_abs_y = self.last_vehicle_ids.ids[self.EV_index].x_position #TODO: (LATER) misleading name: change to amb_abs_x 

        ## Reason 1: time steps >= max_time_steps ##

        # calculate time steps taken in episode so far
        time_step_number = rospy.Time.now().secs - self.episode_start_time
        #if((self.episode_start_time != -1) and (time_step_number >= 20)):
        if((self.episode_start_time != -1) and (time_step_number >= self.max_time_steps)):
            self.is_episode_done = 1
            return

        ## Reason 2: EV reached goal ##
        elif(amb_abs_y >= self.amb_goal_x - 1):
            self.is_episode_done = 2
            return 

        ## Reason 3: any agent reached goal ##
        for agent_index in range((self.num_of_agents + self.num_of_EVs)):
            if (agent_index != self.EV_index):
                agent_abs_y = self.last_vehicle_ids.ids[agent_index].x_position #TODO: (LATER) misleading name: change to agent_abs_x

                if (agent_abs_y > self.amb_goal_x):
                    self.is_episode_done = 3
                    return

        ## Reason 4: gazebo died ##
        if(self.is_gazebo_alive == False):
            self.is_episode_done = 4
            return

        if(((self.last_vehicle_ids.ids[0].x_position - self.agent_start_x) < 0.01) or ((self.last_vehicle_ids.ids[self.EV_index].x_position - self.amb_start_x) < 0.01)):
            self.is_episode_done = 4
            return

        ## 0: not done ##
        self.is_episode_done = 0
        return

 
    # Checks if RL model should be activated for the agent
    # NOTE #: edit as appropriate if environment is extended to work with more agents
    def is_RL_activated(self):

        for agent_index in range((self.num_of_agents + self.num_of_EVs)):
            if (agent_index != self.EV_index):

                agent_x_pos = self.last_vehicle_ids.ids[agent_index].x_position
                amb_x_pos = self.last_vehicle_ids.ids[self.EV_index].x_position
                rel_amb_x = amb_x_pos - agent_x_pos

                # check if EV is inside the agent's window specified by rel_amb_y_min and rel_amb_y_max
                if ((rel_amb_x < self.rel_amb_y_min) or (rel_amb_x > self.rel_amb_y_max)): #TODO: (LATER) misleading name: change to rel_amb_x_min 
                    self.is_activated = False
                else:
                    self.is_activated = True
        return


if __name__ == '__main__':
    rospy.init_node("clear_ev_route_basic_env")

    # create object of class ClearEVRouteBasicEnv
    cEVrbe = ClearEVRouteBasicEnv()

    # execute cEVrbe.closeSim function upon node shutdown
    rospy.on_shutdown(cEVrbe.closeSim)

    rospy.spin()
