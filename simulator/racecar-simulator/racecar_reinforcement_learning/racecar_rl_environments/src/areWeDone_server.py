
#!/usr/bin/env python
import threading
import pdb
import rospy
from racecar_rl_environments.srv import areWeDone, areWeDoneRequest, areWeDoneResponse




class clear_ev_route_basic_env:
    
    def __init__(self):

        self.amb_abs_y = 0#TODO m3rfsh haytgabo mmnen
        self.step_number = 0 #TODO m3rfsh haytgabo mmnen

        self.max_steps = 2 # mawgoda fel reward server
        self.amb_goal_dist = 500
        self.emer_max_speed = 0.5 # mawgoda fel reward server

        self.count_ego_vehicles = 6 # size el array fel states server


    
    def addAy7aga(self,amb_last_velocity):
        amb_last_velocity = self.amb_last_velocity
        myHary = amb_last_velocity + self.emer_spd + 1
        return myHary

    def are_we_done(self, full_state, step_number):
        #full_state: currently not used since we have the ambulance object.

        #amb_abs_y = self.full_state[-1][-1] #Please refer to shape of full_state list in env.measure_full_state()
        amb_abs_y = self.amb_abs_y
        step_number = self.step_number


        #1: steps == max_steps-1
        if(step_number == self.max_steps-1):
            return 1
        #2: goal reached
        elif(amb_abs_y > self.amb_goal_dist - self.emer.max_speed-1 ):
            # DONE: Change NET file to have total distance = 511. Then we can have the condition to compare with 500 directly.
            return 2 #GOAL IS NOW 500-10-1 = 489 cells ahead. To avoid ambulance car eacaping

        for agent_index in range( self.count_ego_vehicles ):
            agent_abs_y = self.full_state[-1][agent_index] # #Please refer to shape of full_state list in env.measure_full_state
                                                            # hidden_state shape: [ agent_abs_y ... for vehicle in vehicles , amb_abs_y]
            if (agent_abs_y > self.amb_goal_dist - self.emer.max_speed - 1):
                return 3


    def serverCallback(self,req):
        #pdb.set_trace()
        
        self.amb_last_velocity =  req.amb_vel_1
        self.emer_spd = req.amb_vel_2

        #anaDa = self.addAy7aga(self.amb_last_velocity)

        
        myreward = self.calc_reward(self.amb_last_velocity, self.isDone,self.number_of_steps,self.max_final_reward,self.min_final_reward,self.max_step_reward, self.min_step_reward)
        
        return rewardResponse(myreward)

    def areWeDoneServer(self):
        server = rospy.Service('areWeDone', reward, self.serverCallback)




    


        
if __name__ == '__main__':
    rospy.init_node("areWeDone_server")

    cEVrbe = clear_ev_route_basic_env()
    cEVrbe.rewardServer()
    # debug # rospy.loginfo("after states server")
    #rospy.loginfo(cEVrbe.reward)
    rospy.spin()

"""