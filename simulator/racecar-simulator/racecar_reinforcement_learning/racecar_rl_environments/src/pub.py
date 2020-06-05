#!/usr/bin/env python
import rospy
from racecar_rl_environments.msg import areWeDone
from std_msgs.msg import Bool


class clear_ev_route_basic_env:
    
    def __init__(self):

        self.amb_abs_y = 0#TODO m3rfsh haytgabo mmnen
        self.step_number = 0 #TODO m3rfsh haytgabo mmnen

        self.max_steps = 2 # mawgoda fel reward server // not sure
        self.amb_goal_dist = 500
        self.emer_max_speed = 0.5 # mawgoda fel reward server

        self.count_ego_vehicles = 6 # size el array fel states server

        self.is_activated = 0
        self.is_episode_done = 0


    def talker():

        pub = rospy.Publisher('areWeDone', areWeDone)
        rospy.init_node('pub', anonymous=True)
        r = rospy.Rate(10) #10hz
        msg = areWeDone()

        msg.is_activated = self.is_activated
        msg.is_episode_done = self.is_episode_done

    
        while not rospy.is_shutdown():

            rospy.loginfo(msg)
            pub.publish(msg)
            r.sleep()




    def are_we_done(self, full_state, step_number):
            #full_state: currently not used since we have the ambulance object.

            #amb_abs_y = self.full_state[-1][-1] #Please refer to shape of full_state list in env.measure_full_state()
            amb_abs_y = self.amb_abs_y    

            #1: steps == max_steps-1
            if(step_number == self.max_steps-1):
                self.is_episode_done = 1
                return self.is_episode_done
            #2: goal reached
            elif(amb_abs_y > self.amb_goal_dist - self.emer_max_speed-1 ):
                # DONE: Change NET file to have total distance = 511. Then we can have the condition to compare with 500 directly.
                #return 2 #GOAL IS NOW 500-10-1 = 489 cells ahead. To avoid ambulance car eacaping
                self.is_episode_done = 2
                return self.is_episode_done 
            for agent_index in range( self.count_ego_vehicles ):
                agent_abs_y = self.full_state[-1][agent_index] # #Please refer to shape of full_state list in env.measure_full_state
                                                                # hidden_state shape: [ agent_abs_y ... for vehicle in vehicles , amb_abs_y]
                # msh 3arf hngeb el agent index ezay ? haytrbt mn states ?  
                if (agent_abs_y > self.amb_goal_dist - self.emer_max_speed - 1):
                    self.is_episode_done = 3
                    return self.is_episode_done



if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass