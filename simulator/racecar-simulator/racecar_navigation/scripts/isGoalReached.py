#!/usr/bin/env python
import rospy
import tf.transformations
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from racecar_navigation.msg import BoolWithHeader
import message_filters


pub = rospy.Publisher('move_car/nav/goalReached', BoolWithHeader, queue_size=1)

class Reached:
    def __init__(self):
        self.nav_goal_position = Point()
        self.nav_feedback_position = Point()
        self.GoalReached = BoolWithHeader()
        self.last_nav_goal_position = Point()
        self.goal_reached = False
        
    
    def set_feedback(self, msg):
        self.nav_feedback_position = msg.pose.pose.position
        h = self.GoalReached.header
        h.stamp = rospy.Time.now()
        if self.goal_reached == False:
            if abs(self.nav_goal_position.x - self.nav_feedback_position.x) <= 1:
                if abs(self.nav_goal_position.y - self.nav_feedback_position.y) <= 1:
                    self.GoalReached.Bool = True
                    pub.publish(self.GoalReached)
                    self.goal_reached = True
        
            else:        
                self.GoalReached.Bool = False
                self.goal_reached = False
                pub.publish(self.GoalReached)
        else:
            #self.GoalReached.Bool = self.goal_reached
            pub.publish(self.GoalReached)


    def set_goal(self, msg):
        self.last_nav_goal_position = self.nav_goal_position
        self.nav_goal_position = msg.pose.position
        self.goal_reached = False

   
        

        
def listener():
    rospy.init_node('isGoalReached', anonymous=True)
    R = Reached()
    rospy.Subscriber("move_base_simple/goal", PoseStamped, R.set_goal)
    rospy.Subscriber("vesc/pf/pose/odom", Odometry, R.set_feedback)
    rospy.spin()

if __name__ == '__main__':
    listener()