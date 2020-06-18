#!/usr/bin/env python
import rospy
import tf.transformations
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from nav_msgs.msg import Path, Odometry
from racecar_navigation.msg import Lanes_Info

#This node publishes information about current and future lanes from move_base

pub1 = rospy.Publisher('move_car/nav/current_laneInfo', Lanes_Info, queue_size=10)
pub2 = rospy.Publisher('move_car/nav/future_laneInfo', Lanes_Info, queue_size=10)

class LanesInfo:
    def __init__(self):
        self.futurePosition = Point()
        self.currentPosition = Point()
        self.currentLanesInfo = Lanes_Info()
        self.currentLanesInfo.map_array = []
        self.futureLanesInfo = Lanes_Info()
        self.futureLanesInfo.map_array = []

    def future_pos(self, msg):
        self.futurePosition = (msg.poses[-1]).pose.position
        self.futureLanesInfo.map_array = self.which_lane(self.futurePosition)
        h = self.futureLanesInfo.header
        h.stamp = rospy.Time.now()
        pub2.publish(self.futureLanesInfo)

    def current_pos(self, msg):
        self.currentPosition = msg.pose.pose.position
        self.currentLanesInfo.map_array = self.which_lane(self.currentPosition)
        h = self.currentLanesInfo.header
        h.stamp = rospy.Time.now()
        pub1.publish(self.currentLanesInfo)

    def which_lane(self, position):
        x_pos = position.x
        y_pos = position.y
        z_pos = position.z
    
        if x_pos >= -50 and x_pos <= 50:
            XRegion = 0
        elif x_pos < -50:
            XRegion = -1
        elif x_pos > 50:
            XRegion = 1

        "Lanes: "
        "y: 0.5| Lane 0  |0| Lane 1 |-0.5| Lane 2 |-1 "
        #if (y_pos >= 0 and y_pos <= 0.5):
        #    Lane = 0
        #elif (y_pos >= -0.5 and y_pos <= 0):
        #    Lane = 1
        #elif (y_pos >= -1 and y_pos <= -0.5):
        #    Lane = 2
        #else:
        #    Lane = -1    

        # determining current lane
        if (y_pos >= 0):
            Lane = 0
        elif (y_pos < -0.525):
            Lane = 2;   
        else:
            Lane = 1
            
        
        map_array = [XRegion, Lane]
        return map_array
        

def listener():
    rospy.init_node('threeLanes_current_future_pos', anonymous=True)
    L = LanesInfo()
    rospy.Subscriber("move_base/TrajectoryPlannerROS/local_plan", Path, L.future_pos)
    rospy.Subscriber("/vesc/odom", Odometry, L.current_pos)
    rospy.spin()

if __name__ == '__main__':
    listener()
