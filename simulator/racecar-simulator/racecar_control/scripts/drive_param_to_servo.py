#!/usr/bin/env python
import rospy
from race.msg import drive_param
from ackermann_msgs.msg import AckermannDriveStamped

# Publisher for AckermannDriveStamped msg
pub = rospy.Publisher('/vesc/ackermann_cmd_mux/input/teleop', AckermannDriveStamped, queue_size=5)

# Input data is drive_param message from topic /drive_parameters
def callback(data):
    msg = AckermannDriveStamped();
    msg.header.stamp = rospy.Time.now();
    msg.header.frame_id = "base_link";

    # velocity from the drive_param message
    msg.drive.speed = data.velocity;
    msg.drive.acceleration = 1;
    msg.drive.jerk = 1;
    # steering angle from the drive_param message
    msg.drive.steering_angle = data.angle
    msg.drive.steering_angle_velocity = 1

    # publish the message
    pub.publish(msg)

def listener():
    rospy.init_node('drive_param_listener', anonymous=True)
    rospy.Subscriber("/drive_parameters", drive_param, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
