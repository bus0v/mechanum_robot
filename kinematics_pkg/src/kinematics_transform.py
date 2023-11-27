#!/usr/bin/env python
#Petr Busov
# This is the kinematics script that will transform the direction input into wheel ouptput

#import rospy
import numpy as np
import math
import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension
from geometry_msgs.msg import Twist
# half width in cm
d1 = 0.1070
# half length in cm
d2 = 0.0830
# wheel radius in cm
R = 0.04
# roller radius in cm

# thetadot is rad/s
# speeds are cm/s

# input is x speed y speed and theta speed
# meaning speed sideways, forward, and the rate at which the robot turns
class Kinematics:
    def __init__(self):
        self.sub = rospy.Subscriber('cmd_vel', Twist, convert)
        np.set_printoptions(precision = 3)
        self.pub = rospy.Publisher('motor',Float32MultiArray,queue_size=50)
        self.rate = rospy.Rate(10)

    def convert(message):
        rospy.loginfo(globalToWheel(np.array([message.linear.x, message.linear.y, message.angular.z])))
        self.pub.publish(globalToWheel(np.array([message.linear.y, message.linear.x, message.angular.z])))

    def globalToWheel(qvel):
        print("this is q_vel %s"%qvel)
    
        transform = np.array([ (-1, 1, (d1+d2)),
                                (1, 1, -(d1+d2)),
                                (-1, 1, -(d1+d2)),
                                (1, 1, (d1+d2)) ])
        wheels = transform.dot(qvel)
        msg = Float32MultiArray()
        msg.layout.data_offset = 0
        msg.layout.dim = [MultiArrayDimension()]
        msg.layout.dim[0].label = "velocities"
        msg.layout.dim[0].size = 4
        msg.data = wheels
        print("this is the array to arduino %s",%msg.data)
        return msg


if __name__ == "__main__":
    rospy.init_node("Kinematics")
    kinematics_node Kinematics()
    subscriber()
