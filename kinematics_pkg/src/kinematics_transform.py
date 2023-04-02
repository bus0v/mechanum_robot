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

np.set_printoptions(precision = 3)
def subscriber():
    # topic, message type, callback function
    sub = rospy.Subscriber('cmd_vel', Twist, convert)
    rospy.spin()

def convert(message):
    rospy.loginfo(globalToWheel(np.array([message.linear.x, message.linear.y, message.angular.z])))
    publisher(globalToWheel(np.array([message.linear.y, message.linear.x, message.angular.z])))

def localToGlobal(xdot, ydot, thetadot, theta):

    robot_frame = np.array([xdot, ydot, thetadot])
    transform = np.array([[ math.cos(theta), math.sin(theta), 0],
                           [-math.sin(theta), math.cos(theta), 0],
                           [0, 0, 1] ])

    q_vel = transform.dot(robot_frame)
    print("this is q_vel %s"%q_vel)
    return(q_vel)

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
    print("this is the array to arduino %s"%msg.data)
    return msg

def publisher(message):
    pub = rospy.Publisher('motor',Float32MultiArray,queue_size=50)
    rate = rospy.Rate(10)
    pub.publish(message)

if __name__ == "__main__":
    rospy.init_node("Kinematics")
    subscriber()
