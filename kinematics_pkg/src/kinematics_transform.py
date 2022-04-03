#!/usr/bin/env python
#Petr Busov
# This is the kinematics script that will transform the direction input into wheel ouptput

#import rospy
import numpy as np
import math
import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
# half width in mm
d1 = 107
# half length in mm
d2 = 83
# wheel radius in mm
R = 40
# roller radius in mm
r = 7.5
# thetadot is rad/s
# speeds are mm/s

# input is x speed y speed and theta speed
# meaning speed sideways, forward, and the rate at which the robot turns


def subscriber():
    # topic, message type, callback function
    sub = rospy.Subscriber('cmd_vel', Twist, convert)
    rospy.spin()

def convert(message):
    rospy.loginfo("linear: %s"%message.linear)
    rospy.loginfo("angular: %s"%message.angular)
    rospy.loginfo("extracted x: %s"%message.linear.x)
    rospy.loginfo("extracted y: %s"%message.linear.y)
    rospy.loginfo("extracted theta: %s"%message.angular.z)
    rospy.loginfo("and converted twist into motor speed")
    rospy.loginfo(globalToWheel(localToGlobal(message.linear.x, message.linear.y, message.angular.z, 0)))

def localToGlobal(xdot, ydot, thetadot, theta):
    
    robot_frame = np.array([xdot, ydot, thetadot])
    transform = np.array([ [math.cos(theta), math.sin(theta), 0],
                           [-math.sin(theta), math.cos(theta), 0],
                           [0, 0, 1] ])

    q_vel = transform.dot(robot_frame)
    print("this is q_vel %s"%q_vel)
    return(q_vel)

def globalToWheel(qvel):
    
    transform = (1/R) * np.array([[-1, 1, (d1+d2)],
                                  [1, 1, -(d1+d2)],
                                  [-1, 1, -(d1+d2)],
                                  [1, 1, (d1+d2)]])
    wheels = transform.dot(qvel)
    print("this is wheels %s"%wheels)
    return wheels

if __name__ == "__main__":
    rospy.init_node("simple_subscriber")
    subscriber()



