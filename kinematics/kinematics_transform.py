#!/usr/bin/env python
#Petr Busov
# This is the kinematics script that will transform the direction input into wheel ouptput

#import rospy
import numpy as np
import math
import rospy
import std_msgs.msg import Float32MultiArray
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
    rospy.loginfo("I recieved a message: %s"%message.data)
    rospy.loginfo("and converted twist into motor speed")
    #print(globalToWheel(localToGlobal(0, 5, 0, 1.5708)))

def localToGlobal(xdot, ydot, thetadot, theta):
    
    robot_frame = np.array([xdot, ydot, thetadot])
    transform = np.array([ [math.cos(theta), math.sin(theta), 0],
                           [-math.sin(theta), math.cos(theta), 0],
                           [0, 0, 1] ])

    q_vel = transform.dot(robot_frame)
    print(q_vel)
    return(q_vel)

def globalToWheel(qvel):
    xdotQ = qvel[0]
    ydotQ = qvel[1]
    thetadotQ = qvel[2]
    qframe = np.array([xdotQ, ydotQ, thetadotQ])
    transform = (1/R) * np.array([[-1, 1, (d1+d2)],
                                  [1, 1, -(d1+d2)],
                                  [-1, 1, -(d1+d2)],
                                  [1, 1, (d1+d2)]])
    wheels = transform.dot(qframe)
    return wheels

if __name__ == "__main__":
    rospy.init_node("simple_subscriber")
    subscriber()



