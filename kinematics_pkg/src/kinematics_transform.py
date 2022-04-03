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
d1 = 10.70
# half length in cm
d2 = 8.30
# wheel radius in cm
R = 4.00
# roller radius in cm
r = 0.75
# thetadot is rad/s
# speeds are cm/s

# input is x speed y speed and theta speed
# meaning speed sideways, forward, and the rate at which the robot turns

np.set_printoptions(precision=3)
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
    rospy.loginfo(globalToWheel(np.array([message.linear.x, message.linear.y, message.angular.z])))
    publisher(globalToWheel(np.array([message.linear.x, message.linear.y, message.angular.z])))

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

    print("this is transform %s"%transform)
    wheels = transform.dot(qvel)
    print("this is wheels %s"%wheels)
    

    msg = Float32MultiArray()
    msg.layout.data_offset = 0
    msg.layout.dim = [MultiArrayDimension()]
    msg.layout.dim[0].label = "velocities"
    msg.layout.dim[0].size = 4
    msg.data = wheels
    print("this is the array to arduino %s"%msg.data)
    print(msg.data[0])
    print(msg.data[1])
    print(msg.data[2])
    print(msg.data[3])
    return msg.data

def publisher(message):
    pub = rospy.Pulisher('motor',Float32MultiArray,queue_size=50)
    rate = rospy.Rate(10)
    pub.publish(message)
    
if __name__ == "__main__":
    rospy.init_node("simple_subscriber")
    subscriber()



