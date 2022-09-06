#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import Int16MultiArray

#define variables
# half width in cm
d1 = 10.70
# half length in cm
d2 = 8.30
# wheel radius in cm
R = 4.00
# roller radius in cm
r = 0.75
#ticks per rotation = 11(ticks) * 30 gear ratio
tpr = 330

x = 0.0
y = 0.0
theta = 0.0
x_dot = 0.0
y_dot = 0.0
theta_dot = 0.0

fr_ticks = 0.0
fl_ticks = 0.0
bl_ticks = 0.0
br_ticks = 0.0

def callback(message):
    #recieve and record encoder ticks
    global fr_ticks, fl_ticks, bl_ticks, br_ticks
    fr_ticks = message.data[0]
    fl_ticks = message.data[1]
    bl_ticks = message.data[2]
    br_ticks = message.data[3]


def listener():
    rospy.init_node('ticks_reciever',anonymous = True)

    rospy.Subscriber("ticks",Int16MultiArray,callback)

    rospy.spin()

if __name__ = '__main__':
    listener()
    #translate encoder ticks into distance
    fr_distance = fr_ticks/330 * R *2 * math.pi
    fl_distance = fl_ticks/330 * R *2 * math.pi
    bl_distance = bl_ticks/330 * R *2 * math.pi
    br_distance = br_ticks/330 * R *2 * math.pi
    current_time = rospy.Time.now()
    last_time = rospy.Time.now()
    #calculate velocities usinf forward kinematics

    #calculate theta

    #calculate speed

    #publish theta

    #publish twist or something
