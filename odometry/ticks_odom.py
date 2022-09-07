#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import Int16MultiArray

class OdomTf:
    def __init__(self):
        #define variables
        # half width in cm
        self.d1 = 10.70
        # half length in cm
        self.d2 = 8.30
        # wheel radius in cm
        self.R = 4.00
        # roller radius in cm
        self.r = 0.75
        #ticks per rotation = 11(ticks) * 30 gear ratio
        self.tpr = 330

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.x_dot = 0.0
        self.y_dot = 0.0
        self.theta_dot = 0.0

        self.fr_ticks = 0.0
        self.fl_ticks = 0.0
        self.bl_ticks = 0.0
        self.br_ticks = 0.0

        self.fr_ticks_prev = 0.0
        self.fl_ticks_prev = 0.0
        self.bl_ticks_prev = 0.0
        self.br_ticks_prev = 0.0

        self.fr_angular = 0.0
        self.fl_angular = 0.0
        self.bl_angular = 0.0
        self.br_angular = 0.0
        # parameters
        self.rate =  rospy.get_param("~rate",10.0)
        self.t_delta = rospy.Duration(1.0/self.rate)
        self.min_time = rospyTime.now() + self.t_delta

        self.last_time = rospy.Time.now()
        rospy.init_node('ticks_reciever',anonymous = True)
        rospy.Subscriber("ticks",Int16MultiArray,ticks_reciever)
        rospy.spin()

    def spin(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()

    def ticks_reciever(message):
        #recieve and record encoder ticks
        self.fr_ticks = message.data[0]
        self.fl_ticks = message.data[1]
        self.bl_ticks = message.data[2]
        self.br_ticks = message.data[3]


    def update(self):
        now = rospy.Time.now()
        if now > self.min_time:
            time_elapsed = now - self.last_time
            time_elapsed.to_sec()
            self.then = now

        self.fr_angular = (self.fr_ticks-self.fr_ticks_prev)/330
        self.fl_angular = (self.fl_ticks-self.fl_ticks_prev)/330
        self.bl_angular = (self.bl_ticks-self.bl_ticks_prev)/330
        self.br_angular = (self.br_ticks-self.br_ticks_prev)/330
        self.fr_ticks_prev = self.fr_ticks
        self.fl_ticks_prev = self.fl_ticks
        self.bl_ticks_prev = self.bl_ticks
        self.br_ticks_prev = self.br_ticks

    def inverse_kinematics():
        self.x_dot = (self.fr_angular + self.fl_angular + self.bl_angular + self.br_angular) *self.R/4
if __name__ = '__main__':
    odomtf = OdomTf()
    odomtf.spin()
    #translate encoder ticks into distance

    #calculate velocities using forward kinematics

    #calculate theta

    #calculate speed

    #publish theta

    #publish twist or something
