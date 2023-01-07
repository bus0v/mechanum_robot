#!/usr/bin/env python
import rospy
import math
import tf2_ros
from std_msgs.msg import Int16MultiArray
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Point
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import tf_conversions
import geometry_msgs.msg

class OdomTf:
    def __init__(self):
        # initialise node
        rospy.init_node('odometryTF',anonymous = True)
        self.rate =  rospy.get_param("~rate",0.5)
        # define variables
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
        self.fr_vel = 0.0
        self.fl_vel = 0.0
        self.bl_vel = 0.0
        self.br_vel = 0.0

        self.fr_ticks_prev = 0.0
        self.fl_ticks_prev = 0.0
        self.bl_ticks_prev = 0.0
        self.br_ticks_prev = 0.0

        self.fr_distance = 0.0
        self.fl_distance = 0.0
        self.bl_distance = 0.0
        self.br_distance = 0.0

        # parameters
        self.t_delta = rospy.Duration(1.0/self.rate)
        self.min_time = rospy.Time.now() + self.t_delta
        self.last_time = rospy.Time.now()
        self.sub_ticks = rospy.Subscriber("ticks",Int16MultiArray, self.ticks_receiver)
        #self.sub_vel = rospy.Subscriber("v_filtered",Int16MultiArray, self.vel_receiver)
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size = 50)
        self.odom_broadcaster = tf2_ros.TransformBroadcaster()
        rospy.loginfo("Initilization complete")

    def spin(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            rate.sleep()

    def ticks_receiver(self, message):
        #recieve and record encoder ticks
        rospy.loginfo("I got ticks")
        if message.data:
            if message.data[0]!=0:
                self.fr_ticks = message.data[0]
            if message.data[1]!=0:
                self.fl_ticks = message.data[1]
            if message.data[2]!=0:
                self.bl_ticks = message.data[2]
            if message.data[3]!=0:
                self.br_ticks = message.data[3]
        
    def vel_receiver(self, message):
        rospy.loginfo("I got speed")
        self.fr_vel = message.data[0]
        self.fl_vel = message.data[1]
        self.bl_vel = message.data[2]
        self.br_vel = message.data[3]

    def update(self):
        now = rospy.Time.now()
        if now > self.min_time:
            time_elapsed = now - self.last_time
            time_elapsed = time_elapsed.to_sec()
            self.then = now
            #translate encoder ticks into distance each wheel rotated in cm
            rospy.loginfo("\n fr_ticks: %.2f",self.fr_ticks)
            rospy.loginfo("\nfr_prev: %d ",self.fr_ticks_prev)
            # make an if statement here that doesn't calcultate stuff if nothing happening
            # if fr_ticks-fr_ticks_prev !=0 then do all this
            self.fr_distance = (self.fr_ticks-self.fr_ticks_prev)/330 * 2 * self.R * math.pi
            self.fl_distance = (self.fl_ticks-self.fl_ticks_prev)/330 * 2 * self.R * math.pi
            self.bl_distance = (self.bl_ticks-self.bl_ticks_prev)/330 * 2 * self.R * math.pi
            self.br_distance = (self.br_ticks-self.br_ticks_prev)/330 * 2 * self.R * math.pi
            rospy.loginfo("\nfr_rotation: %.2f \nfl_rotation: %.2f",self.fr_distance,self.fl_distance)
            #rospy.loginfo("\nbr_rotation: %d \nbl_rotation: %d",self.br_distance,self.bl_distance)
            self.fr_ticks_prev = self.fr_ticks
            self.fl_ticks_prev = self.fl_ticks
            self.bl_ticks_prev = self.bl_ticks
            self.br_ticks_prev = self.br_ticks
            #calculate distances using forward kinematics in cm for the robot in the local frame
            self.x_traveled = (self.fr_distance + self.fl_distance + self.bl_distance + self.br_distance) / 4
            self.y_traveled = (-self.fr_distance + self.fl_distance - self.bl_distance + self.br_distance) / 4
            rospy.loginfo("\nx_traveled: %.2f ",self.x_traveled)
            rospy.loginfo("\ny_traveled: %.2f ",self.y_traveled)
            #calculate theta
            self.theta_traveled = (self.fr_distance - self.fl_distance - self.bl_distance + self.br_distance)  / (4*(self.d1+self.d2))
            #rospy.loginfo("\ntheta: %d ",self.theta)
            #calculate speed
            self.x_dot = self.x_traveled/time_elapsed
            self.y_dot = self.y_traveled/time_elapsed
            self.theta_dot = self.theta_traveled/time_elapsed
            self.last_time = rospy.Time.now()
            #Calculate distance moved total in the global frame
            self.theta = self.theta + self.theta_traveled
            self.x = self.x + (math.cos(self.theta) * self.x_traveled) - math.sin(self.theta) * self.y_traveled
            self.y = self.y + (math.sin(self.theta) * self.x_traveled) + math.cos(self.theta) * self.y_traveled
            rospy.loginfo("\nx_total: %.2f ",self.x)
            rospy.loginfo("\ny_total: %.2f ",self.y)


            #create quaternion since all odometry is 6 DOF
            quaternion = tf_conversions.transformations.quaternion_from_euler(0,0,self.theta)
            #create transform
            trans_stamped = geometry_msgs.msg.TransformStamped()
            trans_stamped.header.stamp = now
            trans_stamped.header.frame_id = "base_link"
            trans_stamped.child_frame_id = "odom"
            trans_stamped.transform.translation.x = self.x
            trans_stamped.transform.translation.y = self.y
            trans_stamped.transform.translation.z = 0.0
            #rospy.loginfo("\nX distance traveled: %d \nY distance %d", self.x, self.y)
            trans_stamped.transform.rotation.x = quaternion[0]
            trans_stamped.transform.rotation.y = quaternion[1]
            trans_stamped.transform.rotation.z = quaternion[2]
            trans_stamped.transform.rotation.w = quaternion[3]

            # publish transform over tf
            self.odom_broadcaster.sendTransform(trans_stamped)

            #publish odometry
            odom = Odometry()
            odom.header.stamp = now
            odom.header.frame_id = "odom"

            # set the pose
            odom.pose.pose = Pose(Point(self.x, self.y, 0.0), Quaternion(*quaternion))
            odom.child_frame_id = "base_link"
            # publish linear and distance velocities of the robot
            odom.twist.twist = Twist(Vector3(self.x_dot,self.y_dot,0.0), Vector3(0,0,self.theta_dot))
            # publish the odometry information over ros
            self.odom_pub.publish(odom)


if __name__ == '__main__':
    odomtf = OdomTf()
    odomtf.spin()
