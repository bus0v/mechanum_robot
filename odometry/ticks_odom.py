#!/usr/bin/env python
import rospy
import math
import tf2_ros
from std_msgs.msg import Int16MultiArray
from tf2_ros.Broadcaster import TransformBroadcaster
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from geometry import tf_conversions

class OdomTf:
    def __init__(self):

        rospy.init_node('ticks_reciever',anonymous = True)
        self.rate =  rospy.get_param("~rate",10.0)
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
        self.t_delta = rospy.Duration(1.0/self.rate)
        self.min_time = rospyTime.now() + self.t_delta
        self.last_time = rospy.Time.now()
        
        rospy.Subscriber("ticks",Int16MultiArray,ticks_reciever)
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size = 50)
        self.odom_broadcaster = TransformBroadcaster()

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
            time_elapsed = time_elapsed.to_sec()
            self.then = now
        #translate encoder ticks into distance each wheel rotated in cm
        self.fr_angular = (self.fr_ticks-self.fr_ticks_prev)/330 * 2 * self.R * math.pi
        self.fl_angular = (self.fl_ticks-self.fl_ticks_prev)/330 * 2 * self.R * math.pi
        self.bl_angular = (self.bl_ticks-self.bl_ticks_prev)/330 * 2 * self.R * math.pi
        self.br_angular = (self.br_ticks-self.br_ticks_prev)/330 * 2 * self.R * math.pi
        self.fr_ticks_prev = self.fr_ticks
        self.fl_ticks_prev = self.fl_ticks
        self.bl_ticks_prev = self.bl_ticks
        self.br_ticks_prev = self.br_ticks
        #calculate distances using forward kinematics in cmfor the robot in the local frame
        self.x = (self.fr_angular + self.fl_angular + self.bl_angular + self.br_angular) / 4
        self.y = (self.fr_angular - self.fl_angular + self.bl_angular - self.br_angular) / 4
        #calculate theta
        self.theta = (self.fr_angular - self.fl_angular - self.bl_angular + self.br_angular)  / (4*(self.d1+self.d2))
        #calculate speed
        self.x_dot = self.x/time_elapsed
        self.y_dot = self.y/time_elapsed
        self.theta_dot = self.theta/time_elapsed

        #Calculate distance moved total in the global frame
        self.x = self.x + (math.cos(self.theta) * self.x) - math.sin(self.theta) * self.y
        self.y = self.y + (math.sin(self.theta) * self.x) + math.cos(self.theta) * self.y
        self.theta = self.theta + self.theta

        #create quaternion since all odometry is 6 DOF
        quaternion = tf_conversions.transformations.quaternion_from_euler(0,0,self.theta)

        # publish transform over tf
        self.odom_broadcaster.SendTransform(
            (x,y,0.),
            quaternion,
            now,
            "base_link",
            "odom"
            )

        #publish odometry
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "odom"

        # set the pose
        odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*quaternion))
        odom.child_frame_id = "base_link"
        # publish linear and angular velocities of the robot
        odom.twist.twist = Twist(Vector3(self.x_dot,self.y_dot,0.0), Vector3(0,0,self.theta_dot))
        # publish the odometry information over ros
        self.odom_pub.publish(odom)
        

    def inverse_kinematics():
        self.x_dot = (self.fr_angular + self.fl_angular + self.bl_angular + self.br_angular) *self.R/4

if __name__ == '__main__':
    odomtf = OdomTf()
    odomtf.spin()
