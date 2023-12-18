
#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension
import numpy as np

def setPID():
    pub = rospy.Publisher("pid_constants", Float32MultiArray, queue_size = 10)
    rospy.init_node('pid_publisher')
    rate = rospy.Rate(10)

    while(not rospy.is_shutdown()):

        kp = float(input("Set kp: "))
        ki = float(input("Set ki: "))
        kd = float(input("set kd: "))
        constants = np.array([kp,ki,kd])
        msg = Float32MultiArray()
        msg.layout.data_offset = 0
        msg.layout.dim = [MultiArrayDimension()]
        msg.layout.dim[0].label = "constants"
        msg.layout.dim[0].size = 3
        msg.data = constants

        pub.publish(msg)
        rate.sleep



if __name__ == '__main__':
    setPID()
