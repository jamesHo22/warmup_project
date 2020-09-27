#!/usr/bin/env python3

# imports and stuff
from std_msgs.msg import Int8MultiArray, Float32
from geometry_msgs.msg import Twist, Vector3, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from tf.transformations import euler_from_quaternion

import rospy
import time
import math

class FiniteControl:

    def __init__(self):
        # init node
        rospy.init_node('finiteControl')

        self.state = self.driving_sentry_shape


    def driving_sentry_shape(self):
        # do some shit
        while not rospy.is_shutdown():
            
            if False: 
                return self.object_follow

    def object_follow(self):
        pass

    def run(self):
        rospy.sleep(1)
        while not rospy.is_shutdown():
            self.state=self.state() # set the state variable equal to function that is returned  






if __name__ == "__main__":
    myFiniteControl = FiniteControl()

    myFiniteControl.run()
