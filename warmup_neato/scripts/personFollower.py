#!/usr/bin/env python3

# importing message stuff
from std_msgs.msg import Int8MultiArray, Float32
from geometry_msgs.msg import Twist, Vector3, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from tf.transformations import euler_from_quaternion

import rospy
import time
import math
import numpy as np

class personFollwer:
    def __init__(self):
        # init node
        rospy.init_node('personFollwer')

        self.scanSub = rospy.Subscriber('/scan', LaserScan, self.recieveScan)
        self.POI = (0,0) # (distance in meters, direction in radians) in polar coordinates in reference to the base link
        self.lidarPoints = []
        # driving
        self.twist = Twist(Vector3(0,0,0), Vector3(0,0,0))
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def recieveScan(self, msg):
        self.lidarPoints = msg.ranges
    
    def getPOI(self):
        minDist = min(self.lidarPoints)
        self.lidarPoints.index()
        



if __name__ == "__main__":
    myPersonFollwer = personFollwer()
    rospy.spin()
