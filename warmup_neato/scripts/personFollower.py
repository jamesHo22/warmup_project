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
        
class personFollwer:
    def __init__(self):
        # init node
        rospy.init_node('personFollwer')

        self.scanSub = rospy.Subscriber('/scan', )


if __name__ == "__main__":
    myPersonFollwer = personFollwer()
    rospy.spin()
    def move_towards(self, angle, distance):
        # given an angle and a distance from the base_link frame, the neato should aim to 
        # move in the right direction and close the gap. 
        # The function should allow for mid-run recalibration

        # self.POI = (meters away from neato, rad detected at)


        while not rospy.is_shutdown():

            if abs(self.POI[0])>.2:
                # drive straight at speed based on distance to drive 
                self.twist.linear.x = self.POI[0]*0.6
                self.pub.publish(self.twist)

            if abs(self.POI[1])>.01:
                # continue turning at angular speed based on angle (in rads) left to cover
                
                # is it - self.POI?  
                self.twist.angular.z = -self.POI[1]*0.5
                self.pub.publish(self.twist)






    # Helper Functions for angle calcs
    def angle_normalize(self, z):
        """ convenience function to map an angle to the range [-pi,pi] """
        return math.atan2(math.sin(z), math.cos(z))

    def angle_diff(self, a, b):
        """ Calculates the difference between angle a and angle b (both should be in radians)
                the difference is always based on the closest rotation from angle a to angle b

            examples:
                angle_diff(.1,.2) -> -.1
                angle_diff(.1, 2*math.pi - .1) -> .2
                angle_diff(.1, .2+2*math.pi) -> -.1
        """
        # a = self.angle_normalize(a)
        # b = self.angle_normalize(b)

        d1 = a-b
        d2 = 2*math.pi - math.fabs(d1)
        if d1 > 0:
            d2 *= -1.0
        if math.fabs(d1) < math.fabs(d2):
            return d1
        else:
            return d2
