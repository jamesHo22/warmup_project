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

class PersonFollower:

    def __init__(self):
        # init node
        rospy.init_node('person_follower')

        # Point of Interest: (distance in meters, direction in radians) in 
        # polar coordinates in reference to the base link
        self.POI = (0,0) 
        self.twist = Twist(Vector3(0,0,0), Vector3(0,0,0))
        rospy.Subscriber('/scan', LaserScan, self.process_scan)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def process_scan(self, msg):
        """ Gets the closest point in scan's distance and angle """
        lidarPoints = msg.ranges
        minIndex = lidarPoints.index(min(lidarPoints))
        self.POI = (lidarPoints[minIndex], math.pi*minIndex/180)

        print(self.POI)
    
    def getPOI(self):
        print(minIndex)

    def sigmoid(self, x):
        return 1/(1+math.exp(-x))

    def run(self):
        # Given an angle and a distance from the base_link frame, the neato should aim to 
        # move in the right direction and close the gap. 
        # The function should allow for mid-run recalibration

        while not rospy.is_shutdown():
            # Checks if neato is close enough to person to stop
            if abs(self.POI[0]) <= .5:
                self.twist.linear.x = 0
                self.twist.angular.z = 0
                self.pub.publish(self.twist)

            else:
                # Checks if heading of neato is not in the direction of the POI
                if abs(self.POI[1]) > .1:
                    # Continue turning at angular speed based on angle (in rads) left to cover
                    
                    # is it - self.POI?  
                    if 0 < self.POI[1] <= math.pi:
                        self.twist.angular.z = self.sigmoid(self.POI[1]) * 0.6
                    else:
                        self.twist.angular.z = -self.sigmoid(self.POI[1]) * 0.6
                else:
                    # Drive straight at speed based on distance to drive
                    self.twist.linear.x = self.POI[0] * 0.5
                    self.twist.angular.z = 0

            self.pub.publish(self.twist)
        
            r.sleep()

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
            
if __name__ == "__main__":
    node = PersonFollower()
    node.run()