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
            
if __name__ == "__main__":
    node = PersonFollower()
    node.run()