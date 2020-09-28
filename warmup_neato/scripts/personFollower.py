#!/usr/bin/env python3

# importing message stuff
from std_msgs.msg import Int8MultiArray, Float32
from geometry_msgs.msg import Twist, Vector3, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from tf.transformations import euler_from_quaternion
from visualization_msgs.msg import Marker

import rospy
import math
import helper

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
        self.marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
        self.person_marker = helper.create_marker("base_link", "person_follow", 0, 0, 0.2)

    def process_scan(self, msg):
        """ Gets the closest point in scan's distance and angle and sets it to the POI"""
        lidarPoints = msg.ranges
        minIndex = lidarPoints.index(min(lidarPoints))
        self.POI = (lidarPoints[minIndex], math.pi*minIndex/180)

    def run(self):
        # Given an angle and a distance from the base_link frame, the neato should aim to 
        # move in the right direction and close the gap. 
        # The function should allow for mid-run recalibration
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            x = self.POI[0]*math.cos(self.POI[1])
            y = self.POI[0]*math.sin(self.POI[1])

            self.person_marker.pose.position.x = x
            self.person_marker.pose.position.y = y
            self.marker_pub.publish(self.person_marker)
            # Checks if neato is close enough to person to stop
            if abs(self.POI[0]) <= .5:
                self.twist.linear.x = 0
                self.twist.angular.z = 0
                self.pub.publish(self.twist)

            else:
                # Checks if heading of neato is not in the direction of the POI
                if abs(self.POI[1]) > .1:
                    # Continue turning at angular speed based on angle (in rads) left to cover
                    # We use a sigmoid function function to scale the motor speeds to between 0 and 1*0.6
                    if 0 < self.POI[1] <= math.pi:
                        self.twist.angular.z = helper.sigmoid(self.POI[1]) * 0.6
                    else:
                        self.twist.angular.z = -helper.sigmoid(self.POI[1]) * 0.6
                else:
                    # Drive straight at speed based on distance to drive
                    self.twist.linear.x = self.POI[0] * 0.5
                    self.twist.angular.z = 0

            self.pub.publish(self.twist)
                       
        
            r.sleep()                 
            
if __name__ == "__main__":
    node = PersonFollower()
    node.run()