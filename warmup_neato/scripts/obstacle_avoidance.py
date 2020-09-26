#!/usr/bin/env python3

# imports 
from std_msgs.msg import Int8MultiArray, Float32
from geometry_msgs.msg import Twist, Vector3, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from tf.transformations import euler_from_quaternion

import rospy
import time
import math
from heapq import nsmallest
# import numpy as np



class naive_avoid_osbtacles:
    def __init__(self):
        # init node
        rospy.init_node('obscale_avoider')

        # setup subscription to lidar scan /scan topic
        self.scanSub = rospy.Subscriber('/scan', LaserScan, self.recieveScan)

        # setup twist attribute and setup publishing to /cmd_vel topic
        self.twist = Twist(Vector3(0,0,0), Vector3(0,0,0))
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def recieveScan(self):
        # upon recieving scan values, set global var values 
        pass

    def operation(self):
        # perform new calcs in every run based on global lidar scan vars

        # determine if operation needs to be standard or different execution needs to be done 
        pass
        



class force_avoid_obstacles:
    def __init__(self):
        # init node
        rospy.init_node('obscale_avoider')

        # setup subscription to lidar scan /scan topic
        self.scanSub = rospy.Subscriber('/scan', LaserScan, self.recieveScan)

        # setup twist attribute and setup publishing to /cmd_vel topic
        self.twist = Twist(Vector3(0,0,0), Vector3(0,0,0))
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        self.POI = (0,0)
        self.lidarPoints = []

    def recieveScan(self):
         # upon recieving scan values, set global var values 
         self.lidarPoints = msg.ranges

    def operation(self):
        while not rospy.is_shutdown():
            # Find angles of negative forces
            angles = self.find_object_directions()

            # calculate net force based on angles walk through 
            net_force = 10*[1,0]
            for angle in angles: 
                net_force += -2*self.angle_to_vector(angle)

            # set POI based on calced 'net force' - AKA new vector direction 
            self.POI_from_net_force(net_force)

            # drive in direction of POI
            if abs(self.POI[1])>.1:
                # continue turning at angular speed based on angle (in rads) left to cover
                if 0 < self.POI[1] <= math.pi:
                    self.twist.angular.z = self.sigmoid(self.POI[1])*0.6
                else:
                    self.twist.angular.z = -self.sigmoid(self.POI[1])*0.6

                self.pub.publish(self.twist)

            else:
                # drive straight at speed based on distance to drive
                self.twist.linear.x = self.sigmoid(self.POI[0])*0.5
                self.twist.angular.z = 0
                self.pub.publish(self.twist)


    def find_object_directions(self):
        # return angles in which objects exist, based on global LIDAR vars 
        # AKA start by finding groupings of angles with similar distances
        # AKA find the centers of all peaks within the 360 degree scan  
        angles = []
        # this algo low key ass 
        smallest_distances = nsmallest(10, self.lidarPoints)
        for distance in smallest_distances:
            angles.append(self.lidarPoints.index(distance))

        # eliminate angle vals that are too close to eachother
        delete = []
        angles = sorted(angles)
        for index in range(1, len(angles)):
            if abs(angles[index] - angles[index-1])< 5:
                delete.append(index)
        
        for i in delete:
            del angles[i]
        # this algo high key ass 
        return angles


    def angle_to_vector(self):
        # return vector direction from angle in degrees
        pass

    def POI_from_net_force(self):
        # set global POI in format (meters, angle (in rads)) based on net_force input
        pass

    def sigmoid(self, x):
        return 1/(1+math.exp(-x))
        

    
        

if __name__ == "__main__":
    myObscaleAvoider = force_avoid_obstacles()
    myObscaleAvoider.operation()