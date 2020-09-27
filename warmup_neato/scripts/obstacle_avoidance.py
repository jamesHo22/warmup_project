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
import numpy as np # L
from heapq import nsmallest


# referenced this lidar obstacle avoidance from git https://github.com/kostaskonkk/datmo

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

    def recieveScan(self, msg):
         # upon recieving scan values, set global var values 
         self.lidarPoints = msg.ranges
        #  print(self.lidarPoints)

    def operation(self):
        while not rospy.is_shutdown():
            main_coef = 10.0
            obstacle_coef = -5.0
            # Find angles, in radians, of obstacles / where to place negative forces
            angles = self.cluster_lidar()
            print('angles: ', angles)

            # angles = [math.pi/3] # test if angle straight in front

            # calculate net force based on angles walk through 
            net_force = main_coef*np.array([1.0,0.0])
            print('net_force to start: ', net_force)
            for angle in angles: 
                net_force += obstacle_coef*self.angle_to_vector(angle)

            print('net_force after sum: ', net_force)
            # set POI based on calced 'net force' - AKA new vector direction 
            self.POI_from_net_force(net_force)
            print('POI val', self.POI)

            # drive in direction of POI
            self.drive()
    


    def drive(self):
        if abs(self.POI[1])>.1:
            # continue turning at angular speed based on angle (in rads) left to cover
            if 0 < self.POI[1] <= math.pi:
                self.twist.angular.z = -self.sigmoid(self.POI[1])*0.6
            else:
                self.twist.angular.z = self.sigmoid(self.POI[1])*0.6

            self.pub.publish(self.twist)

        else:
            # drive straight at speed based on distance to drive
            self.twist.linear.x = self.sigmoid(self.POI[0])*0.5
            self.twist.angular.z = 0
            self.pub.publish(self.twist)

    def find_object_directions(self):
        # 
        # return angles in which objects exist, based on global LIDAR vars 
        # AKA start by finding groupings of angles with similar distances
        # AKA find the centers of all peaks within the 360 degree scan  
        angles = []
        # this algo low key ass 
        smallest_distances = nsmallest(10, self.lidarPoints)
        for distance in smallest_distances:
            angles.append(self.lidarPoints.index(distance))

        # eliminate angle vals that are too close to eachother

        # compare the distances, not angles 
        delete = []
        angles = sorted(angles)
        for index in range(1, len(angles)):
            if abs(angles[index] - angles[index-1])< 5:
                delete.append(index)
        
        for i in delete:
            del angles[i]
        # this algo high key ass 
        return angles



    def cluster_lidar(self):
        # handle case where lidar points does not exist yet
        if len(self.lidarPoints)==0:
            return []
        elif all(elem == math.inf for elem in self.lidarPoints): # handles case where there are no obstacles
            return []
        else:
            # walks through all differences in distances
            threshold_distance = .5 # hardcoded - but in theory should use the Adaptive Breakpoint Detector algorithm
            naive_cluster_edges = [0] # list of starts and ends of groupings
            cluster_centers = []
            # self.lidarPoints = [math.inf, math.inf, math.inf, 20, 20.1, 20.2, 20.4,91,91.3] # for testing
            for i in range(1,len(self.lidarPoints)):
                val = abs(self.lidarPoints[i]-self.lidarPoints[i-1])
                if (val>threshold_distance):
                    # new cluster detected 
                    naive_cluster_edges.append(i-1)
                    naive_cluster_edges.append(i)

            naive_cluster_edges.append(len(self.lidarPoints)-1) # add in final index in lidar array
            naive_cluster_edges = [i for n, i in enumerate(naive_cluster_edges) if i not in naive_cluster_edges[:n]] # remove duplicates
            print('NCE: ', naive_cluster_edges)
            for i in range(1,len(naive_cluster_edges)):
                # assume that each pair of edges is a cluster 
                # include cluster only if it represents an actual object
                print('degrees we are comparing: ', naive_cluster_edges[i-1], 'and ', naive_cluster_edges[i])
                if not (math.isinf(self.lidarPoints[naive_cluster_edges[i-1]]) and math.isinf(self.lidarPoints[naive_cluster_edges[i]])):
                    # grab the 'centers' of these clusters
                    cluster_centers.append((naive_cluster_edges[i-1] + naive_cluster_edges[i])/2) 
            
            print('angles: ', cluster_centers)
            # returns angles of cluster centers in radians
            return list(map(self.deg2rad, cluster_centers))

    def deg2rad(self,angle):
        return(np.deg2rad(angle))

    def angle_to_vector(self, angle):
        # return vector direction with mag 1 from angle in degrees
        return(np.array([math.cos(angle), math.sin(angle)]))
        

    def POI_from_net_force(self, net_Force):
        # set global POI in format (meters, angle (in rads)) based on net_force vector input
        self.POI = [((net_Force[0] ** 2 + net_Force[1] ** 2) ** 0.5), math.atan(net_Force[1]/net_Force[0])]

    def sigmoid(self, x):
        return 1/(1+math.exp(-x))
        




# naive obstacle avoidance code 

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
        

if __name__ == "__main__":
    myObscaleAvoider = force_avoid_obstacles()
    myObscaleAvoider.operation()