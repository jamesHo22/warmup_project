#!/usr/bin/env python3

# importing message stuff
from std_msgs.msg import Int8MultiArray
from geometry_msgs.msg import Twist, Vector3

import rospy
import time

class Square:
    def __init__(self):
        # init node         
        rospy.init_node('square_drive')
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.linear =Vector3(0,0,0)
        self.angular =Vector3(0,0,0)
        self.twist = Twist(self.linear, self.angular)
        # setup sub to odom / imu data
        # self.sub = rospy.Subscriber('/imu', _____, self.response())



    def run(self, speed):
        self.pub.publish(self.twist)
        # r.sleep()

    def response(self):
        self.turn()

    def drive_straight(self):
        pass

    def turn(self):
        self.linear = Vector3(0,0,0)
        self.angular = Vector3(0,1,0)
        self.twist = Twist(self.linear, self.angular)


        
