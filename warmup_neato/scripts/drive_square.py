#!/usr/bin/env python3

# importing message stuff
from std_msgs.msg import Int8MultiArray
from geometry_msgs.msg import Twist, Vector3, Pose
from nav_msgs.msg import Odometry

import rospy
import time

class Square:
    def __init__(self):
        # init node         
        rospy.init_node('square_drive')
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.twist = Twist(Vector3(1,0,0), Vector3(0,0,0))
        self.pub.publish(self.twist)
        self.starting_point = (0,0,0)
        self.sidelength = 1
        self.coef = 1
        self.sub = rospy.Subscriber('/odom', Odometry, self.response)


    


    def run_odom(self):

        # setup sub to odom / imu data
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if distance_driven >= self.sidelength:
                # print('turning')
                self.turn()
            else: 
                # print('driving straight')
                self.drive_straight(distance_driven)
            r.sleep()
        

    def response(self, message):
        print(message.pose.pose.position) 
        # calc distance driven 
        distance_driven = self.get_distance(message.pose.pose.position)
        # print('distance: ', distance_driven)
        self.current_position = (message.pose.pose.position.x,message.pose.pose.position.y,0)


    def get_distance(self, position):
        # grab euclidean distance from starting point to current position 
         return ((position.x-self.starting_point[0])**2+(position.y-self.starting_point[1])**2)**.5


    def get_speed(self, distance_driven):
        speed = (self.sidelength-distance_driven)*(self.coef)
        # print('speed :', speed)
        return(speed)

    def drive_straight(self, distance):
        speed = self.get_speed(distance)
        self.twist = Twist(Vector3(speed,0,0), Vector3(0,0,0))
        self.pub.publish(self.twist)
        

    def turn(self):
        # self.twist = Twist(Vector3(0,0,0), Vector3(0,1,0))
        # self.pub.publish(self.twist)

        # reset starting point in here too
        pass

if __name__ == "__main__":
    mySquare = Square()
    mySquare.run_odom()