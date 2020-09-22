#!/usr/bin/env python3

# importing message stuff
from std_msgs.msg import Int8MultiArray
from geometry_msgs.msg import Twist, Vector3, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

import rospy
import time

class Square:
    def __init__(self):
        # init node         
        rospy.init_node('square_drive')
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.twist = Twist(Vector3(1,0,0), Vector3(0,0,0))
        self.pub.publish(self.twist)
        self.sidelength = 2
        self.coef = 1
        self.sub = rospy.Subscriber('/odom', Odometry, self.response)
        self.subImu = rospy.Subscriber('/imu', Imu, self.responseImu)
        self.current_position = (0, 0)
        self.setpoint = (0, 0)
        self.motorSpeed = 0 
       

    


    def run_odom(self):
        # setup sub to odom / imu data
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            d = self.get_distance()
            if d < self.sidelength:
                self.twist.angular.z = 0
                # keep driving forward
                # get motor speed
                error = abs((d**2-self.sidelength**2))**0.5
                self.motorSpeed = error
                self.currentAngle = 0
                
                
            else:
                self.twist.linear.x = 0
                # calculate the angle to increase by
                self.twist.angular.z = 1
                
                    
                
                # stop driving, turn right 90 deg, set new setpoint as current pos
            self.pub.publish(self.twist)
            r.sleep()
        
    def responseImu(self, message):
        self.currentAngle = message.orientation.w

    def response(self, message):
        '''
        updates the current position class attribute
        '''
        self.current_position = (message.pose.pose.position.x, message.pose.pose.position.y,0)
        


    def get_distance(self):
        '''
        compares the current position class attribute to the last setpoint to calculate distance from it
        '''
        # grab euclidean distance from starting point to current position 
        return ((self.current_position[0]-self.setpoint[0])**2+(self.current_position[1]-self.setpoint[1])**2)**.5


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