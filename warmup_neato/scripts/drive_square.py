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

        # publish first speed at 0
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.twist = Twist(Vector3(0,0,0), Vector3(0,0,0))
        self.pub.publish(self.twist)

        # set some coef's
        self.sidelength = 2
        self.coef = 1
        self.coef_a = 1

        # set your subscribers
        self.sub = rospy.Subscriber('/odom', Odometry, self.response_odom)
        self.subImu = rospy.Subscriber('/imu', Imu, self.response_imu)

        # set data
        self.set_point = (0, 0)
        self.motorSpeed = 0 
        self.current_position = (0, 0)
        self.currentAngle = 0
       

    


    def run_odom(self):
        # setup sub to odom / imu data
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            d = self.get_distance()
            if d < self.sidelength:

                self.twist.angular.z = 0

                # drive forwards
                self.drive_straight()

                # do some error stuff - should we do this within drive_straight()?
                error = abs((d**2-self.sidelength**2))**0.5
                self.motorSpeed = error

                # do some error checking
                # threshold_angle = .5
                # if (self.currentAngle > threshold
                
                
            else:
                self.twist.linear.x = 0
                # calculate the angle to increase by
                self.twist.angular.z = self.get_angular_v()
                
                    
                
                # stop driving, turn right 90 deg, set new set_point as current pos
            self.pub.publish(self.twist)
            r.sleep()
        
    def response_imu(self, message):
        self.currentAngle = message.orientation.w

    def response_odom(self, message):
        '''
        updates the current position class attribute
        '''
        self.current_position = (message.pose.pose.position.x, message.pose.pose.position.y,0)
        


    def get_distance(self):
        '''
        compares the current position class attribute to the last set_point to calculate distance from it
        '''
        # grab euclidean distance from starting point to current position 
        return ((self.current_position[0]-self.set_point[0])**2+(self.current_position[1]-self.set_point[1])**2)**.5

    def get_angular_v(self):
        return( self.coef_a(90-self.currentAngle))

    def get_speed(self, distance_driven):
        speed = (self.sidelength-distance_driven)*(self.coef)
        # print('speed :', speed)
        return(speed)

    def drive_straight(self, distance):
        # calculate required speed
        speed = self.get_speed(distance)
        # set twist attribute based on speed, publish 
        self.twist.linear.x = speed
        self.pub.publish(self.twist)
        

    def turn(self):
        # self.twist = Twist(Vector3(0,0,0), Vector3(0,1,0))
        # self.pub.publish(self.twist)

        # reset starting point in here too
        pass

if __name__ == "__main__":
    mySquare = Square()
    mySquare.run_odom()