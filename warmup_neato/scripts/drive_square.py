#!/usr/bin/env python3

# importing message stuff
from std_msgs.msg import Int8MultiArray, Float32
from geometry_msgs.msg import Twist, Vector3, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion

import rospy
import time
import math

class Square:
    def __init__(self):
        # init node
        rospy.init_node('square_drive')

        # publish first speed at 0
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.twist = Twist(Vector3(0,0,0), Vector3(0,0,0))
        self.pub.publish(self.twist)
        self.pub2 = rospy.Publisher('/theta', Float32, queue_size=10)

        # set some coef's
        self.sidelength = 1
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
        self.set_angle = self.currentAngle + math.pi/2
        self.sides_completed = 0
        
       

    


    def run_odom(self):
        # setup sub to odom / imu data
        r = rospy.Rate(10)
        while not rospy.is_shutdown() and self.sides_completed <= 3 :
            d = self.get_distance()
            print("distance: ", d)
            if d < self.sidelength:
                # print('driving forwards')
                # drive forwards

                # do some error stuff - should we do this within drive_straight()?
                error = abs((d**2-self.sidelength**2))**0.5
                self.motorSpeed = error*0.6
                self.drive_straight()
                
            else:
                # print('turning')

                theta = self.angle_diff(self.currentAngle, self.set_angle)
                # print(theta)
                if abs(theta)<.01:
                    self.reset_vals()
                    print("stop turning")

                self.pub2.publish(Float32(theta))
                self.turn(theta)
                # stop driving, turn right 90 deg, set new set_point as current pos
            self.pub.publish(self.twist)
            r.sleep()

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

    def reset_vals(self):
        
        # self.currentAngle = 0
        print(self.sides_completed)
        self.set_angle = self.angle_normalize(self.currentAngle + math.pi/2)
        self.twist.linear.x = 0
        self.twist.angular.z = 0

        self.sides_completed += 1
        if self.sides_completed == 1:
            self.set_point = (self.current_position[0], self.current_position[1])
        elif self.sides_completed == 2:
            self.set_point = (self.current_position[0], self.current_position[1])
        elif self.sides_completed == 3:
            self.set_point = (self.current_position[0], self.current_position[1])
        


        



        
        
    def response_imu(self, message):
        self.currentAngle = euler_from_quaternion((message.orientation.x, message.orientation.y,message.orientation.z,message.orientation.w))[2]

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

    # def get_angular_v(self):
    #     return( self.coef_a(90-self.currentAngle))

    # def get_speed(self, distance_driven):
    #     speed = (self.sidelength-distance_driven)*(self.coef)
    #     # print('speed :', speed)
    #     return(speed)

    def drive_straight(self):
        # set twist attribute based on speed, publish 
        self.twist.angular.z = 0
        self.twist.linear.x = self.motorSpeed
        self.pub.publish(self.twist)
        

    def turn(self, theta):
        # self.twist = Twist(Vector3(0,0,0), Vector3(0,1,0))
        # self.pub.publish(self.twist)

        # reset starting point in here too
        # print('current angle: ', self.currentAngle)
        self.twist.angular.z = -theta*0.5
        self.twist.linear.x = 0
        self.pub.publish(self.twist)

if __name__ == "__main__":
    mySquare = Square()
    mySquare.run_odom()