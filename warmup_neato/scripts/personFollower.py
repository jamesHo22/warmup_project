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









def move_towards(self, angle, distance):
    # given an angle and a distance from the base_link frame, the neato should aim to 
    # move in the right direction and close the gap. 
    # The function should allow for recalibration (run off of ) 