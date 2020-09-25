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

class personFollwer:
    def __init__(self):
        # init node
        rospy.init_node('personFollwer')

        self.scanSub = rospy.Subscriber('/scan', )

