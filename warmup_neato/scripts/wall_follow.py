#!/usr/bin/env python3

"""
Neato behavior to move parallel to the nearest wall when approaching one
"""

import rospy, time, math
from std_msgs.msg import Int8MultiArray
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan


class WallFollow:
    def __init__(self):         
        rospy.init_node('wall_follower')
        self.target_distance = 1.0
        self.k = 0.5
        # TODO: Sweep a range of angles instead of testing one angle
        # Only follows from left side currently
        self.theta = 45
        self.d1, self.d2 = None, None

        rospy.Subscriber('/scan', LaserScan, self.process_scan)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def process_scan(self, msg):
        """ Populate distances attributes if laser scanner detects something"""
        d1 = msg.ranges[0 + self.theta]
        d2 = msg.ranges[90 + self.theta]
        if msg.ranges[0 + self.theta] > 0.0 and msg.ranges[90 + self.theta] > 0.0:
            self.d1 = d1
            self.d2 = d2
            print(self.d1, self.d2)

    def run(self):
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            # if self.distance_to_wall is not None:
            #     # compute error for proportional control
            #     error = self.distance_to_wall - self.target_distance
            #     m = Twist()
            #     m.linear.x = self.k*error
            #     self.pub.publish(m)
            if self.d1 is not None and self.d2 is not None:
                m = Twist()
                error = self.d1 - self.d2
                # Neato is parallel to the wall
                if abs(error) < .001:
                    m.angular.z = self.k * error
                    m.linear.x = .2
                    print('parallel!!!')
                # Neato is tilted more in the direction of either d1 or d2
                else:
                    m.angular.z = self.k * error
                self.pub.publish(m)

            r.sleep()

    def angle_normalize(z):
        """ convenience function to map an angle to the range [-pi,pi] """
        return math.atan2(math.sin(z), math.cos(z))

    def angle_diff(a, b):
        """ Calculates the difference between angle a and angle b (both should be in radians)
            the difference is always based on the closest rotation from angle a to angle b

            examples:
                angle_diff(.1,.2) -> -.1
                angle_diff(.1, 2*math.pi - .1) -> .2
                angle_diff(.1, .2+2*math.pi) -> -.1
        """
        a = angle_normalize(a)
        b = angle_normalize(b)

        d1 = a-b
        d2 = 2*math.pi - math.fabs(d1)
        if d1 > 0:
            d2 *= -1.0
        if math.fabs(d1) < math.fabs(d2):
            return d1
        else:
            return d2

if __name__ == "__main__":
    node = WallFollow()
    node.run()