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
        # Proportional control scaling factor
        self.k = 0.5
        # Sweep +/- theta on an xy axis for distance mesaurements
        self.theta = 45
        # List of tuples that contain pair of +/- theta angled distances
        self.left_distance_to_wall = []
        self.right_distance_to_wall = []
        self.distance_threshold = 1

        rospy.Subscriber('/scan', LaserScan, self.process_scan)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def process_scan(self, msg):
        """ Populate distances attributes if laser scanner detects something"""
        self.left_distance_to_wall = []
        self.right_distance_to_wall = []

        # Sweeps between 10 degree and theta degree
        for i in range(10, self.theta+1):
            l_d1 = msg.ranges[90 - i]
            l_d2 = msg.ranges[90 + i]
            r_d1 = msg.ranges[270 + i]
            r_d2 = msg.ranges[270 - i]
            if 0 < l_d1 < self.distance_threshold and 0 < l_d1 < self.distance_threshold:
                self.left_distance_to_wall.append((l_d1, l_d2))
            if 0 < r_d1 < self.distance_threshold and 0 < r_d1 < self.distance_threshold:
                self.right_distance_to_wall.append((r_d1, r_d2))

    def run(self):
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            l_diff, l_avg, r_diff, r_avg = 100, 100, 100, 100
            if len(self.left_distance_to_wall) > 0:
                l_diff, l_avg = self.process_distances(self.left_distance_to_wall)
            if len(self.right_distance_to_wall) > 0:
                r_diff, r_avg = self.process_distances(self.right_distance_to_wall)
            
            rotate = None
            # Determine the closest wall on left or right side
            if l_avg < r_avg:
                print(l_avg, r_avg)
                print('left wall priority')
                rotate = l_diff
            elif r_avg < l_avg:
                print(l_avg, r_avg)
                print('right wall priority')
                # Multiply by -1 to use the correct sign when caclulating angular z down the lines
                rotate = r_diff * -1

            if rotate is not None:
                m = Twist()
                # Neato is parallel to the wall
                if abs(rotate) < .01:
                    m.linear.x = .2
                    print('parallel!!!')
                # Neato is tilted more in the direction of either d1 or d2
                else:
                    if rotate > 0:
                        print('turning left', rotate)
                    else:
                        print('turning right', rotate)
                    m.angular.z = self.k * rotate
                self.pub.publish(m)

            r.sleep()

    def process_distances(self, distance_to_wall):
        # Average of all the differences between distance pairs
        dist_diff = 0
        # Average the distances to determine which side has closest wall
        dist_avg = 0
        for i, (d1, d2) in enumerate(distance_to_wall):
            dist_diff += (d1 - d2)
            dist_avg += d1 + d2
        dist_diff /= len(distance_to_wall)
        dist_avg /= (len(distance_to_wall) * 2)
        
        return dist_diff, dist_avg
    
    def linspace(self, start, stop, num):
        return [start + x * (stop - start) / (num - 1) for x in range(num)]

if __name__ == "__main__":
    node = WallFollow()
    node.run()

