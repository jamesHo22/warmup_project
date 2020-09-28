#!/usr/bin/env python3

"""
Neato behavior to move parallel to the nearest wall when approaching one
"""

import rospy, time, math
from std_msgs.msg import Int8MultiArray
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from helper import convert_pose_to_xy_and_theta, create_marker, neato_to_odom
from visualization_msgs.msg import Marker

class WallFollow:
    def __init__(self):         
        rospy.init_node('wall_follower')
        # Proportional control scaling factor
        self.k = 0.5
        # Sweep +/- theta on an xy axis for distance mesaurements
        self.theta = 30
        # List of tuples that contain pair of +/- theta angled distances
        self.left_distance_to_wall = []
        self.right_distance_to_wall = []
        self.forward_distance_to_wall = []
        self.distance_threshold = 1
        # Holds current state (wall at side, wall in front)
        self.state = self.moving_along_wall
        # Current side neato is considering the wall
        self.wall = ''
        self.wall_marker = None
        # Neato odom information
        self.x = 0
        self.y = 0
        self.yaw = 0

        rospy.Subscriber('/scan', LaserScan, self.process_scan)
        rospy.Subscriber('/odom', Odometry, self.process_odom)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)


    def process_scan(self, msg):
        """ Populate distances attributes if laser scanner detects something"""
        self.left_distance_to_wall = []
        self.right_distance_to_wall = []
        self.forward_distance_to_wall = []

        # Sweeps between 10 degree and theta degree
        for i in range(10, self.theta+1):
            l_d1 = msg.ranges[90 - i]
            l_d2 = msg.ranges[90 + i]
            r_d1 = msg.ranges[270 + i]
            r_d2 = msg.ranges[270 - i]
            # Scan between (1 and theta) and (-1 and -theta)
            f_d1 = msg.ranges[0 + i - 9]
            f_d2 = msg.ranges[0 - i + 8]
            if 0 < l_d1 < self.distance_threshold and 0 < l_d1 < self.distance_threshold:
                self.left_distance_to_wall.append((l_d1, l_d2))
            if 0 < r_d1 < self.distance_threshold and 0 < r_d1 < self.distance_threshold:
                self.right_distance_to_wall.append((r_d1, r_d2))
            if 0 < f_d1 < self.distance_threshold and 0 < f_d1 < self.distance_threshold:
                self.forward_distance_to_wall.append((f_d1, f_d2))
    
    def process_odom(self, msg):
        self.x, self.y, self.yaw = convert_pose_to_xy_and_theta(msg.pose.pose)

    def run(self):
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            self.state = self.state()


    def moving_along_wall(self):
        """ Implements moving along wall state. Finds closest wall along left or right side and 
            rotates neato parallel to the wall, moving forward """
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            l_diff, l_avg, r_diff, r_avg = 100, 100, 100, 100
            distance = 0
            if len(self.left_distance_to_wall) > 0:
                l_diff, l_avg = self.calculate_distances(self.left_distance_to_wall)
            if len(self.right_distance_to_wall) > 0:
                r_diff, r_avg = self.calculate_distances(self.right_distance_to_wall)
                        
            rotate = None
            # Determine the closest wall on left or right side
            if l_avg < r_avg:
                print('left wall priority')
                rotate = l_diff
                distance = l_avg
                self.wall = 'left'
            elif r_avg < l_avg:
                print('right wall priority')
                # Multiply by -1 to use the correct sign when caclulating angular z for cmd_vel
                rotate = r_diff * -1
                distance = -r_avg
                self.wall = 'right'

            if rotate is not None:
                m = Twist()
                # Neato is parallel to the wall
                if abs(rotate) < .01:
                    m.linear.x = .2
                    print('parallel!!!')

                    # Calculate wall point in odom coordinates
                    x, y = neato_to_odom(0, distance, self.x, self.y, self.yaw)
                    # Visualize the detected wall using a marker to rviz
                    self.wall_marker = create_marker("odom", "wall_follower", x, y, 0, r=1.0, g=0.6, b=0.6)
                    self.marker_pub.publish(self.wall_marker)
                # Neato is tilted more in the direction of either d1 or d2
                else:
                    if rotate > 0:
                        print('turning left', rotate)
                    else:
                        print('turning right', rotate)
                    # Rotate to an amount proportionate to the avg difference between range 
                    # scans above and below neato horizontal
                    m.angular.z = self.k * rotate
                self.vel_pub.publish(m)

            # Check if there is a wall in front of neato
            if len(self.forward_distance_to_wall) > 0:
                length = len(self.forward_distance_to_wall) * 2
                avg = sum([d1 + d2 for d1, d2 in self.forward_distance_to_wall]) / length
                print('forward object detected', avg)
                if avg < .6:
                    print('Detected forward wall!!')
                    return self.change_wall

            r.sleep()
    
    def change_wall(self):
        """ Implements changing wall state. Neato rotates to for a fixed length to trigger new wall """
        m = Twist()
        print('*****Changing wall*******')
        if self.wall == 'left':
            m.angular.z = -self.k
        else:
            m.angular.z = self.k
        self.vel_pub.publish(m)
        rospy.sleep(2)
        return self.moving_along_wall


    def calculate_distances(self, distance_to_wall):
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

if __name__ == "__main__":
    node = WallFollow()
    node.run()

