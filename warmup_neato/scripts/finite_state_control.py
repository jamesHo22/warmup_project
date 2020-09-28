#!/usr/bin/env python3

# imports and stuff
import rospy, time, math
from std_msgs.msg import Int8MultiArray, Float32
from geometry_msgs.msg import Twist, Vector3, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from helper import convert_pose_to_xy_and_theta, create_marker, \
angle_diff, angle_normalize, sigmoid

class FiniteControl:

    def __init__(self):
        rospy.init_node('finite_control')

        self.tracking_range = 2
        self.angular_speed = .5
        self.state = self.seeking
        # Point of interest, the point to follow
        self.POI = (None, None)
        self.current_position = (0, 0)
        self.current_angle = 0
        # Angle at which the seeking motion starts
        self.starting_angle = self.current_angle
        self.seek_theta = math.radians(60)
        self.seek_step = math.radians(5)

        self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        rospy.Subscriber('scan', LaserScan, self.process_scan)
        rospy.Subscriber('/odom', Odometry, self.process_odom)
        self.marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)

    def seeking(self):
        """ Implements the seeking behavior. Rotates back and forth and 
        scans for a person to follow that comes within the tracing range. """
        r = rospy.Rate(10)
        print('*********Seeking*********')
        while not rospy.is_shutdown():
            # Reached the end of seeking theta, seek the other direction now
            if self.current_angle > self.starting_angle + self.seek_theta or \
            self.current_angle < self.starting_angle - self.seek_theta:
                self.starting_angle = self.current_angle
                self.seek_step *= -1
                print('turn seek around')
                
            # Rotate in seek step increments
            self.rotate_by_angle(self.seek_step)

            # Detects something within tracking range
            # TODO: Use a mass of points, instead of one
            if self.POI[0] is not None: 
                return self.person_follow

            r.sleep()
    
    def rotate_by_angle(self, theta):
        """ Rotates the neato by a specified angle """
        m = Twist()
        # Because neato coordinate system increases counter clockwise, positive
        # angular values turn left, negative angular values turn right
        m.angular.z = self.angular_speed * -1 if theta < 0 else 1
        target_angle = angle_normalize(self.current_angle + theta)
        diff = angle_diff(self.current_angle, target_angle)
        r = rospy.Rate(10)
        # Keep on turning as long as the current angle and target angle are not within a threshold
        while not rospy.is_shutdown() and abs(diff) > .05:
            self.vel_pub.publish(m)
            diff = angle_diff(self.current_angle, target_angle)
            r.sleep()
        m.angular.z = 0
        self.vel_pub.publish(m)

    def process_odom(self, msg):
        x, y, self.current_angle = convert_pose_to_xy_and_theta(msg.pose.pose)
        self.current_position = (x, y)

    def process_scan(self, msg):
        """ Gets the closest point in scan's distance and angle within it's tracking area (front of scan) """
        # Only grabs lidar data in front half
        lidar_0_90 = msg.ranges[0:90]
        lidar_270_end = msg.ranges[270:]
        minIndex = None
        minDistance = None
        if min(lidar_0_90) < min(lidar_270_end):
            minDistance = min(lidar_0_90)
            minIndex = lidar_0_90.index(minDistance)
        else:
            minDistance = min(lidar_270_end)
            minIndex = lidar_270_end.index(minDistance) + 270
        
        # Designate a point of interest if detects something within tracking range
        if minDistance < self.tracking_range:
            self.POI = (minDistance, math.pi*minIndex/180)
            x = self.POI[0]*math.cos(self.POI[1])
            y = self.POI[0]*math.sin(self.POI[1])
            # Visualize a marker for person to follow
            self.marker_pub.publish(create_marker("base_link", "person_follow", x, y, 0.2))
            print('Found a Point of Interest')
        else:
            self.POI = (None, None)
            # Visualize the tracking area using a marker to rviz
            self.marker_pub.publish(create_marker("base_link", "finite_state", self.tracking_range/2, 0, 0, \
                                    Marker.CUBE, self.tracking_range, self.tracking_range*2, .2, r=0, g=0, b=1, a=.2))

    def person_follow(self):
        """ Implements person following behavior """
        m = Twist()
        r = rospy.Rate(10)
        print('*********Person Following*********')
        while not rospy.is_shutdown():
            # Doesn't have a POI
            if self.POI[0] is None:
                return self.seeking
            # Checks if neato is close enough to person to stop
            elif abs(self.POI[0]) <= .5:
                m.linear.x = 0
                m.angular.z = 0
                self.vel_pub.publish(m)

            else:
                # Checks if heading of neato is not in the direction of the POI
                if abs(self.POI[1]) > .1:
                    # Continue turning at angular speed based on angle (in rads) left to cover
                    
                    # is it - self.POI?  
                    if 0 < self.POI[1] <= math.pi:
                        m.angular.z = sigmoid(self.POI[1]) * 0.6
                    else:
                        m.angular.z = -sigmoid(self.POI[1]) * 0.6
                else:
                    # Drive straight at speed based on distance to drive
                    m.linear.x = self.POI[0] * 0.5
                    m.angular.z = 0

            self.vel_pub.publish(m)

            r.sleep() 

    def avoid_obstacle(self):
        pass

    def run(self):
        rospy.sleep(1)
        while not rospy.is_shutdown():
            # set the state variable equal to function that is returned  
            self.state = self.state() 

if __name__ == "__main__":
    myFiniteControl = FiniteControl()
    myFiniteControl.run()
