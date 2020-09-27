#!/usr/bin/env python3

# imports and stuff
import rospy, time, math
from std_msgs.msg import Int8MultiArray, Float32
from geometry_msgs.msg import Twist, Vector3, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion

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
        self.seek_theta = math.radians(90)
        self.seek_step = math.radians(5)

        self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        rospy.Subscriber('scan', LaserScan, self.process_scan)
        rospy.Subscriber('/odom', Odometry, self.process_odom)

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
        target_angle = self.angle_normalize(self.current_angle + theta)
        diff = self.angle_diff(self.current_angle, target_angle)
        r = rospy.Rate(10)
        # Keep on turning as long as the current angle and target angle are not within a threshold
        while not rospy.is_shutdown() and abs(diff) > .05:
            self.vel_pub.publish(m)
            diff = self.angle_diff(self.current_angle, target_angle)
            r.sleep()
        m.angular.z = 0
        self.vel_pub.publish(m)

    def process_odom(self, msg):
        x, y, self.current_angle = self.convert_pose_to_xy_and_theta(msg.pose.pose)
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
            print('Found a Point of Interest')
        else:
            self.POI = (None, None)

    def sigmoid(self, x):
        return 1/(1+math.exp(-x))

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
                        m.angular.z = self.sigmoid(self.POI[1]) * 0.6
                    else:
                        m.angular.z = -self.sigmoid(self.POI[1]) * 0.6
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

    # Helper Functions for angle calcs
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

    def convert_pose_to_xy_and_theta(self, pose):
        """ Convert pose (geometry_msgs.Pose) to a (x,y,yaw) tuple """
        orientation_tuple = (pose.orientation.x,
                                pose.orientation.y,
                                pose.orientation.z,
                                pose.orientation.w)
        angles = euler_from_quaternion(orientation_tuple)
        return (pose.position.x, pose.position.y, angles[2])

if __name__ == "__main__":
    myFiniteControl = FiniteControl()

    myFiniteControl.run()
