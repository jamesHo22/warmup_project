#!/usr/bin/env python3
from std_msgs.msg import Header, Int8MultiArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist, Vector3
import rospy
import tty
import select
import sys
import termios

class Teleop:

    def __init__(self):
        rospy.init_node("myTeleop")
        self.pubMove = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)
    
    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        self.key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return self.key
    
    def run(self):
        r = rospy.Rate(10)
        self.settings = termios.tcgetattr(sys.stdin)
        self.key = None

        while not rospy.is_shutdown() and self.key != '\x03':  
            self.key = self.getKey()
            print(self.key)
            self.moveRobot(self.key)  
            r.sleep()

    def moveRobot(self, key):
        # This is the defaul velocity command to be published
        self.init_twist = Twist(Vector3(0,0,0), Vector3(0,0,0))
        if key == 'w':
            self.init_twist = Twist(Vector3(1,0,0), Vector3(0,0,0))
        elif key == 's':
            self.init_twist = Twist(Vector3(0,0,0), Vector3(0,0,0))
        elif key == 'a':
            self.init_twist = Twist(Vector3(0,0,0), Vector3(0,0,1))
        elif key == 'd':
            self.init_twist = Twist(Vector3(0,0,0), Vector3(0,0,-1))
        elif key == 'x':
            self.init_twist = Twist(Vector3(-1,0,0), Vector3(0,0,0))    
        else:
            self.init_twist = Twist(Vector3(0,0,0), Vector3(0,0,0)) 

        self.pubMove.publish(self.init_twist)
        

			
			

if __name__ == "__main__":

    myTele = Teleop()
    myTele.run()