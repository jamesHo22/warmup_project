#!/usr/bin/env python3
from visualization_msgs.msg import Marker
import rospy


class MarkerTest:
    def __init__(self):
        rospy.init_node('marker_viz')
        self.pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
        
        init_marker = self.create_marker("base_link", 1,2,0)

        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            init_marker.header.stamp=rospy.Time.now()
            # publish marker: 
            self.pub.publish(init_marker)
            # print('publish iteration complete')
            r.sleep()
    
    def create_marker(self, frame, posx, posy, posz):
        marker = Marker()
        marker.header.frame_id = frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = "my_namespace"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = posx
        marker.pose.position.y = posy
        marker.pose.position.z = posz
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 1
        marker.scale.y = 1
        marker.scale.z = 1
        marker.color.a = 1.0 # Don't forget to set the alpha!
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        return marker



if __name__ == "__main__":
    a = MarkerTest()