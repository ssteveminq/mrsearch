#! /usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import *
from geometry_msgs.msg import *
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Point, Quaternion, PointStamped, Twist
from nav_msgs.msg import Path

_ORIGIN_TF ='map'

class path_manager(object):

    def __init__(self, name):
        self.pathpub=rospy.Publisher('goal_agent3',Path,queue_size=10)
        self.desiredpath=Path()
        self.desiredpath.header.stamp =rospy.Time.now()
        self.desiredpath.header.frame_id="map"

        waypoint = PoseStamped()
        waypoint.header.stamp= rospy.Time.now()
        waypoint.header.frame_id = "map"
        waypoint.pose.position.x=7.08
        waypoint.pose.position.y=4.5
        waypoint.pose.orientation.w=1.0
        self.desiredpath.poses.append(waypoint)
        waypoint = PoseStamped()
        waypoint.header.stamp= rospy.Time.now()
        waypoint.header.frame_id = "map"
        waypoint.pose.position.x=6.08
        waypoint.pose.position.y=4.5
        waypoint.pose.orientation.w=1.0
        self.desiredpath.poses.append(waypoint)
        waypoint = PoseStamped()
        waypoint.header.stamp= rospy.Time.now()
        waypoint.header.frame_id = "map"
        waypoint.pose.position.x=4.08
        waypoint.pose.position.y=4.5
        waypoint.pose.orientation.w=1.0
        self.desiredpath.poses.append(waypoint)
        self.starter()
       

    def starter(self,wait=0.0):
        # make sure the cont0roller is running

        r=rospy.Rate(1)
        # rospy.spin()            kkkkkkkk
        while not rospy.is_shutdown():
            self.desiredpath.header.stamp=rospy.Time.now()
            for waypoint in self.desiredpath.poses:
                waypoint.header.stamp=rospy.Time.now()
            self.pathpub.publish(self.desiredpath)
            r.sleep()

            # rospy.spin()            
        # fill ROS message
 

if __name__ == '__main__':
    rospy.init_node('path_publisher_test')
    server = path_manager('path_publisher_test')
    # server.starter()
