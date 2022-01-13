#!/usr/bin/env python
import roslib
import rospy
import actionlib
import roslib
import math
import sys
import rospy
import actionlib
import geometry_msgs.msg
from std_msgs.msg import String
from std_msgs.msg import Int8
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Point, Quaternion, PointStamped
import tf.transformations


class waypoint_manager(object):
    def __init__(self, wait=0.0):
        self.goalpub=rospy.Publisher('goal_agent2',PoseStamped,queue_size=10)

        click_topic='clicked_point'
        rospy.Subscriber(click_topic, PointStamped, self.clicked_Cb)
        rospy.spin()

    def clicked_Cb(self, msg):
        print(msg)
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose.position= msg.point
        pose.pose.orientation.w=1.0
        self.goalpub.publish(pose)
        rospy.loginfo("sending new goal to movebase")

if __name__ == '__main__':
    rospy.init_node('clicknavi_a1')
    navimanager = waypoint_manager()
































