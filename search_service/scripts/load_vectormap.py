#! /usr/bin/python

import rospy
import actionlib
import numpy as np
import os
import sys
import nav_msgs.msg
import geometry_msgs.msg
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose



_FRAME_ID = 'map'
_FILENAME = '/home/knapsack/workspaces/test_ws/src/mrsearch/tmp/vectormaps/campus3.vectormap.txt'


def vectormap_manager():    
    msg = Path()
    msg.header.frame_id = _FRAME_ID
    msg.header.stamp = rospy.Time.now()

    vectormap_pub=rospy.Publisher('/vectormap',Path,queue_size=10)

    map_txt = open(_FILENAME, 'r')

    for line in map_txt:
        pose = PoseStamped()
        # line = map_txt.readline()
        # if not line:
            # print('break')
            # break
        values = line.split(',')
        # print(values[0])
        # print(values[1])
        pose.pose.position.x = float(values[0])
        pose.pose.position.y = float(values[1])
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = 0.
        pose.pose.orientation.y = 0.
        pose.pose.orientation.z = 0.
        pose.pose.orientation.w = 1.
        msg.poses.append(pose)

    map_txt.close()
    rospy.loginfo("Publishing Vectormap...")

    while not rospy.is_shutdown():
        vectormap_pub.publish(msg)




if __name__ == '__main__':
    rospy.init_node('vectormap_manager')
    vectormap_manager()
    # mains()
