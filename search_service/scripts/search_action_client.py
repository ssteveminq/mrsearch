#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
import sys
import tf
import actionlib
import geometry_msgs.msg
from search_service.msg import *


def mains():
    # Initialize
    client = actionlib.SimpleActionClient('search_action',search_service.msg.SearchAction)
    client.wait_for_server()
    goal = search_service.msg.SearchGoal()
    goal.start=True
    print("call search action client")

    # client.send_goal(goal)
    # client.wait_for_result()
    while not rospy.is_shutdown():
        rospy.loginfo("start action")
        client.send_goal(goal)
        client.wait_for_result()
        
if __name__ == '__main__':
    rospy.init_node('search_action_client')
    mains()
