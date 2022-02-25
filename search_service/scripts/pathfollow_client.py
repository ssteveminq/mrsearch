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
    print("main")
    client = actionlib.SimpleActionClient('path_follow_agent_0',search_service.msg.PathFollowAction)
    client.wait_for_server()
    goal = search_service.msg.PathFollowGoal()
    #goal.start=True
    print("call search action client")

    # client.send_goal(goal)
    # client.wait_for_result()
    rospy.loginfo("start action")
    client.send_goal(goal)
    client.wait_for_result()
        
if __name__ == '__main__':
    rospy.init_node('pathfollow_action_client0')
    mains()
