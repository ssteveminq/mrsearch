#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
import sys
import tf
import actionlib
import geometry_msgs.msg
from visual_perception.msg import *


def mains():
    # Initialize
    client = actionlib.SimpleActionClient('getunknowns',visual_perception.msg.UnknownSearchAction)
    client.wait_for_server()
    goal = visual_perception.msg.UnknownSearchGoal()
    #goal.start=True
    print("call search action client")

    # client.send_goal(goal)
    # client.wait_for_result()
    rospy.loginfo("start action")
    client.send_goal(goal)
    client.wait_for_result()
        
if __name__ == '__main__':
    rospy.init_node('unknownsearch_action_client')
    mains()
