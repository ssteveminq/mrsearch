#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
import sys
import tf
import actionlib
import geometry_msgs.msg
from gmm.msg import *


def mains():
    # Initialize
    client = actionlib.SimpleActionClient('Predictions',gmm.msg.PredictionAction)
    client.wait_for_server()
    goal = gmm.msg.PredictionGoal()
    goal.mean_xs=[]
    goal.mean_ys=[]

    goal.mean_xs.append(-26)
    goal.mean_ys.append(-40)

    goal.mean_xs.append(30)
    goal.mean_ys.append(-15)

    # goal.mean_xs.append(-13)
    # goal.mean_ys.append(10)

    goal.mean_xs.append(42)
    goal.mean_ys.append(35)

    goal.mean_xs.append(-36)
    goal.mean_ys.append(-47)

    # goal.mean_xs.append(-23)
    # goal.mean_ys.append(10)

    print("call predictionaction client")

    rospy.loginfo("start action")
    client.send_goal(goal)
    client.wait_for_result()

        
if __name__ == '__main__':
    rospy.init_node('multisearch_action_client')
    mains()
