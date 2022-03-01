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
    goal.mean_ys.append(-20)

    goal.mean_xs.append(-10)
    goal.mean_ys.append(-15)

    # goal.mean_xs.append(-13)
    # goal.mean_ys.append(10)

    goal.mean_xs.append(40)
    goal.mean_ys.append(22)

    goal.mean_xs.append(-25)
    goal.mean_ys.append(-29)

    # goal.mean_xs.append(-23)
    # goal.mean_ys.append(10)

    print("call predictionaction client")

    rospy.loginfo("start action")
    client.send_goal(goal)
    client.wait_for_result()

        
if __name__ == '__main__':
    rospy.init_node('multisearch_action_client')
    mains()
