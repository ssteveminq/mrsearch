#! /usr/bin/python

import rospy
import actionlib
import numpy as np
import rospy
import sys
import tf
import actionlib
import geometry_msgs.msg
from std_msgs.msg import *
from geometry_msgs.msg import *
from geometry_msgs.msg import Pose, PolygonStamped, PoseStamped, PoseArray, Point, Quaternion, PointStamped, Twist
from actionlib_msgs.msg import GoalStatus
import tf
import yaml
from tf import TransformListener
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
from search_service.msg import SetSearchRegionAction, SetSearchRegionFeedback, SetSearchRegionResult, SetSearchRegionGoal

_ORIGIN_TF ='map'

def load_yamlfile():
    with open('../config/wboundary.yaml') as file:
        yaml_data=yaml.load(file, Loader=yaml.Loader)
        # print("yaml_data", yaml_data)
        boundaries= yaml_data['boundaries']
        return boundaries
 

def mains():
    # Initialize
    boundaries = load_yamlfile()
    p = PolygonStamped()
    for boundary in boundaries:
        p.polygon.points.append(Point32(x=boundary[0],y=boundary[1]))
    # p.header = header()

    client = actionlib.SimpleActionClient('set_search_region',SetSearchRegionAction)
    client.wait_for_server()
    goal = SetSearchRegionGoal()
    goal.boundary=p
    print("call search action client")
    rospy.loginfo("start action")
    client.send_goal(goal)
    client.wait_for_result()

        
if __name__ == '__main__':
    rospy.init_node('setsearch_region_action_client')
    mains()


