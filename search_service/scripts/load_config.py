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
from nav_msgs.msg import Path
from actionlib_msgs.msg import GoalStatus
import tf
import yaml
from tf import TransformListener
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
from search_service.msg import SetSearchRegionAction, SetSearchRegionFeedback, SetSearchRegionResult, SetSearchRegionGoal

_ORIGIN_TF ='map'

class agent_info:
    def __init__(self, info_data=None):
        self.init_pos=[0,0]
        self.size_fov=5
        self.speed=1
        self.set_info(info_data)
    def set_info(self, info_data):
        if info_data!=None:
            self.init_pos=info_data['pos']
            self.size_fov=info_data['fov']
            self.speed=info_data['speed']
        


def load_yamlfile():
    with open('../config/config.yaml') as file:
        yaml_data=yaml.load(file, Loader=yaml.Loader)
        # print("yaml_data", yaml_data)
        agent1_info= agent_info(yaml_data['agent1'])
        agent2_info= agent_info(yaml_data['agent1'])


        return agent1_info, agent2_info
 

def mains():
    # Initialize
    agent1_data, agent2_data = load_yamlfile()
    print("agent1_data",agent1_data)
    # p = PolygonStamped()
    # for boundary in boundaries:
        # p.polygon.points.append(Point32(x=boundary[0],y=boundary[1]))
    # p.header = header()

    # client = actionlib.SimpleActionClient('set_search_region',SetSearchRegionAction)
    # client.wait_for_server()
    # goal = SetSearchRegionGoal()
    # goal.boundary=p
    print("call search action client")
    rospy.loginfo("start action")
    client.send_goal(goal)
    client.wait_for_result()

        
if __name__ == '__main__':
    rospy.init_node('setsearch_region_action_client')
    mains()


