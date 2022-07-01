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
from search_service.msg import *

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

def load_search_region_yamlfile():
    with open('config/boundaries.yaml') as file:
        yaml_data=yaml.load(file, Loader=yaml.Loader)
        # print("yaml_data", yaml_data)
        boundaries= yaml_data['boundaries']
        return boundaries
 

def load_yamlfile():
    with open('config/config.yaml') as file:
        yaml_data=yaml.load(file, Loader=yaml.Loader)
        agent1_info= agent_info(yaml_data['agent1'])
        agent2_info= agent_info(yaml_data['agent2'])
        return agent1_info, agent2_info
 

def mains():
    # Initialize
    # boundaries = load_search_region_yamlfile()
    # p = PolygonStamped()
    # for boundary in boundaries:
        # p.polygon.points.append(Point32(x=boundary[0],y=boundary[1]))
    # print("p", p)

    # client = actionlib.SimpleActionClient('set_search_region',SetSearchRegionAction)
    # print("wait_for_server")
    # client.wait_for_server()
    # goal = SetSearchRegionGoal()
    # goal.boundary=p
    # rospy.loginfo("start action")
    # client.send_goal(goal)
    # client.wait_for_result()

    agent1_data, agent2_data = load_yamlfile()
    print("call search action client")
    rospy.loginfo("start action")

    sclient = actionlib.SimpleActionClient('fake_multisearch_action',search_service.msg.FakeMultiSearchAction)
    sclient.wait_for_server()
    sgoal = search_service.msg.FakeMultiSearchGoal()
    sgoal.num_agent=2
    sgoal.agent1_pos_x=agent1_data.init_pos[0]
    sgoal.agent1_pos_y=agent1_data.init_pos[1]
    sgoal.agent1_fov=agent1_data.size_fov
    sgoal.agent1_speed=agent1_data.speed
    sgoal.agent2_pos_x=agent2_data.init_pos[0]
    sgoal.agent2_pos_y=agent2_data.init_pos[1]
    sgoal.agent2_fov=agent2_data.size_fov
    sgoal.agent2_speed=agent2_data.speed
    print("call fake multi search action client")
    sclient.send_goal(sgoal)
    sclient.wait_for_result()

    # client.send_goal(goal)
    # client.wait_for_result()

        
if __name__ == '__main__':
    rospy.init_node('setsearch_region_action_client')
    mains()


