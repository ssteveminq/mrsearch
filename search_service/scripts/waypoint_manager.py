#! /usr/bin/env python

import rospy
import actionlib
import numpy as np
from std_msgs.msg import *
from geometry_msgs.msg import *
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Point, Quaternion, PointStamped, Twist
import trajectory_msgs.msg
from search_service.msg import SearchAction, SearchFeedback, SearchResult
import tf
from tf import TransformListener
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
from obstacle import Obstacle
from raycasting_grid_map import generate_ray_casting_grid_map, calc_grid_map_config, atan_zero_to_twopi
from utils.state_lattice_planner import uniform_terminal_state_sampling_test1, lane_state_sampling_test1
from utils.configuration_space import configuration_space
from utils.cubic_spline_planner import Spline2D, calc_spline_course_trj
from utils.stanley_controller import *
from utils.graph_utils import *
from utils.dynamic_window_approach import *
from  VCD import VerticalCellDecomposition
from collections import defaultdict
import pandas as pd
import os
import re
import time
import argparse
import csv

_ORIGIN_TF ='map'

class Wp_manager(object):

    def __init__(self, name):
        # Init actionserver
        self._action_name = name
        self.robot_pose=np.zeros((5,1))
        self.iters=0;
        #x,y,theta
        self.target_pose = Point()
        self.feedback_ = SearchFeedback()
        self.result_= SearchResult()
        self.target_yaw=0.0
        self.location_map={}
        self.locations=rospy.get_param("waypoints/locations")
        print(self.locations)
        self.locations=rospy.get_param("waypoints/locations")
        self.load_yamlfile()
        print("self.lcoation_map",self.location_map)

        # Preparation to use robot functions
        # self.navi_cli = actionlib.SimpleActionClient('/move_base/move', MoveBaseAction)
        # self._as = actionlib.SimpleActionServer(self._action_name, SearchAction, execute_cb=self.execute_cb, auto_start = False)
        # self._as.start()

    def load_yamlfile(self, ns="waypoints/"):
        for location in self.locations:
            input_string =ns +location
            coord = rospy.get_param(input_string)
            self.location_map[input_string]= coord
            # print("coord[0]:", coord[0])
            # print("coord[1]:", coord[1])
            

    def execute_cb(self, goal):
        print("execute_cb")
        # self._as.set_succeeded()

    def robot_pose_Cb(self, msg):
        # print("execute_cb")
        self.publish_search_map()

    def check_ori_error(self,robot_yaw, target_yaw, err_criterion):
        temp_dist=0.0
        temp_dist=math.pow((robot_yaw-target_yaw),2)
        temp_dist=math.sqrt(temp_dist);
        # rospy.loginfo("temp_yaw_diff: %.4lf ", temp_dist)
        temp_criterion =math.sqrt(math.pow((temp_dist),2))
        # rospy.loginfo("temp_yaw_error: %.4lf ", temp_criterion)

        if (temp_criterion<err_criterion):
            return True

        return False

    def check_pos_error(self, err_criterion):
        #w.r.t map error
        self.x_err = self.target_point.point.x
        self.y_err = self.target_point.point.y

        temp_dist=0.0;
        temp_dist+=math.pow(self.x_err,2)
        temp_dist+=math.pow(self.y_err,2)
        temp_dist=math.sqrt(temp_dist)

        temp_criterion =math.sqrt(math.pow((temp_dist),2))
        rospy.loginfo("distance error: %.4lf ", temp_criterion);
        if (temp_criterion<err_criterion):
            return True

        return False

    def send_vel_command(self):
        self.vel_pub.publish(self.vel_cmd)

if __name__ == '__main__':
    rospy.init_node('waypoint_server')
    server = Wp_manager('waypoint_server')
    rospy.loginfo("waypoint_server created")
    rospy.spin()
