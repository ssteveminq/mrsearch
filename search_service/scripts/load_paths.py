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
from nav_msgs.msg import Odometry, Path
import math

_ORIGIN_TF ='map'

class path_info:
    def __init__(self, info_data=None):
        self.init_pos=[0,0]
        self.m_path=Path()
        self.set_info(info_data)
    def set_info(self, info_data):
        # print("info_data",info_data)
        if info_data!=None:
            for pos in info_data['pos']:
                tmp_pose=PoseStamped()
                tmp_pose.header.frame_id="map"
                tmp_pose.pose.position.x=pos[0]
                tmp_pose.pose.position.y=pos[1]
                quat = tf.transformations.quaternion_from_euler(0, 0, pos[2])
                tmp_pose.pose.orientation= Quaternion(*quat)
                self.m_path.poses.append(tmp_pose)
    def get_path(self):
        return self.m_path



class path_manager(object):
    def __init__(self, wait=0.0):
        self.pathpub1=rospy.Publisher('walrus/search_agent_manager/waypoint_plan',Path,queue_size=10)
        self.pathpub2=rospy.Publisher('jackal/search_agent_manager/waypoint_plan',Path,queue_size=10)

        self.agent1_path=Path()
        self.agent2_path=Path()
        self.load_yamlfile()

        rospy.loginfo("path_manager created")
        # rospy.spin()
        self.num_agent=2
        self.starter()

    def load_yamlfile(self):
        with open('config/visual_paths.yaml') as file:
            yaml_data=yaml.load(file, Loader=yaml.Loader)
            # path_info(yaml_data['walrus'])

            agent1_path_info= path_info(yaml_data['walrus'])
            agent2_path_info= path_info(yaml_data['jackal'])
            # print("self.agent2_path",self.agent2_path)
            self.agent1_path=agent1_path_info.get_path()
            self.agent2_path=agent2_path_info.get_path()
            self.agent1_path.header.stamp=rospy.Time.now()
            self.agent1_path.header.frame_id="map"
            self.agent2_path.header.stamp=rospy.Time.now()
            self.agent2_path.header.frame_id="map"
 

    def starter(self,wait=0.0):
        # make sure the cont0roller is running
        while not rospy.is_shutdown():
            # self.agent1_path.header.stamp=rospy.Time.now()
            # self.agent1_path.header.frame_id="map"
            # self.agent2_path.header.stamp=rospy.Time.now()
            # self.agent2_path.header.frame_id="map"
            self.pathpub1.publish(self.agent1_path)
            self.pathpub2.publish(self.agent2_path)
            rospy.sleep(3.0)
 

if __name__ == '__main__':
    rospy.init_node('path_manager')
    p_manager_ = path_manager()
    p_manager_.starter()
    # mains()


