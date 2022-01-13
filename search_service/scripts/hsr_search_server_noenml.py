#! /usr/bin/env python
import rospy
import actionlib
import numpy as np
from std_msgs.msg import *
from geometry_msgs.msg import *
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped, PoseArray, Point, Quaternion, PointStamped, Twist
import trajectory_msgs.msg
from actionlib_msgs.msg import GoalStatus
from search_service.msg import SearchAction, SearchFeedback, SearchResult
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import JointState
from nav_msgs.msg import OccupancyGrid, Path
import math
from obstacle import Obstacle
from raycasting_grid_map import generate_ray_casting_grid_map, calc_grid_map_config, atan_zero_to_twopi, generate_fov_angle, precasting
from utils.state_lattice_planner import uniform_terminal_state_sampling_test1, lane_state_sampling_test1, lane_state_sampling_test
from utils.configuration_space import configuration_space
from utils.cubic_spline_planner import Spline2D, calc_spline_course_trj
from utils.stanley_controller import *
from utils.graph_utils import *
from utils.dynamic_window_approach import *
from  VCD import VerticalCellDecomposition
from nav_msgs.srv import GetMultiPlan 
import random
import pandas as pd
import os
import re
import time
import argparse
import csv


_ORIGIN_TF ='map'
_OFFSET_TF = 'map'
l_occ = np.log(0.99/0.01)
l_free = np.log(0.01/0.99)


class map_params:
    def __init__(self):
        self.xyreso = 0.5  # x-y grid resolution [m]
        self.yawreso = math.radians(6)  # yaw angle resolution [rad]
        self.xmin=-11.0
        self.xmax=11.0
        self.ymin=0.0
        self.ymax=13.0
        self.xw = int(round((self.xmax - self.xmin) / self.xyreso))
        self.yw = int(round((self.ymax - self.ymin) / self.xyreso))
        self.boundaries=[]
        self.boundaries.append((self.xmin,self.ymin))
        self.boundaries.append((self.xmax,self.ymin))
        self.boundaries.append((self.xmax,self.ymax))
        self.boundaries.append((self.xmin,self.ymax))
        self.sensor_range=5


class Params:
    def __init__(self):
        self.numiters = 4000
        self.dt = 0.2
        self.goal_tol = 0.4
        self.weight_entropy = 0.05
        self.weight_entropy = 0.05
        self.max_vel = 2.0 # m/s
        self.min_vel = 0.0 # m/s
        self.sensor_range_m = 0.5 # m
        # self.animate = 1
        self.area_size=13
        #dwa parameters
        self.max_yawrate = 45.0 * math.pi / 180.0  # [rad/s]
        self.max_accel = 0.3  # [m/ss]
        self.max_dyawrate = 45.0 * math.pi / 180.0  # [rad/ss]
        self.v_reso = 0.02  # [m/s]
        self.yawrate_reso = 0.1 * math.pi / 180.0  # [rad/s]
        self.max_speed = 0.8  # [m/s]
        self.min_speed = 0.0  # [m/s]
        self.predict_time = 2.0  # [s]
        self.to_goal_cost_gain = 1.0
        self.speed_cost_gain = 1.0
        self.robot_radius = 1.0  # [m]



class Map_manager(object):

    def __init__(self, name):
        self._action_name = name
        self.robot_pose=np.zeros((5,1))
        self.agent2_pose=np.zeros((3,1))
        self.iters=0;
        #x,y,theta

        self.weight_entropy = 0.15
        self.weight_travel = 1.7
        self.horizon=25
        self.xmin=-11.0
        self.xmax=11.0
        self.ymin=0.5
        self.ymax=13.0
        self.x_err=0.0
        self.y_err=0.0
        self.vel_cmd = Twist()
        self.target_pose = Point()
        self.feedback_ = SearchFeedback()
        self.result_= SearchResult()
        self.target_yaw=0.0
        self.direction_z=1
        self.gaze_theta_err=0.0
        self.desired_head_pan=0.0
        self.entropy_map = OccupancyGrid()
        self.map_received=False
        self.params_searchmap =  map_params()
        #load waypoints 
        self.frontier_poses=PoseArray()
        self.last_goal=PoseStamped()
        self.start_pose=PoseStamped()
        self.start_pose.header.frame_id='map'
        self.location_map={}
        self.locations=rospy.get_param("waypoints/locations")
        self.load_yamlfile()
        self.agent2_ori=self.start_pose.pose.orientation
        # self.goal_sampling_waypoints(self.params_searchmap)
        self.agent_history=[]
        self.entropy=1.0
        self.occ_entropy=0.0
        self.near_goal=False
        self.called_once=False
        self.no_count=0
        # self.listener = tf.TransformListener()
        # now = rospy.Time.now()-rospy.Duration(3.0)
        # try:
            # self.listener.waitForTransform(_ORIGIN_TF,_OFFSET_TF,rospy.Time(), rospy.Duration(2.0))
        # except:
            # rospy.loginfo("waitTransform failed")
    
        
        # print("self.lcoation_map",self.location_map)
        self.best_trj_pub=rospy.Publisher('/best_trj',Path,queue_size=10)
        self.selected_goal_pub=rospy.Publisher('/search_goal',PoseStamped,queue_size=10)
        self.searchmap_pub = rospy.Publisher('/search_map', OccupancyGrid,queue_size=10)
        self.infomap_pub = rospy.Publisher('/info_map', OccupancyGrid,queue_size=10)

        # jointstates_topic='hsrb/joint_states'
	# rospy.Subscriber(jointstates_topic, JointState, self.joint_state_Cb)

        # agent1_pose_topic='global_pose'
        # rospy.Subscriber(agent1_pose_topic, PoseStamped, self.robot_pose_Cb)

        agent2_pose_topic='global_pose'
        rospy.Subscriber(agent2_pose_topic, PoseStamped, self.agent2_pose_Cb)

        frontiers_pose_topic='frontier_poses'
        rospy.Subscriber(frontiers_pose_topic, PoseArray, self.frontier_poses_Cb)

        # globalmap_topic="/static_obstacle_map_ref"
        globalmap_topic="/scaled_static_map"
        # localmap_agent1_topic="/dynamic_obstacle_map"
        self.global_map=OccupancyGrid()
        # self.agent1_local_map=OccupancyGrid()
        self.agent2_local_map=OccupancyGrid()

        #search_region = log occ map for entropy map w.r.t {map} frame 
        # unknown =0.0 / occ  / free: 
        self.search_region = OccupancyGrid()
        self.search_region.info.resolution = self.params_searchmap.xyreso
        self.search_region.info.width= self.params_searchmap.xw
        self.search_region.info.height= self.params_searchmap.yw
        self.search_region.info.origin.position.x= self.params_searchmap.xmin
        self.search_region.info.origin.position.y= self.params_searchmap.ymin
        self.search_region.data =[0] *(self.search_region.info.width * self.search_region.info.height)

        self.info_region = OccupancyGrid()
        self.info_region.info.resolution = self.params_searchmap.xyreso
        self.info_region.info.width= self.params_searchmap.xw
        self.info_region.info.height= self.params_searchmap.yw
        self.info_region.info.origin.position.x= self.params_searchmap.xmin
        self.info_region.info.origin.position.y= self.params_searchmap.ymin
        self.info_region.data =[0] *(self.info_region.info.width * self.info_region.info.height)

        self.srv_client=rospy.ServiceProxy('planner/planner/make_multiplan', GetMultiPlan)
        self.tolerance=0.5

        # rospy.Subscriber(globalmap_topic, OccupancyGrid, self.globalmap_cb)
        # rospy.Subscriber(localmap_agent1_topic, OccupancyGrid, self.agent1_localmap_cb)
        # localmap_agent2_topic="/costmap_node/costmap/costmap"
        localmap_agent2_topic="dynamic_obstacle_map"
        rospy.Subscriber(localmap_agent2_topic, OccupancyGrid, self.agent2_localmap_cb)

        self.global_map= rospy.wait_for_message(globalmap_topic, OccupancyGrid)
        self.crop_globalmap(self.params_searchmap, self.global_map.info)
        self.initial_entropy= self.get_map_entropy()
        print("total entropy: ", self.initial_entropy)
        self.entropy=1.0

        self._as = actionlib.SimpleActionServer(self._action_name, SearchAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        rospy.loginfo("action_started")

    def get_map_entropy(self):
        entropy_sum=-self.occ_entropy
        for idx in range(len(self.search_region.data)):
            p= 1-1./(1.0+np.exp(self.search_region.data[idx]))
            if p>0.0 and p<1.0:                 
                entropy_sum+=-(p*math.log(p)+(1-p)*math.log(1-p))
        return entropy_sum

    def call_service(self):
        if len(self.agent_history)>0:
            self.start_pose.pose.position.x=self.last_goal.pose.position.x+0.1
            self.start_pose.pose.position.y=self.last_goal.pose.position.y+0.1
        else:
            self.start_pose.pose.position.x=self.agent2_pose[0]+0.15
            self.start_pose.pose.position.y=self.agent2_pose[1]+0.2

        self.start_pose.pose.orientation.w=1.0
        # if self.entropy>0.55:
        #self.goals = self.goal_sampling_waypoints(self.params_searchmap)
        #print("goals:", self.goals)
        # else:
            # self.goals = self.goal_sampling_frontiers()
        if self.entropy<0.4:
            self.goals = self.goal_sampling_waypoints(self.params_searchmap)
        else:
            self.goals = self.goal_sampling_frontiers()
        #k:w
        #self.goals = self.goal_sampling_frontiers()
               
        rospy.wait_for_service('/planner/planner/make_multiplan')
        srv_res=self.srv_client(self.start_pose,self.goals, self.tolerance)
        return srv_res
        # try:
            # srv_res=self.srv_client(self.start_pose,self.goals, self.tolerance)
        # except rospy.ServiceException, e:
            # print("Service call failed: %s", %e)
        # return srv_res
 

    def load_yamlfile(self, ns="waypoints/"):
        for location in self.locations:
            input_string =ns +location
            coord = rospy.get_param(input_string)
            self.location_map[input_string]= coord
 

    def Coord2CellIdx(self,x, y, map_info):
        temp_x = x-map_info.origin.position.x
        temp_y = y-map_info.origin.position.y
        coord_x= math.floor(temp_x/map_info.resolution)
        coord_y= math.floor(temp_y/map_info.resolution)
        # print("coord_x:" ,coord_x, "coord_y:" ,coord_y )
        idx= (int)(coord_x+map_info.width*coord_y)

        return idx
 
    def Idx2Coord(self, idx, map_info):
        res = (int) (idx/map_info.width);
        div = (int) (idx%map_info.width);
        coord_x=(res+0.5)*map_info.resolution+map_info.origin.position.x;
        coord_y=(div+0.5)*map_info.resolution+map_info.origin.position.y;

        return coord_x, coord_y

    def crop_globalmap(self, map_params, map_info):
        print("crop_globalmap")
        #Based on map_parms, this function provides crop_map
        # self.search_region.map_info = map_info
        count=0
        for ix in range(map_params.xw-1):
            for iy in range(map_params.yw-1):
                px = map_params.xmin+ix*map_params.xyreso
                py = map_params.ymin+iy*map_params.xyreso
                global_idx = self.Coord2CellIdx(px,py, map_info)
                search_idx = self.Coord2CellIdx(px,py, self.search_region.info)
                print("px, py: ", px, ", ", py)
                print("global_idx:", global_idx)
                print("search_idx:", search_idx)
                if self.global_map.data[global_idx]>0:
                    self.search_region.data[search_idx]= int(l_occ)
                    count=count+1
                    #entropy calcuation

        p= 1-1./(1.0+np.exp(int(l_occ)))
        self.occ_entropy=count*(-(p*math.log(p)+(1-p)*math.log(1-p)))
        print("occ entropy:", self.occ_entropy)
        print("crop_map finished")
        return
    
    ##for observed cell in local window--> update search_map
    def update_occ_grid_map(self, local_map):
        # point_in = PointStamped()
        # point_in.header.stamp = rospy.Time.now()
        # point_in.header.frame_id= 'map_en'
        # point_in.point= local_map.info.origin.position
        # print("point_in.point: ", point_in)
        # origin_point_map_en=PointStamped()
        # try:
            # transform = self.tf_buffer.lookup_transform(_OFFSET_TF, _ORIGIN_TF,rospy.Time(0),rospy.Duration(1.0))
            # origin_point_map_en =tf2_geometry_msgs.do_transform_point(point_in,transform)
        # except:
            # rospy.loginfo("transformPoint failed")
        origin_point_map_en =local_map.info.origin.position
        for j in range(local_map.info.height-1):
            for i in range(local_map.info.width-1):
                map_idx = j*local_map.info.height+i
                px = origin_point_map_en.x+i*local_map.info.resolution
                py = origin_point_map_en.y+j*local_map.info.resolution
                # px = local_map.info.origin.position.x+i*local_map.info.resolution
                # py = local_map.info.origin.position.y+j*local_map.info.resolution
                # get search cell idx 
                search_idx = self.Coord2CellIdx(px,py, self.search_region.info)

                #Update searchmap according to local measurement
                # if local_map is known (occ or free) and global_map is not occupied by static obstacle 
                if search_idx< len(self.search_region.data):
                    if local_map.data[map_idx]!=-1 and self.search_region.data[search_idx]!=l_occ: #localmap
                        if local_map.data[map_idx]==0:
                            self.search_region.data[search_idx]= int(l_free)
                        # else:
                        # self.search_region.data[search_idx]= l_occ
        self.publish_search_map()
        

    def publish_search_map(self):
        # print(self.search_region.data)
        self.searchmap_pub.publish(self.search_region)
        # rospy.loginfo("publish_searchmap")

    def goal_sampling_frontiers(self):
        self.goals=[]
        for fpose in self.frontier_poses.poses:
            tmp_pose = PoseStamped()
            tmp_pose.pose=fpose
            tmp_pose.pose.orientation.w=1.0
            tmp_pose.header.frame_id="map"
            self.goals.append(tmp_pose)
        # rospy.loginfo("frontier_poses: %d", len(self.frontier_poses.poses))
        return self.goals



    def goal_sampling_waypoints(self, params_searchmap):
        dist_threshold = 2.0
        # agent_pos = [agent_x, agent_y]
        # goals = []
        self.goals=[]

        for loc in self.location_map:
            temppos=[0,0]
            temppos[0]= self.location_map[loc][0]+random.uniform(-0.5,0.5)
            temppos[1]= self.location_map[loc][1]+random.uniform(-0.5,0.5)
            if temppos[0] > params_searchmap.xmax or temppos[0] < params_searchmap.xmin:
                temppos[0]=self.location_map[loc][0]
            if temppos[1] > params_searchmap.ymax or temppos[1] < params_searchmap.ymin:
                temppos[1]=self.location_map[loc][1]

            tmp_pos = PoseStamped()
            tmp_pos.header.frame_id='map'
            tmp_pos.pose.position.x=temppos[0]
            tmp_pos.pose.position.y=temppos[1]
            tmp_pos.pose.orientation.w=1.0
            self.goals.append(tmp_pos)
            # goals.append(temppos)
        # print("goals")
        # print(self.goals)
        ang_reso=math.pi/3
        num_angle = (int)(2*math.pi/ang_reso)
        dist=4.0
        for k in range(num_angle):
            temppos=[0,0]
            temppos[0]= self.agent2_pose[0]+ math.cos(k*ang_reso)*(dist+random.uniform(-0.5,0.5))+random.uniform(-0.5,0.5)
            temppos[1]= self.agent2_pose[1]+math.sin(k*ang_reso)*(dist+random.uniform(-0.5,0.5))+random.uniform(-0.5,0.5)
            if temppos[0] > params_searchmap.xmax or temppos[0] < params_searchmap.xmin:
                continue
            if temppos[1] > params_searchmap.ymax or temppos[1] < params_searchmap.ymin:
                continue

            tmp_pos = PoseStamped()
            tmp_pos.header.frame_id='map'
            tmp_pos.pose.position.x=temppos[0]
            tmp_pos.pose.position.y=temppos[1]
            tmp_pos.pose.orientation.w=1.0
            self.goals.append(tmp_pos)

        for i, fpose in enumerate(self.frontier_poses.poses):
            if i<5:
                tmp_pose = PoseStamped()
                tmp_pose.header.frame_id='map'
                tmp_pose.pose=fpose
                self.goals.append(tmp_pose)

        return self.goals

    def generate_global_trjs(self):
        trjs=[]
        for goal_pos in goals:
            # init_pos = [self.cur_state[0], cur_state[1]]
            cspace.reset_environment(params_global.boundaries,init_pos,goal_pos, obstacles)
            planner.reset_cspace(cspace)
            path, path_idx = planner.search(False, goal_pos)
            if path!=None:
                xs=[]
                ys=[]
                for i in range(len(path)):
                    xs.append(path[i][0])
                    ys.append(path[i][1])
                sp=Spline2D(xs,ys)
                trjs.append(sp)

        return trjs


    def globalmap_cb(self,msg):
        self.global_map= msg
        rospy.loginfo("global_map_cb")
        if self.map_received==False:
            print(self.global_map.info)
            self.crop_globalmap(self.params_searchmap, msg.info)
            self.map_received=True
        else:
            rospy.loginfo("global_map_cb")
            #self.publish_search_map()
    def agent1_localmap_cb(self,msg):
        # rospy.loginfo("agent1 local_callback")
        self.agent1_local_map= msg
        self.update_occ_grid_map(msg)

    def agent2_localmap_cb(self,msg):
        #local map w.r.t. {map_en}
        # rospy.loginfo("agent2 local_callback")
        self.agent2_local_map= msg
        self.update_occ_grid_map(msg)

    def execute_cb(self, goal):
        rospy.loginfo("search execute_cb: current exploration: %.2lf", self.entropy)
        if self.called_once==False:
                self.agent_history.append([self.agent2_pose[0], self.agent2_pose[1]])

        #if self.no_count<5:
        if self.near_goal==False and self.called_once==True:
            rospy.loginfo("Robot is not close to the previous goal. wait until it's near to that gaol")
            rospy.sleep(2.5)
            self.result_.goal.pose.orientation=self.agent2_ori
            self.publish_selected_goal(self.result_.goal)
            #self.no_count=self.no_count+1
            self._as.set_succeeded(self.result_)
            return
        #    else:
        #        self.no_count=0


        # print("search execute_cb")
        # sample_goals = self.goal_sampling_waypoints()
        if self.entropy<0.6:
            self.horizon=30
        if self.entropy<0.3:
            self.horizon=-1
        trjs_candidate =[]

        if self.entropy>0.3:
            res=self.call_service()
            if len(res.multiplan.poses)>0:
                gtrjs = self.Convert_Poses2trjs(res.multiplan.poses)
                sp_gtrjs = self.trjs_to_sample(gtrjs,self.agent_history, self.horizon)
                for gtrj in sp_gtrjs:
                    trjs_candidate.append(gtrj)
                horizon=self.horizon
                print("global_trjs: ", len(trjs_candidate))
                while len(trjs_candidate)<1:
                    # self.agent_history=[]
                    horizon+=5
                    sp_gtrjs = self.trjs_to_sample(gtrjs,self.agent_history, 50)
                    for gtrj in sp_gtrjs:
                        trjs_candidate.append(gtrj)
                    if horizon>80:
                        self.agent_history=[]
            else:
                local_trjs = lane_state_sampling_test(self.agent2_pose, self.params_searchmap, self.search_region)
                for ltrj in local_trjs:
                    trjs_candidate.append(ltrj)

            rospy.loginfo("selecting best_trjs")
            best_trj=self.calc_IG_trjs(trjs_candidate, self.params_searchmap, self.horizon)
            self.publish_selected_path(best_trj)
            self.result_.goal = self.choose_goal_from_trj(best_trj, self.params_searchmap, self.agent_history,self.horizon)
            self.publish_selected_path(best_trj)
        else:
            self.goals = self.goal_sampling_frontiers()
            if len(self.goals)>0:
                num=random.randint(0, len(self.goals)-1)
                self.result_.goal=self.goals[num];
            else:
                rospy.loginfo("The entire map is explored: %.2lf", self.entropy)


        self.agent_history.append([self.result_.goal.pose.position.x, self.result_.goal.pose.position.y])
        #self.publish_selected_path(best_trj)
        # self.spot_movecli.cancel_all_goals()
        self.called_once=True
        self.publish_selected_goal(self.result_.goal)
        self._as.set_succeeded(self.result_)
        self.last_goal = self.result_.goal
        self.entropy = self.get_map_entropy() / self.initial_entropy 
        # rospy.sleep(1.0)
        # self.agent2_move_base(self.result_.goal)
        # rospy.loginfo("action finished")


    def frontier_poses_Cb(self, msg):
        self.frontier_poses=msg

    def robot_pose_Cb(self, msg):
        #HSR pose call back
        # print("execute_cb")
        self.robot_pose[0]=msg.pose.position.x
        self.robot_pose[1]=msg.pose.position.y
        robot_orientation=msg.pose.orientation

        #get yaw angle from quarternion
        orientation_list=[robot_orientation.x, robot_orientation.y, robot_orientation.z,robot_orientation.w]
        roll,pitch,yaw=euler_from_quaternion(orientation_list)
        self.robot_pose[2]=yaw

        # self.start_pose.pose.position.x=self.robot_pose[0]+0.3
        # self.start_pose.pose.position.y=self.robot_pose[1]+0.3
 
        # self.publish_search_map()

    def agent2_pose_Cb(self, msg):
        #a1 pose call back : pose w.r.t map_en frame
        # pose_map=PoseStamped()
        # msg.header.stamp=rospy.Time.now()
        # try: 
            # now= rospy.Time.now()-rospy.Duration(3.0)
            # self.listener.waitForTransform(_ORIGIN_TF,_OFFSET_TF,rospy.Time(0), rospy.Duration(2.0))
            # self.listener.waitForTransform(_ORIGIN_TF,_OFFSET_TF,rospy.Time(), rospy.Duration(3.0))
            # transform = self.tf_buffer.lookup_transform(_OFFSET_TF, _ORIGIN_TF,rospy.Time(0),rospy.Duration(1.0))
            # pose_map=self.listener.transformPose(_OFFSET_TF, pose_map)
            # print("pose_map", pose_map)
        # except:
            # rospy.loginfo("transformPosefailed")
            
        self.agent2_pose[0]=msg.pose.position.x
        self.agent2_pose[1]=msg.pose.position.y
        self.agent2_ori= msg.pose.orientation
        robot_orientation=msg.pose.orientation

        #get yaw angle from quarternion
        orientation_list=[robot_orientation.x, robot_orientation.y, robot_orientation.z,robot_orientation.w]
        # roll,pitch,yaw=euler_from_quaternion(orientation_list)
        yaw=0.0
        self.agent2_pose[2]=yaw

        self.start_pose.pose.position.x=self.agent2_pose[0]+0.2
        self.start_pose.pose.position.y=self.agent2_pose[1]+0.25
 
        self.publish_search_map()


        if self.called_once:
            x_err = self.last_goal.pose.position.x-self.agent2_pose[0]
            y_err = self.last_goal.pose.position.y-self.agent2_pose[1]
            temp_dist=0.0;
            temp_dist+=math.pow(x_err,2)
            temp_dist+=math.pow(y_err,2)
            temp_dist=math.sqrt(temp_dist)
            if temp_dist<1.75:
                self.near_goal=True
            else:
                self.near_goal=False
            #print("near goal:", self.near_goal)
            #rospy.loginfo("near goal: ", self.near_goal)



        # self.publish_search_map()
        
    def agent2_move_base(self, agent2_goal):
        self.spot_movecli.wait_for_server()
        agent2_goal.header.stamp = rospy.Time.now()
        agent2_goal.header.frame_id = "map"
        agent2_movegoal =MoveBaseGoal()
        agent2_movegoal.target_pose = agent2_goal
        self.spot_movecli.send_goal(agent2_movegoal)
        self.spot_movecli.wait_for_result(rospy.Duration(20.0))
        action_state=self.spot_movecli.get_state()
        rospy.loginfo(action_state)
        if action_state == GoalStatus.SUCCEEDED:
            rospy.loginfo("Navigation Succeeded")
        else:
            rospy.loginfo("Still moving...")


    def publish_selected_path(self, trj):
        path_msg = Path()
        path_msg.header.stamp=rospy.Time.now()
        path_msg.header.frame_id='map'
        for i in range(len(trj[0])):
            tmp_pose = PoseStamped()
            # tmp_pose.header.frame_id='map'
            # tmp_pose.header.stamp=rospy.Time.now()
            tmp_pose.pose.position.x = trj[0][i]
            tmp_pose.pose.position.y = trj[1][i]
            path_msg.poses.append(tmp_pose)

        self.best_trj_pub.publish(path_msg)

    def publish_selected_goal(self, goal_pose):
        goal_msg = PoseStamped()
        goal_msg.header.stamp=rospy.Time.now()
        goal_msg.header.frame_id='map'
        goal_msg.pose = goal_pose.pose

        self.selected_goal_pub.publish(goal_msg)


    def Convert_Poses2trjs(self, posesarray):
        trjs=[]
        for path in posesarray:
            xs=[]
            ys=[]
            for pose_idx in range(len(path.poses)):
                if pose_idx%10==0 or (pose_idx==len(path.poses)-1):
                    xs.append(path.poses[pose_idx].position.x)
                    ys.append(path.poses[pose_idx].position.y)
            sp=Spline2D(xs,ys)
            trjs.append(sp)

        return trjs

    def near_history(self,px,py, history):
        #return False
        if self.entropy<0.4:
            min_dist = 2.0
        else:
            min_dist = 2.0
        for i in range(len(history)):
            # print(i, "-th history: ",  history[i])
            tempdist = math.sqrt((px - history[i][0])**2+(py-history[i][1])**2) #distance to gaol
            if tempdist < min_dist:
                return True

        return False




    def get_expected_entropy_infov(self,pos, params_searchmap):

        center_x=pos[0]
        center_y=pos[1]
        minx, miny, maxx, maxy, xw, yw = calc_grid_map_config(params_searchmap.xyreso
                                            , center_x,center_y, params_searchmap.sensor_range)
        print("min_x: ", minx)
        print("min_y:", miny)
        print("max_x:", maxx)
        print("max_y:", maxy)
        precast = precasting(minx,miny, xw, yw, params_searchmap.xyreso,
                            params_searchmap.yawreso, center_x,center_y)
        # print("precast", precast)
        print("center_x: ", center_x, ", center_y: ", center_y)

        #generate min_angle_map[angleid]=distance
        min_angle_map={}
        for ix_local in range(xw-1):
            for iy_local in range(yw-1):
                px = minx+ix_local*params_searchmap.xyreso
                py = miny+iy_local*params_searchmap.xyreso

                if px >= params_searchmap.xmax or px <= params_searchmap.xmin:
                    continue
                if py >= params_searchmap.ymax or py <= params_searchmap.ymin:
                    continue

                gmap_idx = self.Coord2CellIdx(px,py, self.global_map.info)
                if self.global_map.data[gmap_idx]>0:
                    #obs_angle = atan_zero_to_twopi(px-center_x,py-center_y)
                    obs_angle = atan_zero_to_twopi(py-center_y,px-center_x)
                    angleid =  int(math.floor(obs_angle / params_searchmap.yawreso))
                    d= math.sqrt((px-center_x)**2 + (py-center_y)**2)
                    if angleid in min_angle_map:
                        if d<min_angle_map[angleid]:
                            min_angle_map[angleid]=d
                    else:
                        min_angle_map[angleid]=d

        #check feasible index (output: set of infeasible map_indices )
        impossible_idx=[]
        for angle_id, min_dist in min_angle_map.items():
            gridlist = precast[angleid]
            for grid in gridlist:
                if grid.d > min_dist:
                    search_idx = self.Coord2CellIdx(grid.px, grid.py,self.search_region.info)
                    impossible_idx.append(search_idx)
                    
        #print("impossible_idx", impossible_idx)
        #input("---continue")

        # print("impossible_idx", impossible_idx)
        # input("here")
        #iteration for calculating entropy_map
        entropy_sum=0.0
        cell_count=0
        for ix_local in range(xw-1):
            for iy_local in range(yw-1):
                px = minx+ix_local*params_searchmap.xyreso
                py = miny+iy_local*params_searchmap.xyreso
                # print("px: ", px, "py: ", py)
                if px >= params_searchmap.xmax or px <= params_searchmap.xmin:
                    continue
                if py >= params_searchmap.ymax or py <= params_searchmap.ymin:
                    continue

                search_idx = self.Coord2CellIdx(px,py, self.search_region.info)
                #convert log-occ to probability
                if search_idx not in impossible_idx:
                    #print("px: ", px, ", py: ", py)
                    #count unknown cells 
                    if self.search_region.data[search_idx]==0:
                    #if self.search_region.data[search_idx]!=int(l_occ) and self.search_region.data[search_idx]!=int(l_free):
                        cell_count=cell_count+1

                    #p= 1-1./(1.0+np.exp(self.search_region.data[search_idx]))
                    #jf p>0.0 and p<1.0:                 
                    #    entropy_sum+=-(p*math.log(p)+(1-p)*math.log(1-p))
        print("cell counts: ", cell_count)
        entropy_sum=cell_count*0.693147


        #hel=input("---continue")
        # print("impossible_idx")
        # print(impossible_idx)
        # print("len(info_region)", len(self.info_region.data))
        # for i_idx in impossible_idx:
         #    self.info_region.data[i_idx]=100
        


        return entropy_sum

        ##iterating through occ_map
        ##find the min distance for each angle for obstacle points


    def get_entropy_infov(self,pos, params_searchmap):
        #get the expected local measurement entropy map w.r.t global postiion 
        #x= pos[0], y=pos[1]
        px=pos[0]
        py=pos[1]
        minx, miny, maxx, maxy, xw, yw = calc_grid_map_config(params_searchmap.xyreso
                                            , px,py, params_searchmap.sensor_range)
        entropy_sum=0.0
        #iterate local sensor range
        for ix_local in range(xw-1):
            for iy_local in range(yw-1):
                px = minx+ix_local*params_searchmap.xyreso
                py = miny+iy_local*params_searchmap.xyreso
                # print("px: ", px, "py: ", py)
                if px >= params_searchmap.xmax or px <= params_searchmap.xmin:
                    continue
                if py >= params_searchmap.ymax or py <= params_searchmap.ymin:
                    continue

                search_idx = self.Coord2CellIdx(px,py, self.search_region.info)
                #convert log-occ to probability
                # rospy.loginfo("before_entropy")
                p= 1-1./(1.0+np.exp(self.search_region.data[search_idx]))
                # rospy.loginfo("after_entropy")
                if p>0.0 and p<1.0:
                    #convert log-occ to probability
                    entropy_sum+=-(p*math.log(p)+(1-p)*math.log(1-p))

        return entropy_sum


    def calc_IG_trjs(self, trj_candidates, params_global, horizon=25):
        #Calculating Information gain on trajectories
        #1) calculating sampling point for time horizon 
        #2) calculating FOV region over sampling points 
        #3) Collecting IG gain for overlapped region
        #Suppose we have maximum velocity/ arc length
        #sampling points w.r.t distance 
        # emap=self.search_region
        if len(trj_candidates)>0:
            w_t=1.0
            igs=[]
            travels=[]
            tpoints=[]
            for j, trj in enumerate(trj_candidates):
                ig=0
                travel=0
                ref_x = trj[0][0]
                ref_y = trj[1][0]
                for i in range(len(trj[0])):
                        #dx = trj[0][i]-trj[0][i-1]
                        #dy = trj[1][i]-trj[1][i-1]
                     #if i=>0:
                     #   dx = trj[0][i]-trj[0][i-1]
                     #   dy = trj[1][i]-trj[1][i-1]
                     #   travel+=(dx**2+dy**2)**0.5

                      if i%13==0 or i==len(trj[0])-1:
                          if horizon>0:
                              if i<horizon:
                                  # print("i: ", i, "x: ", trj[0][i], ", y: ", trj[1][i], ", ig:", ig)
                                ig+= self.get_expected_entropy_infov([trj[0][i],trj[1][i]],params_global)
                                dx=trj[0][i]-ref_x
                                dy=trj[1][i]-ref_y
                                travel+=(dx**2+dy**2)**0.5
                                ref_x=trj[0][i]
                                ref_y=trj[1][i]
                          else:                     #the infinite horizon case
                              ig+= self.get_expected_entropy_infov([trj[0][i],trj[1][i]],params_global)
                              dx=trj[0][i]-ref_x
                              dy=trj[1][i]-ref_y
                              travel+=(dx**2+dy**2)**0.5
                              ref_x=trj[0][i]
                              ref_y=trj[1][i]
                              self.weight_travel=0.5

                # print("travel:" , travel)
                # print("ig", ig, ", travel: ",travel)
                cost = self.weight_entropy*ig-self.weight_travel*travel
                igs.append(cost)
                travels.append(travel)
                tpoints.append([trj[0][-1], trj[1][-1]])
            print("igs", igs)
            print("travels", travels)
            print("tpoints", tpoints)
            sorted_ig = sorted(((v,i) for i, v in enumerate(igs)),reverse=True)
            max_idx = sorted_ig[0][1]
            print("best trj idx : ", max_idx, "total num of candidates: ", len(igs))
            print("best_ig[] : ", igs[max_idx])
            # print("best information gain : ", sorted_ig[0][0])
            return trj_candidates[max_idx]
        else:
            print("no trajectory candidates")

    def trjs_to_sample(self,trjs,history, horizon=20):
        spline_trjs=[]
        for i, sp in enumerate(trjs):
            ds = 0.3
            s = np.arange(0, sp.s[-1], ds)
            # print("len(s):", len(s), ", sp[-1]:", sp.s[-1])
            rx, ry, ryaw, rk = [], [], [], []
            s_iter=0
            for j, i_s in enumerate(s):
                ix, iy = sp.calc_position(i_s)
                if horizon>0 and j<(2.5*horizon):
                    rx.append(ix)
                    ry.append(iy)
                if horizon<0:
                    rx.append(ix)
                    ry.append(iy)

            #if not self.near_history(rx[-1],ry[-1], history):
            if len(rx)>horizon:
                if not self.near_history(rx[horizon-1],ry[horizon-1], history):
                    spline_trjs.append([rx, ry])
            else:
                spline_trjs.append([rx, ry])

                #if not self.near_history(rx[0],ry[0], history):
                #    spline_trjs.append([rx, ry])

        return spline_trjs


    def choose_goal_from_trj(self, best_trj, params_globalmap, history, horizon=20):
        goal =[0,0]

        if len(best_trj[0])>horizon and horizon>0:
            if not self.near_history(best_trj[0][horizon],best_trj[1][horizon], history):
                goal = [best_trj[0][horizon-1], best_trj[1][horizon-1]]
            else:
                idx=horizon
                while (self.near_history(best_trj[0][idx],best_trj[1][idx], history)==False) and (idx<len(best_trj[0])-1):
                    idx=idx+1
                goal = [best_trj[0][idx], best_trj[1][idx]]
                
        elif horizon<0:
            goal = [best_trj[0][-1], best_trj[1][-1]]
        else:
            goal = [best_trj[0][-1], best_trj[1][-1]]


        if goal[0]> params_globalmap.xmax:
            goal[0]=params_globalmap.xmax-0.5
        elif goal[0]< params_globalmap.xmin:
            goal[0]=params_globalmap.xmin+0.5
        if goal[1]> params_globalmap.ymax:
            goal[1]=params_globalmap.ymax-0.5
        elif goal[1]< params_globalmap.ymin:
            goal[1]=params_globalmap.ymin+0.5

        goal_pos = PoseStamped()
        goal_pos.header.frame_id='map'
        goal_pos.header.stamp=rospy.Time.now()
        goal_pos.pose.position.x = goal[0]
        goal_pos.pose.position.y = goal[1]
        goal_pos.pose.orientation.w = 1.0
        goal_pos.pose.orientation=self.agent2_ori

        return goal_pos


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
    rospy.init_node('search_action')
    server = Map_manager('search_action')
    rospy.loginfo("search_action server created")
    # rospy.spin()
