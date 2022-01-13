#! /usr/bin/env python

import rospy
import actionlib
import numpy as np

from std_msgs.msg import *
from geometry_msgs.msg import *
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Point, Quaternion, PointStamped, Twist
import control_msgs.msg
import controller_manager_msgs.srv
import trajectory_msgs.msg
from search_service.msg import ApproachAction, ApproachFeedback, ApproachResult
from sensor_msgs.msg import JointState
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import tf
from tf import TransformListener
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math

_ORIGIN_TF ='map'
_BASE_TF = 'base_link'

class ApproachServer(object):

    def __init__(self, name):
        # Init actionserver
        self._action_name = name
        self.robot_pose=np.zeros((5,1))
        self.iters=0;
        #x,y,theta
        self.x_err=0.0
        self.y_err=0.0
        self.vel_cmd = Twist()
        self.target_point = PointStamped()
        self.gazetarget_point = PointStamped()
        self.navtarget_pose =PoseStamped()
        self.target_pose = Point()
        self.IsActive = True
        self.IsGoal= False
        self.IsRotated=False
        self.IsGazed=False
        self.feedback_ = ApproachFeedback()
        self.result_= ApproachResult()
        self.target_yaw=0.0
        self.direction_z=1
        self.gaze_theta_err=0.0
        self.desired_head_pan=0.0
        # Preparation to use robot functions

        self.listener = tf.TransformListener()
	self.listener.waitForTransform(_ORIGIN_TF,_BASE_TF, rospy.Time(), rospy.Duration(5.0))
        self.vel_pub = rospy.Publisher('/hsrb/command_velocity', geometry_msgs.msg.Twist,queue_size=10)
        self.headPub = rospy.Publisher('/hsrb/head_trajectory_controller/command', JointTrajectory, queue_size=1)

        jointstates_topic='hsrb/joint_states'
	rospy.Subscriber(jointstates_topic, JointState, self.joint_state_Cb)

        robot_pose_topic='global_pose'
        rospy.Subscriber(robot_pose_topic, PoseStamped, self.robot_pose_Cb)

        self._as = actionlib.SimpleActionServer(self._action_name, villa_navi_service.msg.ApproachAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()


    def process_target(self,target_point, gazetarget_point ):

        # target_point is represented w.r.t base_link
        goal_x = target_point.point.x
        goal_y = target_point.point.y
        goal_theta = target_point.point.z

        diff_x=goal_x;
        diff_y=goal_y;
        distance =0.0
        distance +=math.pow(diff_x,2)
        distance +=math.pow(diff_y,2)
        distance  =math.sqrt(distance)

        # coeff=(float)(1.0/split_size)
        if goal_theta>math.pi:
            goal_theta=goal_theta+2*math.pi
        elif goal_theta<(-1*math.pi):
            goal_theta=goal_theta+2*math.pi

        if goal_theta>0:
            self.direction_z=1
        else:
            self.direction_z=-1

        #gaze target processing
        self.gazetarget_yaw= gazetarget_point.point.z
        # self.gaze_theta_err =gaze_theta - self.robot_pose[3] 

        # if self.gaze_theta_err>1.0:
            # diff_angle=0.5;
        # elif self.gaze_theta_err<-1.0:
            # diff_angle=-0.5;

    def update_angular_command(self, des_vel):
        self.vel_cmd.angular.z=des_vel
        # self.vel_pub.publish(vel_cmd)

    def update_linear_command(self, setzero=0):

        if setzero==0:
            self.vel_cmd.linear.x=0.2*self.x_err
            self.vel_cmd.linear.y=0.2*self.y_err
        else:
            self.vel_cmd.linear.x=0.0
            self.vel_cmd.linear.y=0.0

        # self.vel_pub.publish(vel_cmd)

    def update_head_command(self):
        if self.yaw_err>0.45:
            self.desired_head_pan=self.robot_pose[3]+0.1*self.yaw_err
        else:
            self.desired_head_pan=self.robot_pose[3]+0.4*self.yaw_err

   
    def control_base(self):

        # isObstacle=False
        #check obstacle()
        if self.IsGoal==False:
            if self.is_Obstacle==False:
                if self.check_ori_error(self.robot_pose[2],self.target_yaw,0.1):
                    rospy.loginfo("angular completed")
                    self.update_angular_command(0.0)
                    self.IsRotated=True
                    # return
                else:
                    # self.IsRotated=False
                    self.update_angular_command(self.direction_z*0.2)

                if self.check_gaze_error(self.gazetarget_yaw,0.2):
                    rospy.loginfo("gaze completed")
                    self.IsGazed=True
                else:
                    # self.IsGazed=False
                    self.update_head_command()

                # if (self.check_pos_error(self.navtarget_pose,0.4)):
                if (self.check_pos_error(0.5)):
                    rospy.loginfo("position completed")
                    self.update_linear_command(1)
                    if self.IsGazed==True:
                        self.feedback_.is_possible_go=True
                        self.result_.success=True
                        self.IsGoal=True
                        self.IsActive=False
                        self._as.set_succeeded(self.result_)
                        return
        
                else:
                    self.update_linear_command()
                    self.send_vel_command()
                    # self.send_vel_command()

                # self.send_vel_command()

            else:
                rospy.loginfo("target is occupied")
                
                # if(self.iters<5):
                    # self.navtarget_pose.pose.position.x-=0.1
                    # self.iters+=1
                self.feedback_.is_possible_go=False;
                self._as.publish_feedback(self.feedback_)
                return

    def execute_cb(self, goal):

        self.IsActive = True
        self.IsGoal= False
        self.IsRotated=False
        self.IsGazed=False

        self.navtarget_pose = goal.target
        # target_pose_base=
        orientation_list=[goal.target.pose.orientation.x, goal.target.pose.orientation.y, goal.target.pose.orientation.z, goal.target.pose.orientation.w]
        roll,pitch,yaw=euler_from_quaternion(orientation_list)

        # transform into baselink frame
        # gpose.header.frame_id = 'base_link'
        pose_in = PoseStamped()
        pose_in.header.stamp = rospy.Time(0)
        pose_in.header.frame_id= _ORIGIN_TF
        pose_in.pose = goal.target.pose

        gaze_pose_in = PoseStamped()
        gaze_pose_in.header.stamp = rospy.Time(0)
        gaze_pose_in.header.frame_id= _ORIGIN_TF
        gaze_pose_in.pose = goal.gaze_target.pose

	self.listener.waitForTransform(_ORIGIN_TF,_BASE_TF, rospy.Time(), rospy.Duration(2.0))
        # target_pose_base= self.listener.transformPose(_BASE_TF, pose_in)
        try:
            target_pose_base= self.listener.transformPose(_BASE_TF, pose_in)
            target_gaze_pose_base= self.listener.transformPose(_BASE_TF, gaze_pose_in)
            print "target pose w.r.t base_link", target_pose_base
            orientation_list=[target_pose_base.pose.orientation.x, target_pose_base.pose.orientation.y, target_pose_base.pose.orientation.z, target_pose_base.pose.orientation.w]
            roll,pitch,yaw=euler_from_quaternion(orientation_list)
            gazeorientation_list=[target_gaze_pose_base.pose.orientation.x, target_gaze_pose_base.pose.orientation.y, target_gaze_pose_base.pose.orientation.z, target_gaze_pose_base.pose.orientation.w]
            roll_gaze,pitch_gaze,yaw_gaze=euler_from_quaternion(gazeorientation_list)
        except:
            rospy.loginfo("transform failed")
            self.result_.success=False
            self.feedback_.is_possible_go=False
            self._as.publish_feedback(self.feedback_)
            self._as.set_succeeded(self.result_)
            
            return

        #x, y, theta(yaw)
        self.target_point.point.x= target_pose_base.pose.position.x-0.3
        self.target_point.point.y= target_pose_base.pose.position.y
        self.target_point.point.z= math.atan2(target_pose_base.pose.position.y,target_pose_base.pose.position.x)
        self.target_yaw = self.target_point.point.z+self.robot_pose[2] #rotation angle w.r.t map
        # yelf.target_point.point.z= self.robot_pose[2]

        self.gazetarget_point.point.x= target_gaze_pose_base.pose.position.x
        self.gazetarget_point.point.y= target_gaze_pose_base.pose.position.y
        gaze_x = target_gaze_pose_base.pose.position.x
        gaze_y = target_gaze_pose_base.pose.position.y
        self.gazetarget_point.point.z = math.atan2(gaze_y,gaze_x)

        # self.gazetarget_point.point.z= yaw_gaze

        #check obstacles for target_point
        obs_goal = villa_navi_service.msg.ObsCheckerGoal()
        pose = geometry_msgs.msg.PointStamped()
        pose.header.frame_id=_BASE_TF
        pose.header.stamp=rospy.Time.now()
        pose.point.x= self.target_point.point.x
        pose.point.y= self.target_point.point.y
        # pose.point.y= goal.target.pose.position.y
        obs_goal.pose =pose
        self.obspointPub.publish(pose)

        self.process_target(self.target_point,self.gazetarget_point)

        while self.IsActive ==True:
            self.control_base() 

        # self._as.set_succeeded()

    def joint_state_Cb(self, msg):
        #self_robot_pose[3] = head_pan_angle
        self.robot_pose[3]=msg.position[9]
        self.robot_pose[4]=msg.position[10]


    def robot_pose_Cb(self, msg):
        #self_robot_pose[0] = robot_position_x
        #self_robot_pose[1] = robot_position_y
        #self_robot_pose[2] = robot_theta_yaw

        self.robot_pose[0]=msg.pose.position.x
        self.robot_pose[1]=msg.pose.position.y
        robot_orientation=msg.pose.orientation

        #get yaw angle from quarternion
        orientation_list=[robot_orientation.x, robot_orientation.y, robot_orientation.z,robot_orientation.w]
        roll,pitch,yaw=euler_from_quaternion(orientation_list)
        self.robot_pose[2]=yaw

        pose_in = PoseStamped()
        pose_in.header.stamp = rospy.Time(0)
        pose_in.header.frame_id= _ORIGIN_TF
        pose_in.pose = self.navtarget_pose.pose
        # target_pose_base=PoseStamped()

	self.listener.waitForTransform(_ORIGIN_TF,_BASE_TF, rospy.Time(), rospy.Duration(2.0))
        try:
            target_pose_base= self.listener.transformPose(_BASE_TF, pose_in)
            # print target_pose_base
            self.target_point.point.x= target_pose_base.pose.position.x
            self.target_point.point.y= target_pose_base.pose.position.y
        except:
            rospy.loginfo("print failed")
            return
            


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
        # self.x_err = target_pos.pose.position.x-self.robot_pose[0]
        # self.y_err = target_pos.pose.position.y- self.robot_pose[1]
        self.x_err = self.target_point.point.x
        self.y_err = self.target_point.point.y

        # self.x_err = target_pos.point.x
        # self.y_err = target_pos.point.y
        # self.target_point

        temp_dist=0.0;
        temp_dist+=math.pow(self.x_err,2)
        temp_dist+=math.pow(self.y_err,2)
        temp_dist=math.sqrt(temp_dist)

        temp_criterion =math.sqrt(math.pow((temp_dist),2))
        rospy.loginfo("distance error: %.4lf ", temp_criterion);
        if (temp_criterion<err_criterion):
            return True

        return False

    def check_gaze_error(self, gaze_yaw, err_criterion):

        self.yaw_err = gaze_yaw -(self.robot_pose[3])
        temp_criterion =math.sqrt(math.pow(self.yaw_err,2))

        rospy.loginfo("gaze error: %.4lf ", temp_criterion);
        if (temp_criterion<err_criterion):
            return True
        return False
 
    def send_vel_command(self):
        if self.IsActive:
            if self.IsGoal==False:
                self.vel_pub.publish(self.vel_cmd)

if __name__ == '__main__':
    rospy.init_node('search_action')
    server = ApproachServer('search_action')
    rospy.loginfo("search_action server created")
    rospy.spin()
