#!/usr/bin/env python
import roslib
import rospy
import actionlib
import controller_manager_msgs.srv
import control_msgs.msg
import trajectory_msgs.msg
import roslib
import math
import sys
import rospy
import actionlib
import control_msgs.msg
import controller_manager_msgs.srv
import trajectory_msgs.msg
import geometry_msgs.msg
import controller_manager_msgs.srv
from control_msgs.msg import JointTrajectoryControllerState
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
from std_msgs.msg import Int8
from nav_msgs.msg import Odometry, Path
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from search_service.msg import PathFollowAction, PathFollowFeedback, PathFollowResult
import tf.transformations

class waypoint_manager(object):
    def __init__(self, name, wait=0.0):

        self._action_name = name

        self._as = actionlib.SimpleActionServer(self._action_name, PathFollowAction, execute_cb=self.execute_cb, auto_start = False)
        self.feedback_ = PathFollowFeedback()
        self.result_= PathFollowResult()

        self.cli = actionlib.SimpleActionClient('/move_base/move', MoveBaseAction)
        print("move_base-before")
        # self.cli.wait_for_server()
        print("move_base-after")

        path_topic='agent1/tsp_path'
        rospy.Subscriber(path_topic, Path, self.hsr_path_Cb)
        self.desired_path = Path()

 
        self._as.start()
        rospy.loginfo("action_started")
        rospy.spin()

    def hsr_path_Cb(self, msg):
        print("path_callback")
        self.desired_path = msg

    def execute_cb(self, goal):
        rospy.loginfo("path_follow execute_cb")
        rospy.loginfo("wait_for_move_base")
        self.cli.wait_for_server()
        rospy.loginfo("wait_for_move_base-done")

        movegoal =MoveBaseGoal()
        for pose in self.desired_path.poses:
            print("pose", pose)

            # movegoal.target_pose.header= goal.header
            movegoal.target_pose= pose
            # rospy.loginfo("sending goal", self.goal)
            self.cli.send_goal(movegoal)
            self.cli.wait_for_result(rospy.Duration(10.0))
            rospy.loginfo("moving to the goal")
            action_state=self.cli.get_state()
            rospy.loginfo(action_state)
            if action_state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Navigation Succeeded = next waypoint")

        rospy.loginfo("Done?")
        self.result_.is_done=True
        self._as.set_succeeded(self.result_)
        # rospy.loginfo("sending new goal to move/base")


    # def starter(self,wait=0.0):
        # make sure the cont0roller is running
        
        # r=rospy.Rate(1)
        # rospy.spin()            
        # for waypoint in self.waypoints:
            # self.sendactiongoal(waypoint)
        # fill ROS message

if __name__ == '__main__':
    rospy.init_node('path_follow_mk')
    path_manager = waypoint_manager('path_follow_mk')
    # path_manager.starter()
















