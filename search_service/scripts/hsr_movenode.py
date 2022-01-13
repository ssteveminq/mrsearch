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
from nav_msgs.msg import Odometry
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Point, Quaternion
# import tf.transformations


class waypoint_manager(object):
    def __init__(self, wait=0.0):
        self.cli = actionlib.SimpleActionClient('move_base/move', MoveBaseAction)
        self.cli.wait_for_server()
        self.waypoints=[]
        self.lastgoal=PoseStamped()
        goal_pose_topic='search_goal'
        rospy.Subscriber(goal_pose_topic, PoseStamped, self.search_goal_Cb)

    def search_goal_Cb(self, goal):
        rospy.loginfo("goal callback")
        self.cli.wait_for_server()

        # action_state=self.cli.get_state()
        # if action_state == GoalStatus.ACTIVE:
            # self.cli.wait_for_result(rospy.Duration(1.0))

        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"
        agent2_movegoal =MoveBaseGoal()
        agent2_movegoal.target_pose = goal
        self.cli.send_goal(agent2_movegoal)
        self.cli.wait_for_result()
        self.cli.wait_for_result(rospy.Duration(5.0))
        action_state=self.cli.get_state()
        # rospy.loginfo(action_state)
        if action_state == GoalStatus.SUCCEEDED:
            rospy.loginfo("Navigation Succeeded")
        else:
            rospy.loginfo("Still moving...")



    def sendactiongoal(self,inputpoint):
        # self.goal.x_map=3.0
        # self.goal.y_map=0.0
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose.position = Point(inputpoint.x, inputpoint.y,0.0)
        quat = tf.transformations.quaternion_from_euler(0, 0, inputpoint.z)
        pose.pose.orientation = Quaternion(*quat)
        self.goal.target_pose = pose

        rospy.loginfo("sending new goal to move/base")
        self.cli.send_goal(self.goal)
        self.cli.wait_for_result()
        action_state=self.cli.get_state()
        rospy.loginfo(action_state)
        if action_state == GoalStatus.SUCCEEDED:
            rospy.loginfo("Navigation Succeeded")
    def starter(self,wait=0.0):
        # make sure the cont0roller is running
        # r=rospy.Rate(1)
        rospy.spin()            
        # for waypoint in self.waypoints:
            # self.sendactiongoal(waypoint)
        # fill ROS message
          
if __name__ == '__main__':
    rospy.init_node('waypoint_hsr')
    gaze_manager = waypoint_manager(float(sys.argv[1]) if len(sys.argv) > 1 else 0.0)
    gaze_manager.starter()


































