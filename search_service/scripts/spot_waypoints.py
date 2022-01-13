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
import tf.transformations


class waypoint_manager(object):
    def __init__(self, wait=0.0):
        self.cli = actionlib.SimpleActionClient('spot/move_base', MoveBaseAction)
        self.cli.wait_for_server()
        self.waypoints=[]
        self.goal =MoveBaseGoal()
        waypoint = Point()
        waypoint.x=-7.18
        waypoint.y=-2.0
        waypoint.z=1.8
        self.waypoints.append(waypoint)
        waypoint2 = Point()
        waypoint2.x=-6.8
        waypoint2.y=2.0
        waypoint2.z=1.8
        self.waypoints.append(waypoint2)
        waypoint3 = Point()
        waypoint3.x=0.8
        waypoint3.y=1.75
        waypoint3.z=0.0
        self.waypoints.append(waypoint3)
        waypoint4 = Point()
        waypoint4.x=9.5
        waypoint4.y=4.1
        waypoint4.z=1.9
        self.waypoints.append(waypoint4)
        waypoint5 = Point()
        waypoint5.x=4.08
        waypoint5.y=6.1
        waypoint5.z=1.8
        self.waypoints.append(waypoint5)
        waypoint6 = Point()
        waypoint6.x=-2.28
        waypoint6.y=6.0
        waypoint6.z=-1.7
        self.waypoints.append(waypoint6)
        waypoint7 = Point()
        waypoint7.x=-12.08
        waypoint7.y=5.0
        waypoint7.z=3.14
        self.waypoints.append(waypoint7)




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
        # rospy.spin()            
        for waypoint in self.waypoints:
            self.sendactiongoal(waypoint)
        # fill ROS message
          
if __name__ == '__main__':
    rospy.init_node('waypoint_spot')
    gaze_manager = waypoint_manager(float(sys.argv[1]) if len(sys.argv) > 1 else 0.0)
    gaze_manager.starter()


































