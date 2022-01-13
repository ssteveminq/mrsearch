#!/usr/bin/env python

import actionlib
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import rospy
import tf.transformations

rospy.init_node('test_move_sopt')

# initialize action client
cli = actionlib.SimpleActionClient('spot/move_base', MoveBaseAction)

# wait for the action server to establish connection
cli.wait_for_server()
#-1.8 / 2.2
# 1.6 / 3.0 
# 7.8 / 1.6 
# input goal pose
goal_x = -11.0
goal_y = 2.5
goal_yaw = -2.7

# fill ROS message
pose = PoseStamped()
pose.header.stamp = rospy.Time.now()
pose.header.frame_id = "map"
pose.pose.position = Point(goal_x, goal_y, 0)
quat = tf.transformations.quaternion_from_euler(0, 0, goal_yaw)
pose.pose.orientation = Quaternion(*quat)

goal = MoveBaseGoal()
goal.target_pose = pose

# send message to the action server
cli.send_goal(goal)

# wait for the action server to complete the order
cli.wait_for_result()

# print result of navigation
action_state = cli.get_state()
if action_state == GoalStatus.SUCCEEDED:
        rospy.loginfo("Navigation Succeeded.")
