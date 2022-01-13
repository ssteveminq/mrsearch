#!/usr/bin/python
# -*- coding: utf-8 -*-
import hsrb_interface
from hsrb_interface import geometry
import rospy
import sys
import tf
import actionlib
from geometry_msgs.msg import *
# from navi_service.msg import *
from frontier_exploration.msg import ExploreTaskAction, ExploreTaskResult, ExploreTaskFeedback, ExploreTaskGoal
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Point, Quaternion, PointStamped, PolygonStamped, Polygon, Point32
from visualization_msgs.msg import Marker 
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math

_CONNECTION_TIMEOUT = 10.0

def mains():
		# Initialize
		# client = actionlib.SimpleActionClient('explore_server',ExploreTaskAction)
		# rospy.loginfo("wait_server")
		# client.wait_for_server()
		# rospy.loginfo("requesting_action_server")
		# goal = ExploreTaskGoal()

	input_ = PolygonStamped()
	input_.header.frame_id = 'map'
	input_.header.stamp= rospy.Time.now()

	topic = 'boundary'
	markerpublisher = rospy.Publisher(topic, Marker,queue_size=10)
	robot_pose_topic='global_pose'
	pose_msg = rospy.wait_for_message(robot_pose_topic, PoseStamped,
			timeout=_CONNECTION_TIMEOUT)

	robot_pose_x=pose_msg.pose.position.x
	robot_pose_y=pose_msg.pose.position.y
	rospy.loginfo("robot pose x: %.2lf, y: %.2lf", robot_pose_x, robot_pose_y)


	   
			# for i in range(4):
	point_array=[]
	temppoint = Point32()
	temppoint.x=robot_pose_x+5.0
	temppoint.y=robot_pose_y+5.0
	temppoint.z=0.0
	input_.polygon.points.append(temppoint)
	point_array.append(Point(temppoint.x, temppoint.y, 0.0))

	temppoint2 = Point32()
	temppoint2.x=robot_pose_x-5.0
	temppoint2.y=robot_pose_y+5.0
	input_.polygon.points.append(temppoint2)
	point_array.append(Point(temppoint2.x, temppoint2.y, 0.0))

	temppoint3 = Point32()
	temppoint3.x=robot_pose_x-5.0
	temppoint3.y=robot_pose_y-5.0
	input_.polygon.points.append(temppoint3)
	point_array.append(Point(temppoint3.x, temppoint3.y, 0.0))

	temppoint4 = Point32()
	temppoint4.x=robot_pose_x+5.0
	temppoint4.y=robot_pose_y-5.0
	input_.polygon.points.append(temppoint4)
	point_array.append(Point(temppoint4.x, temppoint4.y, 0.0))


	temppoint5 = Point32()
	temppoint5.x=robot_pose_x+5.1
	temppoint5.y=robot_pose_y+5.0
	temppoint5.z=0.0
	input_.polygon.points.append(temppoint5)
	point_array.append(Point(temppoint5.x, temppoint5.y, 0.0))

	center = PointStamped()
	center.header.frame_id = 'map'
	center.header.stamp= rospy.Time.now()
	center.point = Point(robot_pose_x, robot_pose_y, 0.0)

        print input_.polygon.points


	vpoints = Marker()
	vpoints.header.frame_id = 'map'
	vpoints.header.stamp= rospy.Time.now()
	vpoints.type = vpoints.SPHERE_LIST
	vpoints.action = vpoints.ADD
	vpoints.scale.x = 1.0
	vpoints.scale.y = 1.0
	vpoints.scale.z = 1.0
	vpoints.color.a = 1.0
	vpoints.color.r = 1.0
	vpoints.color.g = 0.0
	vpoints.color.b = 0.0
	vpoints.pose.orientation.w = 1.0
	vpoints.points = point_array
	vpoints.ns = 'explore_points'
	vpoints.lifetime =rospy.Duration(100.0)
	markerpublisher.publish(vpoints)
	rospy.loginfo("marker published")

	rospy.sleep(0.5)


        client = actionlib.SimpleActionClient('explore_server',ExploreTaskAction)
        rospy.loginfo("wait_server")
        client.wait_for_server()
        rospy.loginfo("requesting_action_server")
        goal = ExploreTaskGoal()

        goal.explore_center = center
        goal.explore_boundary = input_

        client.send_goal(goal)
        client.wait_for_result(rospy.Duration(50.0))

        result = client.get_result()
        print result
		
if __name__ == '__main__':
	rospy.init_node('search_client')
	mains()
