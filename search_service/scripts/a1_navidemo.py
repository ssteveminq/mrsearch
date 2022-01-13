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

_ORIGIN_TF='map'
_OFFSET_TF='map_en'


class waypoint_manager(object):
    def __init__(self, wait=0.0):
        self.goalpub=rospy.Publisher('search_goal',PoseStamped,queue_size=10)
        # self.acgoal_sub = rospy.Subscriber()
        # self.cli = actionlib.SimpleActionClient('spot/move_base', MoveBaseAction)
        # self.cli.wait_for_server()
        self.waypoints=[]
        # self.goal =PoseStamped()
        # waypoint = PoseStamped()
        # waypoint.pose.position.x=5.5
        # waypoint.pose.position.y=9.2
        # waypoint.z=1.8
        # quat = tf.transformations.quaternion_from_euler(0, 0, 1.57)
        # waypoint.pose.orientation = Quaternion(*quat)
        # self.waypoints.append(waypoint)
        waypoint = PoseStamped()
        waypoint.pose.position.x=5.5
        waypoint.pose.position.y=9.6
        quat = tf.transformations.quaternion_from_euler(0, 0, 0.0)
        waypoint.pose.orientation = Quaternion(*quat)
        self.waypoints.append(waypoint)
     
        waypoint2 = PoseStamped()
        waypoint2.pose.position.x=5.4
        waypoint2.pose.position.y=9.0
        quat = tf.transformations.quaternion_from_euler(0, 0, 0.0)
        waypoint2.pose.orientation = Quaternion(*quat)
        self.waypoints.append(waypoint2)
        waypoint3 = PoseStamped()
        waypoint3.pose.position.x=3.5
        waypoint3.pose.position.y=7.2
        quat = tf.transformations.quaternion_from_euler(0, 0, -0.00)
        waypoint3.pose.orientation = Quaternion(*quat)
        self.waypoints.append(waypoint3)
        waypoint4 = PoseStamped()
        waypoint4.pose.position.x=-3.8
        waypoint4.pose.position.y=7.0
        quat = tf.transformations.quaternion_from_euler(0, 0, -1.57)
        waypoint4.pose.orientation = Quaternion(*quat)
        self.waypoints.append(waypoint4)

        waypoint7 = PoseStamped()
        waypoint7.pose.position.x=-7.4
        waypoint7.pose.position.y=7.2
        quat = tf.transformations.quaternion_from_euler(0, 0, -1.57)
        waypoint7.pose.orientation = Quaternion(*quat)
        self.waypoints.append(waypoint7)

        waypoint5 = PoseStamped()
        waypoint5.pose.position.x=-8.5
        waypoint5.pose.position.y=11.2
        quat = tf.transformations.quaternion_from_euler(0, 0, 1.57)
        waypoint5.pose.orientation = Quaternion(*quat)
        self.waypoints.append(waypoint5)
        waypoint6 = PoseStamped()
        waypoint6.pose.position.x=1.4
        waypoint6.pose.position.y=11.3
        quat = tf.transformations.quaternion_from_euler(0, 0, 1.57)
        waypoint6.pose.orientation = Quaternion(*quat)
        self.waypoints.append(waypoint6)
        waypoint8 = PoseStamped()
        waypoint8.pose.position.x=6.0
        waypoint8.pose.position.y=11.2
        quat = tf.transformations.quaternion_from_euler(0, 0, 1.57)
        waypoint8.pose.orientation = Quaternion(*quat)
        self.waypoints.append(waypoint8)




    def sendgoal(self, goal_num):
        # self.goal.x_map=3.0
        # self.goal.y_map=0.0
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose= self.waypoints[goal_num].pose
        # pose.pose.position = ,ciPoint(-6.0, -2.0,0.0)
        # pose.pose.orientation.w=1.0
        # quat = tf.transformations.quaternion_from_euler(0, 0, 0.5)
        # pose.pose.orientation = Quaternion(*quat)
        rospy.loginfo("sending new goal to movebase")
        self.goalpub.publish(pose)
        # if action_state == GoalStatus.SUCCEEDED:
            # rospy.loginfo("Navigation Succeeded")
    def starter(self,wait=0.0):
        # make sure the cont0roller is running
        goal_iter=0
        self.sendgoal(goal_iter)
        while not rospy.is_shutdown():
            if goal_iter==len(self.waypoints):
                return
        # r=rospy.Rate(1)
        # rospy.spin()            
        # for waypoint in self.waypoints:
        # self.sendgoal(self.waypoints[0])
            self.sendgoal(goal_iter)
            rospy.sleep(10.0)

            goal_iter=goal_iter+1
        # fill ROS message
          
if __name__ == '__main__':
    rospy.init_node('waypoint_a1')
    gaze_manager = waypoint_manager()
    gaze_manager.starter()
































