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
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from control_msgs.msg import JointTrajectoryControllerState
from std_msgs.msg import String
from std_msgs.msg import Int8
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseArray
from state_lattice_planner.msg import *


class MultiPlan_manager(object):
    def __init__(self, wait=0.0):
        # initialize action client
        # pose_topic='global_pose_a1'
        # rospy.Subscriber(pose_topic,PoseStamped, self.robot_pose_Cb)
        # self.srv_client=rospy.ServiceProxy('/planner/planner/make_multiplan', GetMultiPlan)
        self.path_pub=rospy.Publisher('test_path',Path,queue_size=10)
        self.path=Path()
        self.start_pose=PoseStamped()
        self.start_pose.header.frame_id='map'
        self.start_pose.pose.position.x=1.8
        self.start_pose.pose.position.y=11.3
        self.path.poses.append(self.start_pose)

        tmp_pos = PoseStamped()
        tmp_pos.header.frame_id='map'
        tmp_pos.pose.position.x=2.8
        tmp_pos.pose.position.y=11.4
        self.path.poses.append(tmp_pos)

        tmp_pos2 = PoseStamped()
        tmp_pos2.header.frame_id='map'
        tmp_pos2.pose.position.x=5.0
        tmp_pos2.pose.position.y=12.1
        self.path.poses.append(tmp_pos2)

        tmp_pos3 = PoseStamped()
        tmp_pos3.header.frame_id='map'
        tmp_pos3.pose.position.x=-4.0
        tmp_pos3.pose.position.y=14.1
        self.path.poses.append(tmp_pos3)

        tmp_pos4 = PoseStamped()
        tmp_pos4.header.frame_id='map'
        tmp_pos4.pose.position.x=-4.6
        tmp_pos4.pose.position.y=16.1
        self.path.poses.append(tmp_pos4)

        tmp_pos5 = PoseStamped()
        tmp_pos5.header.frame_id='map'
        tmp_pos5.pose.position.x=-4.0
        tmp_pos5.pose.position.y=17.1
        self.path.poses.append(tmp_pos5)

        tmp_pos6 = PoseStamped()
        tmp_pos6.header.frame_id='map'
        tmp_pos6.pose.position.x=-4.0
        tmp_pos6.pose.position.y=19.1
        self.path.poses.append(tmp_pos6)
        self.path_pub.publish(self.path)

        client = actionlib.SimpleActionClient('local_planner/state_lattice_planner',state_lattice_planner.msg.LocalNaviAction)
        client.wait_for_server()
        goal = state_lattice_planner.msg.LocalNaviGoal()
        goal.path = self.path
        client.send_goal(goal)
        client.wait_for_result()
        print("service called")
 



      
        self.tolerance=0.5
    def robot_pose_Cb(self, msg):
        # rospy.loginfo("call back")
        # br= tf2_ros.TransformBroadcaster()
        self.gpose=msg
        # self.start_pose.pose.position.x=self.gpose.pose.position.x
        # self.start_pose.pose.position.y=self.gpose.pose.position.y

        # fill ROS message
    def call_service(self):
        # rospy.wait_for_service('planner/planner/make_multiplan')
        # try:
        # ans = self.srv_client(self.start_pose,self.goals, self.tolerance)
        # print(ans.multiplan)
    
        PoseArray = ans.multiplan.poses[0]
        test_path = Path()
        test_path.header.frame_id = "odom"
        test_path.poses=[]
        for pose in ans.multiplan.poses[0].poses:
            tmp_pose=PoseStamped()
            tmp_pose.pose=pose
            test_path.poses.append(tmp_pose)
        print("test_path", test_path)

        client = actionlib.SimpleActionClient('follownavi_action',state_lattice_planner.msg.LocalNaviAction)

        client.wait_for_server()
        goal = villa_navi_service.msg.LocalNaviGoal()
        goal.path = test_path
        client.send_goal(goal)
        client.wait_for_result()
        print("service called")
        # except rospy.ServiceException, e:
            # print("Service call failed: %s", %e)
        

if __name__ == '__main__':
    rospy.init_node('localnavi_test')
    plan_manager = MultiPlan_manager(float(sys.argv[1]) if len(sys.argv) > 1 else 0.0)















































# rospy.init_node('test')

# # initialize action client
# cli = actionlib.SimpleActionClient('/hsrb/head_trajectory_controller/follow_joint_trajectory', control_msgs.msg.FollowJointTrajectoryAction)

# # wait for the action server to establish connection
# cli.wait_for_server()

# # make sure the controller is running
# rospy.wait_for_service('/hsrb/controller_manager/list_controllers')
# list_controllers = rospy.ServiceProxy('/hsrb/controller_manager/list_controllers', controller_manager_msgs.srv.ListControllers)
# running = False
# while running == False:
#     rospy.sleep(0.1)
#     for c in list_controllers().controller:
#         if c.name == 'head_trajectory_controller' and c.state == 'running':
#             running = True

# # fill ROS message
# goal = control_msgs.msg.FollowJointTrajectoryGoal()
# traj = trajectory_msgs.msg.JointTrajectory()
# traj.joint_names = ["head_pan_joint", "head_tilt_joint"]
# p = trajectory_msgs.msg.JointTrajectoryPoint()
# p.positions = [0.5, 0.5]
# p.velocities = [0, 0]
# p.time_from_start = rospy.Time(3)
# traj.points = [p]
# goal.trajectory = traj

# # send message to the action server
# cli.send_goal(goal)

# # wait for the action server to complete the order
# cli.wait_for_result()
