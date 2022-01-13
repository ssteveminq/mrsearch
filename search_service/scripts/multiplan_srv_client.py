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
from geometry_msgs.msg import PoseStamped, PoseArray
from nav_msgs.srv import GetMultiPlan 


class MultiPlan_manager(object):
    def __init__(self, wait=0.0):
        # initialize action client
        self.srv_client=rospy.ServiceProxy('/planner/planner/make_multiplan', GetMultiPlan)
        self.start_pose=PoseStamped()
        self.start_pose.header.frame_id='map'
        self.start_pose.pose.position.x=-7.5
        self.start_pose.pose.position.y=7.7
        self.goals=[]
        tmp_pos = PoseStamped()
        tmp_pos.header.frame_id='map'
        tmp_pos.pose.position.x=6
        tmp_pos.pose.position.y=8.0
        self.goals.append(tmp_pos)

        tmp_pos2 = PoseStamped()
        tmp_pos2.header.frame_id='map'
        tmp_pos2.pose.position.x=-4.5
        tmp_pos2.pose.position.y=11.4
        self.goals.append(tmp_pos2)

        tmp_pos3 = PoseStamped()
        tmp_pos3.header.frame_id='map'
        tmp_pos3.pose.position.x=-7.1
        tmp_pos3.pose.position.y=2.5
        self.goals.append(tmp_pos3)
        
      
        self.tolerance=0.5

        # fill ROS message
    def call_service(self):
        rospy.wait_for_service('planner/planner/make_multiplan')
        # try:
        ans = self.srv_client(self.start_pose,self.goals, self.tolerance)
        print(ans)
        print("service called")
        # except rospy.ServiceException, e:
            # print("Service call failed: %s", %e)
        

if __name__ == '__main__':
    rospy.init_node('multimap_manager_test')
    plan_manager = MultiPlan_manager(float(sys.argv[1]) if len(sys.argv) > 1 else 0.0)
    plan_manager.call_service()















































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
