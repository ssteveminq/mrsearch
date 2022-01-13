#!/usr/bin/env python
import roslib
import rospy
import actionlib
import roslib
import math
import sys
import rospy
import geometry_msgs.msg
from std_msgs.msg import String
from std_msgs.msg import Int8
from nav_msgs.msg import Odometry
import geometry_msgs.msg 
from geometry_msgs.msg import PoseStamped, Point, Quaternion
import tf.transformations
import tf
import tf2_ros
# import tf_conversions


class tf_converter(object):
    def __init__(self, wait=0.0):
        # self.goalpub=rospy.Publisher('move_base_simple/goal',PoseStamped,queue_size=10)
        # self.cli = actionlib.SimpleActionClient('spot/move_base', MoveBaseAction)
        # self.cli.wait_for_server()
        rospy.loginfo("hi")
        self.waypoints=[]
        pose_topic='global_pose'
        rospy.Subscriber(pose_topic,PoseStamped, self.robot_pose_Cb)
        initpose_topic='initialpose_gp'
        rospy.Subscriber(initpose_topic,PoseStamped, self.init_pose_Cb)
        self.br= tf2_ros.TransformBroadcaster()
        self.br2= tf2_ros.TransformBroadcaster()
        self.gpose=PoseStamped()
        self.gpose.pose.orientation.w=1.0
        self.initpose=PoseStamped()
        self.initpose.pose.orientation.w=1.0
        rospy.loginfo("hi2")
        # self.br= tf2_ros.TransformBroadcaster()
    def publish_tf(self):
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "map_en"
        t.child_frame_id = "base"
        t.transform.translation.x = self.gpose.pose.position.x
        t.transform.translation.y = self.gpose.pose.position.y
        t.transform.translation.z = 0.0
        t.transform.rotation = self.gpose.pose.orientation
        self.br.sendTransform(t)
    def publish_tf_init(self):
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "map_en"
        t.child_frame_id = "odom"
        t.transform.translation.x = self.initpose.pose.position.x
        t.transform.translation.y = self.initpose.pose.position.y
        t.transform.translation.z = 0.0
        # self.initpose.pose.orientation.w=1.0
        t.transform.rotation = self.initpose.pose.orientation
        # t.transform.roatation.w=1.0
        self.br2.sendTransform(t)


    def robot_pose_Cb(self, msg):
        # rospy.loginfo("call back")
        # br= tf2_ros.TransformBroadcaster()
        self.gpose=msg

    def init_pose_Cb(self, msg):
        # rospy.loginfo("call back")
        # br= tf2_ros.TransformBroadcaster()
        self.initpose=msg
  
            
    def starter(self,wait=0.0):
        # make sure the cont0roller is running

        r=rospy.Rate(2000)
        # rospy.spin()            
        while not rospy.is_shutdown():
            self.publish_tf()
            self.publish_tf_init()
            r.sleep()
            # rospy.spinonce()
        # fill ROS message
          
if __name__ == '__main__':
    rospy.init_node('map_mapen_tf')
    rospy.loginfo("hi")
    tf_manager = tf_converter()
    tf_manager.starter()
    # rospy.spin()


































