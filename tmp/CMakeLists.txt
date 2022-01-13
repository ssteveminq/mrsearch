cmake_minimum_required(VERSION 2.8.3)
project(tmp)

find_package(catkin REQUIRED
  roscpp
  roslib
  rospy
  geometry_msgs
  sensor_msgs
  std_msgs
  std_srvs
  tf
  actionlib
  actionlib_msgs
  message_generation
    )

catkin_metapackage()
