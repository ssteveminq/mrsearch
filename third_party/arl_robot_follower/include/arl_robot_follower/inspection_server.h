///////////////////////////////////////////////////////////////////////////////
//      Title     : inspection_server.h
//      Platforms : Ubuntu 64-bit
//      Copyright : Copyright© The University of Texas at Austin, 2021. All rights reserved.
//                 
//          All files within this directory are subject to the following, unless an alternative
//          license is explicitly included within the text of each file.
//
//          This software and documentation constitute an unpublished work
//          and contain valuable trade secrets and proprietary information
//          belonging to the University. None of the foregoing material may be
//          copied or duplicated or disclosed without the express, written
//          permission of the University. THE UNIVERSITY EXPRESSLY DISCLAIMS ANY
//          AND ALL WARRANTIES CONCERNING THIS SOFTWARE AND DOCUMENTATION,
//          INCLUDING ANY WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
//          PARTICULAR PURPOSE, AND WARRANTIES OF PERFORMANCE, AND ANY WARRANTY
//          THAT MIGHT OTHERWISE ARISE FROM COURSE OF DEALING OR USAGE OF TRADE.
//          NO WARRANTY IS EITHER EXPRESS OR IMPLIED WITH RESPECT TO THE USE OF
//          THE SOFTWARE OR DOCUMENTATION. Under no circumstances shall the
//          University be liable for incidental, special, indirect, direct or
//          consequential damages or loss of profits, interruption of business,
//          or related expenses which may arise from use of software or documentation,
//          including but not limited to those resulting from defects in software
//          and/or documentation, or loss or inaccuracy of data of any kind.
//
///////////////////////////////////////////////////////////////////////////////

#pragma once

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <arl_nav_msgs/GotoRegionAction.h>
#include <arl_robot_follower/FollowTargetAction.h>
#include <arl_robot_follower/InspectTargetAction.h>

#include <tf2_ros/buffer_interface.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

/**
 * Server API
 * 
 * ROS Subscribers
 *  follow_target [geometry_msgs/PoseStamped] - Listens to the pose of the follower robot.
 * 
 * ROS Action Clients
 *  goto_region [arl_nav_msgs/GoToRegionAction] - Sends the navigation commands to the follower robot.
 * 
 * ROS Action Servers
 *  follow_target [arl_robot_follower/FollowTargetAction] - The action provided by this server. Tells the follower robot
 *                                                          to begin following a target frame.
 */

namespace arl_robot_follower {

class InspectionServer {

public:
  InspectionServer(ros::NodeHandle& nh);

  ~InspectionServer();

private:

  // goal callback for the FollowTarget server
  bool executeCB(const arl_robot_follower::InspectTargetGoalConstPtr &goal);

  /**
   * @brief Produce the GoToRegionGoal for the region behind the robot that we want to stay inside.
   * @param goal The follow target goal
   * @return GotoRegionGoal in the target reference frame. This will be the requested distance behind
   *         the target frame (i.e. the negative x direction).
   */
  arl_nav_msgs::GotoRegionGoal getGoalInTargetFrame(const geometry_msgs::PoseStamped target_pose,
                                                    const double distance,
                                                    const double radius);

  double calcMinInspectionDistance(const std::string camera_frame,
                                   geometry_msgs::PoseStamped target_pose,
                                   const double max_theta);

  void waitForSensors(const ros::Duration timeout = ros::Duration(1.0));

  void imagesCb(const sensor_msgs::ImageConstPtr msg);

  void pointcloudCb(const sensor_msgs::PointCloud2ConstPtr msg);

  // actionlib
  actionlib::SimpleActionServer<arl_robot_follower::InspectTargetAction> inspect_server_;
  actionlib::SimpleActionClient<arl_nav_msgs::GotoRegionAction> gottoregion_client_;

  // subscribers
  ros::Subscriber image_sub_;
  ros::Subscriber pointcloud_sub_;

  // callback queue for the follower pose
  ros::CallbackQueue sensors_queue_;

  // node handle
  ros::NodeHandle nh_;
  ros::NodeHandle nh_sensors_;

  // loop rate
  ros::Rate loop_rate_;

  // holds the last received pose for the follower
  sensor_msgs::Image image_;
  sensor_msgs::PointCloud2 pointcloud_;

  // TF items
  tf2_ros::TransformListener listener_;
  tf2_ros::Buffer tf2_buffer_;
};

}  // namespace arl_robot_follower