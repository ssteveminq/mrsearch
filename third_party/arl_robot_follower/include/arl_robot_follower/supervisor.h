///////////////////////////////////////////////////////////////////////////////
//      Title     : supervisor.h
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

#include <queue>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/Path.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <vision_msgs/Detection3DArray.h>

#include <arl_nav_msgs/GotoRegionAction.h>
#include <arl_robot_follower/FollowTargetAction.h>
#include <arl_robot_follower/InspectTargetAction.h>

/**
 * Node API
 * 
 * ROS Parameters
 *  object_detection_threshold [double] - Minimum threshold to confirm the detection of an object.
 *  max_nav_tries [int] - The number of times to try a nav goal before giving up.
 *  max_inspect_tries [int] - The number of times to try a nav goal before giving up.
 *  debug_detected_object [bool] - Boolean flag. If true publish every time a new target is detected.
 * 
 * ROS Subscribers
 *  ~waypoint_plan [nav_msgs/Path] - The waypoint sequence for the demo.
 *  ~detected_objects [vision_msgs/Detection3DArray] - Receives results from the object detection feature.
 * 
 * ROS Publishers
 *  ~detected_objects [] - The waypoint sequence for the demo.
 * 
 * ROS Service Servers
 *  pause [std_srvs/SetBool] - Pause or unpause the demo.
 * 
 * ROS Action Clients
 *  follow_target [arl_robot_follower/FollowTargetAction] - Commands the follower robot to begin following behavior.
 *  inspect_target [arl_robot_follower/InspectTargetAction] - Commands the follower robot to inspect a target of interest.
 *  gotto_region [arl_nav_msgs::GotoRegionAction] - Commands the leader robot to go to a waypoint.
 */

namespace arl_robot_follower {

class Supervisor {

public:
  // basic constructor
  Supervisor();

  // destructor. cancels any goals on the action clients.
  ~Supervisor();

  /**
   * @brief Executes the UTDD autonomous recon demo.
   * @param lead_robot_name The name of the leader robot.
   *                        This robot must have a joint named
   *                        <lead_robot_name>/base_link
   * @param follow_distance The distance at which the follower
   *                        robot should try to stay from the leader.
   * @return True on success.
   */
  bool runDemo(const std::string lead_robot_name,
               const double follow_distance);

private:

  /**
   * @brief Constructs and sends a GotoRegion goal
   *        to the related action client. Use this to
   *        send the leader to the waypoints.
   */
  void sendGoToRegionGoal();

  /**
   * @brief Constructs and sends a InspectTarget goal
   *        to the related action client. Use this to
   *        command the follower.
   */
  void sendInspectionGoal();

  /**
   * @brief Constructs and sends a FollowTarget goal
   *        to the related action client. Use this to
   *        command the follower.
   * @param lead_robot_name The follower will attempt to track the
   *                        <lead_robot_name>/base_link frame.
   * @param follow_distance The follower will attempt to maintain this
   *                        distance to the target frame.
   */
  void sendFollowGoal(const std::string lead_robot_name,
                      const double follow_distance);

  // callback for waypoints_sub_
  // Receives the waypoint plan and copies it to waypoint_queue_
  void waypointsCb(const nav_msgs::PathConstPtr msg);

  // callback for detected_objects_sub_
  // Checks if an target has been detected for the first time (based on position).
  // Adds newly detected target to the inspection task queue.
  void detectedObjectsCb(const vision_msgs::Detection3DArrayConstPtr msg);

  // Pauses or unpauses the demo.
  bool pauseCb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

  // Callback when the follower completes an inspection of a target.
  // If there are more items in the task queue, it will activate the next one.
  void inspectionDoneCb(const actionlib::SimpleClientGoalState& state,
                        const InspectTargetResultConstPtr& result);

  // Callback when the leader reaches a waypoint.
  // If there are more waypoints in the plan, it will activate the next one.
  void waypointDoneCb(const actionlib::SimpleClientGoalState& state,
                      const arl_nav_msgs::GotoRegionResultConstPtr& result);

  /**
   * @brief Tests two detected objects for equality. This is based on the object
   *        classes and their positions.
   * @param first One object to test.
   * @param second The other object to test.
   * @param distance_threshold How close the object positions must be for them
   *                           to be considered the same object.
   * @return True if the two objects are deterimined to be the same.
   */
  bool compareObjects(const vision_msgs::ObjectHypothesisWithPose &first,
                      const vision_msgs::ObjectHypothesisWithPose &second,
                      const double distance_threshold) const;

  // actionlib clients
  actionlib::SimpleActionClient<arl_robot_follower::InspectTargetAction> inspect_client_;
  actionlib::SimpleActionClient<arl_robot_follower::FollowTargetAction> follow_client_;
  actionlib::SimpleActionClient<arl_nav_msgs::GotoRegionAction> gotoregion_client_;

  // services
  ros::ServiceServer pause_server_;

  // subscribers
  ros::Subscriber waypoints_sub_;
  ros::Subscriber detected_objects_sub_;

  // publishers
  ros::Publisher debug_detected_objects_pub;

  // node handles
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  // hold callback data
  sensor_msgs::Image image_;
  sensor_msgs::PointCloud2 pointcloud_;

  // pause status
  bool paused_;

  // the waypoints the leader will drive to
  std::queue<geometry_msgs::PoseStamped> waypoint_queue_;

  // the inspection tasks to be done by the follower
  std::list<vision_msgs::ObjectHypothesisWithPose> inspection_task_queue_;

  // inspection tasks completed by the follower
  std::list<vision_msgs::ObjectHypothesisWithPose> completed_tasks_;

  // we attempt each navigation or inspection task a number of times before giving up
  int nav_attempts_made_;
  int inspect_attempts_made_;
  int total_tgts_;
  
  // cache for resuming a paused follow target goal
  std::string lead_robot_name_;
  double follow_distance_;
};

}  // namespace arl_robot_follower