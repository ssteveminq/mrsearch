///////////////////////////////////////////////////////////////////////////////
//      Title     : inspection_server.cpp
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

#include <arl_robot_follower/inspection_server.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace arl_robot_follower {

InspectionServer::InspectionServer(ros::NodeHandle& nh) :
  nh_(nh),
  nh_sensors_(nh),
  loop_rate_(1.0),
  inspect_server_("inspect_target", boost::bind(&InspectionServer::executeCB, this, _1), false),
  gottoregion_client_("goto_region", true),
  listener_(tf2_buffer_)
{
  tf2_buffer_.setUsingDedicatedThread(true);
  nh_sensors_.setCallbackQueue(&sensors_queue_);
  inspect_server_.start();

  image_sub_ = nh_sensors_.subscribe("image", 10, &InspectionServer::imagesCb, this);
  pointcloud_sub_ = nh_sensors_.subscribe("pointcloud", 10, &InspectionServer::pointcloudCb, this);

  // Set loop rate
  double loop_rate;
  nh_.param<double>("loop_rate", loop_rate, 20.0);
  loop_rate_ = ros::Rate(loop_rate);
}


InspectionServer::~InspectionServer()
{
  inspect_server_.shutdown();
  gottoregion_client_.cancelGoal();
}


bool InspectionServer::executeCB(const arl_robot_follower::InspectTargetGoalConstPtr &goal)
{
  /** Input validation **/
  if (goal->max_distance <= 0) {
    const std::string msg = "Invalid max_distance. Must be positive!";
    ROS_ERROR("INVALID INSPECT MAX DIST. Aborting");
    inspect_server_.setAborted(arl_robot_follower::InspectTargetResult(), msg);
    return true;
  }
  if (goal->min_distance <= 0) {
    const std::string msg = "Invalid min_distance. Must be positive!";
    ROS_ERROR("INVALID INSPECT MIN DIST. Aborting");
    inspect_server_.setAborted(arl_robot_follower::InspectTargetResult(), msg);
    return true;
  }
  if (goal->camera_frame.empty()) {
    const std::string msg = "Invalid camera_frame. Must not be empty!";
    ROS_ERROR("INVALID INSPECT CAMERA FRAME. Aborting");
    inspect_server_.setAborted(arl_robot_follower::InspectTargetResult(), msg);
    return true;
  }

  // check that we're connected to the navigation server
  if ( !gottoregion_client_.isServerConnected() ) {
    const std::string msg = "Not connected to the GoToRegion controller of the follower.";
    ROS_ERROR("INSPECTION NOT CONNECTED TO GOTO REGION GOAL. Aborting");
    inspect_server_.setAborted(arl_robot_follower::InspectTargetResult(), msg);
  }

  // get the min distance to the target for the inspection
  // this is the HORIZONTAL distance, specifically
  double inspection_horz_distance = calcMinInspectionDistance(goal->camera_frame,
                                                              goal->target_pose,
                                                              goal->max_angle);

  const double tol_radius = 1.0;

  // constrain our inspection distance according to the action specification
  inspection_horz_distance = std::min(inspection_horz_distance, goal->max_distance-tol_radius);
  inspection_horz_distance = std::max(inspection_horz_distance, goal->min_distance+tol_radius);

  // create a region goal in the target frame
  const arl_nav_msgs::GotoRegionGoal region_goal = getGoalInTargetFrame(goal->target_pose,
                                                                        inspection_horz_distance,
                                                                        tol_radius);

  // start navigating
  gottoregion_client_.sendGoal(region_goal);

  // start the monitoring loop - wait for the navigation action to conclude
  while (ros::ok() && (gottoregion_client_.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)) {

    // check to see if the gotoregion gave up
    if (gottoregion_client_.getState().isDone()) {
      // resend goal
      ROS_ERROR("RESENDING INSPECT GOAL");
      gottoregion_client_.sendGoal(region_goal);
    }

    // check for cancellation
    if (!inspect_server_.isActive()) {
      gottoregion_client_.cancelGoal();
      ROS_ERROR("INSPECT TGT GOAL CANCELLED BY CLIENT. Aborting");
      const std::string status = "InspectTarget goal cancelled by client.";
      inspect_server_.setAborted(arl_robot_follower::InspectTargetResult(), status);
      return true;
    }

    // check for goal preemption
    if (inspect_server_.isPreemptRequested()) {
      gottoregion_client_.cancelGoal();
      ROS_ERROR("INSPECT TGT GOAL PREEMPTED BY CLIENT. Preemting status");

      const std::string status = "InspectTarget goal preempted by client.";
      inspect_server_.setPreempted(arl_robot_follower::InspectTargetResult(), status);
      return true;
    }

    ros::spinOnce();
    loop_rate_.sleep();
  }


  // collect data
  waitForSensors();

  // put data into result
  arl_robot_follower::InspectTargetResult result;
  result.photo = image_;
  result.pointcloud = pointcloud_;
  inspect_server_.setSucceeded(result);

  return true;
}


arl_nav_msgs::GotoRegionGoal InspectionServer::getGoalInTargetFrame(const geometry_msgs::PoseStamped target_pose,
                                                                            const double distance,
                                                                            const double radius)
{
  arl_nav_msgs::GotoRegionGoal output;

  tf2::Quaternion tf_quaternion;
  tf2::fromMsg(target_pose.pose.orientation, tf_quaternion);
  const double yaw = tf2::getYaw(tf_quaternion) + M_PI;

  // place the robot in a circular region some distance in front of the target
  output.region_center = target_pose;
  output.region_center.header.frame_id = tf2::getFrameId(target_pose);
  output.region_center.pose.position.x += distance*std::cos(yaw);
  output.region_center.pose.position.y += distance*std::sin(yaw);

  // radius of the circular region
  output.radius = radius;

  // set the orientation so that the follower faces towards the target
  const tf2::Quaternion tf_reverse_quaternion = tf2::inverse(tf_quaternion);
  output.region_center.pose.orientation = tf2::toMsg(tf_reverse_quaternion);

  // tolerance on the orientation
  output.angle_threshold = 5.0 / 180.0 * M_PI;

  return output;
}


double InspectionServer::calcMinInspectionDistance(const std::string camera_frame,
                                                           geometry_msgs::PoseStamped target_pose,
                                                           const double max_theta)
{
  // transform target into camera frame
  if ( camera_frame != tf2::getFrameId(target_pose) ) {
    target_pose = tf2_buffer_.transform(target_pose, camera_frame);
  }

  /** remember, in the camera frame y is up and z is forward of the robot **/

  // calc the angle and forward distance
  const double tan = std::tan(max_theta);
  return tan * target_pose.pose.position.y;
}


void InspectionServer::waitForSensors(const ros::Duration timeout)
{
  const double seconds = timeout.toSec();
  sensors_queue_.callAvailable(ros::WallDuration(seconds));
}


void InspectionServer::imagesCb(const sensor_msgs::ImageConstPtr msg)
{
  image_ = *msg;
}


void InspectionServer::pointcloudCb(const sensor_msgs::PointCloud2ConstPtr msg)
{
  pointcloud_ = *msg;
}


}  // namespace arl_robot_follower


int main(int argc, char **argv)
{
  ros::init(argc, argv, "inspection_server");

  ros::NodeHandle nh("~");

  arl_robot_follower::InspectionServer node(nh);
  ros::spin();

  return 0;
}
