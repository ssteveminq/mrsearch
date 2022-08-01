///////////////////////////////////////////////////////////////////////////////
//      Title     : robot_follower_server.cpp
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

#include <arl_robot_follower/robot_follower_server.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace arl_robot_follower {

RobotFollowerServer::RobotFollowerServer(ros::NodeHandle& nh) :
  nh_(nh),
  nh_pose_(nh),
  loop_rate_(1.0),
  follow_server_("follow_target", boost::bind(&RobotFollowerServer::executeCB, this, _1), false),
  gottoregion_client_("goto_region", true),
  listener_(tf2_buffer_)
{
  tf2_buffer_.setUsingDedicatedThread(true);
  nh_pose_.setCallbackQueue(&follower_pose_queue_);
  follow_server_.start();

  follower_pose_sub_ = nh_pose_.subscribe("follower_pose", 10, &RobotFollowerServer::followerPoseCb, this);

  // Set loop rate
  double loop_rate(20.0);
  loop_rate_ = ros::Rate(loop_rate);
}


RobotFollowerServer::~RobotFollowerServer()
{
  follow_server_.shutdown();
  gottoregion_client_.cancelGoal();
}


bool RobotFollowerServer::executeCB(const arl_robot_follower::FollowTargetGoalConstPtr &goal)
{
  if ( !gottoregion_client_.isServerConnected() ) {
    const std::string status = "Not connected to the GoToRegion controller of the follower.";
    follow_server_.setAborted(arl_robot_follower::FollowTargetResult(), status);   
  }

  // produce the GoToRegion goal in the reference frame of the target
  arl_nav_msgs::GotoRegionGoal region_goal = getGoalInTargetFrame(goal);

  // start the monitoring loop
  while (ros::ok()) {

    // check for cancellation
    if (!follow_server_.isActive()) {
      gottoregion_client_.cancelGoal();

      const std::string status = "FollowTarget goal cancelled by client.";
      follow_server_.setAborted(arl_robot_follower::FollowTargetResult(), status);
      return true;
    }

    // check for goal preemption
    if (follow_server_.isPreemptRequested()) {
      gottoregion_client_.cancelGoal();

      const std::string status = "FollowTarget goal preempted by client.";
      follow_server_.setPreempted(arl_robot_follower::FollowTargetResult(), status);
      return true;
    }

    // get follower pose
    waitForFollowerPose();

    region_goal.region_center.header.stamp = ros::Time::now();

    // transform follower pose into the target frame
    if ( tf2::getFrameId(follower_pose_) != goal->target_frame ) {
      follower_pose_ = tf2_buffer_.transform(follower_pose_, goal->target_frame, ros::Duration(1.0));
    }

    // check if the follower is already inside the desired goal region
    double distance;
    if ( !poseInRegion(follower_pose_, region_goal, distance) ) {

      // check if we're already moving
      const actionlib::SimpleClientGoalState state = gottoregion_client_.getState();
      const bool already_moving = !state.isDone();

      if (!already_moving) {
        ROS_ERROR("Sending New Goal");
        // if the follower needs to start moving, transmit the region goal to the planner
        gottoregion_client_.sendGoal(region_goal);
      }
    }

    // publish feedback
    arl_robot_follower::FollowTargetFeedback feedback;
    feedback.distance_to_target = distance;
    follow_server_.publishFeedback(feedback);

    ros::spinOnce();
    loop_rate_.sleep();
  }

  return true;
}


arl_nav_msgs::GotoRegionGoal RobotFollowerServer::getGoalInTargetFrame(const arl_robot_follower::FollowTargetGoalConstPtr &goal)
{
  arl_nav_msgs::GotoRegionGoal output;

  // place the robot in a circular region some distance behind the target
  const double mean_distance = 0.5*(goal->max_distance + goal->min_distance);
  output.region_center.pose.position.x = -mean_distance;
  output.region_center.header.frame_id = goal->target_frame;

  // radius of the circular region
  output.radius = 0.5*(goal->max_distance - goal->min_distance);

  // set the orientation so that the follower faces towards the target
  output.region_center.pose.orientation.w = 1;

  // tolerance on the orientation
  output.angle_threshold = 30.0 / 180.0 * M_PI;

  return output;
}


bool RobotFollowerServer::poseInRegion(const geometry_msgs::PoseStamped &pose,
                                       const arl_nav_msgs::GotoRegionGoal &goal,
                                       double &distance)
{
  const double delta_x = pose.pose.position.x - goal.region_center.pose.position.x;
  const double delta_y = pose.pose.position.y - goal.region_center.pose.position.y;
  const double delta_z = pose.pose.position.z - goal.region_center.pose.position.z;

  distance = std::sqrt( delta_x*delta_x +
                        delta_y*delta_y +
                        delta_z*delta_z );

  return distance <= goal.radius;
}


void RobotFollowerServer::waitForFollowerPose(const ros::Duration timeout)
{
  const double seconds = timeout.toSec();
  follower_pose_queue_.callAvailable(ros::WallDuration(seconds));
}


void RobotFollowerServer::followerPoseCb(geometry_msgs::PoseStampedConstPtr msg)
{
  follower_pose_ = *msg;
}

}  // namespace arl_robot_follower


int main(int argc, char **argv)
{
  ros::init(argc, argv, "robot_follower_server");

  ros::NodeHandle nh("");
  
  arl_robot_follower::RobotFollowerServer node(nh);
  ros::spin();
  
  return 0;
}