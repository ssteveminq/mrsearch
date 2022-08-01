#pragma once

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <arl_robot_follower/InspectTargetAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>


class Listener {
  public:
    geometry_msgs::Pose curiosity_pose;

    bool new_region;

    void inspection_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
        curiosity_pose = msg->pose;
        new_region = 1;
    }

};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "curiosity_inspection_client");
  ros::NodeHandle nh_;
  ros::Rate loop_rate(100);

  double min_distance = 1.0;
  double max_distance = 3.0;
  double max_angle = 3.14;

  Listener listener;

  ros::Subscriber curiosity_sub_ = nh_.subscribe("/walrus/curiosity_region", 10, &Listener::inspection_cb, &listener);

  actionlib::SimpleActionClient<arl_robot_follower::InspectTargetAction> inspect_client_("inspect_target", true);

  ROS_INFO("Waiting for Inspection action server to start.");
  // wait for the action server to start
  inspect_client_.waitForServer();

  while (ros::ok()){

    // current_time = ros::Time::now();
    if (listener.new_region)
    {
      // Create and fill the action goal using the given info
      arl_robot_follower::InspectTargetGoal goal;
      goal.camera_frame = 'base_link';
      goal.target_pose.header.stamp = ros::Time::now();
      goal.target_pose.header.frame_id = 'map';
      goal.target_pose.pose = listener.curiosity_pose;
      goal.min_distance = min_distance;
      goal.max_distance = max_distance;
      goal.max_angle = max_angle;
      

      // Send goal to action server
      inspect_client_.sendGoal(goal);
      // Reset boolean var
      listener.new_region = 0;
    }

    ros::spinOnce();
  }

  return 0;
}
