// ROS Modules
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Header.h>



using namespace std;

class Listener{
public:
    geometry_msgs::PoseWithCovariance pose;
    std_msgs::Header header;

    void loc_cb(const nav_msgs::Odometry::ConstPtr& msg)
    {
        header = msg->header;
        pose = msg->pose;
    }
};


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "tb1_pose_publisher_node");
  ros::NodeHandle n;
  ros::Rate loop_rate(500);

  Listener listener;

  ros::Subscriber tb1_loc_sub = n.subscribe("/tb1/odom", 1000, &Listener::loc_cb, &listener);
  ros::Publisher tb1_pose_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/tb1/pose", 1000);

  // To publish 
  geometry_msgs::PoseWithCovarianceStamped tb1_pose_msg;

  bool first_time = true;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  while (ros::ok()){

    current_time = ros::Time::now();

    tb1_pose_msg.header.stamp = current_time;
    tb1_pose_msg.pose = listener.pose;
    tb1_pose_msg.header.frame_id = "map";

    tb1_pose_pub.publish(tb1_pose_msg);
        
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}