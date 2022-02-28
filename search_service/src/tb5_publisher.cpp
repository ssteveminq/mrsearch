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
  ros::init(argc, argv, "tb5_pose_publisher_node");
  ros::NodeHandle n;
  ros::Rate loop_rate(500);

  Listener listener;

  ros::Subscriber tb5_loc_sub = n.subscribe("/tb5/odom", 1000, &Listener::loc_cb, &listener);
  ros::Publisher tb5_pose_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/tb5/pose", 1000);

  // To publish 
  geometry_msgs::PoseWithCovarianceStamped tb5_pose_msg;

  bool first_time = true;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  while (ros::ok()){

    current_time = ros::Time::now();
    
    tb5_pose_msg.header.stamp = current_time;
    tb5_pose_msg.pose = listener.pose;
    tb5_pose_msg.header.frame_id = "map";

    tb5_pose_pub.publish(tb5_pose_msg);

    last_time = current_time;
        
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}