#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/String.h>
#include <cmath>


using namespace std;

class Listener{
public:
    geometry_msgs::Pose pose;
    geometry_msgs::Pose initpose;

    void loc_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
    {
        pose = msg->pose.pose;
    }

    void initpose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        initpose = msg->pose;
    }
};

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "hsr_odom_publisher_mk");
    ros::NodeHandle n;
    ros::Rate loop_rate(1500);

    Listener listener;

    listener.initpose.orientation.w=1.0;
    listener.pose.orientation.w=1.0;

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Transform transform_init;

    ros::Subscriber loc_sub = n.subscribe("/laser_2d_pose", 1000, &Listener::loc_cb, &listener);
    ros::Subscriber initpose_sub = n.subscribe("/initialpose_gp", 1000, &Listener::initpose_cb, &listener);

    while (ros::ok()){

        //send_tf(map_en --> odom)
        transform.setOrigin(tf::Vector3(listener.pose.position.x, listener.pose.position.y, listener.pose.position.z));
        transform.setRotation(tf::Quaternion(listener.pose.orientation.x, listener.pose.orientation.y, listener.pose.orientation.z,
                               listener.pose.orientation.w));
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_footprint"));

        //transform_init.setOrigin(tf::Vector3(listener.initpose.position.x, listener.initpose.position.y, listener.initpose.position.z));
        //transform_init.setRotation(tf::Quaternion(listener.initpose.orientation.x, listener.initpose.orientation.y, listener.initpose.orientation.z,
                               //listener.initpose.orientation.w));
        //br.sendTransform(tf::StampedTransform(transform_init, ros::Time::now(), "map_en", "odom"));

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;

}

