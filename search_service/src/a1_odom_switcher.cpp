#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "tf/message_filter.h"
#include "tf/tf.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_listener.h"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/String.h>
#include <cmath>


using namespace std;

class Listener{
public:
    geometry_msgs::Pose pose;
    geometry_msgs::Pose initpose;
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener* tf2_listener; 
    int callback_iter;
    bool tf_failed=false;

    //Listener():callback_iter(0)
    //{
    //    tf2_listener = new tf2_ros::TransformListener(tf_buffer);
    //}

    void loc_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        pose = msg->pose;
        tf_failed = false;
        //if(callback_iter>10)
        //{
            //geometry_msgs::PoseStamped pose_in;     //added by mk, ryan
            //pose_in.header.stamp=msg->header.stamp;
            //pose_in.header.frame_id=msg->header.frame_id;
            //pose_in.pose=msg->pose;
     
          //geometry_msgs::PoseStamped pose_out;     //added by mk, ryan
          //try{

              //tf2::doTransform(*msg, *msg, map_en_to_map);
              //tf_buffer.transform(pose_in,pose_out, "map");
              //tf_failed=false;
          //}
          //catch (tf2::TransformException &ex)
          //{
              //tf_failed=true;
              //ROS_WARN("global_pose: Transform Failed: %s", ex.what());
              //sleep(0.10);
              //return;
          //}
          //pose= pose_out.pose;
        
          
          //callback_iter=0;
        //}
        //else
        //{
          //callback_iter++;
          //return;
        //}
    }

    void initpose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        initpose = msg->pose;
    }
};

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "a1_odom_switcher");
    ros::NodeHandle n;
    ros::Rate loop_rate(1000);

    Listener listener;

    listener.initpose.orientation.w=1.0;
    listener.pose.orientation.w=1.0;

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Transform transform_init;

    ros::Subscriber loc_sub = n.subscribe("/global_pose_a1_121", 5, &Listener::loc_cb, &listener);
    // ros::Subscriber initpose_sub = n.subscribe("/initialpose_gp", 1000, &Listener::initpose_cb, &listener);

    while (ros::ok()){

        ros::spinOnce();

        //send_tf(map_en --> odom)
        // if(!listener.tf_failed){
            transform.setOrigin(tf::Vector3(listener.pose.position.x, listener.pose.position.y, listener.pose.position.z));
            transform.setRotation(tf::Quaternion(listener.pose.orientation.x, listener.pose.orientation.y, listener.pose.orientation.z,
                                   listener.pose.orientation.w));
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base"));
        //}

        //transform_init.setOrigin(tf::Vector3(listener.initpose.position.x, listener.initpose.position.y, listener.initpose.position.z));
        //transform_init.setRotation(tf::Quaternion(listener.initpose.orientation.x, listener.initpose.orientation.y, listener.initpose.orientation.z,
                               //listener.initpose.orientation.w));
        //br.sendTransform(tf::StampedTransform(transform_init, ros::Time::now(), "map_en", "odom"));

        loop_rate.sleep();
    }

    return 0;

}

