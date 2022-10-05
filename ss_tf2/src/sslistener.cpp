#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace std;
using namespace ros;

int main(int argc, char** argv){
ros::init(argc, argv, "ss_tf2_listener");

ros::NodeHandle n;
ros::Publisher pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/walrus/pose", 1000); 

tf2_ros::Buffer tfBuffer;
tf2_ros::TransformListener tfListener(tfBuffer); 

ros::Rate rate(10); //rate is 10hz
while (n.ok()){

  geometry_msgs::TransformStamped tfMsg; //TransformStamped message 
  
try{
  tfMsg = tfBuffer.lookupTransform("walrus/base","map",
                            ros::Time(0));
 }
 catch (tf2::TransformException &ex) {
   ROS_WARN("%s",ex.what());
   ros::Duration(1.0).sleep();
   continue;
 }

geometry_msgs::PoseWithCovarianceStamped pwcSmsg; //this message type is a vector

float x;
float y;
float z;
float rx;
float ry;
float rz;
float rw;

x = tfMsg.transform.translation.x; //assigning values from the transform 
y = tfMsg.transform.translation.y;
z = tfMsg.transform.translation.z;

rx = tfMsg.transform.rotation.x; // quaternion has x,y,z,w components 
ry = tfMsg.transform.rotation.y;
rz = tfMsg.transform.rotation.z;
rw = tfMsg.transform.rotation.w;

pwcSmsg.header.frame_id = "map";
pwcSmsg.header.stamp = ros::Time::now();

pwcSmsg.pose.pose.position.x = x; 
pwcSmsg.pose.pose.position.y = y;
pwcSmsg.pose.pose.position.z = z;

pwcSmsg.pose.pose.orientation.x = rx; 
pwcSmsg.pose.pose.orientation.y = ry; 
pwcSmsg.pose.pose.orientation.z = rz; 
pwcSmsg.pose.pose.orientation.w = rw; 

 
pub.publish(pwcSmsg);

}

return 0;
}

   // Questions:
   // what specific topic should be subsribed to/adverstied? is it /tf2? if so, what type of msg are these?
   // how many call back functions are needed? and what do each of them do?
   // general confusion about how to connect subsriber/publisher w transform listsner
   // Should I get rid of tfListener(tfBuffer) object and just have tfListener? ie) is buffer needed?
   // Should I also create a "listener" class? Or is it not needed here
   // will my code include odometry like the example you gave me?
   // can you send me the graph with how all the nodes are related?
   // does move_base need to be included? Overall, am I missing any header files?
   // is /map the target frame and /walrus/base the child frame


//    Class Listener{
//   public:
    
// //Callback function for publishing geometry_msgs::PoseWithCovarinaceStamped message
//       void callBack(const geometry_msgs::PoseWithCovarinaceStamped::ConstPtr& cBptr){ //fix

//           pub.publish(pwcSmsg); 

//           cout << "Publishing: geometry_msgs::PoseWithCovarinaceStamped message" << endl; 
//    }

// // Callback function for /tf2 subsriber
//       void tf_callBack(const geometry_msgs::TransformStamped::ConstPtr& msg){ //do i need this?
//           msg->tfMsg.header.stamp = current_time; //is this super wrong? trying to get cb function to connect transform w walrus and map
//           msg->tfMsg.header.frame_id = "/map";
//           msg->tfMsg.child_frame_id = "/walrus/base";
          
//           tfMsg.transform.translation.x = x;
//           tfMsg.transform.translation.y = y;
//           tfMsg.transform.translation.z = 0.0;
//           tfMsg.transform.rotation = ; //what goes here? a quaternion or an actual number for rotation?
//    }

// pwcSmsg.pose.pose.position.x = x;
// pwcSmsg.pose.pose.position.y = y;
// pwcSmsg.pose.pose.position.z = 0.0;
// pwcSmsg.pose.pose.orientation.w = 1.0; // what value goes here?

// };
// pwcSmsg.pose.covariance[0]=0.5; //accessing 0th index of vector -> correct? or did you want something else
// pwcSmsg.pose.covariance[1]=1.0; //covariance represents rotation about fixed x,y,z axis
// pwcSmsg.pose.covariance[2]=1.5;
// pwcSmsg.pose.covariance[3]=2.0;
// pwcSmsg.pose.covariance[4]=2.5;
// pwcSmsg.pose.covariance[5]=3.0;
// pwcSmsg.pose.covariance[6]=3.5;
// pwcSmsg.pose.covariance[7]=4.0;
// pwcSmsg.pose.covariance[8]=4.5;
