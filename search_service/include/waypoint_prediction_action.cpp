#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <move_base_msgs/MoveBaseAction.h>
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseArray.h"
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "tf/tf.h"
#include <tf/transform_datatypes.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include <sstream>
#include <string>
#include <deque>
#include <signal.h>
#include <boost/thread/thread.hpp>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <navi_msgs/StringStamped.h>
#include <villa_navi_service/PredictWaypointAction.h> 
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <XmlRpcValue.h>
#include <yaml-cpp/yaml.h>

#define MAX_NUM 100
#define TIME_THRESHOLD 25
using namespace std;

typedef std::pair<int,std::string> NodeInfo;
typedef std::map<int,std::string> NodeMapInfo;
typedef std::map<int,std::string>::iterator NodeMapIter;
typedef std::map< std::string, geometry_msgs::Point>::iterator goalMapIter;
typedef std::map< std::string, std::vector<string> >::iterator neighborMapIter;

class WaypointNode{
    public:
        int idx;
        string name;
        geometry_msgs::Point Center;
        geometry_msgs::PoseStamped CenterPose;
        //std::vector<int> neighbors;
        std::vector<string> neighbors;
        bool isEnd;
        WaypointNode(int, std::string, bool);
        void setNeighbor(std::vector<string> neighborset);
        void setCenter(geometry_msgs::Point center_);
};

WaypointNode::WaypointNode(int key, std::string _name, bool isEnd_):idx(key),name(_name),isEnd(isEnd_){


}
void WaypointNode::setCenter(geometry_msgs::Point center_)
{
        Center=center_;
        CenterPose.header.frame_id="map";
        CenterPose.pose.position.x = Center.x;
        CenterPose.pose.position.y = Center.y;
        CenterPose.pose.position.z = 0.0;
        
        double temp_roll =0.0;
        double temp_pitch =0.0;
        double temp_yaw =Center.z;
       
        geometry_msgs::Quaternion q;
        double t0 = cos(temp_yaw * 0.5);
        double t1 = sin(temp_yaw * 0.5);
        double t2 = cos(temp_roll * 0.5);
        double t3 = sin(temp_roll * 0.5);
        double t4 = cos(temp_pitch * 0.5);
        double t5 = sin(temp_pitch * 0.5);
        q.w = t0 * t2 * t4 + t1 * t3 * t5;
        q.x = t0 * t3 * t4 - t1 * t2 * t5;
        q.y = t0 * t2 * t5 + t1 * t3 * t4;
        q.z = t1 * t2 * t4 - t0 * t3 * t5;

       CenterPose.pose.orientation.x=q.x;
       CenterPose.pose.orientation.y=q.y;
       CenterPose.pose.orientation.z=q.z;
       CenterPose.pose.orientation.w=q.w;

}

void WaypointNode::setNeighbor(const std::vector<string> neighborset)
{
    size_t neighbor_size = neighborset.size();
    neighbors.resize(neighbor_size);
    for(size_t i(0);i<neighborset.size();i++)
        neighbors[i]=neighborset[i];
}

typedef std::map<string,WaypointNode> WaypointMapInfo;
typedef std::map<string,WaypointNode>::iterator WaypointMap_iter;

class PredictWaypointAction
{
public:
    
  PredictWaypointAction(std::string name): 
  as_(nh_, name, boost::bind(&PredictWaypointAction::executeCB, this,_1), false),
  action_name_(name),
  last_received_region("home"),
  last_target_region("home"),
  last_time_called(0.0)
  {
    parseparameters(nh_);
    Initialize_waypointmap();

    global_pose.resize(3,0.0);
    globalpose_sub=nh_.subscribe<geometry_msgs::PoseStamped>("/global_pose",10,&PredictWaypointAction::global_pose_callback,this);
    robot_region_sub=nh_.subscribe<navi_msgs::StringStamped>("/regionmap/robot",10,&PredictWaypointAction::robot_region_callback,this);
    target_region_sub=nh_.subscribe<navi_msgs::StringStamped>("/regionmap/target",10,&PredictWaypointAction::target_region_callback,this);
    waypoint_pose_pub=nh_.advertise<geometry_msgs::PoseStamped>("/waypoint_pose",50,true);
    target_pose_pub=nh_.advertise<geometry_msgs::PoseStamped>("/target_pose",50,true);
    Waypoint_marker_pub=nh_.advertise<visualization_msgs::Marker>("/predicted_waypoint_marker", 10);

    
    //register the goal and feeback callbacks
    as_.registerPreemptCallback(boost::bind(&PredictWaypointAction::preemptCB, this));

    //subscribe to the data topic of interest
    as_.start();
  }

  ~PredictWaypointAction(void)
  {
  }
  void Initialize_waypointmap(){

    //set index to each waypoint name
    nodemap.insert(NodeInfo(0,"aisle_west"));
    nodemap.insert(NodeInfo(1,"aisle_center"));
    nodemap.insert(NodeInfo(2,"foyer"));
    nodemap.insert(NodeInfo(3,"sideoffice"));
    nodemap.insert(NodeInfo(4,"stage"));
    nodemap.insert(NodeInfo(5,"home"));
    nodemap.insert(NodeInfo(6,"window_office"));

    //WaypointMap_iter waypoint_iter = WaypointMap.find(cur_region);
    //Iterate nodemap to set center, neighbors. This information is from yaml file.
    //
    //
    int count=0;
    NodeMapInfo::iterator iter = nodemap.begin();
    for(iter;iter!=nodemap.end();iter++)
    {
        //index: iter->first , name is iter->second
        //center information is from goalmaps 
        //neighbors information is from neighbormaps
        WaypointNode* temp_node = new WaypointNode(count,iter->second,true);
        goalMapIter goal_iter=goal_maps.find(iter->second);              //find the center position
        temp_node->setCenter(goal_iter->second);
        neighborMapIter neigh_iter= neighbor_maps.find(iter->second);    //find the neighbor position
        temp_node->setNeighbor(neigh_iter->second);
        WaypointMap.insert(std::pair< std::string,WaypointNode>(iter->second,*temp_node));
        count++;
        delete temp_node;
        //update temp_neighborset with index
        //update temp_center
    }


    /*
    int count=0;
    std::string temp_name= "left_corridor";
    std::vector<int> temp_neighborset;
    geometry_msgs::Point temp_center;
    temp_center.x =-1.5;
    temp_center.y = 0.0;
    temp_center.z =0.0;
    temp_neighborset.push_back(1);
    WaypointNode* temp_node = new WaypointNode(count,temp_name,true);
    temp_node->setNeighbor(temp_neighborset);
    temp_node->setCenter(temp_center);
    WaypointMap.insert(std::pair< std::string,WaypointNode>(temp_name,*temp_node));
    count++;
    delete temp_node;

    temp_neighborset.clear();
    temp_name = "foyer";
    temp_center.x = 1.6;
    temp_center.y = 0.0;
    temp_center.z = 1.5;
    temp_neighborset.push_back(0);
    temp_neighborset.push_back(2);
    temp_neighborset.push_back(3);
    temp_node = new WaypointNode(count,temp_name,true);
    temp_node->setNeighbor(temp_neighborset);
    temp_node->setCenter(temp_center);
    WaypointMap.insert(std::pair< std::string,WaypointNode>(temp_name,*temp_node));
    count++;
    delete temp_node;

    temp_neighborset.clear();
    temp_name = "bedroom";
    temp_center.x = 6.2;
    temp_center.y = 0.0;
    temp_center.z = -3.1;
    temp_neighborset.push_back(1);
    temp_node = new WaypointNode(count,temp_name,true);
    temp_node->setNeighbor(temp_neighborset);
    temp_node->setCenter(temp_center);
    WaypointMap.insert(std::pair< std::string,WaypointNode>(temp_name,*temp_node));
    count++;
    delete temp_node;

    temp_neighborset.clear();
    temp_name = "diningroom";
    temp_center.x = 2.4;
    temp_center.y = -3.6;
    temp_center.z = 2.0;
    temp_neighborset.push_back(1);
    temp_neighborset.push_back(4);
    temp_neighborset.push_back(5);
    temp_node = new WaypointNode(count,temp_name,true);
    temp_node->setNeighbor(temp_neighborset);
    temp_node->setCenter(temp_center);
    WaypointMap.insert(std::pair< std::string,WaypointNode>(temp_name,*temp_node));
    count++;
    delete temp_node;

    temp_neighborset.clear();
    temp_name = "kitchen";
    temp_center.x = 6.3;
    temp_center.y = -3.8;
    temp_center.z = -0.8;
    temp_neighborset.push_back(3);
    temp_neighborset.push_back(6);
    temp_node = new WaypointNode(count,temp_name,true);
    temp_node->setNeighbor(temp_neighborset);
    temp_node->setCenter(temp_center);
    WaypointMap.insert(std::pair< std::string,WaypointNode>(temp_name,*temp_node));
    count++;
    delete temp_node;

    temp_neighborset.clear();
    temp_name = "livingroom";
    temp_center.x = 2.0;
    temp_center.y = -7.2;
    temp_center.z = 1.5;
    temp_neighborset.push_back(3);
    temp_neighborset.push_back(6);
    temp_node = new WaypointNode(count,temp_name,true);
    temp_node->setNeighbor(temp_neighborset);
    temp_node->setCenter(temp_center);
    WaypointMap.insert(std::pair< std::string,WaypointNode>(temp_name,*temp_node));
    count++;
    delete temp_node;

    temp_neighborset.clear();
    temp_name = "office";
    temp_center.x = 7.2;
    temp_center.y = -7.2;
    temp_center.z = -2.2;
    temp_neighborset.push_back(4);
    temp_neighborset.push_back(5);
    temp_neighborset.push_back(7);
    temp_node = new WaypointNode(count,temp_name,true);
    temp_node->setNeighbor(temp_neighborset);
    temp_node->setCenter(temp_center);
    WaypointMap.insert(std::pair< std::string,WaypointNode>(temp_name,*temp_node));
    count++;
    delete temp_node;
    
    temp_neighborset.clear();
    temp_name = "right_cor";
    temp_center.x = 9.0;
    temp_center.y = -7.0;
    temp_center.z = 0.0;
    temp_neighborset.push_back(6);
    temp_node = new WaypointNode(count,temp_name,true);
    temp_node->setNeighbor(temp_neighborset);
    temp_node->setCenter(temp_center);
    WaypointMap.insert(std::pair< std::string,WaypointNode>(temp_name,*temp_node));
    count++;
    delete temp_node;
    */

    std::map<std::string,WaypointNode>::iterator map_it = WaypointMap.begin();
    for(map_it;map_it!=WaypointMap.end();map_it++)
    {
        ROS_INFO("current waypoint name : %s", map_it->first.c_str() );
        ROS_INFO("current waypoint idx : %d", map_it->second.idx);
        ROS_INFO("waypoint center : x: %.3lf , %.3lf" , map_it->second.Center.x, map_it->second.Center.y);
        ROS_INFO("neighbor size : %d",  map_it->second.neighbors.size());
    }

  }

  void robot_region_callback(const navi_msgs::StringStamped::ConstPtr& msg)
  {
      //ROS_INFO("robot_region_callback");
      std::string cur_region =std::string(msg->data); 
      if(cur_region.compare(last_received_region)==0)
      {

          IsRegionChanged=false;
      }
      else
      {
          ROS_INFO("robot's position has been chaned from %s to %s"
                                    ,last_received_region.c_str()
                                    ,cur_region.c_str());
          IsRegionChanged=true;
          robot_dist_neighbor_que.clear();
      }
      //check size of deque
      size_t que_size= robot_regions.size();
      //ROS_INFO("que_size: %d", que_size);
      if(que_size<MAX_NUM)
          robot_regions.push_back(msg->data);
      else{
          robot_regions.pop_front();
          robot_regions.push_back(msg->data);
      }
      last_received_region = std::string(msg->data);

  }

  void target_region_callback(const navi_msgs::StringStamped::ConstPtr& msg)
  {
  
      size_t que_size= target_regions.size();
      if(que_size<MAX_NUM)
          target_regions.push_back(msg->data);
      else{
          target_regions.pop_front();
          target_regions.push_back(msg->data);
      }
  }

  void global_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
   tf::StampedTransform baselinktransform;
   listener.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(2.0));
   listener.lookupTransform("map", "base_link", ros::Time(0), baselinktransform);
   double yaw_tf =   tf::getYaw(baselinktransform.getRotation()); 

   double move_dist = pow(global_pose[0]-msg->pose.position.x,2)
                     +pow(global_pose[1]-msg->pose.position.y,2);
                     +pow(global_pose[2]-yaw_tf,2);
   move_dist = sqrt(move_dist);

   global_pose[0]=msg->pose.position.x;
   global_pose[1]=msg->pose.position.y;
   global_pose[2]=yaw_tf;

   //save context only when robot is moving
   if(move_dist>0.004)
   {
       //ROS_INFO("move distance : %.4lf", move_dist);
       getContext();
   }
   
}
double compute_distance_to_robot(geometry_msgs::Point pos_)
{
    double temp=0.0;
    temp+=pow((pos_.x-global_pose[0]),2);
    temp+=pow((pos_.y-global_pose[1]),2);
    temp=sqrt(temp);
}

//get context : where is robot heading to?
//curregion => get_neighbors()
//

void getContext()
{


   if(robot_regions.size()<1)
        return;

   //ROS_INFO("getContext function -------");

   std::string cur_region= robot_regions[robot_regions.size()-1];
   ROS_INFO("current region:  %s ", cur_region.c_str());

   WaypointMap_iter waypoint_iter = WaypointMap.find(cur_region);
   size_t neighbor_size=   waypoint_iter->second.neighbors.size();
   //waypoint_iter->neighbors
   
   std::vector<double> distance_set;
   distance_set.resize(neighbor_size,0.0);
   double dist=0.0;
   for(size_t i(0);i<neighbor_size;i++)
   {
       //int cur_neighbor_idx = waypoint_iter->second.neighbors[i];
       //NodeMapIter nodeiter = nodemap.find(cur_neighbor_idx);
       //WaypointMap_iter temp_iter = WaypointMap.find(nodeiter->second);
       WaypointMap_iter temp_iter = WaypointMap.find(waypoint_iter->second.neighbors[i]);
       dist = compute_distance_to_robot(temp_iter->second.Center);
       //ROS_INFO("neighbor : %s , distance :%.3lf ", nodeiter->second.c_str(), dist);
       distance_set[i]=dist;
   }

   //save current neighbor distance vector to que
   if(robot_dist_neighbor_que.size()<MAX_NUM)
       robot_dist_neighbor_que.push_back(distance_set);
   else
   {
       robot_dist_neighbor_que.pop_front();
       robot_dist_neighbor_que.push_back(distance_set);
   }
}

  void Publish_waypoint_marker(std::string waypoint_name_)
  {
      WaypointMap_iter waypoint_iter = WaypointMap.find(waypoint_name_);
      double temp_dist=0.25;
      visualization_msgs::Marker waypoint_marker;

      int marker_count =0;
       //visualization_marker

        waypoint_marker.header.frame_id = "/map"; 
        waypoint_marker.header.stamp = ros::Time::now();
        waypoint_marker.ns = "/waypoints";
        waypoint_marker.id = marker_count;

        uint32_t shape = visualization_msgs::Marker::SPHERE;
        waypoint_marker.type = shape;

        waypoint_marker.pose.position.x = (waypoint_iter->second).Center.x;
        waypoint_marker.pose.position.y = (waypoint_iter->second).Center.y;
        waypoint_marker.pose.position.z = 0.25;

        waypoint_marker.pose.orientation.x = 0.0;
        waypoint_marker.pose.orientation.y = 0.0;
        waypoint_marker.pose.orientation.z = 0.0;
        waypoint_marker.pose.orientation.w = 1.0;

        //ROS_INFO("temp dist : %.3lf, temp dist2 : %.3lf, temp dist3 : %.3lf",temp_dist,temp_dist2,temp_dist3);
        waypoint_marker.scale.x = std::abs(temp_dist);
        waypoint_marker.scale.y = std::abs(temp_dist);
        waypoint_marker.scale.z = std::abs(temp_dist);

        waypoint_marker.color.r = 0.9;
        waypoint_marker.color.g = 0.2;
        waypoint_marker.color.b = 0.2;
        waypoint_marker.color.a = 0.7;

        Waypoint_marker_pub.publish(waypoint_marker);

  }

  void preemptCB()
  {
    ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted
    as_.setPreempted();
  }

  void executeCB(const villa_navi_service::PredictWaypointGoalConstPtr &goal)
  {
     // ros::Rate r(1);
     bool success = true;
      ROS_INFO("%s: succeeded",action_name_.c_str());
     double cur_time = ros::Time::now().toSec();

     //if(last_time_called!=0 && abs(cur_time-last_time_called)<TIME_THRESHOLD )
     //{
        //ROS_INFO("I have to wait for time threshold");
        //success=false;
     ////   as_.setPreempted();
        //result_.status =2;
        //feedback_.is_possible_go=false;;
     ////   as_.publishFeedback(feedback_);
        //as_.setSucceeded(result_);
     //}
     
     if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        success = false;
        result_.status =2;
        // break;
      }

      feedback_.is_possible_go=true;
      // publish the feedback
      
      std::string next_waypoint = getRegion();
      ROS_INFO("next waypoint is %s", next_waypoint.c_str());
      WaypointMap_iter waypoint_iter = WaypointMap.find(next_waypoint);

      //if(last_target_region.compare(next_waypoint)==0)
     // {
        //ROS_INFO("I am already in the same region");
        //success = false;
      //  result_.status =2;
        //feedback_.is_possible_go=false;;
        //as_.publishFeedback(feedback_);
       // as_.setSucceeded(result_);
        //as_.setPreempted();
        //success = false;
        //result_.status =2;
      //}
      
      if(success)
      {
          std_msgs::String action_msg;
          action_msg.data=next_waypoint;
          result_.status =1;
          result_.goal_waypoint = action_msg;
          result_.waypoint_point = waypoint_iter->second.Center;
          result_.waypoint_pose = waypoint_iter->second.CenterPose;
          result_.waypoint_pose.header.stamp= ros::Time::now();
          Publish_waypoint_marker(next_waypoint);

          //geometry_msgs::PoseStamped waypoint_Pose;
          //waypoint_Pose.header.stamp = ros::Time::now();
          //waypoint_Pose.header.frame_id = "map";
          //waypoint_Pose.pose.position.x = result_.waypoint_point.x;
          //waypoint_Pose.pose.position.y = result_.waypoint_point.y;
          waypoint_pose_pub.publish(result_.waypoint_pose);
          target_pose_pub.publish(result_.waypoint_pose);

          last_time_called =ros::Time::now().toSec();
          last_target_region = next_waypoint;
          as_.publishFeedback(feedback_);
          as_.setSucceeded(result_);
          ROS_INFO("%s: succeeded",action_name_.c_str());
      }
      else
      {
          result_.status=2;
          as_.setSucceeded(result_);
          ROS_INFO("%s: succeeded",action_name_.c_str());
      
      
      
      }

  }

    std::string getRegion(){

      //ROS_INFO("robot is heading to ot in kitchen");
      size_t neighbor_size = robot_dist_neighbor_que[0].size();
      if(neighbor_size<1)
          return "Failed";

      std::vector<double> avg_delta_dist(neighbor_size);
      for(size_t j(0);j<neighbor_size;j++)
      {
          avg_delta_dist[j]=0.0;
          double temp_delta_dist=0.0;
          for(size_t i(0);i<robot_dist_neighbor_que.size()-1;i++){
              temp_delta_dist = robot_dist_neighbor_que[i+1][j]-robot_dist_neighbor_que[i][j];
              avg_delta_dist[j]+=temp_delta_dist;
          }
          avg_delta_dist[j]= (avg_delta_dist[j])/static_cast<double>(robot_dist_neighbor_que.size());
          //ROS_INFO("neighbor: %d, avg_delta_distance: %.3lf ",j, avg_delta_dist[j]);
      }

      //find minimum index that has the lowest value from avg_delta_dist
      auto minIt = std::min_element(avg_delta_dist.begin(), avg_delta_dist.end());
      int min_idx = minIt-avg_delta_dist.begin();
      double minElement = *minIt;
      //ROS_INFO("minimum idx: %d, min delta_distance: %.3lf ",minIt, minElement);
      
     //find neighbor index from cur location of robot
     std::string cur_region= robot_regions[robot_regions.size()-1];
     WaypointMap_iter waypoint_iter = WaypointMap.find(cur_region);
     //NodeMapIter nodeiter = nodemap.find(waypoint_iter->second.neighbors[min_idx]);
     //WaypointMap_iter temp_iter = WaypointMap.find(waypoint_iter->second.neighbors[min_idx]);
     WaypointMap_iter temp_iter = WaypointMap.find(waypoint_iter->second.neighbors[min_idx]);
     ROS_INFO("---------------------------------------");
     ROS_INFO("robot is heading %s ",waypoint_iter->second.neighbors[min_idx].c_str());
     ROS_INFO("---------------------------------------");

     //return (nodeiter->second);
    return(waypoint_iter->second.neighbors[min_idx]);
  }

void parseparameters(ros::NodeHandle n){
    std::string target_frame;
    n.getParam("villa_navi_service/ref_frame", target_frame);
    std::vector<std::string> locations_vector;
    //std::string locations_list;
    XmlRpc::XmlRpcValue location_list;
    n.getParam("villa_navi_service/location_list", location_list);

    //ROS_INFO("type %s",location_list.getType());
    ROS_ASSERT(location_list.getType() ==XmlRpc::XmlRpcValue::TypeArray);
    for(size_t j(0);j<location_list.size();j++)
    {
        //ROS_INFO_STREAM("Loded "<< "-" << (std::string)(location_list[j]));
    
        std::string location_str = static_cast<std::string>(location_list[j]);
        locations_vector.push_back(location_str);
        LoadPositionfromConfig(n,location_str);
    }
    //ROS_ASSERT(ref_frame.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    //ROS_INFO_STREAM("reference frame: " << target_frame);
}

void LoadPositionfromConfig(ros::NodeHandle n, std::string input_locations)
{

     XmlRpc::XmlRpcValue input_loc;
     std::string param_name = "villa_navi_service/" + input_locations;
     n.getParam(param_name, input_loc);
     //std::vector<double> tmp_pos(3,0.0);
     geometry_msgs::Point tmp_pos;
     std::vector<std::string> neighbor_list;
     for(XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = input_loc.begin(); it != input_loc.end(); ++it) {
         //ROS_INFO_STREAM("Loded "<< input_locations << "-" << (std::string)(it->first) << ",  " << input_loc[it->first]);

         geometry_msgs::Point temp_pos;
         tmp_pos.x=static_cast<double>(input_loc["x"]);
         tmp_pos.y=static_cast<double>(input_loc["y"]);
         //tmp_pos.z=static_cast<double>(input_loc["t"]);
         tmp_pos.z=0.0*static_cast<double>(input_loc["t"]);

        //static_cast<std::vector<std::string>>(input_loc["neighbor"])
    }

    for(size_t j(0);j<input_loc["neighbor"].size();j++)
    {
        neighbor_list.push_back(input_loc["neighbor"][j]);
    }

    goal_maps[input_locations]=tmp_pos;
    neighbor_maps[input_locations]=neighbor_list;

    std::map< std::string, std::vector<string> >::iterator it = neighbor_maps.begin();
    for(it ; it !=neighbor_maps.end();it++ )
    {
        ROS_INFO("neighbors of %s",it->first.c_str() );
        for(size_t (j); j<it->second.size(); j++ )
            ROS_INFO("%d th neighbor: %s",j, it->second[j]);
    }
}

protected:
    
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<villa_navi_service::PredictWaypointAction> as_;
  std::string action_name_;
  int data_count_, goal_;
  float sum_, sum_sq_;
  villa_navi_service::PredictWaypointFeedback feedback_;
  villa_navi_service::PredictWaypointResult result_;
  tf::TransformListener 	  listener;
  ros::Publisher waypoint_pose_pub;
  ros::Publisher Waypoint_marker_pub;
  ros::Publisher target_pose_pub;
  ros::Subscriber sub_;
  ros::Subscriber robot_region_sub;
  ros::Subscriber target_region_sub;
  ros::Subscriber globalpose_sub;
  std::string last_received_region;
  std::string last_target_region;
  double last_time_called;
  bool IsRegionChanged;
  std::vector<double> global_pose;
  std::deque<std::string> robot_regions;
  std::deque<std::string> target_regions;
  std::deque<std::vector<double>> robot_dist_neighbor_que;
  std::deque<std::vector<double>> targt_dist_neighbor_que;
  std::map<std::string,WaypointNode> WaypointMap;
  NodeMapInfo nodemap;
  geometry_msgs::Pose temp_targetpose;
 std::map< std::string, geometry_msgs::Point > goal_maps;
 std::map< std::string, std::vector<string> > neighbor_maps;

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "predict_waypoint");
  PredictWaypointAction predict_action_node(ros::this_node::getName());
  ros::spin();

  return 0;
}
