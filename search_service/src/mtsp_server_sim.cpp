#include "ros/ros.h"
#include <math.h>
#include "spline.h"
#include "cluster.h"
#include <map>
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Twist.h"
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sstream>
#include <signal.h>
#include <boost/thread/thread.hpp>
#include <std_msgs/Bool.h>
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/GetMultiMultiPlan.h>
#include <nav_msgs/GetMultiPlan.h>
#include <search_service/MultiSearchAction.h>
#include <search_service/SearchAction.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib_msgs/GoalID.h>
#include <XmlRpcValue.h>
#include <yaml-cpp/yaml.h>
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "tf/tf.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/utils.h"
#include "cluster.h"
#include <fstream>

#define MAP_RES 0.05
#define NUM_SUBS 3
#define Dist_History 2.0
#define Max_Dist 30.0
#define Dist_GOAL 4.0
#define RAND_RANGE 1.0
#define L_SOCC 4.595 // np.log(0.99/0.01)
#define L_DOCC 2.19 // np.log(0.9/0.1)
#define L_FREE -4.595// np.log(0.01/0.99)
#define CELL_MAX_ENTROPY 0.693147
#define GOAL_THRESHOLD 2.0
#define agent1_namespace "spot"
#define agent2_namespace "spot2"
#define agent3_namespace "spot3"

using namespace std;

double atan_zero_to_twopi(double y, double x){

    double angle = atan2(y, x);
    if(angle < 0.0)
        angle += 2*M_PI;

    return angle;
}

class precastDB
{

public:

    double px;
    double py;
    double dist;
    double angle;
    int ix;
    int iy;
    double value;


public:
    precastDB()
    {
        px=0.0;
        py=0.0;
        dist=0.0;
        angle=0.0;
        ix=0;
        iy=0;
        value=0.0;
    }

    precastDB(double px_, double py_, double dist_, double angle_, int ix_, int iy_)
    {
        px=px_;
        py=py_;
        dist=dist_;
        angle=angle_;
        ix=ix_;
        ix=iy_;
    }

};

class Map_params
{
    public:
        Map_params():xyreso(0.5),yawreso(0.2), xmin(-12.0),xmax(12.0),ymin(-15.0),
        ymax(12.0), sensor_range(7.0)
        {
            xw = int(round((xmax - xmin) / xyreso));
            yw = int(round((ymax - ymin) / xyreso));
        }
        
        void calc_grid_map_config( double agent_x, double agent_y, double& minx, double& miny, double& maxx, double& maxy, double& xw, double& yw)
        {
        
            minx = agent_x - sensor_range;
            maxx = agent_x + sensor_range;
            miny = agent_y - sensor_range;
            maxy = agent_y + sensor_range;
            xw = int((maxx - minx) / xyreso);
            yw = int((maxy - miny) / xyreso);
        }

        //precast
        double xyreso;
        double yawreso;
        double xmin;
        double ymin;
        double xmax;
        double ymax;
        double xw;
        double yw;
        double sensor_range;

};


class MultiSearchManager
{
public:
    
  MultiSearchManager(std::string name): 
  as_(nh_, name, boost::bind(&MultiSearchManager::executeCB, this,_1), false),
  action_name_(name),
  IsGoal(false),
  IsActive(false),
  srv_time(0.0),
  tolerance(0.5),
  search_entropy(1.0),
  neargoal_agent1(false),
  neargoal_agent2(false),
  clustered(false),
  no_count(0),
  horizon(15),
  called_once(false),
  weight_entropy(0.25),
  weight_travel(1.5),
  m_params(NULL)
  {
     m_params= new Map_params();
     parseparameters(nh_);
     search_map.info.resolution = m_params->xyreso;
     search_map.info.width= m_params->xw;
     search_map.info.height= m_params->yw;
     search_map.info.origin.position.x= m_params->xmin;
     search_map.info.origin.position.y= m_params->ymin;
     search_map.data.resize((search_map.info.width * search_map.info.height), 0.0);  //unknown ==> 0 ==> we calculate number of 0 in search map to calculate IG
     //
     //initially the entropy can be computed as #of cells in serchmap * uncertainty
     total_entropy=search_map.data.size()*CELL_MAX_ENTROPY;

     //publishers
    search_map_pub=nh_.advertise<nav_msgs::OccupancyGrid>("/search_map",50,true);
    search_goal_pub=nh_.advertise<geometry_msgs::PoseStamped>("/search_goal",50,true);
    agent1_move_cancel_pub=nh_.advertise<actionlib_msgs::GoalID>("/localnavi_server/cancel",50,true);
    search_entropy_pub=nh_.advertise<std_msgs::Float32>("/search_entropy",50,true);
    search_goal2_pub=nh_.advertise<geometry_msgs::PoseStamped>("/search_goal2",50,true);
    visual_marker_pub= nh_.advertise<visualization_msgs::MarkerArray>("clusters", 5);

    //planner_srv_client= nh_.serviceClient<nav_msgs::GetMultiMultiPlan>("/planner/planner/make_multimultiplan");
    
    //subscribers
    global_map_sub = nh_.subscribe<nav_msgs::OccupancyGrid>("/scaled_static_map", 1, &MultiSearchManager::global_map_callback, this);
    frontiers_poses_sub=nh_.subscribe<geometry_msgs::PoseArray>("/frontier_poses",10,&MultiSearchManager::frontiers_callback,this);
    unknown_poses_sub=nh_.subscribe<geometry_msgs::PoseArray>("/waypoint_poses",10,&MultiSearchManager::unknown_poses_callback,this);
    agent1_localmap_sub= nh_.subscribe<nav_msgs::OccupancyGrid>("/spot/move_base/local_costmap/costmap",10,&MultiSearchManager::agent1_localmap_callback,this);
    agent2_localmap_sub= nh_.subscribe<nav_msgs::OccupancyGrid>("/spot2/move_base/local_costmap/costmap", 10,&MultiSearchManager::agent2_localmap_callback,this);
    agent3_localmap_sub= nh_.subscribe<nav_msgs::OccupancyGrid>("/spot3/move_base/local_costmap/costmap", 10,&MultiSearchManager::agent3_localmap_callback,this);

    agent1_pose_sub=nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("spot/amcl_pose",10,&MultiSearchManager::agent1_pose_callback,this);
    agent2_pose_sub=nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("spot2/amcl_pose",10,&MultiSearchManager::agent2_pose_callback,this);
    agent3_pose_sub=nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("spot3/amcl_pose",10,&MultiSearchManager::agent3_pose_callback,this);

    //receive scaled_global_map
    nav_msgs::OccupancyGrid::ConstPtr shared_map;
    shared_map= ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/scaled_static_map",nh_);
    if(shared_map!= NULL){
     scaled_global_map= *shared_map;
     crop_globalmap(*shared_map);
    }
    total_entropy-=occ_entropy;
    ROS_INFO("total_initial_entropy: %.2f",  total_entropy);

    agent1_pose.resize(3,0.0);
    agent2_pose.resize(3,0.0);
    agent3_pose.resize(3,0.0);
    as_.registerPreemptCallback(boost::bind(&MultiSearchManager::preemptCB, this));
    as_.start();
    ROS_INFO("multi-search action_started");
  }

  ~MultiSearchManager(void)
  {
  }

  void preemptCB()
  {
    ROS_INFO("%s: Preempted", action_name_.c_str());
    //cmd_velocity_pub.publish(vel_cmd);
    IsActive=false;
    ROS_INFO("preempted called");

    // set the action state to preempted
    as_.setPreempted();
  }

  bool transform_subgoal_to_baselinkframe(const geometry_msgs::Pose& inputPose, geometry_msgs::PoseStamped& pose_out)
  {
      geometry_msgs::PoseStamped pose_in;     //added by mk, ryan
      //pose_in.header.stamp=ros::Time(0);
      pose_in.header.stamp=ros::Time().now();
      pose_in.header.frame_id="map";
      pose_in.pose=inputPose;
      pose_in.pose.orientation.x=0.0;
      pose_in.pose.orientation.y=0.0;
      pose_in.pose.orientation.z=0.0;
      pose_in.pose.orientation.w=1.0;

      try{
        listener.transformPose("base", ros::Time(0), pose_in, pose_in.header.frame_id, pose_out);
        return true;
      }catch(tf::TransformException ex){
        std::cout << ex.what() << std::endl;
        sleep(0.1);
        return false;
      }

 
      //try{
          //tf_buffer.transform(pose_in,pose_out, "base");
          //return true;
      //}
      //catch (tf2::TransformException &ex)
      //{
          //ROS_WARN("subgoal to baselink frame: Transform Failed: %s", ex.what());
          //sleep(0.1);
          //return false;
      //}

  }

  void executeCB(const search_service::MultiSearchGoalConstPtr &goal)
  {
     int num_points=2;
     bool success = true;
     bool lead_trj=false;
     if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        success = false;
        return;
        // break;
      }

     if(search_entropy<0.05)
     {
        ROS_INFO("search is finished!!!");
        as_.setSucceeded(result_);
        return;
     }

    //Setting 
    ROS_INFO("MultiSearchManager action is called!!");

    if(called_once)
    {
        agent1_history.poses.push_back(agent1_gpose.pose);
        agent2_history.poses.push_back(agent2_gpose.pose);
        //agents_history.poses.push_back(agent1_gpose.pose);
        //agents_history.poses.push_back(agent2_gpose.pose);
    }

    double sleep_rate= 2.0;
    //if(!neargoal_agent1 and !neargoal_agent2)
    if(!neargoal_agent1 && !neargoal_agent2 && called_once)
    {
      ROS_INFO("Robot is not close to the previous goal. wait until it's near to that goal");
      result_.goal.pose.orientation=agent1_gpose.pose.orientation;
      as_.setSucceeded(result_);
      sleep(2.0);
      no_count++;
      return;
    }
    if(no_count>100)
        no_count=0;
    //search_entropy
    if(search_entropy<0.5)
    {
        horizon=25;
        num_points=3;
    }
    if(search_entropy<0.25)
    {
        horizon=-1;
        num_points=4;
    }

    std::vector<geometry_msgs::PoseArray> agent1_path;
    std::vector<geometry_msgs::PoseArray> agent2_path;
    std::vector<geometry_msgs::PoseArray> agent3_path;
    std::vector<geometry_msgs::PoseStamped> goals_agent1;
    std::vector<geometry_msgs::PoseStamped> goals_agent2;
    std::vector<geometry_msgs::PoseStamped> goals_agent3;

    /*
    if(search_entropy<-0.4)
    {

        if(call_multiplan_service(agent1_path, agent2_path))
        {
            //agent1-path
            std::vector<tk::spline2D> agent1_splines;
            Convert_Poses2trjs(agent1_path, agent1_splines);
            //trjs_to_sample(agent1_splines, agent1_path, agent1_history, horizon);
            trjs_to_sample(agent1_splines, agent1_path, agents_history, horizon);
            int maxidx = calc_IG_trjs(agent1_path, num_points);

            if(maxidx>=0)
            {
                set_goal_from_path(agent1_path[maxidx].poses.back(), result_.goal);
                lead_trj=true;
            }
            else{
                int minidx=0;
                goal_sampling_frontiers(agent1_gpose, goals_agent1, minidx);
                result_.goal=goals_agent1[minidx];
            }

            //agent2-path
            std::vector<tk::spline2D> agent2_splines;
            Convert_Poses2trjs(agent2_path, agent2_splines);
            trjs_to_sample(agent2_splines, agent2_path, agents_history, horizon);
            int maxidx2=0;
            if(lead_trj)
                maxidx2 = calc_IG_trjs_with_leadtrj(agent2_path, agent1_path[maxidx], num_points);
            else
                maxidx2 = calc_IG_trjs_with_leadgoal(agent2_path, result_.goal.pose, num_points);

            if(maxidx2>=0)
                set_goal_from_path(agent2_path[maxidx2].poses.back(), result_.goal2);
            else{

                int minidx=0;
                goal_sampling_frontiers(agent2_gpose, goals_agent2, minidx);
                result_.goal2=goals_agent2[minidx];
                int pcount=0;
                while(near_eachother(result_.goal.pose, result_.goal2.pose) && (pcount< unknown_poses.poses.size()))
                {
                    result_.goal2.pose=unknown_poses.poses[pcount];
                    pcount++;
                }
            }

            called_once=true;
        }
        else{

            ROS_WARN("MultiPlan service failed");

            int minidx=0;
            goal_sampling_frontiers(agent1_gpose, goals_agent1, minidx);
            result_.goal=goals_agent1[minidx];
            goal_sampling_frontiers(agent2_gpose, goals_agent2, minidx);
            result_.goal2=goals_agent2[minidx];

        }
    }
    else{
        int minidx=0;
            goal_sampling_frontiers(agent1_gpose, goals_agent1,minidx);
            result_.goal=goals_agent1[minidx];
            goal_sampling_frontiers(agent2_gpose, goals_agent2,minidx);
            result_.goal2=goals_agent2[minidx];
       
    
    
    }
  */

    //agent1_history.poses.push_back(result_.goal.pose);
    //agent2_history.poses.push_back(result_.goal2.pose);
    //agents_history.poses.push_back(result_.goal.pose);
    //agents_history.poses.push_back(result_.goal2.pose);
    //last_goal_agent1=result_.goal;
    //last_goal_agent2=result_.goal2;
    search_entropy = get_searchmap_entropy()/ total_entropy;
    //cancel previous search action
    //actionlib_msgs::GoalID cancel_msg;
    ROS_INFO("current entropy: %.2lf", search_entropy);
    //search_goal_pub.publish(result_.goal);
    //search_goal2_pub.publish(result_.goal2);
    //std_msgs::Float32 entropy_msg;
    //entropy_msg.data=search_entropy;
    //search_entropy_pub.publish(entropy_msg);
    //as_.setSucceeded(result_);
    ROS_INFO("%s: succeeded", action_name_.c_str());
    return;

    //save the current global_pose w.r.t map
    //double ros_rate = 2.0;
    //while(ros::ok() && IsActive && !as_.isPreemptRequested())
    //{
        //as_.publishFeedback(feedback_);
        //ros::spinOnce();
        //r.sleep();
    //}

  }
 

  void frontiers_callback(const geometry_msgs::PoseArray::ConstPtr& msg)
  {
  
      ROS_INFO("frontier_callback");
      frontier_poses=*msg;
  
  
  }

  void unknown_poses_callback(const geometry_msgs::PoseArray::ConstPtr& msg)
  {
  
      ROS_INFO("unknown_pose_callback");
      unknown_poses=*msg;
      clustering();
  
  }


  void agent2_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
  {
      //* global_pose w.r.t "map" frame
      //agent2_gpose=*msg;
      ROS_INFO("agent3_callback");
      agent2_gpose.pose=msg->pose.pose;
      agent2_pose[0]=msg->pose.pose.position.x;
      agent2_pose[1]=msg->pose.pose.position.y;
      agent2_pose[2]=tf2::getYaw(msg->pose.pose.orientation);
      agent2_pose_updated=true;

      if(called_once)
        {
            double temp_dist = sqrt(pow(last_goal_agent2.pose.position.x-agent2_pose[0],2)
                                        +pow(last_goal_agent2.pose.position.y-agent2_pose[1],2));
            if(temp_dist<GOAL_THRESHOLD)
                neargoal_agent2=true;
            else
                neargoal_agent2=false;
    }
  }

  void agent3_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
  {
      //* global_pose w.r.t "map" frame
      //agent2_gpose=*msg;
      ROS_INFO("agent3_callback");
      agent3_gpose.pose=msg->pose.pose;
      agent3_pose[0]=msg->pose.pose.position.x;
      agent3_pose[1]=msg->pose.pose.position.y;
      agent3_pose[2]=tf2::getYaw(msg->pose.pose.orientation);
      agent3_pose_updated=true;

      if(called_once)
        {
            double temp_dist = sqrt(pow(last_goal_agent3.pose.position.x-agent3_pose[0],2)
                                        +pow(last_goal_agent3.pose.position.y-agent3_pose[1],2));
            if(temp_dist<GOAL_THRESHOLD)
                neargoal_agent3=true;
            else
                neargoal_agent3=false;
    }
  }


  void agent1_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
  {
      //* global_pose w.r.t "map" frame
      agent1_gpose.pose=msg->pose.pose;
      agent1_pose[0]=msg->pose.pose.position.x;
      agent1_pose[1]=msg->pose.pose.position.y;
      agent1_pose[2]=tf2::getYaw(msg->pose.pose.orientation);
      global_pose_a1_updated=true;

      if(called_once)
      {
          double temp_dist = sqrt(pow(last_goal_agent1.pose.position.x-agent1_pose[0],2)
                                        +pow(last_goal_agent1.pose.position.y-agent1_pose[1],2));
            if(temp_dist<GOAL_THRESHOLD)
                neargoal_agent1=true;
            else
                neargoal_agent1=false;
      }

      search_map_pub.publish(search_map);
  }
  

  void agent1_localmap_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    agent1_local_map=*msg;
    agent1_local_map_updated = true;
    update_occ_grid_map(msg);

    std_msgs::Float32 entropy_msg;
    entropy_msg.data=search_entropy;
    search_entropy_pub.publish(entropy_msg);
    //check_obstacle
}

  void agent2_localmap_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    agent2_local_map=*msg;
    agent2_local_map_updated = true;

    update_occ_grid_map(msg);
    //check_obstacle
}

  void agent3_localmap_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    agent3_local_map=*msg;
    agent3_local_map_updated = true;

    update_occ_grid_map(msg);
    //check_obstacle
}



void global_map_callback(const nav_msgs::OccupancyGridConstPtr& msg)
{
    ROS_INFO("global map callback");
    global_map = *msg;
    global_map_updated = true;
}


//search mapupdate function
void update_occ_grid_map(const nav_msgs::OccupancyGridConstPtr& msg)
{
    //origin_point_map_en =local_map.info.origin.position
    double px, py =0.0;
    int map_idx,search_idx =0;

    for(int j(0); j< msg->info.height;j++)
        for(int i(0); i< msg->info.width;i++)
        {
            map_idx = j*msg->info.width+i;
            px = msg->info.origin.position.x+(i+0.5)*msg->info.resolution;
            py = msg->info.origin.position.y+(j+0.5)*msg->info.resolution;
            search_idx = Coord2CellNum(px,py, search_map);         // get search cell idx 
            //Update searchmap according to local measurement
            // if local_map is known (occ or free) and global_map is not occupied by static obstacle 
            if(search_idx<search_map.data.size())
            {
                if(msg->data[map_idx]!=-1 and search_map.data[search_idx]!=int(L_SOCC)) 
                    //if local measurment is not unknown and not filled with static obstacle//
                    if(msg->data[map_idx]==0)
                        search_map.data[search_idx]= int(L_FREE);
                    else
                    {
                        //not unknown, not static obstacle, not free --> dynamic obstacle
                        search_map.data[search_idx]= int(L_DOCC);
                    }
            }
        }


    search_map_pub.publish(search_map);
}


double get_searchmap_entropy()
{

 // count the number of cells whose value is 0
 int map_count=0;
    for(int j(0); j< search_map.info.height;j++)
        for(int i(0); i< search_map.info.width;i++)
        {
            int map_idx = j*search_map.info.width+i;
            if(search_map.data[map_idx]==0)
                map_count++;
        }
    return map_count*CELL_MAX_ENTROPY;
}

void crop_globalmap(const nav_msgs::OccupancyGrid global_map)
{
    //"Crop global_map-->fill search map with static obstacle "
    //searchmap info / globalmap_info
    ROS_INFO("Crop global_map");
    int count=0;

    double px, py =0.0;
    int global_idx,search_idx =0;

    //iteration for search map to apply globalmap info 
    for(int j(0); j< search_map.info.height;j++)
        for(int i(0); i< search_map.info.width;i++)
        {
            px = search_map.info.origin.position.x+(i+0.5)*search_map.info.resolution;
            py = search_map.info.origin.position.y+(j+0.5)*search_map.info.resolution;
            global_idx = Coord2CellNum(px,py, global_map);  
            search_idx = Coord2CellNum(px,py, search_map);         // get search cell idx 
            //if static_obstacle data in global_map
            if(global_map.data[global_idx]>0)
            {
                    search_map.data[search_idx]= int(L_SOCC);
                    count++;
            }
            //Update searchmap according to local measurement
            // if local_map is known (occ or free) and global_map is not occupied by static obstacle 
        }

        occ_entropy=double(count)*CELL_MAX_ENTROPY;  //the amount of entropy reduced
        ROS_INFO("occ_entropy_sum: %.2lf ", occ_entropy);
        ROS_INFO("crop_map finished");
        return;
}



void Idx2Globalpose(int idx, std::vector<double>& global_coord, const nav_msgs::OccupancyGrid& inputmap_)
{
    global_coord.resize(2,0.0);

    int res = (int) (idx/ inputmap_.info.width);
    int div = (int) (idx%inputmap_.info.width);

    global_coord[0]=res*inputmap_.info.resolution+inputmap_.info.origin.position.x;
    global_coord[1]=div*inputmap_.info.resolution+inputmap_.info.origin.position.y;
}


    int Coord2CellNum(double _x, double _y, const nav_msgs::OccupancyGrid& inputmap_)
    {	
        //ROS_INFO("x: %.2lf, y: %.2lf", _x, _y);
        std::vector<int> target_Coord;
        target_Coord.resize(2,0);
        //double reference_origin_x;
        //double reference_origin_y;

        //reference_origin_x=inputmap_.info.origin.position.x;
        //reference_origin_y=inputmap_.info.origin.position.y;

        double  temp_x  = _x-inputmap_.info.origin.position.x;
        double  temp_y = _y-inputmap_.info.origin.position.y;

        target_Coord[0]= (int) floor(temp_x/inputmap_.info.resolution);
        target_Coord[1]= (int) floor(temp_y/inputmap_.info.resolution);

        int index= target_Coord[0]+inputmap_.info.width*target_Coord[1];
        return index;
        //ROS_INFO("targertcoord: x: %d, y: %d", target_Coord[0], target_Coord[1]);
    }


    bool call_multiplan_service_single(std::vector<geometry_msgs::PoseArray>& path_agent1)
    {

     if(agent1_history.poses.size()>0)
            {
                startpose_agent1.header.stamp=ros::Time().now();
                startpose_agent1.header.frame_id="map";
                startpose_agent1.pose.position.x=last_goal_agent1.pose.position.x+0.1;
                startpose_agent1.pose.position.y=last_goal_agent1.pose.position.y+0.1;
            }
            else{

                startpose_agent1.header.stamp=ros::Time().now();
                startpose_agent1.header.frame_id="map";
                startpose_agent1.pose=agent1_gpose.pose;
                
            }
       startpose_agent1.pose.orientation.w=1.0;

       std::vector<geometry_msgs::PoseStamped> goals_agent1;
       if(search_entropy>0.45)
       {
            goal_sampling_waypoints(agent1_gpose, goals_agent1);
       }
       else{
          int idx=0; 
          goal_sampling_frontiers(agent1_gpose, goals_agent1, idx);
       
       }

       nav_msgs::GetMultiPlan srv_;
       srv_.request.start = startpose_agent1;
       srv_.request.goals= goals_agent1;
       //std::cout<<"startpose_agent1: "<<startpose_agent1<<std::endl;
       std::cout<<"startpose_agent1"<<std::endl;
       std::cout<<"x: "<< startpose_agent1.pose.position.x<<", y: "<<startpose_agent1.pose.position.y<<std::endl;
       std::cout<<"-------------------------"<<std::endl;
       std::cout<<"size of goals: "<<goals_agent1.size()<<std::endl;

       if(planner_srv_client_single.call(srv_))
       {
          ROS_INFO("get MultiPlan service finished!!");
          path_agent1= srv_.response.multiplan.poses;
          return true;
       }
       else
       {
           ROS_WARN("Failed to call service multiplan");
           return false;
       }
    }


    bool call_multiplan_service(std::vector<geometry_msgs::PoseArray>& path_agent1, std::vector<geometry_msgs::PoseArray>& path_agent2)
    {
        if(agent1_history.poses.size()>0)
        {

            startpose_agent1.header.stamp=ros::Time().now();
            startpose_agent1.header.frame_id="map";
            startpose_agent1.pose.position.x=last_goal_agent1.pose.position.x+0.1;
            startpose_agent1.pose.position.y=last_goal_agent1.pose.position.y+0.1;
        }
        else{

            startpose_agent1.header.stamp=ros::Time().now();
            startpose_agent1.header.frame_id="map";
            startpose_agent1.pose=agent1_gpose.pose;
        }

        if(agent2_history.poses.size()>0)
        {

            startpose_agent2.header.stamp=ros::Time().now();
            startpose_agent2.header.frame_id="map";
            startpose_agent2.pose.position.x=last_goal_agent2.pose.position.x+0.1;
            startpose_agent2.pose.position.y=last_goal_agent2.pose.position.y+0.1;
        }
        else{

            startpose_agent2.header.stamp=ros::Time().now();
            startpose_agent2.header.frame_id="map";
            startpose_agent2.pose=agent2_gpose.pose;
        }

       startpose_agent1.pose.orientation.w=1.0;
       startpose_agent2.pose.orientation.w=1.0;


       std::vector<geometry_msgs::PoseStamped> goals_agent1;
       std::vector<geometry_msgs::PoseStamped> goals_agent2;

       if(search_entropy>0.35)
       {
            goal_sampling_waypoints(agent1_gpose, goals_agent1);
            goal_sampling_waypoints(agent2_gpose, goals_agent2);
       }
       else
       {
           int minidx=0;
            goal_sampling_frontiers(agent1_gpose, goals_agent1,minidx);
            goal_sampling_frontiers(agent2_gpose, goals_agent2,minidx);
       }

       nav_msgs::GetMultiMultiPlan srv_;
       srv_.request.start = startpose_agent1;
       srv_.request.start2 = startpose_agent2;
       srv_.request.goals= goals_agent1;
       srv_.request.goals2 = goals_agent2;
       std::cout<<"startpose_agent1"<<std::endl;
       std::cout<<"x: "<< startpose_agent1.pose.position.x<<", y: "<<startpose_agent1.pose.position.y<<std::endl;
       std::cout<<"size of goals: "<<goals_agent1.size()<<std::endl;
       std::cout<<"-------------------------"<<std::endl;
       std::cout<<"startpose_agent2"<<std::endl;
       std::cout<<"x: "<< startpose_agent2.pose.position.x<<", y: "<<startpose_agent2.pose.position.y<<std::endl;
       std::cout<<"size of goals: "<<goals_agent2.size()<<std::endl;
       std::cout<<"-------------------------"<<std::endl;



       if(planner_srv_client.call(srv_))
       {
          ROS_INFO("get MultiMultiPlan service finished!!");
          ROS_INFO("path1 size: %d , path2 size: %d", srv_.response.multiplan.poses.size(), srv_.response.multiplan2.poses.size());
          path_agent1= srv_.response.multiplan.poses;
          path_agent2= srv_.response.multiplan2.poses;
          return true;
       }
       else
       {
           ROS_WARN("Failed to call service multiplan");
           return false;
       }
    }


void goal_sampling_waypoints(const geometry_msgs::PoseStamped agent_pos, std::vector<geometry_msgs::PoseStamped>& goals)
{
    double randx, randy=0.0;
    goals.clear();
    geometry_msgs::PoseStamped tmp_pose;
    tmp_pose.header.stamp =ros::Time().now();
    tmp_pose.header.frame_id="map";
    if(search_entropy>0.5){
        std::map< std::string, std::vector<double> >::iterator map_iter=goal_maps.begin();
        for(map_iter; map_iter!=goal_maps.end();map_iter++)
        {

            randx = (static_cast<float>(rand())/static_cast <float>(RAND_MAX)-0.5)*2*RAND_RANGE;
            randy = (static_cast<float>(rand())/static_cast <float>(RAND_MAX)-0.5)*2*RAND_RANGE;
            //cout<<"randx:" <<randx << "randy:"<<randy <<std::endl;
            tmp_pose.pose.position.x=map_iter->second[0]+randx;
            tmp_pose.pose.position.y=map_iter->second[1]+randy;
            if (tmp_pose.pose.position.x> m_params->xmax || tmp_pose.pose.position.x< m_params->xmin)
                tmp_pose.pose.position.x=map_iter->second[0];
            if (tmp_pose.pose.position.y> m_params->ymax || tmp_pose.pose.position.y< m_params->ymin)
                tmp_pose.pose.position.y=map_iter->second[0];

            //
            //
            //should consider searchmap if dynamic obstacle is occupied
            int search_idx = Coord2CellNum(tmp_pose.pose.position.x, tmp_pose.pose.position.y, search_map);         // get search cell idx 
            double dist_goal = sqrt(pow(tmp_pose.pose.position.x-agent_pos.pose.position.x,2)+pow(tmp_pose.pose.position.y-agent_pos.pose.position.y,2));
            if(search_idx<search_map.data.size())
            {
                if(search_map.data[search_idx]!=int(L_SOCC) and search_map.data[search_idx]!=int(L_DOCC)) 
                {
                    if(dist_goal<Max_Dist )
                        goals.push_back(tmp_pose);
                }
            }
            //goals.push_back(tmp_pose);

        }
    }
    else{
    
         for(int j(0); j< unknown_poses.poses.size();j++)
        {
            tmp_pose.pose=unknown_poses.poses[j];
            double dist_goal = sqrt(pow(tmp_pose.pose.position.x-agent_pos.pose.position.x,2)+pow(tmp_pose.pose.position.y-agent_pos.pose.position.y,2));

            if(dist_goal<Max_Dist )
                goals.push_back(tmp_pose);
        }
    }


    for(int i(0); i< frontier_poses.poses.size();i++) 
    {
        tmp_pose.pose=frontier_poses.poses[i];

        double dist_goal = sqrt(pow(tmp_pose.pose.position.x-agent_pos.pose.position.x,2)+pow(tmp_pose.pose.position.y-agent_pos.pose.position.y,2));
        if(dist_goal<Max_Dist )
            goals.push_back(tmp_pose);
    }

    //for(int j(0); j< unknown_poses.poses.size();j++)
    //{
    
        //tmp_pose.pose=unknown_poses.poses[j];
        //goals.push_back(tmp_pose);
    //}

    return;
}

//This function generate 2d spline using sampled points from nav_msg/Path  
//This function will make (s-x), (s-y) splines using arc length (s)
void Convert_Poses2trjs(const std::vector<geometry_msgs::PoseArray>& paths, std::vector<tk::spline2D>& spline_set)
{
    spline_set.clear();
    //ToDo: check path size
    ROS_INFO("global paths size: %d", paths.size());
    int path_const=20;

    for(int i(0); i< paths.size();i++){
        std::vector<double> xs;
        std::vector<double> ys;
        std::cout<<i<<"-th poses size:"<<paths[i].poses.size()<<std::endl;
        path_const=20;
        if(paths[i].poses.size()<80)
            path_const=10;

        for(int j(0); j< paths[i].poses.size();j++)
        {
            if(j%path_const==0 || (j==paths[i].poses.size()-1)){
                xs.push_back(paths[i].poses[j].position.x);
                ys.push_back(paths[i].poses[j].position.y);
            }
        }
        if(xs.size()>2)
        {
            tk::spline2D spline2d(xs,ys);
            spline_set.push_back(spline2d);
        }
        else{
        
            xs.clear();
            ys.clear();
            for(int j(0); j< paths[i].poses.size();j++)
            {
                xs.push_back(paths[i].poses[j].position.x);
                ys.push_back(paths[i].poses[j].position.y);
            }

            if(xs.size()>2)
            {
                tk::spline2D spline2d(xs,ys);
                spline_set.push_back(spline2d);
            }
            else
            {
                ROS_WARN("failed to sample trajectroies");
            }
        
        }
    }
}



bool near_history(const geometry_msgs::PoseArray trjs_, const geometry_msgs::PoseArray history_)
{
    //if the last point is near one of history (range: Dist_history), return true. 
    bool near_hiostory=false;
    for(int i(0); i< history_.poses.size();i++)
    {
        double dist = sqrt( pow(trjs_.poses.back().position.x-history_.poses[i].position.x,2)+pow(trjs_.poses.back().position.y-history_.poses[i].position.y,2));
        if(dist<Dist_History )
            return true;
    }
    return false;
}

bool near_eachother(const geometry_msgs::Pose pos1, const geometry_msgs::Pose pos2)
{
    //if the last point is near one of history (range: Dist_history), return true. 
    //bool near_=false;
    double dist = sqrt( pow(pos1.position.x-pos2.position.x,2)+pow(pos1.position.y-pos2.position.y,2));
    if(dist<Dist_GOAL)
        return true;
    
    return false;
}



   
//void trjs_to_sample(std::vector<tk::spline2D>& spline_set, std::vector<nav_msgs::Path>& trjs, int horizon=20)
void trjs_to_sample(std::vector<tk::spline2D>& spline_set, std::vector<geometry_msgs::PoseArray>& trjs, 
                    const geometry_msgs::PoseArray hisotry_, int horizon=15)
{
    //output: trjs (sampling points from trajectory with (ds=0.5) using horizon

    std::cout<<"trjs_to_sample function, current_horizon:  "<<horizon<<std::endl;
    double ds=0.5;
    double tmp_x, tmp_y=0.0;
    trjs.clear();
    //trjs.resize(spline_set.size());
    for(int i(0); i <spline_set.size();i++)
    {
        geometry_msgs::PoseArray tmp_posearray;
        double s_length = spline_set[i].get_length();
        //std::cout<<i<<"-th path length: "<< s_length<<std::endl;
        std::vector<double> rangeset = tk::range(0.0, s_length, ds);
        //for(int k(0);k<rangeset.size();k++)
            //std::cout<<"k: "<<k<< ",rangeset[k]" << rangeset[k] <<std::endl;
        auto s_iter=rangeset.begin();
        int count_iter=0;
        for(s_iter;s_iter!=rangeset.end();s_iter++)
        {
            if(count_iter<horizon && horizon>0)
            {
                spline_set[i].calc_positions(*s_iter,tmp_x,tmp_y);
                geometry_msgs::Pose tmp_pose;
                tmp_pose.position.x=tmp_x;
                tmp_pose.position.y=tmp_y;
                //std::cout<<"s_iter" <<*s_iter<<", x: "<<tmp_x <<", y: "<<tmp_y<<std::endl;
                tmp_posearray.poses.push_back(tmp_pose);
                count_iter++;
            }
            else if(horizon<0)
            {
                spline_set[i].calc_positions(*s_iter,tmp_x,tmp_y);
                geometry_msgs::Pose tmp_pose;
                tmp_pose.position.x=tmp_x;
                tmp_pose.position.y=tmp_y;
                tmp_posearray.poses.push_back(tmp_pose);
            }
        }

        //trjs.push_back(tmp_posearray);
        //last point check!!
        if(search_entropy>0.4)
        {
            if(!near_history(tmp_posearray, hisotry_))
            {
                trjs.push_back(tmp_posearray);
            }
        }
        else
            trjs.push_back(tmp_posearray);

    }
}

//double calc_min_distance_from_trj(double px, double py, std::vector<geometry_msgs::Point>& trj)
double calc_min_distance_from_trj(double px, double py, const geometry_msgs::PoseArray trj)
{
    double min_dist = 100.0;
    for(size_t i(0);i<trj.poses.size();i++)
    {
        double tempdist = sqrt(pow(px-trj.poses[i].position.x,2)+pow(py-trj.poses[i].position.y,2));
        if(tempdist < min_dist)
            min_dist = tempdist;
    }

    return min_dist;
}

double get_expected_entropy_infov(const geometry_msgs::Point pose)
{
    double minx,miny,maxx,maxy,xw,yw=0.0;
    double agent_x = pose.x;
    double agent_y = pose.y;

    m_params->calc_grid_map_config(agent_x,agent_y, minx, miny, maxx, maxy, xw, yw);
    std::vector<int> impossible_idx;
    get_impossible_indices(pose, impossible_idx);

    double entropy_sum=0.0;
    int cell_count=0;

    for(int i(0);i < xw; i++){
        for (int j(0); j<yw; j++){
            double px = i * m_params->xyreso + minx;
            double py = j * m_params->xyreso + miny;
        
            if(px > m_params->xmax || px < m_params->xmin)
                continue;
            if(py > m_params->ymax || py < m_params->ymin)
                continue;

            int search_idx = Coord2CellNum(px,py, search_map);

            //convert log-occ to probability
            if( std::find(impossible_idx.begin(), impossible_idx.end(), search_idx) ==impossible_idx.end())
            {
                if(search_map.data[search_idx]==0)
                    cell_count++;
            }
        }
    }
 
    //ROS_INFO("cell counts: %d ", cell_count);
    entropy_sum=double(cell_count)*CELL_MAX_ENTROPY;

    return entropy_sum;
}



double get_expected_entropy_infov_trj( const geometry_msgs::Point pose,geometry_msgs::PoseArray leader_trj, double  dist_th=6.0)
{

    double minx,miny,maxx,maxy,xw,yw=0.0;
    double agent_x = pose.x;
    double agent_y = pose.y;

    m_params->calc_grid_map_config(agent_x,agent_y, minx, miny, maxx, maxy, xw, yw);
    std::vector<int> impossible_idx;
    get_impossible_indices(pose, impossible_idx);

    double entropy_sum=0.0;
    int cell_count=0;

    for(int i(0);i < xw; i++){
        for (int j(0); j<yw; j++){
            double px = i * m_params->xyreso + minx;
            double py = j * m_params->xyreso + miny;
        
            if(px > m_params->xmax || px < m_params->xmin)
                continue;
            if(py > m_params->ymax || py < m_params->ymin)
                continue;

            int search_idx = Coord2CellNum(px,py, search_map);
            double min_dist = calc_min_distance_from_trj(px,py,leader_trj);

            //convert log-occ to probability
            if( std::find(impossible_idx.begin(), impossible_idx.end(), search_idx) ==impossible_idx.end() && 
                    (min_dist>dist_th)){

                if(search_map.data[search_idx]==0)
                    cell_count++;
            }
        }
    }
 
    ROS_INFO("cell counts: %d ", cell_count);
    entropy_sum=double(cell_count)*CELL_MAX_ENTROPY;

    return entropy_sum;

}

double get_expected_entropy_infov_leadgoal( const geometry_msgs::Point pose,geometry_msgs::Pose leader_goal, double  dist_th=6.0)
{

    double minx,miny,maxx,maxy,xw,yw=0.0;
    double agent_x = pose.x;
    double agent_y = pose.y;

    m_params->calc_grid_map_config(agent_x,agent_y, minx, miny, maxx, maxy, xw, yw);
    std::vector<int> impossible_idx;
    get_impossible_indices(pose, impossible_idx);

    double entropy_sum=0.0;
    int cell_count=0;

    for(int i(0);i < xw; i++){
        for (int j(0); j<yw; j++){
            double px = i * m_params->xyreso + minx;
            double py = j * m_params->xyreso + miny;
        
            if(px > m_params->xmax || px < m_params->xmin)
                continue;
            if(py > m_params->ymax || py < m_params->ymin)
                continue;

            int search_idx = Coord2CellNum(px,py, search_map);

            //double dist= sqrt(pow(px-agent_x,2)+ pow((py-agent_y),2));
            //double min_dist = calc_min_distance_from_trj(px,py,leader_trj);

            //convert log-occ to probability
            if( std::find(impossible_idx.begin(), impossible_idx.end(), search_idx) ==impossible_idx.end())
            {
                if(search_map.data[search_idx]==0)
                    cell_count++;
            }
        }
    }
 
    ROS_INFO("cell counts: %d ", cell_count);
    entropy_sum=double(cell_count)*CELL_MAX_ENTROPY;

    return entropy_sum;

}




void precasting(const geometry_msgs::Point pose, std::map<int,std::vector<precastDB> >& precastdb)
{
    precastdb.clear();
    double minx,miny,maxx,maxy,xw,yw=0.0;
    double agent_x = pose.x;
    double agent_y = pose.y;
    int angleid=0;
    m_params->calc_grid_map_config(agent_x,agent_y, minx, miny, maxx, maxy, xw, yw);

    for(int i(0);i < xw; i++){
        for (int j(0); j<yw; j++){
            double px = i * m_params->xyreso + minx;
            double py = j * m_params->xyreso + miny;
        
            double dist = sqrt(pow(px-agent_x, 2) + pow(py-agent_y,2));
            double angle = atan_zero_to_twopi((py-agent_y), (px-agent_x));
            angleid = int(floor(angle / m_params->yawreso));

            precastDB pc(px,py,dist,angle,i,j);

            auto it = precastdb.find(angleid);
            if(it !=precastdb.end())
            {
                it->second.push_back(pc);
            }
            else{
                std::vector<precastDB> DB_vec(1, pc);
                precastdb[angleid]=DB_vec;
            }
        }
    }

    //check function
    //auto map_it =precastdb.begin() ;
    //for(map_it; map_it!=precastdb.end();map_it++)
    //{
        //std::cout<<"angle_id: "<< map_it->first<<", precast_db size: "<< map_it->second.size()<<std::endl;
        //for(int i(0); i< map_it->second.size();i++)
        //{
        //std::cout<<"precast_db size: "<< map_it->second.size()<<std::endl;
        //}
    //}
}


void get_impossible_indices(const geometry_msgs::Point pose, std::vector<int>& impossible_idx)
{
    impossible_idx.clear();
    double agent_x = pose.x;
    double agent_y = pose.y;
    double minx,miny,maxx,maxy,xw,yw=0.0;
    m_params->calc_grid_map_config(agent_x,agent_y, minx, miny, maxx, maxy, xw, yw);

    std::map<int,std::vector<precastDB> > premap;
    precasting(pose,premap);

    //min_angle_map
    std::map<int, double> min_anglemap;

    for(int i(0);i < xw; i++){
        for (int j(0); j<yw; j++){
            double px = i * m_params->xyreso + minx;
            double py = j * m_params->xyreso + miny;
        
            if(px > m_params->xmax || px < m_params->xmin)
                continue;
            if(py > m_params->ymax || py < m_params->ymin)
                continue;

            int gmap_idx = Coord2CellNum(px,py, global_map);
            if(global_map.data[gmap_idx]>0)
            {
                double obs_angle = atan_zero_to_twopi(py-agent_y,px-agent_x);
                int angleid =  int(floor(obs_angle / m_params->yawreso));
                double dist= sqrt(pow(px-agent_x,2)+ pow((py-agent_y),2));

                auto it = min_anglemap.find(angleid);
                if(it !=min_anglemap.end())
                {
                    if(dist<it->second)
                    {
                        min_anglemap[angleid]=dist;
                    }
                }
                else{
                    min_anglemap[angleid]=dist;
                }
            }
        }
    }

    auto mapit = min_anglemap.begin();
    for(mapit; mapit!=min_anglemap.end();mapit ++)
    {
        auto pre_iter= premap.find(mapit->first);
        for(int j(0);j< pre_iter->second.size();j++ )
        {
            
            //for the grid point such that distacne is greater than min_distance, add impossible_idx
           if((pre_iter->second)[j].dist>mapit->second)
           {
            int gmap_idx = Coord2CellNum((pre_iter->second)[j].px,(pre_iter->second)[j].py, search_map);
            impossible_idx.push_back(gmap_idx);
           }
        }
    }


}

int calc_IG_trjs_with_leadtrj(const std::vector<geometry_msgs::PoseArray> trjs_candidates, const geometry_msgs::PoseArray lead_trj, int num_points=2)
{

  
    if(trjs_candidates.size()>0)
    {
        std::vector<double> utilities, igs,travels;
        //tpoints=[]
        for(auto it = trjs_candidates.begin(); it!= trjs_candidates.end(); ++it)
        {
            int pose_size = (*it).poses.size();
            int mid_point = floor(pose_size/2);
            double ig=0.0;
            double travel=0.0;
            std::vector<int> idx_set(num_points,0);
            if(num_points==2)
            {
                idx_set[0]=mid_point;
                idx_set[1]=pose_size-1;
            }
            else{

                int size_const= floor(pose_size/num_points);
                for(int k(0);k<num_points-1;k++)
                {
                    idx_set[k]=size_const*(k+1);
                }
                idx_set[num_points-1]=pose_size-1;
            
            }
        
            //first point (should be saved because we are calculating the expected travel cost from this point)
            double ref_x=(*it).poses.front().position.x;
            double ref_y=(*it).poses.front().position.y;
            //mid point
            //end point
            for(int k(0);k<num_points;k++)
            {
              //ig+=get_expected_entropy_infov_trj((*it).poses[idx_set[k]].position, lead_trj);
              travel+= sqrt(pow((*it).poses[idx_set[k]].position.x-ref_x,2)+pow((*it).poses[idx_set[k]].position.x-ref_y,2));
              ref_x=(*it).poses[idx_set[k]].position.x;
              ref_y=(*it).poses[idx_set[k]].position.y;
              double min_dist = calc_min_distance_from_trj(ref_x,ref_y,lead_trj);
              if(min_dist >3.0)
                  ig+=get_expected_entropy_infov((*it).poses[idx_set[k]].position);
            }

            double utility = weight_entropy*ig - weight_travel*travel;
            std::cout<<"ig: "<<ig<<", travel: "<<travel <<", utility: "<<utility<<std::endl;
            utilities.push_back(utility);
            igs.push_back(ig);
            travels.push_back(travel);
        }

        //find l
        auto maxIt = std::max_element(utilities.begin(), utilities.end());
        double minElement = *maxIt;
        int max_idx = maxIt -utilities.begin();
        std::cout<<"max_index" << max_idx<<std::endl;
        std::cout<<"max_utilities" << *maxIt<<std::endl;
        return max_idx;

    }
    else
    {
        ROS_WARN("Something wrong!!!----- No trjs candidates ----");
        return -1;
    }

}

int calc_IG_trjs_with_leadgoal(const std::vector<geometry_msgs::PoseArray> trjs_candidates, const geometry_msgs::Pose lead_goal, int num_points=2)
{

    if(trjs_candidates.size()>0)
    {
        std::vector<double> utilities, igs,travels;
        //tpoints=[]
        for(auto it = trjs_candidates.begin(); it!= trjs_candidates.end(); ++it)
        {
            int pose_size = (*it).poses.size();
            int mid_point = floor(pose_size/2);
            double ig=0.0;
            double travel=0.0;
            std::vector<int> idx_set(num_points,0);
            if(num_points==2)
            {
                idx_set[0]=mid_point;
                idx_set[1]=pose_size-1;
            }
            else{

                int size_const= floor(pose_size/num_points);
                for(int k(0);k<num_points-1;k++)
                {
                    idx_set[k]=size_const*(k+1);
                }
                idx_set[num_points-1]=pose_size-1;
            
            }
        
            //first point (should be saved because we are calculating the expected travel cost from this point)
            double ref_x=(*it).poses.front().position.x;
            double ref_y=(*it).poses.front().position.y;
            //mid point
            //end point
            for(int k(0);k<num_points;k++)
            {
              //ig+=get_expected_entropy_infov_trj((*it).poses[idx_set[k]].position, lead_trj);
              travel+= sqrt(pow((*it).poses[idx_set[k]].position.x-ref_x,2)+pow((*it).poses[idx_set[k]].position.x-ref_y,2));
              ref_x=(*it).poses[idx_set[k]].position.x;
              ref_y=(*it).poses[idx_set[k]].position.y;
              double tmp_dist=sqrt(pow(ref_x-lead_goal.position.x,2)+pow(ref_y-lead_goal.position.y,2));
              if(tmp_dist>7.0)
                  ig+=get_expected_entropy_infov((*it).poses[idx_set[k]].position);
            }

            double utility = weight_entropy*ig - weight_travel*travel;
            std::cout<<"ig: "<<ig<<", travel: "<<travel <<", utility: "<<utility<<std::endl;
            utilities.push_back(utility);
            igs.push_back(ig);
            travels.push_back(travel);
        }

        //find l
        auto maxIt = std::max_element(utilities.begin(), utilities.end());
        double minElement = *maxIt;
        int max_idx = maxIt -utilities.begin();
        std::cout<<"max_index" << max_idx<<std::endl;
        std::cout<<"max_utilities" << *maxIt<<std::endl;
        return max_idx;

    }
    else
    {
        ROS_WARN("Something wrong!!!----- No trjs candidates ----");
        return -1;
    }

}








int calc_IG_trjs(const std::vector<geometry_msgs::PoseArray> trjs_candidates, int num_points=2)
{
 // Calculating Information gain on trajectories
 // 1) calculating sampling point for time horizon 
 // 2) calculating FOV region over sampling points 
 // 3) Collecting IG gain for overlapped region
 // 4) input trjs should be re-sampled w.r.t arc length for managing horizon
 // double weight_t=1.0
  
    if(trjs_candidates.size()>0)
    {
        std::vector<double> utilities, igs,travels;
        //tpoints=[]
        for(auto it = trjs_candidates.begin(); it!= trjs_candidates.end(); ++it)
        {
            int pose_size = (*it).poses.size();
            int mid_point = floor(pose_size/2);
            double ig=0.0;
            double travel=0.0;
            std::vector<int> idx_set(num_points,0);
            if(num_points==2)
            {
                idx_set[0]=mid_point;
                idx_set[1]=pose_size-1;
            }
            else{

                int size_const= floor(pose_size/num_points);
                for(int k(0);k<num_points-1;k++)
                {
                    idx_set[k]=size_const*(k+1);
                }
                idx_set[num_points-1]=pose_size-1;
            
            }
        
            //first point (should be saved because we are calculating the expected travel cost from this point)
            double ref_x=(*it).poses.front().position.x;
            double ref_y=(*it).poses.front().position.y;
            //mid point
            //end point
            for(int k(0);k<num_points;k++)
            {
              ig+=get_expected_entropy_infov((*it).poses[idx_set[k]].position);
              travel+= sqrt(pow((*it).poses[idx_set[k]].position.x-ref_x,2)+pow((*it).poses[idx_set[k]].position.x-ref_y,2));
              ref_x=(*it).poses[idx_set[k]].position.x;
              ref_y=(*it).poses[idx_set[k]].position.y;
            }

            double utility = weight_entropy*ig - weight_travel*travel;
            utilities.push_back(utility);
            igs.push_back(ig);
            travels.push_back(travel);
            std::cout<<"ig: "<<ig<<", travel: "<<travel <<", utility: "<<utility<<std::endl;
        }


        //find l
        auto maxIt = std::max_element(utilities.begin(), utilities.end());
        double minElement = *maxIt;
        int max_idx = maxIt -utilities.begin();
        std::cout<<"max_index" << max_idx<<std::endl;
        std::cout<<"max_utility" << *maxIt<<std::endl;
        return max_idx;

    }
    else
    {
        ROS_WARN("Something wrong!!!----- No trjs candidates ----");
        return -1;
    }
}



//void goal_sampling_frontiers(const geometry_msgs::PoseStamped agent_pos, geometry_msgs::PoseArray& goals)
void goal_sampling_frontiers(const geometry_msgs::PoseStamped agent_pos, std::vector<geometry_msgs::PoseStamped>& goals, int& min_dist_idx)
{
    goals.clear();
    geometry_msgs::PoseStamped tmp_pose;
    tmp_pose.header.stamp=ros::Time().now();
    tmp_pose.header.frame_id="map";

    //find minimum_dist from agent_pos
    std::vector<double> distances;
    double min_dist=100;
    //double min_dist_idx=0;
    double tmp_dist=0.0;
    for(int i(0); i< unknown_poses.poses.size();i++) 
    {
        tmp_pose.pose=unknown_poses.poses[i];
        goals.push_back(tmp_pose);
        tmp_dist=sqrt(pow(tmp_pose.pose.position.x-agent_pos.pose.position.x,2)+pow(tmp_pose.pose.position.y-agent_pos.pose.position.y,2));
        if(tmp_dist<min_dist)
        {
            min_dist=tmp_dist;
            min_dist_idx=i;
        }

    }

    if(min_dist==100)
    {
         for(int i(0); i< frontier_poses.poses.size();i++) 
        {
            tmp_pose.pose=frontier_poses.poses[i];

            tmp_dist=sqrt(pow(tmp_pose.pose.position.x-agent_pos.pose.position.x,2)+pow(tmp_pose.pose.position.y-agent_pos.pose.position.y,2));
            if(tmp_dist<min_dist)
            {
                min_dist=tmp_dist;
                min_dist_idx=i;
            }
        }
    
    }
    return;
}


void LoadPositionfromConfig(ros::NodeHandle n, std::string input_locations)
{
      XmlRpc::XmlRpcValue input_loc;
      std::string param_name = "waypoints/" + input_locations;
      std::cout<<param_name<<std::endl;
      n.getParam(param_name, input_loc);
      std::vector<double> tmp_pos(2,0.0);
      tmp_pos[0]=static_cast<double>(input_loc[0]);
      tmp_pos[1]=static_cast<double>(input_loc[1]);
      goal_maps[input_locations]=tmp_pos;
}



   void parseparameters(ros::NodeHandle n)
  {
      std::string target_frame;
      n.getParam("waypoints/ref_frame", target_frame);
      std::vector<std::string> locations_vector;
      //std::string locations_list;
      XmlRpc::XmlRpcValue location_list;
      ROS_INFO_STREAM("reference frame: " << target_frame);
      n.getParam("waypoints/locations", location_list);
      //ROS_INFO("type %s",location_list.getType());
      ROS_ASSERT(location_list.getType() ==XmlRpc::XmlRpcValue::TypeArray);
      for(size_t j(0);j<location_list.size();j++)
      {
          //ROS_INFO_STREAM("Loded "<< "-" << (std::string)(location_list[j]));
          std::string location_str = static_cast<std::string>(location_list[j]);
          locations_vector.push_back(location_str);
          LoadPositionfromConfig(n,location_str);
      }
      ROS_INFO("parsing finished");
      //ROS_ASSERT(ref_frame.getType() == XmlRpc::XmlRpcValue::TypeStruct);
      //print loaded parameters
      //std::map< std::string, std::vector<double> >::iterator map_iter=goal_maps.begin();
      //for(map_iter; map_iter!=goal_maps.end();map_iter++)
      //{
          //std::cout<<map_iter->first <<" : " <<map_iter->second[0]<< ", "<<map_iter->second[1]<<std::endl;
      //}

}

void clustering()
{
    //std::vector<std::vector<double> >agent_xs;
    //std::vector<std::vector<double> >agent_ys;
    agent_xs.resize(3);
    agent_ys.resize(3);
    if(!clustered)
    {
        int data_size = unknown_poses.poses.size();

        std::vector<double> xs;
        std::vector<double> ys;

        for(size_t i(0);i<data_size;i++)
        {
        
            double tx = unknown_poses.poses[i].position.x;
            double ty = unknown_poses.poses[i].position.y;
            xs.push_back(tx);
            ys.push_back(ty);
        }

        
        
        std::vector<geometry_msgs::Pose> Posevec;
        Posevec.push_back(agent1_gpose.pose);
        Posevec.push_back(agent2_gpose.pose);
        Posevec.push_back(agent3_gpose.pose);


        std::vector<double> weights;
        weights.push_back(1.0);
        weights.push_back(1.0);
        weights.push_back(1.0);
          
        HCluster test_cluster(3, xs,ys, Posevec, weights);
       for(size_t i(0);i<3;i++)
        {
            test_cluster.get_labeled_x_y(i, agent_xs[i],agent_ys[i]);
        }

       std::ofstream outfile;
        outfile.open("/home/mk/workspaces/data/cluster_sol.csv", std::ofstream::out | std::ofstream::trunc);
        for(int i(0);i<xs.size();i++)
        {
            outfile<<i<<"\t"<< xs[i]<<"\t"<<ys[i]<<"\t"<<test_cluster.m_labels[i]<<"\n";
        }
        outfile.close();

      std::ofstream outfile2;
        outfile2.open("/home/mk/workspaces/data/cluster_con.csv", std::ofstream::out | std::ofstream::trunc);
        for(int i(0);i<Posevec.size();i++)
        {
            outfile2<<i<<"\t"<< Posevec[i].position.x<<"\t"<<Posevec[i].position.y<<"\n";
        }
        outfile2.close();


        clustered=true;

    }
    else{

    //std::vector<double> agent0_ys;
    //std::vector<double> agent1_xs;
    //std::vector<double> agent1_ys;
    //std::vector<double> agent2_xs;
    //std::vector<double> agent2_ys;

     
    std_msgs::ColorRGBA blue;
    blue.r = 0; blue.g = 0; blue.b = 1.0; blue.a = 1.0;
    std_msgs::ColorRGBA red;
    red.r = 1.0; red.g = 0; red.b = 0; red.a = 1.0;
    std_msgs::ColorRGBA green;
    green.r = 0; green.g = 1.0; green.b = 0; green.a = 1.0;

     //ROS_DEBUG("visualising %lu frontiers", frontiers.size());
     visualization_msgs::MarkerArray markers_msg;
    std::vector<visualization_msgs::Marker>& markers = markers_msg.markers;
    //visualization_msgs::Marker m;

     visualization_msgs::Marker m;
        m.header.frame_id ="map";
        m.header.stamp = ros::Time::now();
        m.ns = "unknowns";
        m.scale.x = 1.0;
        m.scale.y = 1.0;
        m.scale.z = 1.0;
        m.color.r = 0;
        m.color.g = 0;
        m.color.b = 255;
        m.color.a = 255;
        // lives forever
        m.lifetime = ros::Duration(0);
        m.frame_locked = true;


    size_t id = 0;
    for(size_t i(0);i<3;i++)
    {
        m.action = visualization_msgs::Marker::ADD;
        //size_t id = 0;
        for (int j(0);j<agent_xs[i].size();j++) {
            m.type = visualization_msgs::Marker::SPHERE;
            m.id = int(id);
            m.pose.position.x = agent_xs[i][j];
            m.pose.position.y = agent_ys[i][j];
            m.pose.position.z = 0.5;
            m.pose.orientation.w = 1.0;
            m.scale.x = 1.1;
            m.scale.y = 1.1;
            m.scale.z = 1.1;
            //m.points = frontier.points;
            
            if(i==0)
                m.color=red;
            else if(i==1)
                m.color=green;
            else
                m.color=blue;
                
            markers.push_back(m);
            ++id;
        }
   

    }
    visual_marker_pub.publish(markers_msg);

    }
}


void set_goal_from_path(const geometry_msgs::Pose pose_, geometry_msgs::PoseStamped& goal)
{

    goal.header.stamp=ros::Time().now();
    goal.header.frame_id="map";
    goal.pose=pose_;
    goal.pose.orientation.w=1.0;
}



  
protected:
    
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<search_service::MultiSearchAction> as_;
  ros::ServiceClient planner_srv_client;
  ros::ServiceClient planner_srv_client_single;
  std::string action_name_;
  int data_count_, goal_;
  float sum_, sum_sq_;
  search_service::MultiSearchFeedback feedback_;
  search_service::MultiSearchResult result_;

  ros::Subscriber agent1_pose_sub;
  ros::Subscriber agent2_pose_sub;
  ros::Subscriber agent3_pose_sub;
  ros::Subscriber agent1_localmap_sub;
  ros::Subscriber agent2_localmap_sub;
  ros::Subscriber agent3_localmap_sub;
  ros::Subscriber frontiers_poses_sub;
  ros::Subscriber unknown_poses_sub;

  ros::Subscriber global_map_sub;
  ros::Publisher agent1_path_pub;
  ros::Publisher agent2_path_pub;
  ros::Publisher agent3_path_pub;
  ros::Publisher search_map_pub;
  ros::Publisher search_goal_pub;
  ros::Publisher search_goal2_pub;
  ros::Publisher search_goal3_pub;
  ros::Publisher agent1_move_cancel_pub;
  ros::Publisher search_entropy_pub;
  ros::Publisher visual_marker_pub;
  int direction_z;

  geometry_msgs::PoseStamped agent1_gpose;
  geometry_msgs::PoseStamped agent2_gpose;
  geometry_msgs::PoseStamped agent3_gpose;

  geometry_msgs::PoseStamped searchgoal_agent1;
  geometry_msgs::PoseStamped searchgoal_agent2;
  geometry_msgs::PoseStamped searchgoal_agent3;
  geometry_msgs::PoseStamped startpose_agent1;
  geometry_msgs::PoseStamped startpose_agent2;
  geometry_msgs::PoseStamped startpose_agent3;
  geometry_msgs::PoseStamped last_goal_agent1;
  geometry_msgs::PoseStamped last_goal_agent2;
  geometry_msgs::PoseStamped last_goal_agent3;
  geometry_msgs::PoseArray agent1_history;
  geometry_msgs::PoseArray agent2_history;
  geometry_msgs::PoseArray agent3_history;
  geometry_msgs::PoseArray agents_history;
  geometry_msgs::PoseArray frontier_poses;
  geometry_msgs::PoseArray unknown_poses;

  //location map--villa_navigation
  geometry_msgs::PoseArray subgoals;
  nav_msgs::OccupancyGrid agent1_local_map;
  nav_msgs::OccupancyGrid agent2_local_map;
  nav_msgs::OccupancyGrid agent3_local_map;
  nav_msgs::OccupancyGrid scaled_global_map;
  nav_msgs::OccupancyGrid global_map;
  nav_msgs::OccupancyGrid search_map;
  nav_msgs::OccupancyGrid info_map;
  tf::TransformListener   listener;

  //std::vector<double> agent1_pose;
  //std::vector<double> agent2_pose;
  //std::vector<double> agent3_pose;

  bool IsGoal;
  bool IsActive;
  double srv_time; 
  double tolerance;
  bool agent1_local_map_updated;
  bool agent2_local_map_updated;
  bool agent3_local_map_updated;
  bool global_map_updated;
  bool global_pose_a1_updated;
  bool agent1_pose_updated;
  bool agent2_pose_updated;
  bool agent3_pose_updated;
  bool called_once;
  bool neargoal_agent1;
  bool neargoal_agent2;
  bool neargoal_agent3;
  double search_entropy;
  double total_entropy;
  double occ_entropy;
  int no_count;
  int horizon;
  bool clustered;

  double weight_entropy; 
  double weight_travel; 

  std::vector<std::vector<double> >agent_xs;
  std::vector<std::vector<double> >agent_ys;


  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener* tf2_listener; 
  geometry_msgs::TransformStamped map_en_to_map;
  geometry_msgs::TransformStamped map_to_base;
  Map_params* m_params;
  std::map< std::string, std::vector<double> > goal_maps;
  std::map<int, std::vector<precastDB>> precast_map;
public:

  //StateLatticePlannerROS* pslp_planner;

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "multisearch_action");
  MultiSearchManager manager(ros::this_node::getName());
  double ros_rate = 2;
  ros::Rate r(ros_rate);

  while (ros::ok())
  {
      
	 ros::spinOnce();
     r.sleep();
  }
  ros::spin();

  return 0;
}
