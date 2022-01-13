#include "ros/ros.h"
#include <math.h>
#include "spline.h"
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

#define MAP_RES 0.05
//#define M_PI 3.141592
#define NUM_SUBS 3
#define Dist_History 2.0
#define Max_Dist 30.0
//#define RAND_MAX 1
#define RAND_RANGE 0.35
#define L_SOCC 4.595 // np.log(0.99/0.01)
#define L_DOCC 2.19 // np.log(0.9/0.1)
#define L_FREE -4.595// np.log(0.01/0.99)
#define CELL_MAX_ENTROPY 0.693147
#define GOAL_THRESHOLD 2.0

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
    //precastDB():px_(0.0),py_(0.0),dist_(0,0),angle_(0.0),ix(0),iy(0){}
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
        Map_params():xyreso(0.5),yawreso(0.2), xmin(-11.0),xmax(11.0),ymin(0.0),
        ymax(13.0), sensor_range(7.0)
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
class Server_params
{
   public:
        Server_params():weight_entropy(0.15),weight_travel(1.7), horizon(15),x_err(0.0),y_err(0.0)
        {
            //xw = int(round((xmax - xmin) / xyreso));
            //yw = int(round((ymax - ymin) / xyreso));
        }

       double weight_entropy; 
       double weight_travel; 
       int horizon;
       //double xmin=-11.0;
       //double xmax=11.0
       //double ymin=1.0
       //double ymax=13.0
       double x_err;
       double y_err;
       //double vel_cmd = Twist()
       //double target_pose = Point()
       //double feedback_ = MultiSearchFeedback()
       //double result_= MultiSearchResult()
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
  no_count(0),
  horizon(15),
  called_once(false),
  weight_entropy(0.25),
  weight_travel(1.0),
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
     //self.srv_client=rospy.ServiceProxy('planner/planner/make_multimultiplan', GetMultiMultiPlan)
     //self.tolerance=0.5

     //publishers
    search_map_pub=nh_.advertise<nav_msgs::OccupancyGrid>("/search_map",50,true);
    search_goal_pub=nh_.advertise<geometry_msgs::PoseStamped>("/search_goal2",50,true);
    //agent1_move_cancel_pub=nh_.advertise<actionlib_msgs::GoalID>("/localnavi_server/cancel",50,true);
    search_entropy_pub=nh_.advertise<std_msgs::Float32>("/search_entropy",50,true);

    //search_goal2_pub=nh_.advertise<geometry_msgs::PoseStamped>("/search_goal2",50,true);

    //planner_srv_client= nh_.serviceClient<nav_msgs::GetMultiMultiPlan>("/planner/planner/make_multimultiplan");
    planner_srv_client_single= nh_.serviceClient<nav_msgs::GetMultiPlan>("/planner/planner/make_multiplan");
    

    //subscribers
    global_map_sub = nh_.subscribe<nav_msgs::OccupancyGrid>("/scaled_static_map", 1, &MultiSearchManager::global_map_callback, this);
    frontiers_poses_sub=nh_.subscribe<geometry_msgs::PoseArray>("/frontier_poses",10,&MultiSearchManager::frontiers_callback,this);
    unknown_poses_sub=nh_.subscribe<geometry_msgs::PoseArray>("/unknown_poses",10,&MultiSearchManager::unknown_poses_callback,this);
    agent1_pose_sub=nh_.subscribe<geometry_msgs::PoseStamped>("/global_pose",10,&MultiSearchManager::global_pose_a1_callback,this);
    agent1_localmap_sub= nh_.subscribe<nav_msgs::OccupancyGrid>("/dynamic_obstacle_map",10,&MultiSearchManager::agent1_localmap_callback,this);
    //agent2_pose_sub=nh_.subscribe<geometry_msgs::PoseStamped>("/global_pos_hsre",10,&MultiSearchManager::global_pose_hsr_callback,this);
    //agent1_localmap_sub= nh_.subscribe<nav_msgs::OccupancyGrid>("/dynamic_obstacle_map",10,&MultiSearchManager::agent2_localmap_callback,this);
    //agent2_localmap_sub= nh_.subscribe<nav_msgs::OccupancyGrid>("/costmap_node/costmap/costmap",10,&MultiSearchManager::agent1_localmap_callback,this);




    //receive scaled_global_map
    nav_msgs::OccupancyGrid::ConstPtr shared_map;
    shared_map= ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/scaled_static_map",nh_);
    if(shared_map!= NULL){
     scaled_global_map= *shared_map;
     crop_globalmap(*shared_map);
    }
    total_entropy-=occ_entropy;
    ROS_INFO("total_initial_entropy: %.2f",  total_entropy);

    global_pose_a1.resize(3,0.0);
    global_pose_hsr.resize(3,0.0);
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

        //pose_out.pose.orientation.x=0;
        //pose_out.pose.orientation.y=0;
        //pose_out.pose.orientation.z=0;
        //pose_out.pose.orientation.w=1;
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
    //IsActive=true;
    //IsGoal=false;
    ROS_INFO("MultiSearchManager action is called!!");

    if(called_once)
    {
        agent1_history.poses.push_back(agent1_pose.pose);
        //agent2_history.poses.push_back(agent2_pose.pose);
    }

    double sleep_rate= 2.0;
    //if(!neargoal_agent1 and !neargoal_agent2)
    if(!neargoal_agent1 && called_once)
    {
      ROS_INFO("Robot is not close to the previous goal. wait until it's near to that goal");
      ROS_INFO("search_entropy: %2.lf", search_entropy);
      
      result_.goal.pose.orientation=agent1_pose.pose.orientation;
      //search_goal_pub.publish(result_.goal);
      as_.setSucceeded(result_);
      sleep(2.0);
      //self.publish_selected_goal(self.result_.goal)
      //self.result_.goal.pose.orientation=self.agent2_ori
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
    std::vector<geometry_msgs::PoseStamped> goals_agent1;


    if(search_entropy>0.35)
    {

        if(call_multiplan_service_single(agent1_path))
        {
            //agent1-path
            std::vector<tk::spline2D> agent1_splines;
            Convert_Poses2trjs(agent1_path, agent1_splines);
            trjs_to_sample(agent1_splines, agent1_path, agent1_history, horizon);
            int maxidx = calc_IG_trjs(agent1_path, num_points);
        if(maxidx>=0)
            set_goal_from_path(agent1_path[maxidx].poses.back(), result_.goal);
        else
        {

            int minidx=0;
            goal_sampling_frontiers(agent1_pose, goals_agent1, minidx);
            result_.goal=goals_agent1[minidx];
        }

        called_once=true;
        }
        else{
            ROS_WARN("MultiPlan service failed");
            //geometry_msgs::PoseArray goals_agent1;
            int minidx=0;
            goal_sampling_frontiers(agent1_pose, goals_agent1, minidx);
            result_.goal=goals_agent1[minidx];
        }
    }
    else{
    
   	ROS_INFO("sampling from frontiers");
        int minidx=0;
        goal_sampling_frontiers(agent1_pose, goals_agent1,minidx);
        result_.goal=goals_agent1[minidx];
    }

        
    agent1_history.poses.push_back(result_.goal.pose);
    last_goal_agent1=result_.goal;
    search_entropy = get_searchmap_entropy()/ total_entropy;
    //cancel previous search action
    actionlib_msgs::GoalID cancel_msg;
    ROS_INFO("current entropy: %.2lf", search_entropy);
    search_goal_pub.publish(result_.goal);
    std_msgs::Float32 entropy_msg;
    entropy_msg.data=search_entropy;
    search_entropy_pub.publish(entropy_msg);
    as_.setSucceeded(result_);
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
  
      frontier_poses=*msg;
  }

  void unknown_poses_callback(const geometry_msgs::PoseArray::ConstPtr& msg)
  {
  
      unknown_poses=*msg;
  
  }



  void global_pose_hsr_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
      //* global_pose w.r.t "map" frame
      agent2_pose=*msg;
      global_pose_hsr[0]=msg->pose.position.x;
      global_pose_hsr[1]=msg->pose.position.y;
      global_pose_hsr[2]=tf2::getYaw(msg->pose.orientation);
      global_pose_hsr_updated=true;

      if(called_once)
        {
            double temp_dist = sqrt(pow(last_goal_agent2.pose.position.x-global_pose_hsr[0],2)
                                        +pow(last_goal_agent2.pose.position.y-global_pose_hsr[1],2));
            if(temp_dist<GOAL_THRESHOLD)
                neargoal_agent2=true;
            else
                neargoal_agent2=false;
    }
  }


  void global_pose_a1_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
      //* global_pose w.r.t "map" frame
      agent1_pose=*msg;
      global_pose_a1[0]=msg->pose.position.x;
      global_pose_a1[1]=msg->pose.position.y;
      global_pose_a1[2]=tf2::getYaw(msg->pose.orientation);
      global_pose_a1_updated=true;

      if(called_once)
      {
          double temp_dist = sqrt(pow(last_goal_agent1.pose.position.x-global_pose_a1[0],2)
                                        +pow(last_goal_agent1.pose.position.y-global_pose_a1[1],2));
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
    //check_obstacle
}

  void agent2_localmap_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    agent2_local_map=*msg;
    agent2_local_map_updated = true;

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
        target_Coord[1]= (int)floor(temp_y/inputmap_.info.resolution);

        int index= target_Coord[0]+inputmap_.info.width*target_Coord[1];
        return index;
        //ROS_INFO("targertcoord: x: %d, y: %d", target_Coord[0], target_Coord[1]);
    }


    bool CheckRotation(double cur_angle, double desired_angle, double err_criterion)
    {
        double temp_dist=0.0;

        temp_dist=pow((cur_angle-desired_angle),2);
        temp_dist=sqrt(temp_dist);
        ROS_INFO("temp_yaw_diff: %.4lf ", temp_dist);

        double temp_criterion =sqrt(pow((temp_dist-err_criterion),2));
        //ROS_INFO("temp_yaw_error: %.4lf ", temp_criterion);

        if(temp_criterion<err_criterion)
            return true;

        return false;
    }

    bool checkDistance(std::vector<double>& pos1, std::vector<double>& pos2, double err_criterion)
    {
        if((pos1.size()<1) || (pos2.size()<1))
        {
            ROS_INFO("vector size wrong");
            return false;
        }
        double temp_dist=0.0;
        for(size_t i(0);i<2;i++) 
            temp_dist+=pow((pos1[i]-pos2[i]),2);

        temp_dist=sqrt(temp_dist);

        ROS_INFO("temp_dist: %.4lf ", temp_dist);
        //double temp_criterion =sqrt(pow((temp_dist-err_criterion),2));

        //ROS_INFO("distance error: %.4lf ", temp_criterion);
        if( temp_dist<err_criterion)
            return true;

        return false;
    }


   void get_cmd_gains(std::vector<double>& cur_pos, std::vector<double>& goal_pos, std::vector<double>& gains)
    {

      try{
          map_to_base= tf_buffer.lookupTransform("map", "base", ros::Time(0), ros::Duration(1.0) );
      }
      catch(tf2::TransformException &ex){
      
          ROS_WARN("lookup Transform Failed: %s", ex.what());
      }
      geometry_msgs::PoseStamped pose_in;     //added by mk, ryan
      pose_in.header.stamp=ros::Time(0);
      pose_in.header.frame_id="map";
      pose_in.pose.position.x=goal_pos[0];
      pose_in.pose.position.y=goal_pos[1];
      pose_in.pose.orientation.w=1.0;
 
      geometry_msgs::PoseStamped pose_out;     //added by mk, ryan
      try{
          //tf2::doTransform(*msg, *msg, map_en_to_map);
          tf_buffer.transform(pose_in,pose_out, "base");
      }
      catch (tf2::TransformException &ex)
      {
          ROS_WARN("global_pose: Transform Failed: %s", ex.what());
          sleep(0.10);
          return;
      }
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
                startpose_agent1.pose=agent1_pose.pose;
                
            }
       startpose_agent1.pose.orientation.w=1.0;

       std::vector<geometry_msgs::PoseStamped> goals_agent1;
       if(search_entropy>0.3)
       {
            goal_sampling_waypoints(agent1_pose, goals_agent1);
       }
       else{
       
           int minidx=0;
           goal_sampling_frontiers(agent1_pose, goals_agent1, minidx);
       
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
            startpose_agent1.pose.position.x=last_goal_agent1.pose.position.x+0.1;
            startpose_agent1.pose.position.y=last_goal_agent1.pose.position.y+0.1;
        }
        else{
            startpose_agent1.pose=agent1_pose.pose;
        }

        if(agent2_history.poses.size()>0)
        {
            startpose_agent2.pose.position.x=last_goal_agent2.pose.position.x+0.1;
            startpose_agent2.pose.position.y=last_goal_agent2.pose.position.y+0.1;
        }
        else{
            startpose_agent2.pose=agent2_pose.pose;
        }

       startpose_agent1.pose.orientation.w=1.0;
       startpose_agent2.pose.orientation.w=1.0;


       std::vector<geometry_msgs::PoseStamped> goals_agent1;
       std::vector<geometry_msgs::PoseStamped> goals_agent2;
       if(search_entropy>0.25)
       {
            goal_sampling_waypoints(agent1_pose, goals_agent1);
            goal_sampling_waypoints(agent2_pose, goals_agent2);
       }
       else
       {
       
           int minidx=0;
            goal_sampling_frontiers(agent1_pose, goals_agent1, minidx);
            goal_sampling_frontiers(agent2_pose, goals_agent2, minidx);
       
       }

       nav_msgs::GetMultiMultiPlan srv_;
       srv_.request.start = startpose_agent1;
       srv_.request.start2 = startpose_agent2;
       srv_.request.goals= goals_agent1;
       srv_.request.goals2 = goals_agent2;

       //if(planner_srv_client.call(srv_))
       //{
          //ROS_INFO("get MultiPlan service finished!!");
          //path_agent1= srv_.response.multiplan.poses;
          //path_agent2= srv_.response.multiplan2.poses;
          //return true;
       //}
       //else
       //{
           //ROS_WARN("Failed to call service multiplan");
           //return false;
       //}

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

            //tmp_pose.pose.position.x=map_iter->second[0];
            //tmp_pose.pose.position.y=map_iter->second[1];
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
        //std::cout<<i<<"-th poses size:"<<paths[i].poses.size()<<std::endl;
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
        if(xs.size()>3)
        {
            tk::spline2D spline2d(xs,ys);
            spline_set.push_back(spline2d);
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
double calc_min_distance_from_trj(double px, double py, geometry_msgs::PoseArray& trj)
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


//double get_expected_entropy_infov_trj( const geometry_msgs::Point pose,std::vector<geometry_msgs::Point> leader_trj, double  dist_th=5.0)
double get_expected_entropy_infov_trj( const geometry_msgs::Point pose,geometry_msgs::PoseArray leader_trj, double  dist_th=5.0)
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

int calc_IG_trjs_with_leadtrj(const std::vector<geometry_msgs::PoseArray> trjs_candidates, const geometry_msgs::PoseArray, int num_points=2)
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
            idx_set[0]=mid_point;
            idx_set[1]=pose_size-1;
        
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
            idx_set[0]=mid_point;
            idx_set[1]=pose_size-1;
        
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
//void goal_sampling_frontiers(const geometry_msgs::PoseStamped agent_pos, std::vector<geometry_msgs::PoseStamped>& goals)
//{
    //goals.clear();
    //geometry_msgs::PoseStamped tmp_pose;
    //tmp_pose.header.stamp=ros::Time().now();
    //tmp_pose.header.frame_id="map";

    //for(int i(0); i< frontier_poses.poses.size();i++) 
    //{
        //tmp_pose.pose=frontier_poses.poses[i];
        //goals.push_back(tmp_pose);
    //}

    //return;
//}

//void goal_sampling_frontiers(const geometry_msgs::PoseStamped agent_pos, std::vector<geometry_msgs::PoseStamped>& goals, int& min_dist_idx)
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
    
	 ROS_INFO("frontiers");
    }
    else
    {
	 ROS_INFO("unknown_poses");
    }




    return;
}




void LoadPositionfromConfig(ros::NodeHandle n, std::string input_locations)
{
      XmlRpc::XmlRpcValue input_loc;
      std::string param_name = "waypoints/" + input_locations;
      n.getParam(param_name, input_loc);
      std::vector<double> tmp_pos(2,0.0);
      tmp_pos[0]=static_cast<double>(input_loc[0]);
      tmp_pos[1]=static_cast<double>(input_loc[1]);
      goal_maps[input_locations]=tmp_pos;
}



    //void replanning(int& goal_idx)
    //{
        //nav_msgs::GetPlan plan_srv;
        //plan_srv.request.start=m_pose;
        //plan_srv.request.goal=m_goalpose;
        //plan_srv.request.tolerance=0.5;
        //double total_distance_to_goal=sqrt(pow((m_pose.pose.position.x-m_goalpose.pose.position.x),2)+pow((m_pose.pose.position.y-m_goalpose.pose.position.y),2));
        //int num_subgoals=NUM_SUBS;
        //if(total_distance_to_goal>10.0)
            //num_subgoals=NUM_SUBS+int((total_distance_to_goal-10.0)/2.0);
        //else if(total_distance_to_goal<5.0)
            //num_subgoals=1;

        //if(planner_srv_client.call(plan_srv))
        //{
            //subgoals.poses.clear();
            //int path_length =plan_srv.response.plan.poses.size();
            //int path_const = floor(path_length/num_subgoals);
            //subgoals.poses.clear();
            //subgoals.poses.push_back(plan_srv.response.plan.poses[0].pose);
            //for(int i(0);i<(num_subgoals-1);i++)
            //{
                //int pose_idx=(i+1)*path_const;
                //subgoals.poses.push_back(plan_srv.response.plan.poses[pose_idx].pose);
            //}
            //subgoals.poses.push_back(plan_srv.response.plan.poses[path_length-1].pose);
            //goal_idx=0;
            //ROS_INFO("Replanning Succeeded! ---Subgoal--Size: %d", subgoals.poses.size());

        //}
        //else{
            //ROS_INFO("replanning failed");
        //}


    //}
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
      //ROS_ASSERT(ref_frame.getType() == XmlRpc::XmlRpcValue::TypeStruct);
      //print loaded parameters
      //std::map< std::string, std::vector<double> >::iterator map_iter=goal_maps.begin();
      //for(map_iter; map_iter!=goal_maps.end();map_iter++)
      //{
          //std::cout<<map_iter->first <<" : " <<map_iter->second[0]<< ", "<<map_iter->second[1]<<std::endl;
      //}

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
  //ros::ServiceClient planner_srv_client;
  ros::ServiceClient planner_srv_client_single;
  std::string action_name_;
  int data_count_, goal_;
  float sum_, sum_sq_;
  search_service::MultiSearchFeedback feedback_;
  search_service::MultiSearchResult result_;

  ros::Subscriber agent1_pose_sub;
  ros::Subscriber agent2_pose_sub;
  ros::Subscriber agent1_localmap_sub;
  ros::Subscriber agent2_localmap_sub;
  ros::Subscriber frontiers_poses_sub;
  ros::Subscriber unknown_poses_sub;

  ros::Subscriber global_map_sub;
  ros::Publisher agent1_path_pub;
  ros::Publisher agent2_path_pub;
  ros::Publisher search_map_pub;
  ros::Publisher search_goal_pub;
  ros::Publisher search_goal2_pub;
  ros::Publisher agent1_move_cancel_pub;
  ros::Publisher search_entropy_pub;
  int direction_z;

  geometry_msgs::PoseStamped agent1_pose;
  geometry_msgs::PoseStamped agent2_pose;

  geometry_msgs::PoseStamped searchgoal_agent1;
  geometry_msgs::PoseStamped searchgoal_agent2;
  geometry_msgs::PoseStamped startpose_agent1;
  geometry_msgs::PoseStamped startpose_agent2;
  geometry_msgs::PoseStamped last_goal_agent1;
  geometry_msgs::PoseStamped last_goal_agent2;
  geometry_msgs::PoseArray agent1_history;
  geometry_msgs::PoseArray agent2_history;
  geometry_msgs::PoseArray frontier_poses;
  geometry_msgs::PoseArray unknown_poses;

  //location map--villa_navigation
  geometry_msgs::PoseArray subgoals;
  nav_msgs::OccupancyGrid agent1_local_map;
  nav_msgs::OccupancyGrid agent2_local_map;
  nav_msgs::OccupancyGrid scaled_global_map;
  nav_msgs::OccupancyGrid global_map;
  nav_msgs::OccupancyGrid search_map;
  nav_msgs::OccupancyGrid info_map;
  tf::TransformListener   listener;


  std::vector<double> global_pose_a1;
  std::vector<double> global_pose_hsr;

  bool IsGoal;
  bool IsActive;
  double srv_time; 
  double tolerance;
  bool agent1_local_map_updated;
  bool agent2_local_map_updated;
  bool global_map_updated;
  bool global_pose_a1_updated;
  bool global_pose_hsr_updated;
  bool called_once;
  bool neargoal_agent1;
  bool neargoal_agent2;
  double search_entropy;
  double total_entropy;
  double occ_entropy;
  int no_count;
  int horizon;

  double weight_entropy; 
  double weight_travel; 


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
  //StateLatticePlannerROS planner;
  //manager.pslp_planner= &planner;
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
