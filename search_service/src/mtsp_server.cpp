#include "ros/ros.h"
#include <math.h>
#include "spline.h"
#include "cluster.h"
#include "map_util.h"
#include <map>
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PolygonStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/Twist.h"
#include "geometry_tools.h"
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/GetPlan.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sstream>
#include <signal.h>
#include <boost/thread/thread.hpp>
#include <std_msgs/Bool.h>
#include <search_service/MultiSearchAction.h>
#include <search_service/MultiSearchResult.h>
#include <search_service/SetSearchRegionAction.h>
#include <search_service/SetSearchRegionGoal.h>
#include <search_service/SetSearchRegionResult.h>
#include <search_service/MultiSearchResult.h>
#include <search_service/SearchAction.h>
#include <visual_perception/UnknownSearchAction.h>
#include <search_service/TSPSolveAction.h>
#include <search_service/TSPSolveResult.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
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


class MultiSearchManager
{
public:
    
  MultiSearchManager(std::string name): 
  as_(nh_, name, boost::bind(&MultiSearchManager::executeCB, this,_1), false),
  as_region(nh_, "set_search_region", boost::bind(&MultiSearchManager::executeCB_SR, this,_1), false),
  ac_("getunknowns", true),
  ac_tsp("tsp_solve_action", true),
  action_name_(name),
  IsActive(false),
  IsCalled(false),
  srv_time(0.0),
  tolerance(0.5),
  search_entropy(1.0),
  clustered(false),
  pathUpdated(false),
  smoothpath(false),
  called_once(false),
  weight_entropy(0.25),
  weight_travel(1.5),
  m_params(NULL)
  {
     nh_.param("MAX_X", MAX_X, {16.0});
     nh_.param("MIN_X", MIN_X, {-15.0});
     nh_.param("MAX_Y", MAX_Y, {20.0});
     nh_.param("MIN_Y", MIN_Y, {-30.0});
     nh_.param("AGENT1_POSE_TOPIC", agent1_pose_topic, {"global_pose_a1_121"});
     nh_.param("AGENT2_POSE_TOPIC", agent2_pose_topic, {"spot2/odom"});
     nh_.param("AGENT3_POSE_TOPIC", agent3_pose_topic, {"spot3/odom"});
     nh_.param("AGENT1_MAP_TOPIC", agent1_map_topic, {"costmap_node/costmap/costmap"});
     nh_.param("AGENT2_MAP_TOPIC", agent2_map_topic, {"spot2/costmap"});
     nh_.param("AGENT3_MAP_TOPIC", agent3_map_topic, {"spot3/costmap"});
     nh_.param("NUM_AGETNT", NUMAGENTS, {3});

     nh_.getParam("AGENT1_POSE_TOPIC", agent1_pose_topic);
     nh_.getParam("AGENT2_POSE_TOPIC", agent2_pose_topic);
     nh_.getParam("AGENT3_POSE_TOPIC", agent3_pose_topic);
     nh_.getParam("NUM_AGENT", NUMAGENTS);
     nh_.getParam("AGENT1_MAP_TOPIC", agent1_map_topic);
     nh_.getParam("AGENT2_MAP_TOPIC", agent2_map_topic);
     nh_.getParam("AGENT3_MAP_TOPIC", agent3_map_topic);
     nh_.getParam("MAX_X", MAX_X);
     nh_.getParam("MIN_X", MIN_X);
     nh_.getParam("MAX_Y", MAX_Y);
     nh_.getParam("MIN_Y", MIN_Y);

     ROS_INFO("agent1_pose_topic: %s",agent1_pose_topic.c_str());
     ROS_INFO("agent2_pose_topic: %s",agent2_pose_topic.c_str());
     ROS_INFO("agent3_pose_topic: %s",agent3_pose_topic.c_str());
     ROS_INFO("agent1_map_topic: %s",agent1_map_topic.c_str());
     ROS_INFO("agent2_map_topic: %s",agent2_map_topic.c_str());
     ROS_INFO("agent3_map_topic: %s",agent3_map_topic.c_str());
     ROS_INFO("num_agent: %d",NUMAGENTS);
     ROS_INFO("max_x: %.2lf",MAX_X);
     ROS_INFO("max_y: %.2lf",MAX_Y);
     ROS_INFO("min_x: %.2lf",MIN_X);
     ROS_INFO("min_y: %.2lf",MIN_Y);

     m_params= new Map_params(MAX_X, MAX_Y, MIN_X, MIN_Y);
     search_map.info.resolution = m_params->xyreso;
     search_map.info.width= m_params->xw;
     search_map.info.height= m_params->yw;
     search_map.info.origin.position.x= m_params->xmin;
     search_map.info.origin.position.y= m_params->ymin;
     search_map.data.resize((search_map.info.width * search_map.info.height), 0.0);  //unknown ==> 0 ==> we calculate number of 0 in search map to calculate IG

     //initially the entropy can be computed as #of cells in serchmap * uncertainty
     total_entropy=search_map.data.size()*CELL_MAX_ENTROPY;

     agent_poses.poses.resize(NUMAGENTS);
     //self.tolerance=0.5

     //publishers
     search_map_pub=nh_.advertise<nav_msgs::OccupancyGrid>("/search_map",50,true);
     agent1_move_cancel_pub=nh_.advertise<actionlib_msgs::GoalID>("/localnavi_server/cancel",50,true);
     search_entropy_pub=nh_.advertise<std_msgs::Float32>("/search_entropy",50,true);
     visual_marker_pub= nh_.advertise<visualization_msgs::MarkerArray>("clusters", 5);
     polygon_pub = nh_.advertise<geometry_msgs::PolygonStamped>("current_polygon", 10, true);

     agent1_path_pub = nh_.advertise<nav_msgs::Path>("agent1_path", 50, true);
     agent2_path_pub = nh_.advertise<nav_msgs::Path>("agent2_path", 50, true);
     agent3_path_pub = nh_.advertise<nav_msgs::Path>("agent3_path", 50, true);


     //subscribers
     global_map_sub = nh_.subscribe<nav_msgs::OccupancyGrid>("/scaled_static_map", 1, &MultiSearchManager::global_map_callback, this);
     agent1_localmap_sub= nh_.subscribe<nav_msgs::OccupancyGrid>(agent1_map_topic,10,&MultiSearchManager::agent1_localmap_callback,this);
     agent2_localmap_sub= nh_.subscribe<nav_msgs::OccupancyGrid>(agent2_map_topic, 10,&MultiSearchManager::agent2_localmap_callback,this);
     agent3_localmap_sub= nh_.subscribe<nav_msgs::OccupancyGrid>(agent3_map_topic, 10,&MultiSearchManager::agent3_localmap_callback,this);

     agent1_pose_sub=nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>(agent1_pose_topic,10,&MultiSearchManager::agent1_pose_callback,this);
     agent2_pose_sub=nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>(agent2_pose_topic,10,&MultiSearchManager::agent2_pose_callback,this);
     agent3_pose_sub=nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>(agent3_pose_topic,10,&MultiSearchManager::agent3_pose_callback,this);

     planner_srv_client= nh_.serviceClient<nav_msgs::GetPlan>("/spot/move_base/GlobalPlanner//make_plan");


     //receive scaled_global_map
     nav_msgs::OccupancyGrid::ConstPtr shared_map;
     shared_map= ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/scaled_static_map",nh_);
     if(shared_map!= NULL){
         scaled_global_map= *shared_map;
         crop_globalmap(*shared_map, geometry_msgs::Polygon());
     }
     total_entropy-=occ_entropy;
     ROS_INFO("total_initial_entropy: %.2f",  total_entropy);

     agent1_pose.resize(3,0.0);
     agent2_pose.resize(3,0.0);
     agent3_pose.resize(3,0.0);
     as_region.start();
     ROS_INFO("Set-search region_started");
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
    IsActive=false;
    ROS_INFO("preempted called");
    // set the action state to preempted
    as_.setPreempted();
  }

  void executeCB_SR(const search_service::SetSearchRegionGoalConstPtr &goal)
  {

     updateBoundsFromPolygon(goal->boundary.polygon,MIN_X,MIN_Y,MAX_X, MAX_Y);
     //calculate_ max,max_y, min_x, min_y
     m_params= new Map_params(MAX_X, MAX_Y, MIN_X, MIN_Y);
     search_map.info.resolution = m_params->xyreso;
     search_map.info.width= m_params->xw;
     search_map.info.height= m_params->yw;
     search_map.info.origin.position.x= m_params->xmin;
     search_map.info.origin.position.y= m_params->ymin;
     search_map.data.resize((search_map.info.width * search_map.info.height), 0.0);  //unknown ==> 0 ==> we calculate number of 0 in search map to calculate IG
     ROS_INFO("width: %.2lf, height: %.2lf ", m_params->xw, m_params->yw);

     nav_msgs::OccupancyGrid::ConstPtr shared_map;
     shared_map= ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/scaled_static_map",nh_);
     if(shared_map!= NULL){
         scaled_global_map= *shared_map;
         crop_globalmap(*shared_map, goal->boundary.polygon);
     }
     total_entropy-=occ_entropy;
     ROS_INFO("total_initial_entropy: %.2f",  total_entropy);
     

    polygon_.polygon.points.clear();
    polygon_.header.frame_id="map";
    geometry_msgs::Point32 tmp_pnt;
    for (const auto & point : goal->boundary.polygon.points)
      {
         tmp_pnt.x = point.x;
         tmp_pnt.y = point.y;
         tmp_pnt.z = point.z;
         polygon_.polygon.points.push_back(tmp_pnt);
      }
      
    polygon_pub.publish(polygon_);

     search_map_pub.publish(search_map);
     result_region.success=true;
     as_region.setSucceeded(result_region);

  }


  void executeCB(const search_service::MultiSearchGoalConstPtr &goal)
  {
      
     ROS_INFO("Resizing Map with Search_Polygon");


     ROS_INFO("MultiSearchManager action is called!!");
     ac_.waitForServer(ros::Duration(5.0));
     ROS_INFO("WaypointGeneration Server is running....!!");
     ac_tsp.waitForServer(ros::Duration(5.0));
     ROS_INFO("TSP_solve Server is running....!!");
     bool success = true;

    double sleep_rate= 2.0;
    ros::Rate r(sleep_rate);

     //search_entropy = get_searchmap_entropy()/ total_entropy;
     if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        success = false;
        return;
        // break;
      }


     //Call Search Service
    while(ros::ok() && !as_.isPreemptRequested())
    {
     search_entropy = get_searchmap_entropy()/ total_entropy;
     ROS_INFO("current_entropy: %.2lf", search_entropy);

     
     //Call Waypoint Generateion (UnknwonSearch action)
     visual_perception::UnknownSearchGoal unknwongoal;
     ac_.sendGoal(unknwongoal);
     bool finished_before_timeout = ac_.waitForResult(ros::Duration(20.0)); //Wait for 20s
     if(finished_before_timeout)
     {
         ROS_INFO("waypoints are generated");
         auto res_ = ac_.getResult();
         waypoints= res_->waypoints;
         ROS_INFO("clustering started");
         //Waypoints = > clustering 
         clustering(NUMAGENTS);
         //Call TSP Solver
         search_service::TSPSolveGoal tspgoal;
         tspgoal.num_agent = NUMAGENTS;
         tspgoal.clusters = clustered_poses;
         tspgoal.poses = agent_poses;
         //call TSP Solve Action
         ac_tsp.sendGoal(tspgoal);
         ROS_INFO("TSP Solve started");
         bool finished_before_timeout_tsp = ac_tsp.waitForResult(ros::Duration(30.0));
         if(finished_before_timeout_tsp )
         {
              ROS_INFO("TSP solution obtained. Path will be published");
              auto tsp_res=ac_tsp.getResult();
              result_.paths= tsp_res->paths;
              //save_paths for each agents
              agent_paths = tsp_res->paths;
              result_.cur_entropy = search_entropy;
              as_.setSucceeded(result_);
              pathUpdated=true;
              publish_paths(NUMAGENTS);


              return;
         }
         else{
                ROS_INFO("can't obtain tsp path");
         }
     }
     else{
             ROS_INFO("Waypoint Generateion is not completed");
         }

     ros::spinOnce();
     r.sleep();

    }

  }

  void updateBoundsFromPolygon(const geometry_msgs::Polygon &polygon, double& min_x, double& min_y, 
          double& max_x, double& max_y){

      min_x = std::numeric_limits<double>::infinity();
      min_y = std::numeric_limits<double>::infinity();
      max_x = -std::numeric_limits<double>::infinity();
      max_y = -std::numeric_limits<double>::infinity();

      for (const auto & point : polygon.points)
      {
        min_x = std::min(min_x, static_cast<double>(point.x));
        min_y = std::min(min_y, static_cast<double>(point.y));
        max_x = std::max(max_x, static_cast<double>(point.x));
        max_y = std::max(max_y, static_cast<double>(point.y));
      }
      min_x = floor(min_x);
      min_y = floor(min_y);
      max_x = floor(max_x);
      max_y = floor(max_y);

      ROS_INFO("min_x: %.2lf, max_x: %.2lf ,min_y: %.2lf, max_y: %.2lf", min_x, max_x, min_y,max_y);
    }

  void unknown_poses_callback(const geometry_msgs::PoseArray::ConstPtr& msg)
  {
      //ROS_INFO("waypoints_pose_callback");
      waypoints=*msg;
      clustering(NUMAGENTS);
  }

  void agent2_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
  {
      ROS_INFO("agent2_callback");
      agent2_gpose.pose=msg->pose.pose;
      agent2_pose[0]=msg->pose.pose.position.x;
      agent2_pose[1]=msg->pose.pose.position.y;
      agent2_pose[2]=tf2::getYaw(msg->pose.pose.orientation);
      agent2_pose_updated=true;

      agent_poses.poses[1]=msg->pose.pose;

  }

  void agent3_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
  {
      //* global_pose w.r.t "map" frame
      //ROS_INFO("agent3_callback");
      agent3_gpose.pose=msg->pose.pose;
      agent3_pose[0]=msg->pose.pose.position.x;
      agent3_pose[1]=msg->pose.pose.position.y;
      agent3_pose[2]=tf2::getYaw(msg->pose.pose.orientation);
      agent3_pose_updated=true;

      agent_poses.poses[2]=msg->pose.pose;
  }


  void agent1_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
  {
      //* global_pose w.r.t "map" frame
      agent1_gpose.pose=msg->pose.pose;
      agent1_pose[0]=msg->pose.pose.position.x;
      agent1_pose[1]=msg->pose.pose.position.y;
      agent1_pose[2]=tf2::getYaw(msg->pose.pose.orientation);
      global_pose_a1_updated=true;

      agent_poses.poses[0]=msg->pose.pose;

      search_map_pub.publish(search_map);

     if(pathUpdated)
         publish_paths(NUMAGENTS);
  }
  

  void agent1_localmap_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    agent1_local_map=*msg;
    agent1_local_map_updated = true;
    update_occ_grid_map(msg);

    //publish search map
    std_msgs::Float32 entropy_msg;
    entropy_msg.data=search_entropy;
    search_entropy_pub.publish(entropy_msg);
    //check_obstacle
    if(pathUpdated)
        publish_paths(NUMAGENTS);
}

  void agent2_localmap_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    agent2_local_map=*msg;
    agent2_local_map_updated = true;

    update_occ_grid_map(msg);
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

void crop_globalmap(const nav_msgs::OccupancyGrid global_map, const geometry_msgs::Polygon _polygon)
{
    //"Crop global_map-->fill search map with static obstacle "
    //searchmap info / globalmap_info
    ROS_INFO("Crop_global_map");
    int count=0;

    double px, py =0.0;
    int global_idx,search_idx =0;

    //iteration for search map to apply globalmap info 
    geometry_msgs::Point tmp_pnt;
    for(int j(0); j< search_map.info.height;j++)
        for(int i(0); i< search_map.info.width;i++)
        {
            px = search_map.info.origin.position.x+(i)*search_map.info.resolution;
            py = search_map.info.origin.position.y+(j)*search_map.info.resolution;
            global_idx = Coord2CellNum(px,py, global_map);  
            search_idx = Coord2CellNum(px,py, search_map);         // get search cell idx 
            //if static_obstacle data in global_map
            if(global_map.data[global_idx]==100)
            {
                    ROS_INFO("px:%.2lf ,py: %.2lf", px,py);
                    search_map.data[search_idx]= int(L_SOCC);
                    count++;
            }
            else{
                    search_map.data[search_idx]= 0.0;
            }

            tmp_pnt.x=px;
            tmp_pnt.y=py;
            if(!pointInPolygon(tmp_pnt, _polygon))
                search_map.data[search_idx]=int(L_SOCC);


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

        double  temp_x  = _x-inputmap_.info.origin.position.x;
        double  temp_y = _y-inputmap_.info.origin.position.y;

        target_Coord[0]= (int) floor(temp_x/inputmap_.info.resolution);
        target_Coord[1]= (int) floor(temp_y/inputmap_.info.resolution);

        int index= target_Coord[0]+inputmap_.info.width*target_Coord[1];
        return index;
        //ROS_INFO("targertcoord: x: %d, y: %d", target_Coord[0], target_Coord[1]);
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

 }
void publish_paths(int n_agent)
{
    ROS_INFO("paths are being published");

    if(smoothpath)
    {
        ROS_INFO("smooth");
        agent1_path_pub.publish(smooth_paths[0]);
        agent2_path_pub.publish(smooth_paths[1]);
    
    }
    else{
    
        ROS_INFO("nonsmooth");
        agent1_path_pub.publish(agent_paths[0]);
        agent2_path_pub.publish(agent_paths[1]);
    
    }
    //else{
    
        //ROS_INFO("3");
        //agent1_path_pub.publish(agent_paths[0]);
        //agent2_path_pub.publish(agent_paths[1]);
        //agent3_path_pub.publish(agent_paths[2]);
    //}
    

    nav_msgs::GetPlan srv_;
    if(!smoothpath)
    {
        smooth_paths.clear();
        for(int n(0); n<NUMAGENTS;n++)
        {
            nav_msgs::Path smooth_path;
            ROS_INFO("agent!!");
            srv_.request.start.pose = agent_poses.poses[n];
            srv_.request.start.header.frame_id = "map";
            srv_.request.start.header.stamp= ros::Time::now();
            std::vector<int> real_dst;
            for(size_t i(0);i<agent_paths[n].poses.size();i++)
            {
                //srv_.request.start = agent1_gpose;
                srv_.request.goal= agent_paths[n].poses[i];
                srv_.request.goal.header.frame_id= "map";
                if(planner_srv_client.call(srv_))
                {
                    real_dst.push_back(srv_.response.plan.poses.size());
                }
                else{

                    ROS_INFO("failed");
                    
                }

            }
            auto minIt = std::min_element(real_dst.begin(), real_dst.end());
            double minElement = *minIt;
            int min_idx = minIt -real_dst.begin();
     

            srv_.request.start.pose = agent_poses.poses[n];
            for(size_t i(min_idx);i<agent_paths[n].poses.size();i++)
            {
                int real_idx=i;
                if(real_idx>agent_paths[n].poses.size())
                    real_idx-=agent_paths[n].poses.size();
                //srv_.request.start = agent1_gpose;
                srv_.request.goal= agent_paths[n].poses[real_idx];
                if(planner_srv_client.call(srv_))
               {
                  ROS_INFO("get MultiPlan service finished!!");
                  nav_msgs::Path path_agent1= srv_.response.plan;
                  for(size_t j(0);j<path_agent1.poses.size();j++)
                      smooth_path.poses.push_back(path_agent1.poses[j]);
                  srv_.request.start = agent_paths[n].poses[real_idx];
               }
                else{
                
                    ROS_INFO("failed");
                
                }
                

            }
            smooth_path.header.frame_id="map";
            smooth_path.header.stamp=ros::Time::now();
            smooth_paths.push_back(smooth_path);

            //if(n==NUMAGENTS-1)
                smoothpath=true;
        }
    }
    //smooth_path.header.frame_id="map";
    //agent3_path_pub.publish(smooth_path);

    return;
}

/*This function assigns waypoints to each agents
//Input:
//Output:
*/


void clustering(int n_agent )
{
    ROS_INFO("Clustering for %d agents!!", n_agent);
    agent_xs.resize(n_agent);
    agent_ys.resize(n_agent);
    if(!clustered)
    {
        int data_size = waypoints.poses.size();
        std::vector<double> xs;
        std::vector<double> ys;

        for(size_t i(0);i<data_size;i++)
        {
            double tx = waypoints.poses[i].position.x;
            double ty = waypoints.poses[i].position.y;
            xs.push_back(tx);
            ys.push_back(ty);
        }

        std::vector<double> weights;
        weights.push_back(1.0);
        
        std::vector<geometry_msgs::Pose> Posevec;
        Posevec.push_back(agent1_gpose.pose);
        if(n_agent>2)
        {
            //two agents
            //geometry_msgs::Pose tmp;
            Posevec.push_back(agent2_gpose.pose);
            weights.push_back(1.0);
            Posevec.push_back(agent3_gpose.pose);
            weights.push_back(1.0);
        }
        else if(n_agent>1)
        {
            Posevec.push_back(agent2_gpose.pose);
            weights.push_back(1.0);
        }

       HCluster test_cluster(n_agent, xs,ys, Posevec, weights);
       for(size_t i(0);i<n_agent;i++)
        {
            test_cluster.get_labeled_x_y(i, agent_xs[i],agent_ys[i]);
        }

       //Save clustered_poses to geometry_msgs/PoseArray
       clustered_poses.clear();
       for(size_t i(0);i<n_agent;i++)
       {
           geometry_msgs::PoseArray tmp_posearray;
           for(int k(0);k<agent_xs[i].size();k++)
           {
               geometry_msgs::Pose tmp_pos;
               tmp_pos.position.x=agent_xs[i][k];
               tmp_pos.position.y=agent_ys[i][k];
               tmp_posearray.poses.push_back(tmp_pos);
           }
           clustered_poses.push_back(tmp_posearray);
       }

        clustered=true;

    }
    else{

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
    for(size_t i(0);i<n_agent;i++)
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
  actionlib::SimpleActionServer<search_service::SetSearchRegionAction> as_region;
  actionlib::SimpleActionClient<visual_perception::UnknownSearchAction> ac_;
  actionlib::SimpleActionClient<search_service::TSPSolveAction> ac_tsp;

  ros::ServiceClient planner_srv_client;
  //ros::ServiceClient planner_srv_client_single;
  std::string action_name_;
  int data_count_, goal_;
  float sum_, sum_sq_;
  search_service::MultiSearchFeedback feedback_;
  search_service::MultiSearchResult result_;
  search_service::SetSearchRegionResult result_region;


  //Topic names
  std::string map_frame;
  std::string base_frame;
  std::string agent1_pose_topic;
  std::string agent2_pose_topic;
  std::string agent3_pose_topic;
  std::string agent1_map_topic;
  std::string agent2_map_topic;
  std::string agent3_map_topic;

  double MAX_X;
  double MAX_Y;
  double MIN_X;
  double MIN_Y;
  int NUMAGENTS;

  //Subscribers
  ros::Subscriber agent1_pose_sub;
  ros::Subscriber agent2_pose_sub;
  ros::Subscriber agent3_pose_sub;
  ros::Subscriber agent1_localmap_sub;
  ros::Subscriber agent2_localmap_sub;
  ros::Subscriber agent3_localmap_sub;
  ros::Subscriber unknown_poses_sub;
  ros::Subscriber global_map_sub;

  //Publishers
  ros::Publisher agent1_path_pub;
  ros::Publisher agent2_path_pub;
  ros::Publisher agent3_path_pub;
  ros::Publisher search_map_pub;
  ros::Publisher agent1_move_cancel_pub;
  ros::Publisher search_entropy_pub;
  ros::Publisher visual_marker_pub;
  ros::Publisher polygon_pub;
  int direction_z;

  geometry_msgs::PoseStamped agent1_gpose;
  geometry_msgs::PoseStamped agent2_gpose;
  geometry_msgs::PoseStamped agent3_gpose;
  geometry_msgs::PoseArray waypoints;
  geometry_msgs::PoseArray agent_poses;
  std::vector<geometry_msgs::PoseArray> clustered_poses;

  //location map--villa_navigation
  geometry_msgs::PoseArray subgoals;
  nav_msgs::OccupancyGrid agent1_local_map;
  nav_msgs::OccupancyGrid agent2_local_map;
  nav_msgs::OccupancyGrid agent3_local_map;
  nav_msgs::OccupancyGrid scaled_global_map;
  nav_msgs::OccupancyGrid global_map;
  nav_msgs::OccupancyGrid search_map;
  tf::TransformListener   listener;

  std::vector<double> agent1_pose;
  std::vector<double> agent2_pose;
  std::vector<double> agent3_pose;

  std::vector<nav_msgs::Path> agent_paths;
  geometry_msgs::PolygonStamped polygon_;

  bool IsActive;
  bool IsCalled;
  bool pathUpdated;
  bool smoothpath;

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
  double search_entropy;
  double total_entropy;
  double occ_entropy;
  bool clustered;

  double weight_entropy; 
  double weight_travel; 

  std::vector<std::vector<double> >agent_xs;
  std::vector<std::vector<double> >agent_ys;
  std::vector<nav_msgs::Path> smooth_paths;


  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener* tf2_listener; 
  geometry_msgs::TransformStamped map_en_to_map;
  geometry_msgs::TransformStamped map_to_base;
  Map_params* m_params;
  std::map< std::string, std::vector<double> > goal_maps;
  std::map<int, std::vector<precastDB>> precast_map;
public:

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "multisearch_action");
  ROS_INFO("node_info");
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
