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
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_real_distribution.hpp>
#include <std_msgs/Bool.h>
#include <search_service/SetSearchRegionAction.h>
#include <search_service/SetSearchRegionGoal.h>
#include <search_service/SetSearchRegionResult.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib_msgs/GoalID.h>
#include <XmlRpcValue.h>
#include <yaml-cpp/yaml.h>
#include <fstream>

#define MAP_RES 0.05
#define NUM_SUBS 3
#define Dist_History 2.0
#define Max_Dist 30.0
#define Dist_GOAL 4.0
#define RAND_RANGE 1.0
#define L_SOCC 99.//4.595 // np.log(0.99/0.01)
#define L_DOCC 2.19 // np.log(0.9/0.1)
#define L_FREE -10000// np.log(0.01/0.99)
#define CELL_MAX_ENTROPY 0.693147
#define GOAL_THRESHOLD 2.0


using namespace std;

double atan_zero_to_twopi(double y, double x){

    double angle = atan2(y, x);
    if(angle < 0.0)
        angle += 2*M_PI;

    return angle;
}

double calc_dist(geometry_msgs::Pose pos1, geometry_msgs::Pose pos2)
{

    double dist_=pow(pos1.position.x-pos2.position.x, 2);
    dist_+=pow(pos1.position.y-pos2.position.y, 2);

    return dist_;

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


class MARLMapManager
{
public:
    
  MARLMapManager(std::string name): 
  as_region(nh_, "set_search_region", boost::bind(&MARLMapManager::executeCB_SR, this,_1), false),
  action_name_(name),
  IsActive(false),
  IsCalled(false),
  srv_time(0.0),
  search_entropy(1.0),
  called_once(false),
  weight_entropy(0.25),
  weight_travel(1.5),
  m_params(NULL)
  {
     nh_.param("MAX_X", MAX_X, {20.0});
     nh_.param("MIN_X", MIN_X, {-1.0});
     nh_.param("MAX_Y", MAX_Y, {12.0});
     nh_.param("MIN_Y", MIN_Y, {0.0});
     nh_.param("AGENT1_POSE_TOPIC", agent1_pose_topic, {"/a1_950/goal_pose"});
     nh_.param("AGENT2_POSE_TOPIC", agent2_pose_topic, {"/hsr/global_pose"});
     nh_.param("AGENT1_MAP_TOPIC", agent1_map_topic, {"/a1_950/fov_map"});
     nh_.param("AGENT2_MAP_TOPIC", agent2_map_topic, {"/hsr/fov_map"});
     nh_.param("NUM_AGENT", NUMAGENTS, {2});
     nh_.param("BOOL_INITPOSES", BOOL_INITPOSES, {1});

     nh_.getParam("AGENT1_POSE_TOPIC", agent1_pose_topic);
     nh_.getParam("AGENT2_POSE_TOPIC", agent2_pose_topic);
     nh_.getParam("NUM_AGENT", NUMAGENTS);
     nh_.getParam("BOOL_INITPOSES", BOOL_INITPOSES);
     nh_.getParam("AGENT1_MAP_TOPIC", agent1_map_topic);
     nh_.getParam("AGENT2_MAP_TOPIC", agent2_map_topic);
     nh_.getParam("MAX_X", MAX_X);
     nh_.getParam("MIN_X", MIN_X);
     nh_.getParam("MAX_Y", MAX_Y);
     nh_.getParam("MIN_Y", MIN_Y);

     ROS_INFO("agent1_pose_topic: %s",agent1_pose_topic.c_str());
     ROS_INFO("agent2_pose_topic: %s",agent2_pose_topic.c_str());
     ROS_INFO("agent1_map_topic: %s",agent1_map_topic.c_str());
     ROS_INFO("agent2_map_topic: %s",agent2_map_topic.c_str());
     ROS_INFO("num_agent: %d",NUMAGENTS);
     ROS_INFO("init_pose: %d",BOOL_INITPOSES);
     ROS_INFO("max_x: %.2lf",MAX_X);
     ROS_INFO("max_y: %.2lf",MAX_Y);
     ROS_INFO("min_x: %.2lf",MIN_X);
     ROS_INFO("min_y: %.2lf",MIN_Y);

     m_params= new Map_params(MAX_X, MAX_Y, MIN_X, MIN_Y);
     search_map.info.resolution = m_params->xyreso;
     search_map.info.width = m_params->xw;
     search_map.info.height = m_params->yw;
     search_map.info.origin.position.x = m_params->xmin;
     search_map.info.origin.position.y = m_params->ymin;
     search_map.data.resize((search_map.info.width * search_map.info.height), 0.0);  //unknown ==> 0 ==> we calculate number of 0 in search map to calculate IG

     // initially the entropy can be computed as #of cells in serchmap * uncertainty
     // set size for member variables
     agent_poses.poses.resize(NUMAGENTS);

     // publishers
     search_map_pub=nh_.advertise<nav_msgs::OccupancyGrid>("/search_map",50,true);
     search_entropy_pub=nh_.advertise<std_msgs::Float32>("/search_entropy",50,true);
     polygon_pub = nh_.advertise<geometry_msgs::PolygonStamped>("/current_polygon", 10, true);

    // subscribers
     global_map_sub = nh_.subscribe<nav_msgs::OccupancyGrid>("/map", 1, &MARLMapManager::global_map_callback, this);
     agent1_localmap_sub= nh_.subscribe<nav_msgs::OccupancyGrid>(agent1_map_topic,10,&MARLMapManager::agent1_localmap_callback,this);
     agent2_localmap_sub= nh_.subscribe<nav_msgs::OccupancyGrid>(agent2_map_topic, 10,&MARLMapManager::agent2_localmap_callback,this);
     agent1_pose_sub=nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>(agent1_pose_topic,10,&MARLMapManager::agent1_pose_callback,this);
     agent2_pose_sub=nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>(agent2_pose_topic,10,&MARLMapManager::agent2_pose_callback,this);

     // receive scaled_global_map (static map)
     nav_msgs::OccupancyGrid::ConstPtr shared_map;
     shared_map= ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/map",nh_);
     if(shared_map!= NULL){
         scaled_global_map= *shared_map;
         crop_globalmap(*shared_map, geometry_msgs::Polygon());
     }


     as_region.start();
     ROS_INFO("Set search region started");
  }

  ~MARLMapManager(void)
  {
  }

  /* Set_Search_region action callback function */
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
     //total_entropy-=occ_entropy;
     //ROS_INFO("total_initial_entropy: %.2f",  total_entropy);
     
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
     total_entropy=get_searchmap_entropy();
     search_map_pub.publish(search_map);
     result_region.success=true;
     as_region.setSucceeded(result_region);

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

  void agent2_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
  {
      //ROS_INFO("agent2_callback");
      agent2_gpose.pose=msg->pose.pose;
      agent2_pose_updated=true;
      agent_poses.poses[1]=msg->pose.pose;

  }

  void agent1_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
  {
      //* global_pose w.r.t "map" frame
      agent1_gpose.pose=msg->pose.pose;
      global_pose_a1_updated=true;

      agent_poses.poses[0]=msg->pose.pose;
      search_map_pub.publish(search_map);

  }

  void agent1_localmap_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    agent1_local_map=*msg;
    agent1_local_map_updated = true;
    //  std::cout << "agent1 costmap grid received" << std::endl;
    update_occ_grid_map(msg);
    //publish search map
    std_msgs::Float32 entropy_msg;
    entropy_msg.data=search_entropy;
    search_entropy_pub.publish(entropy_msg);
}

  void agent2_localmap_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    agent2_local_map=*msg;
    agent2_local_map_updated = true;

    update_occ_grid_map(msg);
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
    double px, py = 0.0;
    int map_idx,search_idx = 0;

    for(int j(0); j< msg->info.height;j++)
        for(int i(0); i< msg->info.width;i++)
        {
            map_idx = j*msg->info.width+i;
            px = msg->info.origin.position.x+(i+0.05)*msg->info.resolution;
            py = msg->info.origin.position.y+(j+0.05)*msg->info.resolution;
            search_idx = Coord2CellNum(px,py, search_map);         // get search cell idx 
            //Update searchmap according to local measurement
            // if local_map is known (occ or free) and global_map is not occupied by static obstacle 
            if(search_idx<search_map.data.size())
            {
                if(msg->data[map_idx]!=-1 and search_map.data[search_idx]!=int(L_SOCC)) 
                    //if local measurment is not unknown and not filled with static obstacle//
                    if(msg->data[map_idx]!= -1)
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
    //"Crop global_map --> fill search map with static obstacle "
    ROS_INFO("Update Entropy Map with Static Obstacles");
    int count = 0;
    double px, py = 0.0;
    int global_idx, search_idx = 0;

    //iteration for search map to apply globalmap info 
    geometry_msgs::Point tmp_pnt;
    for(int j(0); j<search_map.info.height; j++)
        for(int i(0); i<search_map.info.width; i++)
        {
            px = search_map.info.origin.position.x+(i)*search_map.info.resolution;
            py = search_map.info.origin.position.y+(j)*search_map.info.resolution;
            global_idx = Coord2CellNum(px,py, global_map);
            search_idx = Coord2CellNum(px,py, search_map);// get search cell idx
            //if static_obstacle data in global_map
            // if(global_map.data[global_idx])
            //     std::cout << "global data = " << global_map.data[global_idx] << std::endl;
            if(global_map.data[global_idx] != 0)
            {
                    search_map.data[search_idx]= int(L_SOCC);// rg here
                    count++;
            }
            else{
                    search_map.data[search_idx]= 1;
            }

            tmp_pnt.x=px;
            tmp_pnt.y=py;
            if(!pointInPolygon(tmp_pnt, _polygon))
                search_map.data[search_idx]=int(L_SOCC); //rg here L_SOCC
            //Update searchmap according to local measurement
            // if local_map is known (occ or free) and global_map is not occupied by static obstacle 
        }
        occ_entropy=double(count)*CELL_MAX_ENTROPY;  //the amount of entropy reduced
        ROS_INFO("occ_entropy_sum: %.2lf ", occ_entropy);
        // ROS_INFO("crop_map finished");
        return;
}



/* function: cell_index to global pose w.r.t. global map*/
void Idx2Globalpose(int idx, std::vector<double>& global_coord, const nav_msgs::OccupancyGrid& inputmap_)
{
    global_coord.resize(2,0.0);

    int res = (int) (idx/ inputmap_.info.width);
    int div = (int) (idx%inputmap_.info.width);

    global_coord[0]=res*inputmap_.info.resolution+inputmap_.info.origin.position.x;
    global_coord[1]=div*inputmap_.info.resolution+inputmap_.info.origin.position.y;
}


/* function--> (x,y) to  cell_index */
int Coord2CellNum(double _x, double _y, const nav_msgs::OccupancyGrid& inputmap_)
{	
    //ROS_INFO("x: %.2lf, y: %.2lf", _x, _y);
    std::vector<int> target_Coord;
    target_Coord.resize(2,0);

    double  temp_x  = _x-inputmap_.info.origin.position.x;
    double  temp_y = _y-inputmap_.info.origin.position.y;

    target_Coord[0]= (int) floor(temp_x/inputmap_.info.resolution);
    target_Coord[1]= (int) floor(temp_y/inputmap_.info.resolution);

    int index= target_Coord[0]+inputmap_.info.width*target_Coord[1];
    return index;
}



protected:
    
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<search_service::SetSearchRegionAction> as_region;

  std::string action_name_;
  int data_count_, goal_;
  float sum_, sum_sq_;
  search_service::SetSearchRegionResult result_region;


  //Topic names
  std::string map_frame;
  std::string base_frame;

  std::string agent1_pose_topic;
  std::string agent2_pose_topic;

  std::string agent1_map_topic;
  std::string agent2_map_topic;

  double MAX_X;
  double MAX_Y;
  double MIN_X;
  double MIN_Y;
  int NUMAGENTS;
  int BOOL_INITPOSES;

  //Subscribers
  ros::Subscriber agent1_pose_sub;
  ros::Subscriber agent2_pose_sub;

  ros::Subscriber agent1_localmap_sub;
  ros::Subscriber agent2_localmap_sub;

  ros::Subscriber global_map_sub;

  //Publishers
  ros::Publisher search_map_pub;
  ros::Publisher search_entropy_pub;
  ros::Publisher polygon_pub;
  int direction_z;

  std::vector<geometry_msgs::PoseStamped> agents_gpose;
  std::vector<nav_msgs::OccupancyGrid> agents_maps;

  geometry_msgs::PoseStamped agent1_gpose;
  geometry_msgs::PoseStamped agent2_gpose;

  geometry_msgs::PoseArray agent_poses;

  nav_msgs::OccupancyGrid agent1_local_map;
  nav_msgs::OccupancyGrid agent2_local_map;
  nav_msgs::OccupancyGrid scaled_global_map;
  nav_msgs::OccupancyGrid global_map;
  nav_msgs::OccupancyGrid search_map;

  std::vector<nav_msgs::Path> agent_paths;
  geometry_msgs::PolygonStamped polygon_;

  bool IsActive;
  bool IsCalled;

  double srv_time; 
  bool agent1_local_map_updated;
  bool agent2_local_map_updated;
  bool global_map_updated;
  bool global_pose_a1_updated;
  bool agent1_pose_updated;
  bool agent2_pose_updated;
  bool called_once;
  double search_entropy;
  double total_entropy;
  double occ_entropy;

  double weight_entropy; 
  double weight_travel; 

  std::vector<std::vector<double> >agent_xs;
  std::vector<std::vector<double> >agent_ys;

  Map_params* m_params;
  std::map< std::string, std::vector<double> > goal_maps;
  std::map<int, std::vector<precastDB>> precast_map;
public:

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "marl_map_manager");
  ROS_INFO("node_info");
  MARLMapManager manager(ros::this_node::getName());
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
