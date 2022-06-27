#include <string>
#include <boost/thread/mutex.hpp>


// ros stuff
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include "tf/message_filter.h"
#include <tf/transform_datatypes.h>
#include <actionlib/server/simple_action_server.h>
// messages
#include <std_msgs/Int8.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/PointCloud.h>
#include <message_filters/time_sequencer.h>
#include <message_filters/subscriber.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/transform_listener.h"
#include "tf2/utils.h"

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
//#include <pcl_ros/point_cloud.h>
//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>

#include <visual_perception/VisualUnknownSearchAction.h>
#include <visual_perception/VisualUnknownSearchResult.h>
#include <visual_perception/VisualUnknownSearchFeedback.h>

//#include <visualization_msgs/Marker.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <frontier_exploration/Frontier.h>
#include <boost/foreach.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_real_distribution.hpp>
#include <XmlRpcValue.h>
#include <yaml-cpp/yaml.h>

// log files
#include <fstream>
#include <vector>
#include <queue>
#include <math.h>

#define FOVW 36       //field of view width
#define MATH_PI 3.14159265359
#define Target_Dist_person 1.0
#define LASER_Dist_person  2.5

#define P_S_given_H 0.8
#define P_S_given_Hc 0.5
#define P_Sc_given_H 0.01
#define P_Sc_given_Hc 0.99

#define HUMANS_DETECTED 1
#define HUMAN_OCCUPIED 1
#define NO_HUMANS_DETECTED 0
#define PROB_THRESH 0.1

#define RAND_RANGE 10.0
#define NUM_POS 3


using namespace std;

typedef std::pair<double,int> mypair;
bool comparator( const mypair& l, const mypair& r)
{ return l.second< r.second; } 


class visual_prediction_manager
{
public:
  /// constructor
  visual_prediction_manager(std::string name);
  visual_prediction_manager();
  /// destructor
  virtual ~visual_prediction_manager();

  /// callback for messages
  void publish_target_belief();
  void publish_searchmap();
  void publish_human_candidates();
  void global_pose_a1_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void global_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
  void human_poses_callback(const geometry_msgs::PoseArray::ConstPtr& message);
  void target_poses_callback(const geometry_msgs::PoseArray::ConstPtr& message);
  void searchmap_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
  
  bool Comparetwopoistions(std::vector<double> pos,std::vector<double> pos2,double criterion);
  bool check_staticObs(float x_pos,float y_pos);
  //bool getlinevalue(int line_type,double input_x, double input_y);
  double getDistance_from_Vec(std::vector<double> origin, double _x, double _y);
  //bool check_cameraregion(float x_pos,float y_pos);

  int  CoordinateTransform_Global2_staticMap(float global_x, float global_y);
  int  CoordinateTransform_Global2_beliefMap(double global_x, double global_y);
  void Mapidx2GlobalCoord(int map_idx, std::vector<double>& global_coords);
  int Coord2CellNum(double _x, double _y, const nav_msgs::OccupancyGrid& inputmap_);
    
  void update_human_occ_belief(int update_type);
  void spin();

  //void executeCB(const visual_perception::GetFrontierGoalConstPtr &goal);

  //costmap
  frontier_exploration::Frontier buildNewUnknown(unsigned int initial_cell, unsigned int reference, std::vector<bool>& frontier_flag);
  bool isNewFrontierCell(unsigned int idx, const std::vector<bool>& frontier_flag);
  bool isNewUnknown(unsigned int idx, const std::vector<bool>& frontier_flag);
  bool isCellValueandFree(unsigned int idx, int value_);
  bool hasFreenbr(unsigned int idx);
  double frontierCost(const frontier_exploration::Frontier& frontier);
  bool nearestCell(unsigned int &result, unsigned int start, unsigned char val);
  void SplitFrontiers(const frontier_exploration::Frontier& frontier, unsigned int reference_,  std::vector<frontier_exploration::Frontier>& newfrontiers);
  std::vector<unsigned int> nhood8(unsigned int idx);
  std::vector<unsigned int> nhood4(unsigned int idx);
  //std::vector<frontier_exploration::Frontier> searchFrom(geometry_msgs::Point position);
  std::vector<int> FindCellFrom(geometry_msgs::Point position, int value_);
  unsigned char* getCharMap() const; 
  void visualizeFrontiers(const std::vector<frontier_exploration::Frontier>& frontiers);
  void getNextFrontiers(const std::vector<frontier_exploration::Frontier>& frontiers);
  void insert_unknownmap(const std::vector<int> idx_set);
  //void reset_search_map();
  void find_avg_pose(const std::vector<int> idx_set, geometry_msgs::Pose& avg_point);
  void LoadPositionfromConfig(ros::NodeHandle n, std::string input_locations);
  void parseparameters(ros::NodeHandle n);
  void FindUnknowns();
  std::vector<frontier_exploration::Frontier> Unknown_search(geometry_msgs::Point position, std::vector<bool>& visited_flag, int target_value_);
  double get_visited_ratio(std::vector<bool>& visited_flag);
  void executeCB(const visual_perception::VisualUnknownSearchGoalConstPtr &goal);

private:

  ros::NodeHandle nh_;
  std::string action_name_;
  actionlib::SimpleActionServer<visual_perception::VisualUnknownSearchAction> as_;
  visual_perception::VisualUnknownSearchResult result_;
  visual_perception::VisualUnknownSearchFeedback feedback_;
  std::map< std::string, std::vector<double> > goal_maps;

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

  ros::Publisher camera_visible_region_pub;
  ros::Publisher human_target_pub;
  ros::Publisher searchmap_pub;
  ros::Publisher human_candidates_pub;
  ros::Publisher frontier_marker_pub;
  ros::Publisher nextfrontier_pose_pub;
  ros::Publisher cmd_velocity_pub;
  ros::Publisher unknown_map_pub;
  ros::Publisher unknown_posearray_pub;

  ros::Subscriber globalpose_sub;
  ros::Subscriber Scaled_static_map_sub;
  ros::Subscriber searchmap_sub;
  ros::Subscriber target_poses_sub;

  int max_frontier_size_;
  unsigned int min_frontier_size_;
  unsigned int costmap_size_x;
  unsigned int costmap_size_y;
  unsigned char* costmap_;
  
  // tf listener
  //tf::TransformListener     robot_state_;
  tf::TransformListener     listener;
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener* tf2_listener;
  geometry_msgs::TransformStamped map_en_to_map;

  std::string fixed_frame_;
  boost::mutex filter_mutex_;

  std::vector<int> visiblie_idx_set;
  std::vector<int> human_occupied_idx;

  std::vector<double> global_pose;
  std::vector<double> global_pose_a1;
  std::vector< std::vector< double > > cur_target;
  std::vector<double> Head_Pos; 

  int num_of_detected_human;
  int num_of_detected_target;
  nav_msgs::OccupancyGrid camera_visible_region;
  nav_msgs::OccupancyGrid dynamic_belief_map;
  nav_msgs::OccupancyGrid Human_Belief_Scan_map;
  nav_msgs::OccupancyGrid Human_Belief_type_map;
  nav_msgs::OccupancyGrid Target_Search_map;
  nav_msgs::OccupancyGrid unknown_map;
  geometry_msgs::PoseArray human_candidates_poses;
  
  std::vector<double> m_dyn_occupancy;
  std::vector<double> m_prob_occupancy;
	
  
  std::vector<int> index_of_target_occ_cells_updated_recently;
  std::map<int, float> map_index_of_target_cells_to_prob;
  

  //costmap
  double initial_yaw;
  double potential_scale_;
  double gain_scale_;
  double blacklist_radius_;
  double map_res;

  int last_markers_count_;
  bool isTargetDetected;
  bool isActionActive;
  bool Is_Searchmap_received;
  bool map_dimension_changed;

  frontier_exploration::Frontier left_frontiers;
  frontier_exploration::Frontier right_frontiers;
  geometry_msgs::Point left_average;
  geometry_msgs::Point right_average;
  geometry_msgs::PoseArray unknown_poses;

}; // class


