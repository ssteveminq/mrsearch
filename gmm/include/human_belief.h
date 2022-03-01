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
#include <sensor_msgs/PointCloud.h>
#include <message_filters/time_sequencer.h>
#include <message_filters/subscriber.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/OccupancyGrid.h>

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_real_distribution.hpp>

#include <gmm/PredictionAction.h>
#include <gmm/PredictionResult.h>
#include <gmm/PredictionFeedback.h>

#include <GaussianMixtureModel/ExpectationMaximization.h>
#include <GaussianMixtureModel/GaussianMixtureModelFactory.h>
#include <GaussianUtils/GaussianDistributionFactory.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// log files
#include <fstream>
#include <vector>

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


typedef std::pair<double,int> mypair;
bool comparator( const mypair& l, const mypair& r)
{ return l.first < r.first; } 



class belief_manager
{
public:
  /// constructor
  //
  belief_manager();
  belief_manager(std::string name);
  /// destructor
  virtual ~belief_manager();

  /// callback for messages
  void publish_human_belief();
  void publish_gmm();
  void global_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
  void human_yolo_callback(const visualization_msgs::MarkerArray::ConstPtr& message);
  void people_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void searchmap_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
  

  bool Comparetwopoistions(std::vector<double> pos,std::vector<double> pos2,double criterion);
  bool check_staticObs(float x_pos,float y_pos);
  double getDistance_from_Vec(std::vector<double> origin, double _x, double _y);

  int  globalcoord_To_SScaled_map_index(float x_pos,float y_pos);
  int  CoordinateTransform_Global2_staticMap(float global_x, float global_y);
  int  CoordinateTransform_Global2_beliefMap(double global_x, double global_y);
  void Mapidx2GlobalCoord(int map_idx, std::vector<double>& global_coords, nav_msgs::OccupancyGrid& map_);
    
  void update_human_occ_belief(int update_type);
  void publish_clusters();
  void update_particles();
  void publish_meanpose();
  void spin();

  void put_human_occ_map_yolo();
  void executeCB(const gmm::PredictionGoalConstPtr &goal);

private:

  ros::NodeHandle nh_;
  std::string action_name;
  actionlib::SimpleActionServer<gmm::PredictionAction> as_;
  gmm::PredictionResult result_;
  gmm::PredictionFeedback feedback_;

  ros::Publisher camera_visible_region_pub;
  ros::Publisher mean_pub;
  ros::Publisher belief_pub;
  ros::Publisher human_candidates_pub;
  ros::Publisher visual_marker_pub;

  ros::Subscriber people_yolo_sub;
  ros::Subscriber people_poses_sub;
  ros::Subscriber globalpose_sub;
  ros::Subscriber joint_state_sub;
  ros::Subscriber searchmap_sub;

  bool Is_Searchmap_received;
  bool map_dimension_changed;
  bool isCalled;

  geometry_msgs::PoseArray sample_poses;
  geometry_msgs::PoseArray mean_poses;

  // tf listener
  tf::TransformListener     robot_state_;
  tf::TransformListener     listener;

  double MAX_X;
  double MAX_Y;
  double MIN_X;
  double MIN_Y;



  std::vector<gauss::gmm::Cluster> clusters;

  std::string fixed_frame_;
  boost::mutex filter_mutex_;

  std::vector<int> visiblie_idx_set;
  std::vector<int> human_occupied_idx;

  std::vector<double> global_pose;
  std::vector< std::vector< double > > cur_people;

  int num_of_detected_human;
  nav_msgs::OccupancyGrid Scaled_map;
  nav_msgs::OccupancyGrid camera_visible_region;
  nav_msgs::OccupancyGrid belief_map;
  nav_msgs::OccupancyGrid Human_Belief_Scan_map;
  //nav_msgs::OccupancyGrid Human_Belief_type_map;
  nav_msgs::OccupancyGrid search_map;
  geometry_msgs::PoseArray human_candidates_poses;
  
  std::vector<double> m_dyn_occupancy;
  std::vector<double> m_prob_occupancy;
	
  
  std::vector<int> index_of_human_occ_cells_updated_recently;
  std::map<int, float> map_index_of_human_cells_to_prob;

}; // class


