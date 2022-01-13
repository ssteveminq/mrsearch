#ifndef __STATE_LATTICE_PLANNER_ROS_H
#define __STATE_LATTICE_PLANNER_ROS_H

#include <ros/ros.h>
#include <math.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"


#include "state_lattice_planner/state_lattice_planner.h"

class StateLatticePlannerROS
{
public:
    StateLatticePlannerROS(void);

    void process(void);
    bool process(const geometry_msgs::PoseStamped& local_goal, bool is_finalgoal, const std::vector<double>& robot_pose, bool& replan);
    void local_goal_callback(const geometry_msgs::PoseStampedConstPtr&);
    void local_map_callback(const nav_msgs::OccupancyGridConstPtr&);
    void global_map_callback(const nav_msgs::OccupancyGridConstPtr&);
    void odom_callback(const nav_msgs::OdometryConstPtr&);
    void target_velocity_callback(const geometry_msgs::TwistConstPtr&);
    void pose_callback(const geometry_msgs::PoseStampedConstPtr&);
    template<typename TYPE>
    void get_obstacle_map(const nav_msgs::OccupancyGrid&, state_lattice_planner::ObstacleMap<TYPE>&);
    bool check_collision_rosmap(const nav_msgs::OccupancyGrid& map_, const std::vector<Eigen::Vector3d>& trajectory, const std::vector<double>& robot_pose);
    int get_mapidx_from_xy(const nav_msgs::OccupancyGrid& map_, const double x_, const double y_);
    void get_xy_from_index(const nav_msgs::OccupancyGrid& map_, int idx_, std::vector<double>& Coord);
    void smooth_velocity(geometry_msgs::Twist& cmd_vel);
    void smoothing_velcommand(geometry_msgs::Twist& cmd_vel_des, geometry_msgs::Twist& cur);
    void update_linear_y(geometry_msgs::Pose base_pose,geometry_msgs::Twist& cmd_vel);
    void rotate_trajectories(const std::vector<MotionModelDiffDrive::Trajectory> originals,   std::vector<MotionModelDiffDrive::Trajectory>& rotated);

protected:
    void visualize_trajectories(const std::vector<MotionModelDiffDrive::Trajectory>&, const double, const double, const double, const int, const ros::Publisher&);
    void visualize_trajectory(const MotionModelDiffDrive::Trajectory&, const double, const double, const double, const ros::Publisher&);

    double HZ;
    std::string ROBOT_FRAME;
    int N_P;
    int N_H;
    int N_S;
    double MAX_ALPHA;
    double MAX_PSI;
    double MAX_ACCELERATION;
    double TARGET_VELOCITY;
    std::string LOOKUP_TABLE_FILE_NAME;
    int MAX_ITERATION;
    double OPTIMIZATION_TOLERANCE;
    double MAX_YAWRATE;
    double MAX_D_YAWRATE;
    double MAX_WHEEL_ANGULAR_VELOCITY;
    double WHEEL_RADIUS;
    double TREAD;
    double IGNORABLE_OBSTACLE_RANGE;
    bool VERBOSE;
    int CONTROL_DELAY;
    double TURN_DIRECTION_THRESHOLD;
    bool ENABLE_SHARP_TRAJECTORY;
    bool ENABLE_CONTROL_SPACE_SAMPLING;
    double MAP_RANGE;
    double SUBGOAL_RANGE;
    double GOAL_RANGE;

    ros::NodeHandle nh;
    ros::NodeHandle local_nh;

    ros::Publisher velocity_pub;
    ros::Publisher candidate_trajectories_pub;
    ros::Publisher candidate_trajectories_pub2;
    ros::Publisher candidate_trajectories_no_collision_pub;
    ros::Publisher selected_trajectory_pub;
    ros::Publisher selected_trajectory_goal_pub;
    ros::Publisher local_goal_pub;
    ros::Publisher goal_status_pub;
    ros::Subscriber global_map_sub;
    ros::Subscriber local_map_sub;
    ros::Subscriber local_goal_sub;
    ros::Subscriber pose_sub;
    ros::Subscriber odom_sub;
    ros::Subscriber target_velocity_sub;
    tf::TransformListener listener;

    //tf2_ros::Buffer tf_buffer;
    //tf2_ros::TransformListener* tf2_listener; 
    //geometry_msgs::TransformStamped map_to_base;
    geometry_msgs::PoseStamped local_goal;
    geometry_msgs::PoseStamped cur_pose;
    nav_msgs::OccupancyGrid local_map;
    nav_msgs::OccupancyGrid global_map;
    geometry_msgs::Twist current_velocity;
    geometry_msgs::Twist previous_velocity;
    bool local_goal_subscribed;
    bool local_map_updated;
    bool global_map_updated;
    bool odom_updated;
    int local_map_count;
    bool goal_out_of_map;
    int stack_count;
    int zero_count;
    double rot_gain;
    bool goal_reached;
    double last_time_trj;
    MotionModelDiffDrive::Trajectory cur_trj;
    StateLatticePlanner planner;
};

#endif //__STATE_LATTICE_PLANNER_ROS_H
