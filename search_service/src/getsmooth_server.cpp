#include "ros/ros.h"
#include <math.h>
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Twist.h"
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <Eigen/Dense>
#include <sstream>
#include <signal.h>
#include <boost/thread/thread.hpp>
#include <std_msgs/Bool.h>
#include <search_service/GetSmoothPathAction.h>
#include <search_service/GetSmoothPathGoal.h>
#include <search_service/GetSmoothPathResult.h>
#include <actionlib/server/simple_action_server.h>
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "tf/tf.h"


#define CELL_MAX_ENTROPY 0.693147

using namespace std;

class PathManager
{
public:
    
  PathManager(std::string name, std::string planner_topic_): 
  as_(nh_, name, boost::bind(&PathManager::executeCB, this,_1), false),
  action_name_(name)
  {
     nh_.param("PLANNER_TOPIC", planner_topic,{"/tb1/move_base/make_plan"});
     planner_topic = planner_topic_;
     ROS_INFO("global planner topic: %s", planner_topic.c_str());
     planner_srv_client= nh_.serviceClient<nav_msgs::GetPlan>(planner_topic);

    //register the goal and feeback callbacks
    as_.registerPreemptCallback(boost::bind(&PathManager::preemptCB, this));
    //subscribe to the data topic of interest
    as_.start();

  }

  ~PathManager(void)
  {
  }

  void preemptCB()
  {
    ROS_INFO("%s: Preempted", action_name_.c_str());
    ROS_INFO("preempted called");
    // set the action state to preempted
    as_.setPreempted();
  }

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




  double get_expected_entropy_infov(const geometry_msgs::Point pose, const nav_msgs::OccupancyGrid& inputmap_)
  {
    double sensor_range=8;
    double xyreso=0.5;
    double agent_x = pose.x;
    double agent_y = pose.y;
    double minx = agent_x - sensor_range;
    double  maxx = agent_x + sensor_range;
    double miny = agent_y - sensor_range;
    double maxy = agent_y + sensor_range;
    double  xw = int((maxx - minx) / xyreso);
   double  yw = int((maxy - miny) / xyreso);
   double min_map_x = inputmap_.info.origin.position.x;
   double min_map_y = inputmap_.info.origin.position.y;
   double max_map_x = inputmap_.info.origin.position.x+inputmap_.info.width;
   double max_map_y = inputmap_.info.origin.position.y+inputmap_.info.height;


    double entropy_sum=0.0;
    int cell_count=0;

    for(int i(0);i < xw; i++){
        for (int j(0); j<yw; j++){
            double px = i * xyreso + minx;
            double py = j * xyreso + miny;
        
            if(px > max_map_x|| px < min_map_x)
                continue;

            if(py > max_map_y|| px < min_map_y)
                continue;

            int search_idx = Coord2CellNum(px,py, inputmap_);
            //convert log-occ to probability
            if(inputmap_.data[search_idx]==0.0)
                cell_count++;
        }
    }
 
    //ROS_INFO("cell counts: %d ", cell_count);
    entropy_sum=double(cell_count)*CELL_MAX_ENTROPY;

    return entropy_sum;
}






  double calc_dist(geometry_msgs::Pose pos1, geometry_msgs::Pose pos2)
  {

      double dist_=pow(pos1.position.x-pos2.position.x, 2);
      dist_+=pow(pos1.position.y-pos2.position.y, 2);
        return dist_;

  }

  void executeCB(const search_service::GetSmoothPathGoalConstPtr &goal)
  {

    nav_msgs::GetPlan srv_;
     // ros::Rate r(1);
     bool success = true;
     // ROS_INFO("%s: succeeded",action_name_.c_str());
     if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        success = false;
        // break;
      }

    //Setting 
    ROS_INFO("GetSmoothPath action is called!!");

    nav_msgs::Path smooth_path;
    srv_.request.start.pose = goal->start_pos; //current agent_pose
    srv_.request.start.header.frame_id = "map";
    srv_.request.start.header.stamp= ros::Time::now();
    std::vector<int> real_dst;
    std::vector<int> idx_set;
    int max_idx = 0;
    int min_idx = 0;
    nav_msgs::Path tmp_plan;

    real_dst.resize(goal->input_path.poses.size(),0.0);
    double min_dist=1000.0;
    for(size_t i(0);i<goal->input_path.poses.size();i++)
    {
        idx_set.push_back(i);
        srv_.request.goal= goal->input_path.poses[i];
        srv_.request.goal.header.frame_id= "map";
        if(planner_srv_client.call(srv_))
        {
           double tmp_dst=0.0;
           if(srv_.response.plan.poses.size()>1)
           {
               for(int m(0);m<srv_.response.plan.poses.size()-1;m++)
                   tmp_dst+=calc_dist(srv_.response.plan.poses[m].pose, srv_.response.plan.poses[m+1].pose);
           }
           else{
               tmp_dst = calc_dist(srv_.request.start.pose, srv_.response.plan.poses[0].pose);
               double len_pose =srv_.response.plan.poses.size();
               //ROS_INFO("len_pose: %f", len_pose);
            }
            real_dst[i]=tmp_dst;
            ROS_INFO("tmp_dst: %.2lf", tmp_dst);
            if(real_dst[i]<min_dist)
            {
                min_dist =real_dst[i];
                tmp_plan=srv_.response.plan;
            }
                //calculate the path length to the pose(for finding the nearest pose from the robot location)
        }
        else{
            ROS_INFO("global planner make plan service failed");
            real_dst[i]=500.0;
        }
    }
    if(real_dst.size()>0)
    {
        auto minIt = std::min_element(real_dst.begin(), real_dst.end());
        double minElement = *minIt;
        min_idx = minIt -real_dst.begin();

        srv_.request.start.pose= goal->input_path.poses[min_idx].pose;
        idx_set.erase(std::remove(idx_set.begin(),idx_set.end(),min_idx), idx_set.end());
        for(size_t k(0);k<tmp_plan.poses.size();k++)
            smooth_path.poses.push_back(tmp_plan.poses[k]);
    }
    else{
        min_idx=0;
        idx_set.erase(std::remove(idx_set.begin(),idx_set.end(),min_idx), idx_set.end());
    }
    //after first point, iterate with idx_set 
    real_dst.clear();
    //call service from current point
    //check the pose length to the pose in idx_set
    bool max_iter_reached=false;
    int idx_iter=0;
    int cur_idx = min_idx;
    double max_dst=0.0;
    while(idx_set.size()>0){
        //ROS_INFO("idx_set.size() : %d", idx_set.size());
        max_dst=-1000.0;
        srv_.request.start.pose =goal->input_path.poses[cur_idx].pose;
        srv_.request.start.header.frame_id = "map";
        srv_.request.start.header.stamp =ros::Time::now();
        real_dst.resize(idx_set.size(),0.0);
        for(size_t j(0);j<idx_set.size();j++)
        {
            srv_.request.goal= goal->input_path.poses[idx_set[j]];
            srv_.request.goal.header.frame_id= "map";
            srv_.request.goal.header.stamp = ros::Time::now();
            if(planner_srv_client.call(srv_))
            {
                double tmp_dst=0.0;
                double tmp_gain=0.0;
                if(srv_.response.plan.poses.size()>1){
                    int end_point= srv_.response.plan.poses.size()-1;
                    //int mid_point = floor(end_point/2);
                    //tmp_gain+=get_expected_entropy_infov(srv_.response.plan.poses[mid_point].pose.position, goal->search_map);
                    tmp_gain+=get_expected_entropy_infov(srv_.response.plan.poses[end_point].pose.position,goal->search_map);
                    
                    

                    for(int m(0);m<srv_.response.plan.poses.size()-1;m++)
                    {
                        tmp_dst+=calc_dist(srv_.response.plan.poses[m].pose, srv_.response.plan.poses[m+1].pose);
                    }
                }
                else{
                    tmp_dst = calc_dist(srv_.request.start.pose, srv_.response.plan.poses[0].pose);
                    int len_pose =srv_.response.plan.poses.size();
                }

                //ROS_INFO("tmp_gain: %.2lf", tmp_gain);
                //ROS_INFO("tmp_dst: %.2lf", tmp_dst);
                real_dst[j]=0.01*tmp_gain-15*tmp_dst;
                if(real_dst[j]>max_dst)
                {
                    max_dst=real_dst[j];
                    tmp_plan=srv_.response.plan;
                }
            }
            else{
                real_dst[j]=500.0;
                ROS_INFO("failed to get plan");
                smooth_path.poses.push_back(goal->input_path.poses[idx_set[j]]);
            }
        }
        auto maxIt = std::max_element(real_dst.begin(), real_dst.end());
        double minElement = *maxIt;
        max_idx = maxIt -real_dst.begin();
        cur_idx=idx_set[max_idx];
        idx_set.erase(std::remove(idx_set.begin(),idx_set.end(),cur_idx), idx_set.end());
        for(size_t k(0);k<tmp_plan.poses.size();k++)
            smooth_path.poses.push_back(tmp_plan.poses[k]);
        idx_iter++;
        if(idx_iter>15)
            break;
    }//while

    if(smooth_path.poses.size()>0)
    {
        smooth_path.header.frame_id="map";
        smooth_path.header.stamp=ros::Time::now();
        result_.output_path=smooth_path;
        ROS_INFO("smooth_path Updated");
        as_.setSucceeded(result_);
    }
    else{
        ROS_INFO("pathsize is none!");
    }


  }



protected:
    
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<search_service::GetSmoothPathAction> as_;
  std::string action_name_;
  int data_count_, goal_;
  float sum_, sum_sq_;
  search_service::GetSmoothPathFeedback feedback_;
  search_service::GetSmoothPathResult result_;
  ros::ServiceClient planner_srv_client;
  std::string planner_topic;


};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "get_smooth_path_action");
  std::cout<<"path_topic: "<<argv[1]<<std::endl;
  PathManager smoothpath_manager(ros::this_node::getName(), argv[1]);
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
