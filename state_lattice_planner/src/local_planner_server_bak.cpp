#include "ros/ros.h"
#include <math.h>
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Twist.h"
#include "state_lattice_planner/state_lattice_planner_ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <Eigen/Dense>
#include <sstream>
#include <signal.h>
#include <boost/thread/thread.hpp>
#include <std_msgs/Bool.h>
#include <state_lattice_planner/LocalNaviAction.h> 
#include <nav_msgs/GetPlan.h>
#include <actionlib/server/simple_action_server.h>
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "tf/tf.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/utils.h"
      

#define DYN_OFFSET_X 3.5
#define DYN_OFFSET_Y 3.5
#define MAP_RES 0.05
#define NUM_SUBS 3
#define Max_speed 0.40
#define Min_speed 0.10
#define Max_angular_speed 0.25
#define ROBOT_FRAME "base"


using namespace std;

class SLPlanner
{
public:
    
  SLPlanner(std::string name): 
  as_(nh_, name, boost::bind(&SLPlanner::executeCB, this,_1), false),
  action_name_(name),
  IsGoal(false),
  IsRotated(false),
  IsActive(false),
  direction_z(1),
  split_size(3),
  srv_time(0.0),
  current_path_idx(0),
  callback_iter(0),
  IsFinalSubgoal(false)
  {
    planner_srv_client= nh_.serviceClient<nav_msgs::GetPlan>("/planner/planner/make_plan");
    cmd_velocity_pub= nh_.advertise<geometry_msgs::Twist>("/cmd_vel",10, true);
    subgoal_pub= nh_.advertise<geometry_msgs::PoseStamped>("/subgoal",10, true);
    dynamicmap_sub =nh_.subscribe<nav_msgs::OccupancyGrid>("/costmap_node/costmap/costmap", 30, &SLPlanner::dynamicmapCallback,this); 
    global_map_sub = nh_.subscribe("/scaled_static_map", 1, &SLPlanner::global_map_callback, this);
    globalpose_sub=nh_.subscribe<geometry_msgs::PoseStamped>("/global_pose_a1",10,&SLPlanner::global_pose_callback,this);
    //odom_sub=nh_.subscribe<nav_msgs::Odometry>("/odom",10,&SLPlanner::odom_callback,this);
    Scaled_dynamic_map_pub=nh_.advertise<nav_msgs::OccupancyGrid>("/scaled_dynamic_map", 10, true);
    fast_occ_pub=nh_.advertise<std_msgs::Bool>("/Is_Occupied", 10, true);

    geometry_msgs::PointStamped input_point;
    input_point.header.frame_id="base";
    input_point.point.x=0.5;
    input_point.point.y=0.1;
    input_point.point.z=0.0;
    PointSet.push_back(input_point);
    input_point.header.frame_id="base";
    input_point.point.x=0.5;
    input_point.point.y=-0.1;
    input_point.point.z=0.0;
    PointSet.push_back(input_point);

    global_pose.resize(3,0.0);
    navtarget_pose.resize(2,0.0);
    //register the goal and feeback callbacks
    as_.registerPreemptCallback(boost::bind(&SLPlanner::preemptCB, this));
    //subscribe to the data topic of interest
    as_.start();
  }

  ~SLPlanner(void)
  {
  }

  void preemptCB()
  {
    ROS_INFO("%s: Preempted", action_name_.c_str());
    geometry_msgs::Twist vel_cmd;
                    vel_cmd.linear.x=0.00;
                    vel_cmd.linear.y=0.0;
                    vel_cmd.linear.z=0.0;

                    vel_cmd.angular.x=0.0;
                    vel_cmd.angular.y=0.0;
                    vel_cmd.angular.z=0.0;
                    cmd_velocity_pub.publish(vel_cmd);
                    IsActive=false;
                    ROS_INFO("preempted called");

    // set the action state to preempted
    as_.setPreempted();
  }
  void publish_subgoal(geometry_msgs::PoseStamped& pose_)
  {
     //geometry_msgs::PoseStamped subgoal_pos;
    //subgoal_pos.header.stamp=ros::Time::now();
    //subgoal_pos.header.frame_id="map";
    //subgoal_pos.pose.position.x=navtarget_pose[0];
    //subgoal_pos.pose.position.y=navtarget_pose[1];
    //subgoal_pos.pose.orientation.w=1.0;
    subgoal_pub.publish(pose_);
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

  void executeCB(const state_lattice_planner::LocalNaviGoalConstPtr &goal)
  {
     bool success = true;
     if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        success = false;
        // break;
      }

    //Setting 
    IsRotated=false;
    IsActive=true;
    IsGoal=false;

    ROS_INFO("SLPlanner action is called!!");
    //save the current global_pose w.r.t map
    //path-follow
    //goal->path.poses.end()
    if(goal->path.poses.size()<1)
    {

        std::cout<<"goal-path wrong-try to call this action again"<<std::endl;
        return;
    }

    auto it =std::prev(goal->path.poses.end());
    m_goalpose.pose=it->pose;
    m_goalpose.header.frame_id="map";

    double total_distance_to_goal=sqrt(pow((it->pose.position.x-global_pose[0]),2)+pow(it->pose.position.y-global_pose[1],2));
    int num_subgoals=NUM_SUBS;
    if(total_distance_to_goal>10.0)
        num_subgoals=NUM_SUBS+int((total_distance_to_goal-10.0)/2.0);
    else if(total_distance_to_goal<5.0)
        num_subgoals=2;

        
    int path_length = goal->path.poses.size();
    int path_const = floor(path_length/num_subgoals);
    subgoals.poses.clear();
    
    subgoals.poses.push_back(goal->path.poses[0].pose);
    for(int i(0);i<(num_subgoals-1);i++)
    {
        int pose_idx=(i+1)*path_const;
        //if(pose_idx>goal->path.poses.size())
            //subgoals.poses.push_back(goal->path.poses[path_length-1].pose);
        //else
            subgoals.poses.push_back(goal->path.poses[pose_idx].pose);
            //subgoals.poses.push_back(goal->path.poses[path_length-1].pose);
        //ROS_INFO("- %d th point: (%.2lf, %.2lf)", i, goal->path.poses[i].pose.position.x, goal->path.poses[i].pose.position.y);
    }

    subgoals.poses.push_back(goal->path.poses[path_length-1].pose);
    ROS_INFO("---Subgoal--Size: %d", subgoals.poses.size());

    //subgoals.poses.push_back(goal->path.poses[path_length-1].pose);
    geometry_msgs::PoseStamped pose_base;
    geometry_msgs::Twist vel_cmd;
    double ros_rate = 3.0;
    ros::Rate r(ros_rate);
    int cur_goal_idx=0;
    //get first subgoal index
    bool is_finalgoal=false;
    int failure_count=0;
    bool bool_wait=true;
    bool bool_asst_ctrl=false;
    bool bool_replan=false;
    while(ros::ok() && IsActive && !as_.isPreemptRequested())
    {
        //transform subgoal to base_link frame
        if(bool_replan)
        {
            replanning(cur_goal_idx);
            is_finalgoal=false;
            bool_wait=true;
            bool_asst_ctrl=false;
            bool_replan=false;
            failure_count=0;
        
        }
        bool bool_tf=transform_subgoal_to_baselinkframe(subgoals.poses[cur_goal_idx], pose_base);
        if(!bool_tf)
        {
            bool_wait=true;
            failure_count++;

            if(failure_count>6)
            {
                bool_replan=true;
                //subgoal updates with replanning
                //replanning(cur_goal_idx);
                failure_count=0;
            
            }
            else if(failure_count>3)
            {
                bool_asst_ctrl=true;
                //failure_count=0;
                //control with global pos
            }
        }
        else
        {
            //control with global pos
            bool_wait=false;
            failure_count=0;
        }
        
            
        if(!bool_wait)
        {
            //publish_subgoal
            publish_subgoal(pose_base);   //base_frame
            if(cur_goal_idx==subgoals.poses.size()-1)
                is_finalgoal=true;

            if(pslp_planner->process(pose_base, is_finalgoal,global_pose, bool_replan))
            {
                //this block will be called only when the robot arrives subgoals or finalgoal.
                //check current path_idx
                if(is_finalgoal)
                {
                    IsActive=false;
                    feedback_.is_possible_go=true;
                    Sending_zerovelcmd();
                    result_.success=feedback_.is_possible_go;
                    as_.setSucceeded(result_);
                }
                else
                    cur_goal_idx++;

            }
        }
        else{
            if(bool_asst_ctrl)
            {
                //control with global pos
                if(global_pose_updated)
                {
                    ROS_INFO("control with global pose");
                    get_cmd_gains(subgoals.poses[cur_goal_idx], vel_cmd);
                    cmd_velocity_pub.publish(vel_cmd);
                    global_pose_updated=false;
                    bool_asst_ctrl=false;
                }
            
            }
            //else
                //Sending_zerovelcmd();
        
        }

        as_.publishFeedback(feedback_);
    	ros::spinOnce();
        r.sleep();
    }

  }
  void odom_callback(const nav_msgs::OdometryConstPtr& msg){
  
      global_pose[0]=msg->pose.pose.position.x;
      global_pose[1]=msg->pose.pose.position.y;
  
  }

  void global_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
      //* global_pose w.r.t "map" frame
      m_pose=*msg;
      global_pose[0]=msg->pose.position.x;
      global_pose[1]=msg->pose.position.y;
      global_pose[2]=tf2::getYaw(msg->pose.orientation);
      global_pose_updated=true;
      /*-----------------------------global fram w.r.t map_en ------------------------

    //map_en_to_map= tf_buffer.lookupTransform("map_en", "map", ros::Time(0), ros::Duration(1.0) );
    if(callback_iter>20)
    {
        geometry_msgs::PoseStamped pose_in;     //added by mk, ryan
        pose_in.header.stamp=ros::Time(0);
        pose_in.header.frame_id=msg->header.frame_id;
        pose_in.pose=msg->pose;
 
      geometry_msgs::PoseStamped pose_out;     //added by mk, ryan
      try{

          listener.transformPose("/map", ros::Time(0), *msg, msg->header.frame_id, pose_out);
          //tf2::doTransform(*msg, *msg, map_en_to_map);
          //tf_buffer.transform(pose_in,pose_out, "map");
      }
      catch (tf::TransformException &ex)
      {
          ROS_WARN("global_pose: Transform Failed: %s", ex.what());
          sleep(0.10);
          return;
      }

      m_currentpose = msg->pose;
      global_pose[0]=pose_out.pose.position.x;
      global_pose[1]=pose_out.pose.position.y;
      global_pose[2]=0.0;

      tf::StampedTransform baselinktransform;
      try{
          listener.waitForTransform("map", "base", ros::Time(0), ros::Duration(1.0));
          listener.lookupTransform("map", "base", ros::Time(0), baselinktransform);
          double yaw_tf =   tf::getYaw(baselinktransform.getRotation()); 
          global_pose[2]=yaw_tf;
          global_pose_updated=true;
      }
      catch(tf::TransformException &ex){
      
          ROS_WARN("yaw Transform Failed: %s", ex.what());
          return;
      }
      ROS_INFO("global_pose: %.2lf, %.2lf, %.2lf", global_pose[0], global_pose[1], global_pose[2]);
      */
      /*

      bool isObstacle=false;
      try{
          listener.waitForTransform("map", "base", ros::Time(0), ros::Duration(2.0));
      }
      catch(tf::TransformException &ex){
      }

      for(size_t point_idx(0);point_idx<PointSet.size();point_idx++)
      {
            geometry_msgs::PointStamped point_out;

          try{
                listener.transformPoint("map_en", PointSet[point_idx], point_out);
                //ROS_INFO("transformed point : x : %.3lf, y : %.3lf ", point_out.point.x,point_out.point.y);
                //int map_idx=Coord2CellNum(point_out.point.x,point_out.point.y);
                //std::vector<double> front_point(2,0.0);
                //Idx2Globalpose(map_idx,front_point);
                //ROS_INFO("front point : x : %.3lf, y : %.3lf ",front_point[0], front_point[1]);

                //ToDo: FixMe
                //if(Scaled_dynamic_map.data[map_idx]>1.0)
                //{
                    //isObstacle=true;
                    //break;
                //}
          }
          catch(tf::TransformException &ex){
          
              ROS_WARN("Transform Failed: %s", ex.what());
              return;
          }
      }

      
      callback_iter=0;
    }
    else
    {
      callback_iter++;
        return;
    
    }
    */
  }
  
  void dynamicmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    dynamic_map=*msg;
    local_map_updated = true;
    //check_obstacle
}

void global_map_callback(const nav_msgs::OccupancyGridConstPtr& msg)
{
    ROS_INFO("global map callback");
    global_map = *msg;
    global_map_updated = true;
}



    void Idx2Globalpose(int idx, std::vector<double>& global_coord)
    {
        global_coord.resize(2,0.0);

        int res = (int) (idx/ Scaled_dynamic_map.info.width);
        int div = (int) (idx%Scaled_dynamic_map.info.width);

        global_coord[0]=res*Scaled_dynamic_map.info.resolution+Scaled_dynamic_map.info.origin.position.x;
        global_coord[1]=div*Scaled_dynamic_map.info.resolution+Scaled_dynamic_map.info.origin.position.y;
    }


    bool process_target(double goal_x, double goal_y,double goal_theta){


        float diff_x=goal_x-global_pose[0];
        float diff_y=goal_y-global_pose[1];
        float dist =sqrt(pow(diff_x,2)+pow(diff_y,2));

        if(dist>2.5)
            split_size=5.0;
        else if(dist<0.75)
            split_size=0;
        else
            split_size=3.0;

    
        //ROS_INFO("diff x : %.3lf , y : %.3lf ",diff_x,diff_y);
        navtarget_pose.resize(3,0.0);
        navtarget_pose[0]=global_pose[0]+diff_x;
        navtarget_pose[1]=global_pose[1]+diff_y;
        
        //ROS_INFO("global theta : %.3lf, navigation_theta",global_pose[2],navtarget_pose[2]);
        //ROS_INFO("nglobal thetax : %.3lf  ",global_pose[2]);
        //ROS_INFO("target x : %.3lf , y : %.3lf ",navtarget_pose[0],navtarget_pose[1]);
        geometry_msgs::Vector3Stamped gV, tV;

        gV.vector.x=navtarget_pose[0]-global_pose[0];
        gV.vector.y=navtarget_pose[1]-global_pose[1];
        gV.vector.z=0.5;

        gV.header.stamp=ros::Time();
        gV.header.frame_id="/map";
        try{
            listener.transformVector("/base",gV,tV);
        }
        catch(tf::TransformException &ex){

          ROS_WARN("lookup Transform Failed: %s", ex.what());
        }

        double target_angle_baselink= atan2(tV.vector.y,tV.vector.x);
        //ROS_INFO("target_angle_baselink: %.3lf", target_angle_baselink);
        navtarget_pose[2]=(target_angle_baselink)+global_pose[2];
        //ROS_INFO("nav target_angle: %.3lf, global_pose : %.3lf", navtarget_pose[2], global_pose[2]);

        //ToDo : origientation (x,ytheta)
        double diff_theta=navtarget_pose[2]-global_pose[2];
        if(diff_theta>M_PI)
            diff_theta=diff_theta-2*M_PI;
        else if (diff_theta < (-1 * M_PI))
            diff_theta=diff_theta+2*M_PI;

        //ROS_INFO("diff_theta: %.3lf", diff_theta);

        if(diff_theta>0)
            direction_z=1;
        else
            direction_z=-1;

        //Check if the robot can go straight or not
        double coeff=(double)(1.0/(double)split_size);
           int count=0;
           return true;
    }

    int Coord2CellNum(double _x, double _y)
    {	
        //ROS_INFO("x: %.2lf, y: %.2lf", _x, _y);
        std::vector<int> target_Coord;
        target_Coord.resize(2,0);

        double reference_origin_x;
        double reference_origin_y;

        reference_origin_x=Scaled_dynamic_map.info.origin.position.x;
        reference_origin_y=Scaled_dynamic_map.info.origin.position.y;

        double  temp_x  = _x-reference_origin_x;
        double  temp_y = _y-reference_origin_y;

        target_Coord[0]= (int) (temp_x/MAP_RES);
        target_Coord[1]= (int)(temp_y/MAP_RES);

        //ROS_INFO("targertcoord: x: %d, y: %d", target_Coord[0], target_Coord[1]);

        std::vector<int> dynamicCoord;
        dynamicCoord.resize(2);
        dynamicCoord[0]=Scaled_dynamic_map.info.origin.position.x+Scaled_dynamic_map.info.resolution*target_Coord[0]+0.5*Scaled_dynamic_map.info.resolution;
        dynamicCoord[1]=Scaled_dynamic_map.info.origin.position.y+Scaled_dynamic_map.info.resolution*target_Coord[1]+0.5*Scaled_dynamic_map.info.resolution;

        int index= target_Coord[0]+Scaled_dynamic_map.info.width*target_Coord[1];
        //ROS_INFO("dynamic_map idx : %d", index);
        return index;
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

   void get_cmd_gains(geometry_msgs::Pose& goal_pos, geometry_msgs::Twist& cmd_vel)
    {

        std::vector<double> gains(2,0.0);
        gains[0] = goal_pos.position.x-global_pose[0];
        gains[1] = goal_pos.position.y-global_pose[1];
        double diff_theta = atan2(gains[1],gains[0])-global_pose[2];


        for(int i(0);i<2;i++)
        {
            if(abs(gains[i])>1.0)
               {
                   if(gains[i]>0.0)
                       gains[i]=Max_speed;
                   else
                       gains[i]=-Max_speed;
               }
               else if (abs(gains[i])<0.3)
               {
                    if(gains[i]>0.0)
                       gains[i]=Min_speed;
                   else
                       gains[i]=-Min_speed;
               }
               else
                   gains[i]=0.5*gains[i];
        }
        if(fabs(diff_theta)>0.6)
        {
            cmd_vel.linear.x=0.0;
            cmd_vel.linear.y=0.0;
            if(diff_theta>0)
                cmd_vel.angular.z=0.12;
            else
                cmd_vel.angular.z=-0.12;
        
        }
        else{
            cmd_vel.linear.x=gains[0];
            cmd_vel.linear.y=gains[1];
        }


    
    
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

        if((cur_pos.size()<1) || (goal_pos.size()<1))
        {
            ROS_INFO("vector size wrong");
            return;
        }
        gains.resize(2,0.0);
        gains[0]=pose_out.pose.position.x;
        gains[1]=pose_out.pose.position.y;
        for(size_t i(0);i<2;i++) 
        {
           if(abs(gains[i])>1.0)
           {
               if(gains[i]>0.0)
                   gains[i]=Max_speed;
               else
                   gains[i]=-Max_speed;
           }
           else if (abs(gains[i])<0.3)
           {
                if(gains[i]>0.0)
                   gains[i]=Min_speed;
               else
                   gains[i]=-Min_speed;
           }
           else
               gains[i]=0.5*gains[i];
               
        }
        ROS_INFO("Updated gain: %.2lf, %.2lf", gains[0],gains[1]);
    }

    void Sending_zerovelcmd()
    {

        geometry_msgs::Twist vel_cmd;
        vel_cmd.linear.x=0.00; vel_cmd.linear.y=0.0;  vel_cmd.linear.z=0.0;
        vel_cmd.angular.x=0.0; vel_cmd.angular.y=0.0; vel_cmd.angular.z=0.0;
        cmd_velocity_pub.publish(vel_cmd);

    }
    void Sending_velcmd(float x_, float y_, float theta_)
    {

        geometry_msgs::Twist vel_cmd;
        vel_cmd.linear.x=x_; vel_cmd.linear.y=y_;  vel_cmd.linear.z=0.0;
        vel_cmd.angular.x=0.0; vel_cmd.angular.y=0.0; vel_cmd.angular.z=theta_;
        cmd_velocity_pub.publish(vel_cmd);
    }



    void Sending_velcmd()
    {
        bool isObstacle=false;
            if(!IsGoal)
            {
                if(!IsRotated)
                {
                    //control angular z cmd_vel 
                    //
                    if(CheckRotation(global_pose[2],navtarget_pose[2],0.125))
                    {
                        IsRotated=true;
                        ROS_INFO("============Ratate done!!==========");
                        return;
                    }
                    ROS_INFO("Rotating");
                    geometry_msgs::Twist vel_cmd;
                    vel_cmd.linear.x=0.00; vel_cmd.linear.y=0.0;  vel_cmd.linear.z=0.0;
                    vel_cmd.angular.x=0.0; vel_cmd.angular.y=0.0; vel_cmd.angular.z=direction_z*Max_angular_speed;
                    cmd_velocity_pub.publish(vel_cmd);
                }
                else
                {
                    //control linear x,y cmd_vel 
                    ROS_INFO("Moving Forward");
                    ROS_INFO("current_path_idx: %d", current_path_idx);
                    double error_eps=0.0;

                    if(current_path_idx ==subgoals.poses.size()-1)
                    {
                        IsFinalSubgoal=true;
                        error_eps=0.2;
                    }
                    else
                        error_eps=0.5;

                    if(checkDistance(global_pose,navtarget_pose,error_eps))
                    {
                        ROS_INFO("====sub goal acheived======");
                        //check curent idx
                        if(!IsFinalSubgoal) //if it is not final subgoal , update path index
                        {
                            ROS_INFO("========Not Final Subgoal=========");
                            current_path_idx++;
                            navtarget_pose[0]=subgoals.poses[current_path_idx].position.x;
                            navtarget_pose[1]=subgoals.poses[current_path_idx].position.y;
                            IsRotated=false;

                            if(current_path_idx ==subgoals.poses.size() )
                                IsFinalSubgoal=true;

                            return;
                        }
                        else{//if it if final goal
                         
                            //if(checkDistance(global_pose,navtarget_pose,0.25))
                            //{
                                ROS_INFO("====Total goal acheived======");
                                IsGoal=true;
                                IsActive=false;
                                feedback_.is_possible_go=true;
                                Sending_zerovelcmd();
                                result_.success=feedback_.is_possible_go;
                                as_.setSucceeded(result_);
                            //}
                            //else{
                                 //std::vector<double> gains(2,0.0);
                                //get_cmd_gains(global_pose,navtarget_pose,gains);
                                //geometry_msgs::Twist vel_cmd;
                                //vel_cmd.linear.x=0.5*gains[0];
                                //vel_cmd.linear.y=0.5*gains[1];
                                //vel_cmd.linear.z=0.0;
                                //vel_cmd.angular.x=0.0;
                                //vel_cmd.angular.y=0.0;
                                //vel_cmd.angular.z=0.0;
                                //cmd_velocity_pub.publish(vel_cmd);
                            
                            //}

                        }

                        return;
                    }

                    if(!isObstacle)
                    {
                        ROS_INFO("NO OBS in front of the robot, robot alligned");
                        std::vector<double> gains(2,0.0);
                        get_cmd_gains(global_pose,navtarget_pose,gains);
                        geometry_msgs::Twist vel_cmd;
                        vel_cmd.linear.x=gains[0];
                        vel_cmd.linear.y=0.75*gains[1];
                        vel_cmd.linear.z=0.0;
                        vel_cmd.angular.x=0.0;
                        vel_cmd.angular.y=0.0;
                        vel_cmd.angular.z=0.0;
                        cmd_velocity_pub.publish(vel_cmd);
                        //IsObs.data=isObstacle;
                        //fast_occ_pub.publish(IsObs);
                    }
                    else
                    {
                        //IsObs.data=isObstacle;
                        //fast_occ_pub.publish(IsObs);
                        ROS_INFO("Front is occupied");

                        double cur_time = ros::Time::now().toSec();
                        double time_diff= cur_time-srv_time;
                               
                        if(time_diff<5.0)
                            return;
                        else{
                            srv_time=ros::Time::now().toSec();
                        }

                        ROS_INFO("Front is occupied");
                        //IsActive=false;
                        return;
                    
                    }
                }

            }
        //}
    }

    void replanning(int& goal_idx)
    {
        nav_msgs::GetPlan plan_srv;
        plan_srv.request.start=m_pose;
        plan_srv.request.goal=m_goalpose;
        plan_srv.request.tolerance=0.5;
        double total_distance_to_goal=sqrt(pow((m_pose.pose.position.x-m_goalpose.pose.position.x),2)+pow((m_pose.pose.position.y-m_goalpose.pose.position.y),2));
        int num_subgoals=NUM_SUBS;
        if(total_distance_to_goal>10.0)
            num_subgoals=NUM_SUBS+int((total_distance_to_goal-10.0)/2.0);
        else if(total_distance_to_goal<5.0)
            num_subgoals=1;

        if(planner_srv_client.call(plan_srv))
        {
            subgoals.poses.clear();
            int path_length =plan_srv.response.plan.poses.size();
            int path_const = floor(path_length/num_subgoals);
            subgoals.poses.clear();
            subgoals.poses.push_back(plan_srv.response.plan.poses[0].pose);
            for(int i(0);i<(num_subgoals-1);i++)
            {
                int pose_idx=(i+1)*path_const;
                subgoals.poses.push_back(plan_srv.response.plan.poses[pose_idx].pose);
            }
            subgoals.poses.push_back(plan_srv.response.plan.poses[path_length-1].pose);
            goal_idx=0;
            ROS_INFO("Replanning Succeeded! ---Subgoal--Size: %d", subgoals.poses.size());

        }
        else{
            ROS_INFO("replanning failed");
        }


    }



  
protected:
    
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<state_lattice_planner::LocalNaviAction> as_;
  ros::ServiceClient planner_srv_client;
  std::string action_name_;
  int data_count_, goal_;
  float sum_, sum_sq_;
  state_lattice_planner::LocalNaviFeedback feedback_;
  state_lattice_planner::LocalNaviResult result_;
  ros::Subscriber globalpose_sub;
  ros::Subscriber odom_sub;
  ros::Subscriber target_sub;
  ros::Subscriber dynamicmap_sub;
  ros::Subscriber global_map_sub;
  ros::Publisher cmd_velocity_pub;
  ros::Publisher subgoal_pub;
  ros::Publisher fast_occ_pub;
  ros::Publisher Scaled_dynamic_map_pub;
  int direction_z;

  geometry_msgs::PoseStamped m_pose;
  geometry_msgs::PoseStamped m_goalpose;
  geometry_msgs::PoseArray subgoals;
  nav_msgs::OccupancyGrid Scaled_dynamic_map;
  nav_msgs::OccupancyGrid dynamic_map;
  nav_msgs::OccupancyGrid global_map;
  tf::TransformListener   listener;

  std::vector<double> navtarget_pose;
  std::vector<double> global_pose;
  //std::vector<int> checklist_mapidxset;
  std::map<int,bool> checklistmap;
  std::vector<bool> checklistbool;
  std::vector<geometry_msgs::PointStamped> PointSet;
  //ros::ServiceClient m_client;
  int split_size;
  bool IsGoal;
  bool IsRotated;
  bool IsActive;
  bool IsFinalSubgoal;
  double srv_time; 
  int current_path_idx;
  bool local_map_updated;
  bool global_map_updated;
  bool global_pose_updated;

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener* tf2_listener; 
  geometry_msgs::TransformStamped map_en_to_map;
  geometry_msgs::TransformStamped map_to_base;
  int callback_iter;
public:

  StateLatticePlannerROS* pslp_planner;

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "localnavi_server");
  SLPlanner sl_planner(ros::this_node::getName());
  StateLatticePlannerROS planner;
  sl_planner.pslp_planner= &planner;
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
