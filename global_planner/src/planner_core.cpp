/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/
#include <global_planner/planner_core.h>
#include <pluginlib/class_list_macros.h>
#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>

#include <global_planner/dijkstra.h>
#include <global_planner/astar.h>
#include <global_planner/grid_path.h>
#include <global_planner/gradient_path.h>
#include <global_planner/quadratic_calculator.h>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(global_planner::GlobalPlanner, nav_core::BaseGlobalPlanner)

namespace global_planner {

void GlobalPlanner::outlineMap(unsigned char* costarr, int nx, int ny, unsigned char value) {
    unsigned char* pc = costarr;
    for (int i = 0; i < nx; i++)
        *pc++ = value;
    pc = costarr + (ny - 1) * nx;
    for (int i = 0; i < nx; i++)
        *pc++ = value;
    pc = costarr;
    for (int i = 0; i < ny; i++, pc += nx)
        *pc = value;
    pc = costarr + nx - 1;
    for (int i = 0; i < ny; i++, pc += nx)
        *pc = value;
}

GlobalPlanner::GlobalPlanner() :
        costmap_(NULL), initialized_(false), allow_unknown_(true) {
}

GlobalPlanner::GlobalPlanner(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id) :
        costmap_(NULL), initialized_(false), allow_unknown_(true) {
    //initialize the planner
    initialize(name, costmap, frame_id);
}

GlobalPlanner::~GlobalPlanner() {
    if (p_calc_)
        delete p_calc_;
    if (planner_)
        delete planner_;
    if (path_maker_)
        delete path_maker_;
    if (dsrv_)
        delete dsrv_;
}

void GlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
    initialize(name, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());
}

void GlobalPlanner::initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id) {
    if (!initialized_) {
        ros::NodeHandle private_nh("~/" + name);
        costmap_ = costmap;
        frame_id_ = frame_id;

        unsigned int cx = costmap->getSizeInCellsX(), cy = costmap->getSizeInCellsY();

        private_nh.param("old_navfn_behavior", old_navfn_behavior_, false);
        if(!old_navfn_behavior_)
            convert_offset_ = 0.5;
        else
            convert_offset_ = 0.0;

        bool use_quadratic;
        private_nh.param("use_quadratic", use_quadratic, true);
        if (use_quadratic)
            p_calc_ = new QuadraticCalculator(cx, cy);
        else
            p_calc_ = new PotentialCalculator(cx, cy);

        bool use_dijkstra;
        private_nh.param("use_dijkstra", use_dijkstra, true);
        if (use_dijkstra)
        {
            DijkstraExpansion* de = new DijkstraExpansion(p_calc_, cx, cy);
            if(!old_navfn_behavior_)
                de->setPreciseStart(true);
            planner_ = de;
        }
        else
            planner_ = new AStarExpansion(p_calc_, cx, cy);

        bool use_grid_path;
        private_nh.param("use_grid_path", use_grid_path, false);
        if (use_grid_path)
            path_maker_ = new GridPath(p_calc_);
        else
            path_maker_ = new GradientPath(p_calc_);

        orientation_filter_ = new OrientationFilter();

        plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
        plans_pub_ = private_nh.advertise<visualization_msgs::MarkerArray>("plans", 1);
        potential_pub_ = private_nh.advertise<nav_msgs::OccupancyGrid>("potential", 1);

        private_nh.param("allow_unknown", allow_unknown_, true);
        planner_->setHasUnknown(allow_unknown_);
        private_nh.param("planner_window_x", planner_window_x_, 0.0);
        private_nh.param("planner_window_y", planner_window_y_, 0.0);
        private_nh.param("default_tolerance", default_tolerance_, 0.0);
        private_nh.param("publish_scale", publish_scale_, 100);
        private_nh.param("outline_map", outline_map_, true);

        make_plan_srv_ = private_nh.advertiseService("make_plan", &GlobalPlanner::makePlanService, this);
        make_multiplan_srv_ = private_nh.advertiseService("make_multiplan", &GlobalPlanner::makeMultiPlanService, this);
        make_multimultiplan_srv_ = private_nh.advertiseService("make_multimultiplan", &GlobalPlanner::makeMultiMultiPlanService, this);

        dsrv_ = new dynamic_reconfigure::Server<global_planner::GlobalPlannerConfig>(ros::NodeHandle("~/" + name));
        dynamic_reconfigure::Server<global_planner::GlobalPlannerConfig>::CallbackType cb = boost::bind(
                &GlobalPlanner::reconfigureCB, this, _1, _2);
        dsrv_->setCallback(cb);

        initialized_ = true;
    } else
        ROS_WARN("This planner has already been initialized, you can't call it twice, doing nothing");

}

void GlobalPlanner::reconfigureCB(global_planner::GlobalPlannerConfig& config, uint32_t level) {
    planner_->setLethalCost(config.lethal_cost);
    path_maker_->setLethalCost(config.lethal_cost);
    planner_->setNeutralCost(config.neutral_cost);
    planner_->setFactor(config.cost_factor);
    publish_potential_ = config.publish_potential;
    orientation_filter_->setMode(config.orientation_mode);
    orientation_filter_->setWindowSize(config.orientation_window_size);
}

void GlobalPlanner::clearRobotCell(const geometry_msgs::PoseStamped& global_pose, unsigned int mx, unsigned int my) {
    if (!initialized_) {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }

    //set the associated costs in the cost map to be free
    costmap_->setCost(mx, my, costmap_2d::FREE_SPACE);
}

bool GlobalPlanner::makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp) {
    makePlan(req.start, req.goal, resp.plan.poses);

    resp.plan.header.stamp = ros::Time::now();
    resp.plan.header.frame_id = frame_id_;

    return true;
}

bool GlobalPlanner::makeMultiPlanService(nav_msgs::GetMultiPlan::Request& req, nav_msgs::GetMultiPlan::Response& resp) {
    makeMultiPlan(req.start, req.goals, resp.multiplan.poses);
    publishMultiPlans( resp.multiplan.poses);

    resp.multiplan.header.stamp = ros::Time::now();
    resp.multiplan.header.frame_id = frame_id_;

    return true;
}


bool GlobalPlanner::makeMultiMultiPlanService(nav_msgs::GetMultiMultiPlan::Request& req, nav_msgs::GetMultiMultiPlan::Response& resp){

    makeMultiPlan(req.start, req.goals, resp.multiplan.poses);
    makeMultiPlan(req.start2, req.goals2, resp.multiplan2.poses);
    publishMultiPlans( resp.multiplan.poses, resp.multiplan2.poses);

    resp.multiplan.header.stamp = ros::Time::now();
    resp.multiplan.header.frame_id = frame_id_;

    resp.multiplan2.header.stamp = ros::Time::now();
    resp.multiplan2.header.frame_id = frame_id_;

    return true;
}


void GlobalPlanner::mapToWorld(double mx, double my, double& wx, double& wy) {
    wx = costmap_->getOriginX() + (mx+convert_offset_) * costmap_->getResolution();
    wy = costmap_->getOriginY() + (my+convert_offset_) * costmap_->getResolution();
}

bool GlobalPlanner::worldToMap(double wx, double wy, double& mx, double& my) {
    double origin_x = costmap_->getOriginX(), origin_y = costmap_->getOriginY();
    double resolution = costmap_->getResolution();
    //ROS_INFO("wx: %.2lf, wy: %.2lf, origin_x: %.2lf, origin_y: %.2lf ", wx, wy, origin_x, origin_y);
    //ROS_INFO("costmap resolution: %.2lf", resolution);
    //std::cout<<"wx, wy, origin_x, origin_y"<<wx << wy << origin_x << origin_y <<std::endl;

    if (wx < origin_x || wy < origin_y)
        return false;

    mx = (wx - origin_x) / resolution - convert_offset_;
    my = (wy - origin_y) / resolution - convert_offset_;

    if (mx < costmap_->getSizeInCellsX() && my < costmap_->getSizeInCellsY())
        return true;

    return false;
}

bool GlobalPlanner::makeMultiPlan(const geometry_msgs::PoseStamped& start, const std::vector<geometry_msgs::PoseStamped>& goals,
                std::vector<geometry_msgs::PoseArray>& plans){
    return makeMultiPlan(start, goals, default_tolerance_, plans);
}

bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                           std::vector<geometry_msgs::PoseStamped>& plan) {
    return makePlan(start, goal, default_tolerance_, plan);
}

bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                           double tolerance, std::vector<geometry_msgs::PoseStamped>& plan) {
    boost::mutex::scoped_lock lock(mutex_);
    if (!initialized_) {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return false;
    }

    //clear the plan, just in case
    plan.clear();

    ros::NodeHandle n;
    std::string global_frame = frame_id_;

    //until tf can handle transforming things that are way in the past... we'll require the goal to be in our global frame
    if (goal.header.frame_id != global_frame) {
        ROS_ERROR(
                "The goal pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", global_frame.c_str(), goal.header.frame_id.c_str());
        return false;
    }

    if (start.header.frame_id != global_frame) {
        ROS_ERROR(
                "The start pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", global_frame.c_str(), start.header.frame_id.c_str());
        return false;
    }

    double wx = start.pose.position.x;
    double wy = start.pose.position.y;

    unsigned int start_x_i, start_y_i, goal_x_i, goal_y_i;
    double start_x, start_y, goal_x, goal_y;

    //ROS_INFO("wx, wy: ", wx, wy);

    if (!costmap_->worldToMap(wx, wy, start_x_i, start_y_i)) {
        ROS_WARN(
                "The robot's start position is off the global costmap. Planning will always fail, are you sure the robot has been properly localized?");
        //ROS_INFO("wx, wy: ", wx, wy);
        //ROS_INFO("start_x_i, start_y_i: ", start_x_i, start_y_i);
        return false;
    }
    if(old_navfn_behavior_){
        start_x = start_x_i;
        start_y = start_y_i;
    }else{
        worldToMap(wx, wy, start_x, start_y);
    }

    wx = goal.pose.position.x;
    wy = goal.pose.position.y;

    if (!costmap_->worldToMap(wx, wy, goal_x_i, goal_y_i)) {
        ROS_WARN_THROTTLE(1.0,
                "The goal sent to the global planner is off the global costmap. Planning will always fail to this goal.");
        return false;
    }
    if(old_navfn_behavior_){
        goal_x = goal_x_i;
        goal_y = goal_y_i;
    }else{
        worldToMap(wx, wy, goal_x, goal_y);
    }

    //clear the starting cell within the costmap because we know it can't be an obstacle
    clearRobotCell(start, start_x_i, start_y_i);

    int nx = costmap_->getSizeInCellsX(), ny = costmap_->getSizeInCellsY();

    //make sure to resize the underlying array that Navfn uses
    p_calc_->setSize(nx, ny);
    planner_->setSize(nx, ny);
    path_maker_->setSize(nx, ny);
    potential_array_ = new float[nx * ny];

    if(outline_map_)
        outlineMap(costmap_->getCharMap(), nx, ny, costmap_2d::LETHAL_OBSTACLE);

    bool found_legal = planner_->calculatePotentials(costmap_->getCharMap(), start_x, start_y, goal_x, goal_y,
                                                    nx * ny * 2, potential_array_);

    if(!old_navfn_behavior_)
        planner_->clearEndpoint(costmap_->getCharMap(), potential_array_, goal_x_i, goal_y_i, 2);
    if(publish_potential_)
        publishPotential(potential_array_);

    if (found_legal) {
        //extract the plan
        if (getPlanFromPotential(start_x, start_y, goal_x, goal_y, goal, plan)) {
            //make sure the goal we push on has the same timestamp as the rest of the plan
            geometry_msgs::PoseStamped goal_copy = goal;
            goal_copy.header.stamp = ros::Time::now();
            plan.push_back(goal_copy);
        } else {
            ROS_ERROR("Failed to get a plan from potential when a legal potential was found. This shouldn't happen.");
        }
    }else{
        ROS_ERROR("Failed to get a plan.");
    }

    // add orientations if needed
    orientation_filter_->processPath(start, plan);

    //publish the plan for visualization purposes
    publishPlan(plan);
    delete potential_array_;
    return !plan.empty();
}

bool GlobalPlanner::makeMultiPlan(const geometry_msgs::PoseStamped& start, const std::vector<geometry_msgs::PoseStamped>& goals,
                           double tolerance, std::vector<geometry_msgs::PoseArray>& plans) {
    boost::mutex::scoped_lock lock(mutex_);
    if (!initialized_) {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return false;
    }

    //clear the plan, just in case
    plans.clear();
    //plans.resize(goals.size());
    //plan.clear();

    ros::NodeHandle n;
    std::string global_frame = frame_id_;

    //until tf can handle transforming things that are way in the past... we'll require the goal to be in our global frame
    for(size_t i(0); i<goals.size(); i++)
    {
        if (goals[i].header.frame_id != global_frame) {
            ROS_ERROR(
                    "The goal pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", global_frame.c_str(), goals[i].header.frame_id.c_str());
            return false;
        }
    }

    if (start.header.frame_id != global_frame) {
        ROS_ERROR(
                "The start pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", global_frame.c_str(), start.header.frame_id.c_str());
        return false;
    }

    double wx = start.pose.position.x;
    double wy = start.pose.position.y;

    unsigned int start_x_i, start_y_i, goal_x_i, goal_y_i;
    double start_x, start_y, goal_x, goal_y;

    if (!costmap_->worldToMap(wx, wy, start_x_i, start_y_i)) {
        ROS_WARN(
                "The robot's start position is off the global costmap. Planning will always fail, are you sure the robot has been properly localized?");
        return false;
    }
    if(old_navfn_behavior_){
        start_x = start_x_i;
        start_y = start_y_i;
    }else{
        worldToMap(wx, wy, start_x, start_y);
    }

    //ROS_INFO("wx: %.2lf, wy: %.2lf", wx, wy);
    //ROS_INFO("start_x_i: %.2lf, start_y_i: %.2lf", start_x_i, start_y_i);
    //ROS_INFO("goal size: %d", goals.size());


    for(size_t i(0); i<goals.size(); i++){

        wx = goals[i].pose.position.x;
        wy = goals[i].pose.position.y;

        if (!costmap_->worldToMap(wx, wy, goal_x_i, goal_y_i)) {
            ROS_WARN_THROTTLE(1.0,
                    "The goal sent to the global planner is off the global costmap. Planning will always fail to this goal.");
            return false;
        }
        if(old_navfn_behavior_){
            goal_x = goal_x_i;
            goal_y = goal_y_i;
        }else{
            worldToMap(wx, wy, goal_x, goal_y);
        }

        //clear the starting cell within the costmap because we know it can't be an obstacle
        clearRobotCell(start, start_x_i, start_y_i);

        int nx = costmap_->getSizeInCellsX(), ny = costmap_->getSizeInCellsY();

        //make sure to resize the underlying array that Navfn uses
        p_calc_->setSize(nx, ny);
        planner_->setSize(nx, ny);
        path_maker_->setSize(nx, ny);
        potential_array_ = new float[nx * ny];

        if(outline_map_)
            outlineMap(costmap_->getCharMap(), nx, ny, costmap_2d::LETHAL_OBSTACLE);

        bool found_legal = planner_->calculatePotentials(costmap_->getCharMap(), start_x, start_y, goal_x, goal_y,
                                                        nx * ny * 2, potential_array_);

        if(!old_navfn_behavior_)
            planner_->clearEndpoint(costmap_->getCharMap(), potential_array_, goal_x_i, goal_y_i, 2);
        if(publish_potential_)
            publishPotential(potential_array_);

        geometry_msgs::PoseArray temp_path;
        if (found_legal) {
            //extract the plan
            if (getPlanFromPotential_posearray(start_x, start_y, goal_x, goal_y, goals[i], temp_path)) {
                //ROS_INFO("temp_path's size : %d", temp_path.poses.size());
                //make sure the goal we push on has the same timestamp as the rest of the plan
                //geometry_msgs::PoseStamped goal_copy = goals[i];
                geometry_msgs::Pose goal_copy = goals[i].pose;
                //goal_copy.header.stamp = ros::Time::now();
                temp_path.poses.push_back(goal_copy);
                plans.push_back(temp_path);
            } else {
                ROS_ERROR("Failed to get a plan from potential when a legal potential was found. This shouldn't happen.");
            }
        }else{
            ROS_ERROR("Failed to get a plan.");
        }

        // add orientations if needed
        //orientation_filter_->processPath(start, plans[i]);

        delete potential_array_;
    }

    //publish the plan for visualization purposes
    publishMultiPlans(plans);

    return !plans.empty();
}

void GlobalPlanner::publishMultiPlans(const std::vector<geometry_msgs::PoseArray>& paths) {
    if (!initialized_) {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }

    visualization_msgs::MarkerArray paths_marker;
    paths_marker.markers.clear();
    //ROS_INFO("total path size: %d",paths.size());
    int pt_idx=0;

    for(int i(0);i<paths.size();i++)
        {
            //ROS_INFO("each path size: %d",paths[i].poses.size());
            for(int j(0); j< paths[i].poses.size(); j++){
                if(j%25==0)
                {
                    //visualization_marker
                    visualization_msgs::Marker markers_point;

                    markers_point.header.frame_id = "/map"; 
                    markers_point.header.stamp = ros::Time::now();
                    markers_point.ns = "/paths_point";
                    markers_point.id = pt_idx++;

                    uint32_t shape = visualization_msgs::Marker::SPHERE;
                    markers_point.type = shape;

                    markers_point.pose.position.x = paths[i].poses[j].position.x;
                    markers_point.pose.position.y = paths[i].poses[j].position.y;
                    markers_point.pose.position.z = 1;

                    markers_point.pose.orientation.x = 0.0;
                    markers_point.pose.orientation.y = 0.0;
                    markers_point.pose.orientation.z = 0.0;
                    markers_point.pose.orientation.w = 1.0;

                    double temp_dist=0.5;
                    markers_point.scale.x = std::abs(temp_dist);
                    markers_point.scale.y = std::abs(temp_dist);
                    markers_point.scale.z = std::abs(temp_dist);

                    markers_point.color.r = 0.13;
                    markers_point.color.g = 0.8;
                    markers_point.color.b = 0.5;
                    markers_point.color.a = 0.85;
                    paths_marker.markers.push_back(markers_point);
                }
            }
        }

      //ROS_INFO("paths_marker_size: %d", paths_marker.markers.size());
      plans_pub_.publish(paths_marker);

     //Extract the plan in world co-ordinates, we assume the path is all in the same frame
    //for (unsigned int i = 0; i < path.size(); i++) {
        //gui_path.poses[i] = path[i];
    //}

    //plan_pub_.publish(gui_path);
}

void GlobalPlanner::publishMultiPlans(const std::vector<geometry_msgs::PoseArray>& paths, const std::vector<geometry_msgs::PoseArray>& paths2) {
    if (!initialized_) {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }

    visualization_msgs::MarkerArray paths_marker;
    paths_marker.markers.clear();
    ROS_INFO("total path size: %d",paths.size());
    int pt_idx=0;

    for(int i(0);i<paths.size();i++)
        {
            ROS_INFO("each path size: %d",paths[i].poses.size());
            for(int j(0); j< paths[i].poses.size(); j++){
                if(j%25==0)
                {
                    //visualization_marker
                    visualization_msgs::Marker markers_point;

                    markers_point.header.frame_id = "/map"; 
                    markers_point.header.stamp = ros::Time::now();
                    markers_point.ns = "/paths_point";
                    markers_point.id = pt_idx++;

                    uint32_t shape = visualization_msgs::Marker::SPHERE;
                    markers_point.type = shape;

                    markers_point.pose.position.x = paths[i].poses[j].position.x;
                    markers_point.pose.position.y = paths[i].poses[j].position.y;
                    markers_point.pose.position.z = 1;

                    markers_point.pose.orientation.x = 0.0;
                    markers_point.pose.orientation.y = 0.0;
                    markers_point.pose.orientation.z = 0.0;
                    markers_point.pose.orientation.w = 1.0;

                    double temp_dist=0.5;
                    markers_point.scale.x = std::abs(temp_dist);
                    markers_point.scale.y = std::abs(temp_dist);
                    markers_point.scale.z = std::abs(temp_dist);

                    markers_point.color.r = 0.13;
                    markers_point.color.g = 0.9;
                    markers_point.color.b = 0.1;
                    markers_point.color.a = 0.65;
                    paths_marker.markers.push_back(markers_point);
                }
            }
        }

    for(int i(0);i<paths2.size();i++)
        {
            //ROS_INFO("each path size: %d",paths2[i].poses.size());
            for(int j(0); j< paths2[i].poses.size(); j++){
                if(j%25==0)
                {
                    //visualization_marker
                    visualization_msgs::Marker markers_point;

                    markers_point.header.frame_id = "/map"; 
                    markers_point.header.stamp = ros::Time::now();
                    markers_point.ns = "/paths_point";
                    markers_point.id = pt_idx++;

                    uint32_t shape = visualization_msgs::Marker::SPHERE;
                    markers_point.type = shape;

                    markers_point.pose.position.x = paths2[i].poses[j].position.x;
                    markers_point.pose.position.y = paths2[i].poses[j].position.y;
                    markers_point.pose.position.z = 1;

                    markers_point.pose.orientation.x = 0.0;
                    markers_point.pose.orientation.y = 0.0;
                    markers_point.pose.orientation.z = 0.0;
                    markers_point.pose.orientation.w = 1.0;

                    double temp_dist=0.5;
                    markers_point.scale.x = std::abs(temp_dist);
                    markers_point.scale.y = std::abs(temp_dist);
                    markers_point.scale.z = std::abs(temp_dist);

                    markers_point.color.r = 0.83;
                    markers_point.color.g = 0.1;
                    markers_point.color.b = 0.2;
                    markers_point.color.a = 0.65;
                    paths_marker.markers.push_back(markers_point);
                }
            }
        }

      //ROS_INFO("paths_marker_size: %d", paths_marker.markers.size());
      plans_pub_.publish(paths_marker);

     //Extract the plan in world co-ordinates, we assume the path is all in the same frame
    //for (unsigned int i = 0; i < path.size(); i++) {
        //gui_path.poses[i] = path[i];
    //}

    //plan_pub_.publish(gui_path);
}





void GlobalPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path) {
    if (!initialized_) {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }

    //create a message for the plan
    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());

    gui_path.header.frame_id = frame_id_;
    gui_path.header.stamp = ros::Time::now();

    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for (unsigned int i = 0; i < path.size(); i++) {
        gui_path.poses[i] = path[i];
    }

    plan_pub_.publish(gui_path);
}

bool GlobalPlanner::getPlanFromPotential(double start_x, double start_y, double goal_x, double goal_y,
                                      const geometry_msgs::PoseStamped& goal,
                                       std::vector<geometry_msgs::PoseStamped>& plan) {
    if (!initialized_) {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return false;
    }

    std::string global_frame = frame_id_;

    //clear the plan, just in case
    plan.clear();

    std::vector<std::pair<float, float> > path;

    if (!path_maker_->getPath(potential_array_, start_x, start_y, goal_x, goal_y, path)) {
        ROS_ERROR("NO PATH!");
        return false;
    }

    ros::Time plan_time = ros::Time::now();
    for (int i = path.size() -1; i>=0; i--) {
        std::pair<float, float> point = path[i];
        //convert the plan to world coordinates
        double world_x, world_y;
        mapToWorld(point.first, point.second, world_x, world_y);

        geometry_msgs::PoseStamped pose;
        pose.header.stamp = plan_time;
        pose.header.frame_id = global_frame;
        pose.pose.position.x = world_x;
        pose.pose.position.y = world_y;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;
        plan.push_back(pose);
    }
    if(old_navfn_behavior_){
            plan.push_back(goal);
    }
    return !plan.empty();
}

bool GlobalPlanner::getPlanFromPotential_posearray(double start_x, double start_y, double goal_x, double goal_y,
                                      const geometry_msgs::PoseStamped& goal,
                                      geometry_msgs::PoseArray& plan) {
    if (!initialized_) {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return false;
    }

    std::string global_frame = frame_id_;

    //clear the plan, just in case
    plan.poses.clear();

    std::vector<std::pair<float, float> > path;

    if (!path_maker_->getPath(potential_array_, start_x, start_y, goal_x, goal_y, path)) {
        ROS_ERROR("NO PATH!");
        return false;
    }

    ros::Time plan_time = ros::Time::now();
    for (int i = path.size() -1; i>=0; i--) {
        std::pair<float, float> point = path[i];
        //convert the plan to world coordinates
        double world_x, world_y;
        mapToWorld(point.first, point.second, world_x, world_y);

        geometry_msgs::Pose pose;
        pose.position.x = world_x;
        pose.position.y = world_y;
        pose.position.z = 0.0;
        pose.orientation.x = 0.0;
        pose.orientation.y = 0.0;
        pose.orientation.z = 0.0;
        pose.orientation.w = 1.0;
        plan.poses.push_back(pose);
        //ROS_INFO("pos_x: %.2lf, pos_y: %.2lf", world_x, world_y);
    }
    if(old_navfn_behavior_){
            plan.poses.push_back(goal.pose);
    }
    return !plan.poses.empty();
}



void GlobalPlanner::publishPotential(float* potential)
{
    int nx = costmap_->getSizeInCellsX(), ny = costmap_->getSizeInCellsY();
    double resolution = costmap_->getResolution();
    nav_msgs::OccupancyGrid grid;
    // Publish Whole Grid
    grid.header.frame_id = frame_id_;
    grid.header.stamp = ros::Time::now();
    grid.info.resolution = resolution;

    grid.info.width = nx;
    grid.info.height = ny;

    double wx, wy;
    costmap_->mapToWorld(0, 0, wx, wy);
    grid.info.origin.position.x = wx - resolution / 2;
    grid.info.origin.position.y = wy - resolution / 2;
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation.w = 1.0;

    grid.data.resize(nx * ny);

    float max = 0.0;
    for (unsigned int i = 0; i < grid.data.size(); i++) {
        float potential = potential_array_[i];
        if (potential < POT_HIGH) {
            if (potential > max) {
                max = potential;
            }
        }
    }

    for (unsigned int i = 0; i < grid.data.size(); i++) {
        if (potential_array_[i] >= POT_HIGH) {
            grid.data[i] = -1;
        } else
            grid.data[i] = potential_array_[i] * publish_scale_ / max;
    }
    potential_pub_.publish(grid);
}

} //end namespace global_planner
