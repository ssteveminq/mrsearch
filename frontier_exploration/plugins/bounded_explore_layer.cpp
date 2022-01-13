#include <frontier_exploration/bounded_explore_layer.h>

#include <pluginlib/class_list_macros.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseArray.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/footprint.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>

#include <frontier_exploration/Frontier.h>
#include <frontier_exploration/UpdateBoundaryPolygon.h>
#include <frontier_exploration/GetNextFrontier.h>
#include <frontier_exploration/BlacklistPoint.h>
#include <frontier_exploration/frontier_search.h>
#include <frontier_exploration/geometry_tools.h>

PLUGINLIB_EXPORT_CLASS(frontier_exploration::BoundedExploreLayer, costmap_2d::Layer)

namespace frontier_exploration
{

    using costmap_2d::LETHAL_OBSTACLE;
    using costmap_2d::NO_INFORMATION;
    using costmap_2d::FREE_SPACE;

    BoundedExploreLayer::BoundedExploreLayer():marker_seq(0), tf_listener_(buffer){}

    BoundedExploreLayer::~BoundedExploreLayer(){
        polygonService_.shutdown();
        frontierService_.shutdown();
        delete dsrv_;
        dsrv_ = 0;
    }

    void BoundedExploreLayer::onInitialize(){

        ros::NodeHandle nh_("~/" + name_);
        frontier_cloud_pub = nh_.advertise<sensor_msgs::PointCloud2>("frontiers",5);
        blacklist_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("blacklist", 5);
        frontier_marker_pub = nh_.advertise<visualization_msgs::MarkerArray>("frontier_list", 5);
        configured_ = false;
        marked_ = false;

        bool explore_clear_space;
        nh_.param("explore_clear_space", explore_clear_space, true);
        if(explore_clear_space){
            default_value_ = NO_INFORMATION;
        }else{
            default_value_ = FREE_SPACE;
        }

        matchSize();
        last_markers_count_=0;

        blacklist_radius_=0.2;
        nh_.param<bool>("resize_to_boundary", resize_to_boundary_, true);
        //nh_.param<std::string>("frontier_travel_point", frontier_travel_point_, "closest");
        nh_.param<std::string>("frontier_travel_point", frontier_travel_point_, "middle");
        nh_.param<int>("min_frontier_size", min_frontier_size_, 1);
        //added by mk
        //nh_.param<double>("potential_scale", potential_scale_, 1e-3);
        nh_.param<double>("potential_scale", potential_scale_, 1e-3);
        nh_.param<double>("gain_scale", gain_scale_, 1.0);
        nh_.param<double>("blacklist_radius",blacklist_radius_, 0.2);
        //nh_.param<double>("gain_scale", gain_scale_, 1.0);


        polygonService_ = nh_.advertiseService("update_boundary_polygon", &BoundedExploreLayer::updateBoundaryPolygonService, this);
        frontierService_ = nh_.advertiseService("get_next_frontier", &BoundedExploreLayer::getNextFrontierService, this);
        blacklistPointService_ = nh_.advertiseService("blacklist_point", &BoundedExploreLayer::blacklistPointService, this);
        clearBlacklistService_ = nh_.advertiseService("clear_blacklist", &BoundedExploreLayer::clearBlacklistService, this);

        dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh_);
        dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
                    &BoundedExploreLayer::reconfigureCB, this, _1, _2);
        dsrv_->setCallback(cb);

    }


    void BoundedExploreLayer::matchSize(){
        Costmap2D* master = layered_costmap_->getCostmap();
        resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
                  master->getOriginX(), master->getOriginY());
    }


    void BoundedExploreLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level){
        enabled_ = config.enabled;
    }

    bool BoundedExploreLayer::getNextFrontierService(frontier_exploration::GetNextFrontier::Request &req, frontier_exploration::GetNextFrontier::Response &res){
        return getNextFrontier(req.start_pose, res.next_frontier);
    }

    bool BoundedExploreLayer::getNextFrontier(geometry_msgs::PoseStamped start_pose, geometry_msgs::PoseStamped &next_frontier){

        ROS_INFO("get next frontier");
        //wait for costmap to get marked with boundary
        ros::Rate r(10);
        while(!marked_){
            ros::spinOnce();
            r.sleep();
        }

        if(start_pose.header.frame_id != layered_costmap_->getGlobalFrameID()){
            //error out if no transform available
            //
            //if(!tf_listener_.waitForTransform(layered_costmap_->getGlobalFrameID(), start_pose.header.frame_id,ros::Time::now(),ros::Duration(10))) {
                //ROS_ERROR_STREAM("Couldn't transform from "<<layered_costmap_->getGlobalFrameID()<<" to "<< start_pose.header.frame_id);
                //return false;
            //}
            geometry_msgs::TransformStamped transformStamped; 
            try{
                transformStamped= buffer.lookupTransform(layered_costmap_->getGlobalFrameID(), start_pose.header.frame_id,ros::Time(0));
            }
            catch (tf2::TransformException &ex)
            {
                ROS_WARN("%s", ex.what());
                ros::Duration(1.0).sleep();
                return false;
            
            }
            //if(!buffer.lookupTransform(layered_costmap_->getGlobalFrameID(), start_pose.header.frame_id,ros::Time::now(),ros::Duration(10))) {
                //ROS_ERROR_STREAM("Couldn't transform from "<<layered_costmap_->getGlobalFrameID()<<" to "<< start_pose.header.frame_id);
                //return false;
            //}
            geometry_msgs::PoseStamped temp_pose = start_pose;
            tf2::doTransform(start_pose, temp_pose, transformStamped);
            //tf_listener_.transformPose(layered_costmap_->getGlobalFrameID(),temp_pose,start_pose);
        }

        //initialize frontier search implementation
        FrontierSearch frontierSearch(*(layered_costmap_->getCostmap()), min_frontier_size_, frontier_travel_point_,potential_scale_,gain_scale_);
        //get list of frontiers from search implementation
        //std::vector<Frontier> frontier_list = frontierSearch.searchFrom(start_pose.pose.position);
        auto frontier_list = frontierSearch.searchFrom(start_pose.pose.position);

        if(frontier_list.size() == 0){
            ROS_INFO("No frontiers found, exploration complete");
            ROS_DEBUG("No frontiers found, exploration complete");
            return false;
        }

        //visualize_frontiers(frontier_list);
        visualizeFrontiers(frontier_list);
        //create placeholder for selected frontier
        Frontier selected;
        selected.min_distance = std::numeric_limits<double>::infinity();

        //pointcloud for visualization purposes
        pcl::PointCloud<pcl::PointXYZI> frontier_cloud_viz;
        pcl::PointXYZI frontier_point_viz(50);
        int max;

        BOOST_FOREACH(Frontier frontier, frontier_list){
            //load frontier into visualization poitncloud
            frontier_point_viz.x = frontier.travel_point.x;
            frontier_point_viz.y = frontier.travel_point.y;
            frontier_cloud_viz.push_back(frontier_point_viz);

            //ROS_INFO("list travel points:  %.3lf, %.3lf", frontier.travel_point.x, frontier.travel_point.y);
            //ROS_INFO("min_distance : %.3lf", frontier.min_distance);
            //check if this frontier is the nearest to robot
            //if (frontier.min_distance < selected.min_distance && !anyPointsNearby(frontier.travel_point, blacklist_, blacklist_radius_)){
            //if (frontier.min_distance < selected.min_distance){
            if (frontier.min_distance < selected.min_distance && !anyPointsNearby(frontier.travel_point, blacklist_, blacklist_radius_)){
                selected = frontier;
                max = frontier_cloud_viz.size()-1;
            }
        }
        //ROS_INFO("max: %d", max);
        //ROS_INFO("list travel points - %.3lf, %.3lf", frontier.travel_point.x, frontier,travel_point.y);

        if (std::isinf(selected.min_distance)) {
            ROS_INFO("No valid (non-blacklisted) frontiers found, exploration complete");
            ROS_DEBUG("No valid (non-blacklisted) frontiers found, exploration complete");
            return false;
        }

        //color selected frontier
        frontier_cloud_viz[max].intensity = 100;

        //publish visualization point cloud-----------------------------
        //sensor_msgs::PointCloud2 frontier_viz_output;
        //pcl::toROSMsg(frontier_cloud_viz,frontier_viz_output);
        //frontier_viz_output.header.frame_id = layered_costmap_->getGlobalFrameID();
        //frontier_viz_output.header.stamp = ros::Time::now();
        //frontier_cloud_pub.publish(frontier_viz_output);
        //---------------------------------------------------------------

        //set goal pose to next frontier
        next_frontier.header.frame_id = layered_costmap_->getGlobalFrameID();
        next_frontier.header.stamp = ros::Time::now();

        next_frontier.pose.position = selected.travel_point;
        tf2::Quaternion q;
        q.setEuler(yawOfVector(start_pose.pose.position, next_frontier.pose.position), 0,0);
        next_frontier.pose.orientation.w=q.getW();
        next_frontier.pose.orientation.x=q.getX();
        next_frontier.pose.orientation.y=q.getY();
        next_frontier.pose.orientation.z=q.getZ();

        //next_frontier.pose.orientation = tf::createQuaternionMsgFromYaw( yawOfVector(start_pose.pose.position, next_frontier.pose.position) );
        return true;

    }

    /*
    //this function is not used - has bug
    bool BoundedExploreLayer::visualize_frontiers(std::vector<Frontier> _frontier_list){
    
        ROS_INFO("visualize frontiers");
    
        visualization_msgs::MarkerArray marker_array;
        visualization_msgs::Marker marker;
        geometry_msgs::Pose pose_;
        marker_seq=0;

        uint32_t shape = visualization_msgs::Marker::SPHERE;

        //for (const auto& elem : _frontier_list){
        BOOST_FOREACH(Frontier frontier, _frontier_list){
            visualization_msgs::Marker marker;
            marker.header.frame_id = layered_costmap_->getGlobalFrameID();
            marker.header.stamp = ros::Time::now();
            marker.header.seq = ++marker_seq;
            marker.ns = "fronteier_marker";
            marker.id = 0;
            marker.type = shape;
            pose_.position.x= frontier.travel_point.x;
            pose_.position.y= frontier.travel_point.y;
            pose_.position.z= 0.5;
            marker.pose=pose_;

            double temp_dist =0.3;
            //marker.scale=geometry_msgs::Vector3(temp_dist,temp_dist,temp_dist);
            marker.scale.x = std::abs(temp_dist);
            marker.scale.y = std::abs(temp_dist);
            marker.scale.z = std::abs(temp_dist);

            marker.color.r = 0.0;
            marker.color.g = 0.2;
            marker.color.b = 0.8;
            marker.color.a = 0.85;

            marker.lifetime = ros::Duration(0);
            marker_array.markers.push_back(marker);
        }
        frontier_marker_pub.publish(marker_array);
    }*/

    void BoundedExploreLayer::visualizeFrontiers(const std::vector<Frontier>& frontiers)
    {
        std_msgs::ColorRGBA blue;
        blue.r = 0;
        blue.g = 0;
        blue.b = 1.0;
        blue.a = 1.0;
        std_msgs::ColorRGBA red;
        red.r = 1.0;
        red.g = 0;
        red.b = 0;
        red.a = 1.0;
        std_msgs::ColorRGBA green;
        green.r = 0;
        green.g = 1.0;
        green.b = 0;
        green.a = 1.0;

        ROS_DEBUG("visualising %lu frontiers", frontiers.size());
        visualization_msgs::MarkerArray markers_msg;
        std::vector<visualization_msgs::Marker>& markers = markers_msg.markers;
        visualization_msgs::Marker m;

        m.header.frame_id =layered_costmap_->getGlobalFrameID();
        m.header.stamp = ros::Time::now();
        m.ns = "frontiers";
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

        // weighted frontiers are always sorted
        double min_cost = frontiers.empty() ? 0. : frontiers.front().cost;
        m.action = visualization_msgs::Marker::ADD;
        size_t id = 0;
        for (auto& frontier : frontiers) {
            m.type = visualization_msgs::Marker::POINTS;
            m.id = int(id);
            m.pose.position = {};
            m.scale.x = 0.1;
            m.scale.y = 0.1;
            m.scale.z = 0.1;
            m.points = frontier.points;
            if (goalOnBlacklist(frontier.centroid)) {
                m.color = red;
            } else {
                m.color = blue;
            }
            markers.push_back(m);
            ++id;
            m.type = visualization_msgs::Marker::SPHERE;
            m.id = int(id);
            m.pose.position = frontier.initial;
            // scale frontier according to its cost (costier frontiers will be smaller)
            double scale = std::min(std::abs(min_cost * 0.4 / frontier.cost), 0.5);
            m.scale.x = scale;
            m.scale.y = scale;
            m.scale.z = scale;
            m.points = {};
            m.color = green;
            markers.push_back(m);
            ++id;
        }
        size_t current_markers_count = markers.size();

        // delete previous markers, which are now unused
        m.action = visualization_msgs::Marker::DELETE;
        for (; id < last_markers_count_; ++id) {
            m.id = int(id);
            markers.push_back(m);
        }

        last_markers_count_ = current_markers_count;
        //marker_array_publisher_.publish(markers_msg);
        frontier_marker_pub.publish(markers_msg);
    }



    bool BoundedExploreLayer::updateBoundaryPolygonService(frontier_exploration::UpdateBoundaryPolygon::Request &req, frontier_exploration::UpdateBoundaryPolygon::Response &res){

        return updateBoundaryPolygon(req.explore_boundary);

    }

    void BoundedExploreLayer::reset(){

        //reset costmap_ char array to default values
        marked_ = false;
        configured_ = false;
        memset(costmap_, default_value_, size_x_ * size_y_ * sizeof(unsigned char));

    }

    bool BoundedExploreLayer::updateBoundaryPolygon(geometry_msgs::PolygonStamped polygon_stamped){

        //clear existing boundary, if any
        polygon_.points.clear();

        //error if no transform available between polygon and costmap
        //if(!tf_listener_.waitForTransform(layered_costmap_->getGlobalFrameID(), polygon_stamped.header.frame_id,ros::Time::now(),ros::Duration(10))) {
            //ROS_ERROR_STREAM("Couldn't transform from "<<layered_costmap_->getGlobalFrameID()<<" to "<< polygon_stamped.header.frame_id);
            //return false;
        //}
        geometry_msgs::TransformStamped transformStamped; 
            try{
                transformStamped= buffer.lookupTransform(layered_costmap_->getGlobalFrameID(), polygon_stamped.header.frame_id,ros::Time(0));
            }
            catch (tf2::TransformException &ex)
            {
                ROS_WARN("%s", ex.what());
                ros::Duration(1.0).sleep();
                return false;
            
            }

        //Transform all points of boundary polygon into costmap frame
        geometry_msgs::PointStamped in, out;
        in.header = polygon_stamped.header;
        BOOST_FOREACH(geometry_msgs::Point32 point32, polygon_stamped.polygon.points){
            in.point = costmap_2d::toPoint(point32);
            tf2::doTransform(in, out, transformStamped);
            //tf_listener_.transformPoint(layered_costmap_->getGlobalFrameID(),in,out);
            polygon_.points.push_back(costmap_2d::toPoint32(out.point));
        }

        //if empty boundary provided, set to whole map
        if(polygon_.points.empty()){
            geometry_msgs::Point32 temp;
            temp.x = getOriginX();
            temp.y = getOriginY();
            polygon_.points.push_back(temp);
            temp.y = getSizeInMetersY();
            polygon_.points.push_back(temp);
            temp.x = getSizeInMetersX();
            polygon_.points.push_back(temp);
            temp.y = getOriginY();
            polygon_.points.push_back(temp);
        }

        if(resize_to_boundary_){
            updateOrigin(0,0);

            //Find map size and origin by finding min/max points of polygon
            double min_x = std::numeric_limits<double>::infinity();
            double min_y = std::numeric_limits<double>::infinity();
            double max_x = -std::numeric_limits<double>::infinity();
            double max_y = -std::numeric_limits<double>::infinity();

            BOOST_FOREACH(geometry_msgs::Point32 point, polygon_.points){
                min_x = std::min(min_x,(double)point.x);
                min_y = std::min(min_y,(double)point.y);
                max_x = std::max(max_x,(double)point.x);
                max_y = std::max(max_y,(double)point.y);
            }

            //resize the costmap to polygon boundaries, don't change resolution
            int size_x, size_y;
            worldToMapNoBounds(max_x - min_x, max_y - min_y, size_x, size_y);
            layered_costmap_->resizeMap(size_x, size_y, layered_costmap_->getCostmap()->getResolution(), min_x, min_y);
            matchSize();
        }

        configured_ = true;
        marked_ = false;
        return true;
    }


    void BoundedExploreLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y){

        //check if layer is enabled and configured with a boundary
        if (!enabled_ || !configured_){ return; }

        //update the whole costmap
        *min_x = getOriginX();
        *min_y = getOriginY();
        *max_x = getSizeInMetersX()+getOriginX();
        *max_y = getSizeInMetersY()+getOriginY();

    }

    void BoundedExploreLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j){
        //check if layer is enabled and configured with a boundary
        if (!enabled_ || !configured_){ return; }

        //draw lines between each point in polygon
        MarkCell marker(costmap_, LETHAL_OBSTACLE);

        //circular iterator
        for(int i = 0, j = polygon_.points.size()-1; i < polygon_.points.size(); j = i++){

            int x_1, y_1, x_2, y_2;
            worldToMapEnforceBounds(polygon_.points[i].x, polygon_.points[i].y, x_1,y_1);
            worldToMapEnforceBounds(polygon_.points[j].x, polygon_.points[j].y, x_2,y_2);

            raytraceLine(marker,x_1,y_1,x_2,y_2);
        }
        //update the master grid from the internal costmap
        mapUpdateKeepObstacles(master_grid, min_i, min_j, max_i, max_j);


    }

    void BoundedExploreLayer::mapUpdateKeepObstacles(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j){
        if (!enabled_)
            return;

        unsigned char* master = master_grid.getCharMap();
        unsigned int span = master_grid.getSizeInCellsX();

        for (int j = min_j; j < max_j; j++)
        {
            unsigned int it = span*j+min_i;
            for (int i = min_i; i < max_i; i++)
            {
                //only update master grid if local costmap cell is lethal/higher value, and is not overwriting a lethal obstacle in the master grid
                if(master[it] != LETHAL_OBSTACLE && (costmap_[it] == LETHAL_OBSTACLE || costmap_[it] > master[it])){
                    master[it] = costmap_[it];
                }
                it++;
            }
        }
        marked_ = true;
    }
    

    bool BoundedExploreLayer::goalOnBlacklist(const geometry_msgs::Point& goal)
    {
        constexpr static size_t tolerace = 5;
        costmap_2d::Costmap2D* costmap2d = layered_costmap_->getCostmap();

        // check if a goal is on the blacklist for goals that we're pursuing
        for (auto& frontier_goal : blacklist_) {
            double x_diff = fabs(goal.x - frontier_goal.x);
            double y_diff = fabs(goal.y - frontier_goal.y);

            if (x_diff < tolerace * costmap2d->getResolution() &&
                    y_diff < tolerace * costmap2d->getResolution())
                return true;
        }
        return false;
    }



    bool BoundedExploreLayer::blacklistPointService(frontier_exploration::BlacklistPoint::Request &req, frontier_exploration::BlacklistPoint::Response &res) {
        // Add point to blacklist
        blacklist_.push_back(req.point);
        ROS_WARN("Blacklist point added %f, %f", req.point.x, req.point.y);

        // Show point in blacklist topic
        visualization_msgs::Marker marker;
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.ns = "blacklist";
        marker.id = blacklist_.size();
        marker.action = visualization_msgs::Marker::ADD;

        //marker.header.frame_id = global_frame_;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();

        marker.pose.position = req.point;
        marker.pose.orientation.w = 1.0;

        // Scale is the diameter of the shape
        marker.scale.x = 2 * blacklist_radius_;
        marker.scale.y = 2 * blacklist_radius_;
        // Circle
        marker.scale.z = 0.05;

        marker.color.r = 1.0;
        marker.color.a = 0.6;

        blacklist_marker_pub_.publish(marker);

        // All is good :)
        return true;
    }

    bool BoundedExploreLayer::clearBlacklistService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp) {
        // Clear the list
        blacklist_.clear();
        ROS_WARN("Blacklist cleared");

        // Delete all markers from visualization
        visualization_msgs::Marker marker;
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.ns = "blacklist";
        // The constant does not exist in ROS Indigo, although functionality is implemented. We use our own.
        marker.action = DELETEALL;

        // All is good :)
        return true;
    }
}
