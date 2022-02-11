#include "plugin_client.h"
#include <actionlib/client/simple_action_client.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include "geometry_tools.h"

#include <search_service/SetSearchRegionAction.h>
#include <search_service/SetSearchRegionGoal.h>
#include <visualization_msgs/Marker.h>
#include <string>


  void PluginClient::vizPubCb()
  {
      visualization_msgs::Marker points, line_strip;

      points.header = line_strip.header = input_.header;
      points.ns = line_strip.ns = "explore_points";
      points.lifetime = line_strip.lifetime = ros::Duration();

      points.id = 0;
      line_strip.id = 1;

      points.type = visualization_msgs::Marker::SPHERE_LIST;
      line_strip.type = visualization_msgs::Marker::LINE_STRIP;

      if (!input_.polygon.points.empty())
      {
          points.action = line_strip.action = visualization_msgs::Marker::ADD;
          points.pose.orientation.w = line_strip.pose.orientation.w = 1.0;

          points.scale.x = points.scale.y = 0.1;
          line_strip.scale.x = 0.05;

          for (const auto & point : input_.polygon.points)
          {
              line_strip.points.push_back(costmap_2d::toPoint(point));
              points.points.push_back(costmap_2d::toPoint(point));
          }

          if (waiting_for_center_)
          {
              line_strip.points.push_back(costmap_2d::toPoint(input_.polygon.points.front()));
              points.color.a = points.color.r = line_strip.color.r = line_strip.color.a = 1.0;
          }
          else
          {
              points.color.a = points.color.b = line_strip.color.b = line_strip.color.a = 1.0;
          }
      }
      else
      {
          points.action = line_strip.action = visualization_msgs::Marker::DELETE;
      }
      ROS_WARN_ONCE("Change marker topic to search_polygon_marker before continuing.");
      // publish points and lines to be viewable in rviz
      point_viz_pub_.publish(points);
      point_viz_pub_.publish(line_strip);
  }

  void PluginClient::pointCb(const geometry_msgs::PointStampedConstPtr& point)
  {
      double average_distance = polygonPerimeter(input_.polygon) / input_.polygon.points.size();

      if (waiting_for_center_)
      {
          // flag is set so this is the last point of boundary polygon, i.e. center

          if (!pointInPolygon(point->point, input_.polygon))
          {
              ROS_ERROR("Center not inside polygon, restarting");
          }
          else
          {
              ROS_INFO("Search Polygon Selected!!");
              //ROS_INFO("Sending goal");
              search_service::SetSearchRegionGoal goal;
              actionlib::SimpleActionClient<search_service::SetSearchRegionAction> ac_("set_search_region", true);
              ac_.waitForServer();
              //goal.start_point = *point;
              goal.boundary = input_;
              ac_.sendGoal(goal);
              ROS_INFO("sending_goal");
              bool finished_before_timeout = ac_.waitForResult(ros::Duration(50.0));
              if(finished_before_timeout)
              {
                  actionlib::SimpleClientGoalState state = ac_.getState();
                  ROS_INFO("action finished: %s ", state.toString().c_str());
              
              }
               //send the name of the plugin you want to use to get goals
              //goal.strategy_plugin = plugin_name_;
          }
          waiting_for_center_ = false;
          input_.polygon.points.clear();
      }
      else if (input_.polygon.points.empty())
      {
          // first control point, so initialize header of boundary polygon

          input_.header = point->header;
          input_.polygon.points.push_back(costmap_2d::toPoint32(point->point));
      }
      else if (input_.header.frame_id != point->header.frame_id)
      {
          ROS_ERROR("Frame mismatch, restarting polygon selection");
          input_.polygon.points.clear();
      }
      else if (input_.polygon.points.size() > 1 &&
        pointsNearby(input_.polygon.points.front(), point->point, average_distance*0.1))
      {
          // check if last boundary point, i.e. nearby to first point

          if (input_.polygon.points.size() < 3)
          {
              ROS_ERROR("Not a valid polygon, restarting");
              input_.polygon.points.clear();
          }
          else
          {
              waiting_for_center_ = true;
              ROS_WARN("If you are done, Please select any point inside the polygon");
              ROS_WARN("Otherwise, you can select any point outside the polygon to clear points");
          }
      }
      else
      {
          // otherwise, must be a regular point inside boundary polygon
          input_.polygon.points.push_back(costmap_2d::toPoint32(point->point));
          input_.header.stamp = ros::Time::now();
      }
  }

  PluginClient::PluginClient() :
      nh_(),
      private_nh_("~"),
      waiting_for_center_(false)
  {
      input_.header.frame_id = "map";
      point_ = nh_.subscribe("/clicked_point", 10, &PluginClient::pointCb, this);
      point_viz_pub_ = nh_.advertise<visualization_msgs::Marker>("search_polygon_marker", 10);
      point_viz_timer_ = nh_.createWallTimer(ros::WallDuration(0.1), boost::bind(&PluginClient::vizPubCb, this));
      ROS_INFO("Please use the 'Point' tool in Rviz to select an exporation boundary.");
  }


int main(int argc, char** argv)
{
    ros::init(argc, argv, "plugin_client");

    PluginClient client;
    ros::spin();
    return 0;
}
