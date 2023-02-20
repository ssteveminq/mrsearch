#include "ros/ros.h"
#include "map_util.h"
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>


// using namespace Eigen;
//
class Listener{
public:
    //geometry_msgs::Pose pose;
    ros::NodeHandle m_node;
    ros::Publisher   Scaled_static_map_pub;
    geometry_msgs::Pose initpose;
    Map_params* m_params;
	nav_msgs::OccupancyGrid Scaled_static_map;
    double MAX_X;
    double MAX_Y;
    double MIN_X;
    double MIN_Y;
    double XY_RES_SCALED;
    bool is_initialized;

    Listener(){

        m_node.param("MAX_X", MAX_X, {25.0});
        m_node.param("MIN_X", MIN_X, {-1.0});
        m_node.param("MAX_Y", MAX_Y, {10.25});
        m_node.param("MIN_Y", MIN_Y, {0.0});
        m_node.getParam("MAX_X", MAX_X);
        m_node.getParam("MIN_X", MIN_X);
        m_node.getParam("MAX_Y", MAX_Y);
        m_node.getParam("MIN_Y", MIN_Y);

        m_node.param("XY_RES_SCALED", XY_RES_SCALED, {0.05});

        m_params= new Map_params(MAX_X, MAX_Y, MIN_X, MIN_Y, XY_RES_SCALED);

        Scaled_static_map.info.resolution = m_params->xyreso;
        Scaled_static_map.info.width = m_params->xw;
        Scaled_static_map.info.height = m_params->yw;
        Scaled_static_map.info.origin.position.x = m_params->xmin;
        Scaled_static_map.info.origin.position.y = m_params->ymin;
        Scaled_static_map.data.resize((Scaled_static_map.info.width * Scaled_static_map.info.height), 0.0);  //unknown ==> 0 ==> we calculate number of 0 in search map to calculate IG

        int map_size_ =Scaled_static_map.info.width * Scaled_static_map.info.height;
        ROS_INFO("xw: % .2lf ", m_params->xw);

        ROS_INFO("yw: % .2lf ", m_params->yw);
        ROS_INFO("map_size: %d",map_size_); 
        ROS_INFO("scaled_static_map initialized");
        is_initialized=true;
     
        Scaled_static_map_pub=m_node.advertise<nav_msgs::OccupancyGrid>("/scaled_static_map", 10, true);
    }


    void map_cb(const nav_msgs::OccupancyGrid::ConstPtr& msg)
    {
        ROS_INFO("Map_CB");
        double small_pos_x, small_pos_y=0.0;
        double dist_x,dist_y=0.0;
        int map_coord_i,map_coord_j=0;
        int numcount=0;
        int	original_width=msg->info.width;
        int	original_height= msg->info.height;
        double original_x=msg->info.origin.position.x;
        double original_y=msg->info.origin.position.y;
        double original_maxx=msg->info.origin.position.x+original_width*msg->info.resolution;
        double original_maxy=msg->info.origin.position.y+original_height*msg->info.resolution;
        double original_res=msg->info.resolution;
        //double original_y=-51.225;;
        
        m_params= new Map_params(original_maxx,original_maxy,original_x,original_y, XY_RES_SCALED);
        Scaled_static_map.info.resolution = m_params->xyreso;
        Scaled_static_map.info.width= m_params->xw;
        Scaled_static_map.info.height= m_params->yw;
        Scaled_static_map.info.origin.position.x= m_params->xmin;
        Scaled_static_map.info.origin.position.y= m_params->ymin;
        Scaled_static_map.data.resize((Scaled_static_map.info.width * Scaled_static_map.info.height), 0.0);  //unknown ==> 0 ==> we calculate number of 0 in search map to calculate IG
        //double original_res=0.05;

        if(is_initialized){

		//for static space map
		std::map<int,int> occupancyCountMap;
		int scaled_res= 1; //(XY_RES_SCALED/original_res);
        // std::cout << "xy_scaled, original res = " << XY_RES_SCALED << ", " << original_res << std::endl;
        // std::cout << "SCALED RESOLUTION OF STATIC MAP = " << scaled_res << std::endl;
		int map_idx=0;
		int scaled_result=0;

		for(int j(0);j<Scaled_static_map.info.height;j++)
		    for(int i(0);i<Scaled_static_map.info.width;i++)
		    {

			map_idx=j*Scaled_static_map.info.width+i;
			double pos_x=i*Scaled_static_map.info.resolution+Scaled_static_map.info.origin.position.x;
			double pos_y=j*Scaled_static_map.info.resolution+Scaled_static_map.info.origin.position.y;

			numcount=0;
			for(int k(0);k<scaled_res;k++)
			    for(int j(0);j<scaled_res;j++)
			    {
				small_pos_x=pos_x+j*original_res;
				small_pos_y=pos_y+k*original_res;
				dist_x= small_pos_x-original_x;
				dist_y= small_pos_y-original_y;
				map_coord_i=floor(dist_x/original_res);
				map_coord_j=floor(dist_y/original_res);
				
				int map_data_index=original_width*map_coord_j+map_coord_i;
				float temp_occupancy= msg->data[map_data_index];
				if(temp_occupancy>0)
                   numcount++;
			    }

			    if(numcount)
				scaled_result=100;
			    else
				scaled_result=0;

			    Scaled_static_map.data[map_idx]=scaled_result;
                // if(Scaled_static_map.data[map_idx]) 
                    //std::cout << "SCALED MAP DATA = " << Scaled_static_map.data[map_idx] << std::endl;
		}

		 //find index from
		 Scaled_static_map.header.stamp =  ros::Time::now();
		 Scaled_static_map.header.frame_id = "map"; 
		 Scaled_static_map_pub.publish(Scaled_static_map);
	}

    }

    void initpose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        initpose = msg->pose;
    }

};



int main(int argc, char **argv)
{
  ros::init(argc, argv, "Map_converter");
  
  Listener problemmanager; 

  ros::Subscriber staticmap_sub;
  ros::NodeHandle n;
  staticmap_sub = n.subscribe<nav_msgs::OccupancyGrid>("/map", 30, &Listener::map_cb,&problemmanager); 
  ros::Rate loop_rate(10);

  while (ros::ok())
  {
  	 ros::spinOnce();
     loop_rate.sleep();  
  }

  ros::spin();

  return 0;
}




