#include "visible_region.h"



using namespace std;
using namespace tf;
using namespace message_filters;

static const double       sequencer_delay            = 0.5; //TODO: this is probably too big, it was 0.8
static const unsigned int sequencer_internal_buffer  = 100;
static const unsigned int sequencer_subscribe_buffer = 10;
static const unsigned int num_particles_tracker      = 1000;
static const double       tracker_init_dist          = 4.0;

bool IsNotInitilized = true;

// constructor
visible_manager::visible_manager(ros::NodeHandle nh, std::string pose_topic_="amcl_pose", std::string camera_frame_="front_camera", std::string map_name_="local_costmap")
  : nh_(nh),
    robot_state_(),
    agent_local_map_updated(false)
{

  agent_pose_topic = pose_topic_;
  camera_frame=camera_frame_;
  lasermap_name=map_name_;

  cout<<"agent_pose_toic: "<<agent_pose_topic <<std::endl;
  cout<<"agent_map_laser_toic: "<<lasermap_name<<std::endl;
  cout<<"CAMERA_frame: "<<camera_frame<<std::endl;

  camera_visible_region_pub=nh_.advertise<nav_msgs::OccupancyGrid>("/camera_region_map", 10, true);
  globalpose_sub=nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>(agent_pose_topic,10,&visible_manager::global_pose_callback,this);
  laser_costmap_sub= nh_.subscribe<nav_msgs::OccupancyGrid>(lasermap_name,10,&visible_manager::agent_localmap_callback,this);
  global_pose.resize(3,0.0);

  //camera region
  camera_visible_region.info.width=10;
  camera_visible_region.info.height= 10; //30
  camera_visible_region.info.resolution=0.5;
  camera_visible_region.info.origin.position.x=-2.5; // -7.5
  camera_visible_region.info.origin.position.y=-2.5; //-7.5
  camera_visible_region.data.resize(camera_visible_region.info.width*camera_visible_region.info.height,0.0);


}

// destructor
visible_manager::~visible_manager()
{
  // delete sequencer
  // delete all trackers
};



void visible_manager::publish_cameraregion()
{

   getCameraregion();
   FilterwithLaserCostmap();
   camera_visible_region.header.stamp =  ros::Time::now();
   camera_visible_region.header.frame_id = "map"; 
   camera_visible_region_pub.publish(camera_visible_region);


}


void visible_manager::FilterwithLaserCostmap()
{


  size_t fov_size = camera_visible_region.data.size();
  if(agent_local_map_updated){
  
      for(int i(0);i<fov_size;i++)
      {
          if((agent_local_map.data[i]<0) && (camera_visible_region.data[i]>1))
              camera_visible_region.data[i]=-1;
      }
  }

}


void visible_manager::getCameraregion()
{

  double global_robot_x= global_pose[0];
  double global_robot_y= global_pose[1];
  double global_robot_theta = global_pose[2]; //base_yaw+head_yaw

  camera_visible_region.info.origin.position.x=global_robot_x-2.5;//7.5
  camera_visible_region.info.origin.position.y=global_robot_y-2.5;//7.5



  visiblie_idx_set.clear();
  global_robot_theta=0.0;
  //Iteration for belief grid
  for(int i(0);i<camera_visible_region.info.width;i++)
    for(int j(0);j<camera_visible_region.info.height;j++)
  {
    int belief_map_idx=j*camera_visible_region.info.height+i;

    // double map_ogirin_x = camera_visible_region.info.origin.position.x+global_robot_x;
    // double map_ogirin_y = camera_visible_region.info.origin.position.y+global_robot_y;

    double map_ogirin_x = camera_visible_region.info.origin.position.x;
    double map_ogirin_y = camera_visible_region.info.origin.position.y;


    double trans_vector_x=(i+0.5)*camera_visible_region.info.resolution;
    double trans_vector_y=(j+0.5)*camera_visible_region.info.resolution;

    double rot_trans_vector_x = cos(global_robot_theta)*trans_vector_x-sin(global_robot_theta)*trans_vector_y;
    double rot_trans_vector_y = sin(global_robot_theta)*trans_vector_x+cos(global_robot_theta)*trans_vector_y;

    double belief_global_x=map_ogirin_x+rot_trans_vector_x;
    double belief_global_y=map_ogirin_y+rot_trans_vector_y;

    //solve
    bool line1_result =getlinevalue(1,belief_global_x,belief_global_y);
    bool line2_result =getlinevalue(2,belief_global_x,belief_global_y);


    if( line1_result && line2_result )
    {
      camera_visible_region.data[belief_map_idx]=10;//30;  
      visiblie_idx_set.push_back(belief_map_idx);         //save cell_id 
    }
    else
      camera_visible_region.data[belief_map_idx]=-1.0; 
  }



}


bool visible_manager::getlinevalue(int line_type,double input_x, double input_y)
{

  double global_robot_theta = global_pose[2];
  // double global_robot_theta =Camera_angle;
  double theta_1=-FOVW*MATH_PI/180+global_robot_theta;
  double theta_2= FOVW*MATH_PI/180+global_robot_theta;
  
  double m_1=tan(theta_1);
  double m_2=tan(theta_2);

  int isspecial=0;

  if(theta_1<-MATH_PI/2.0 && theta_2 >-MATH_PI/2.0)
  {
    double temp=m_2;
    isspecial=1;
  }
  else if(theta_2> MATH_PI/2.0 && (theta_1 <MATH_PI/2.0))
  {
    isspecial=2;
  }
  else if (theta_1<-MATH_PI/2.0 && theta_2 <-MATH_PI/2.0)
  {
    isspecial=5;
  }
  else if(theta_2< -MATH_PI/2.0)
  {
    isspecial=3;
  }

  else if(theta_1>MATH_PI/2.0 && theta_2> MATH_PI/2.0)
  {
    isspecial=4;  
  }

   // std::cout<<"camera region section : "<<isspecial<<std::endl;
  
  double m=0.0;
  double coeff_sign=1.0;

  double global_robot_x= global_pose[0];
  double global_robot_y= global_pose[1];

  double res =0.0;

  switch(line_type){
  case 1:
      m=m_1;
      coeff_sign=-1.0;

      if(isspecial==0)
          coeff_sign=-1.0;
      else if(isspecial==1)
        coeff_sign=1.0;
      else if(isspecial==2)
        coeff_sign=-1.0;  
      else if(isspecial==4)
        coeff_sign=1.0; 
      else if(isspecial==5)
        coeff_sign=1.0; 

      break;
  case 2:
      m=m_2;
      coeff_sign=-1.0;
      if(isspecial==1)
        coeff_sign=1.0; 
      else if(isspecial==0)
        coeff_sign=1.0; 
      else if(isspecial==3)
        coeff_sign=1.0;
           

      break;
  default:
    std::cout<<"Wrong line type"<<std::endl;
      m=m_1;
    }

  res= m*input_x-m*global_robot_x+global_robot_y-input_y;

  if(res*coeff_sign>0 || res==0)
    return true;
  else
    return false;

}




void visible_manager::scaled_static_map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    ROS_INFO("map Received");
    std::cout <<"static_Width: " << msg->info.width << std::endl;
    std::cout <<"static_Height: " << msg->info.height << std::endl;
    std::cout << "static_X origin:" << msg->info.origin.position.x << std::endl;
    std::cout << "static_Y origin:" << msg->info.origin.position.y << std::endl;
    std::cout <<"static_Resolution: " << msg->info.resolution << std::endl;   

    // Copy Data;
    Scaled_map = (*msg);
    Scaled_map.header.stamp =  ros::Time::now();
    Scaled_map.header.frame_id = "map";
 
}



bool visible_manager::check_staticObs(float x_pos,float y_pos)
{
  
  //return true if it is occupied with obstacles
  if (Scaled_map.data.size()>0)
  {   int obs_idx=globalcoord_To_SScaled_map_index(x_pos,y_pos);
    

    if(Scaled_map.data[obs_idx]>0)
        return true;
    else
      return false;
  }

}


bool visible_manager::check_cameraregion(float x_pos,float y_pos)
{

  if(abs(x_pos)<7.0 && abs(y_pos)<7.0)
  {
  //return true if it is occupied with obstacles
  if (camera_visible_region.data.size()>0)
  {   
    int obs_idx=CoordinateTransform_Global2_staticMap(x_pos,y_pos);
    
    if(obs_idx<camera_visible_region.data.size()){
      if(camera_visible_region.data[obs_idx]>10.0)
        return true;
      else
        return false;
    }
  }

  }

  return true;
}



int visible_manager::globalcoord_To_SScaled_map_index(float x_pos,float y_pos)
{
   std::vector<float> cur_coord(2,0.0);
  
   //for case of using static map
  float reference_origin_x =-4;
  float reference_origin_y =-4;
  float Grid_STEP=0.5;
  int num_grid=24;

  float  temp_x  = x_pos-reference_origin_x;
  float  temp_y = y_pos-reference_origin_y;

  cur_coord[0]= (int) (temp_x/Grid_STEP);
  cur_coord[1]= (int)(temp_y/Grid_STEP);

  int robot_pos_id=num_grid*cur_coord[1]+cur_coord[0];

  return robot_pos_id;

}


double visible_manager::getDistance_from_Vec(std::vector<double> origin, double _x, double _y)
{
  double temp=0.0;

  temp=(origin[0]-_x)*(origin[0]-_x);
  temp+=(origin[1]-_y)*(origin[1]-_y);
  temp=sqrt(temp);

  return temp;

}

void visible_manager::global_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  //ROS_INFO("pose_callback");
    std::cout<<"pos_callback"<<std::endl;

   global_pose[0]=msg->pose.pose.position.x;
   global_pose[1]=msg->pose.pose.position.y;

   tf::StampedTransform baselinktransform;
   listener.waitForTransform("map", camera_frame, ros::Time(0), ros::Duration(2.0));
   listener.lookupTransform("map", camera_frame, ros::Time(0), baselinktransform);
   double yaw_tf =   tf::getYaw(baselinktransform.getRotation()); 

    global_pose[2]=yaw_tf;

}


// callback for messages
int visible_manager::CoordinateTransform_Global2_staticMap(float global_x, float global_y)
{
  double reference_origin_x=camera_visible_region.info.origin.position.x;
  double reference_origin_y=camera_visible_region.info.origin.position.y;

  //Find the coordinate w.r.t map origin
  double  temp_x  = global_x - reference_origin_x;
  double  temp_y  = global_y - reference_origin_y;

  //Find the map cell idx for x, y
  std::vector<int> human_coord(2,0);
  human_coord[0]= (int) (temp_x/camera_visible_region.info.resolution);
  human_coord[1]= (int) (temp_y/camera_visible_region.info.resolution);

  //Find the map index from cell x, y
  int static_map_idx= human_coord[0]+camera_visible_region.info.width*human_coord[1];

  // std::cout<<"map_idx : "<<static_map_idx<<std::endl;
  return static_map_idx;
   
}

void visible_manager::agent_localmap_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    agent_local_map=*msg;
    //ROS_INFO("laser_map_received");
    agent_local_map_updated = true;
    //update_occ_grid_map(msg);
    //publish search map
}

bool visible_manager::Comparetwopoistions(std::vector<double> pos,std::vector<double> pos2, double criterion)
{
  //return true if there are in criterion distance 
  double temp_dist=0.0;
  for(int i(0);i<2;i++) 
  {
    temp_dist+=pow((pos[i]-pos2[i]),2);
  }

  temp_dist=sqrt(temp_dist);

  if(temp_dist<criterion)
    return true;
  

  return false;

}

// filter loop
void visible_manager::spin()
{
  ROS_INFO("People tracking manager started.");

  while (ros::ok())
  {
    publish_cameraregion();
    //Publish_nav_target();
    // ------ LOCKED ------
    boost::mutex::scoped_lock lock(filter_mutex_);
    lock.unlock();
    // ------ LOCKED ------

    // sleep
    ros::Duration(0.4).sleep();

    ros::spinOnce();
  }
};


// ----------
// -- MAIN --
// ----------
int main(int argc, char **argv)
{
  // Initialize ROS
  ros::init(argc, argv, "visible_region_node");
  ros::NodeHandle(nh);

  std::cout<<"pose_topic: "<<argv[1]<<std::endl;
  std::cout<<"camera_frame: "<<argv[2]<<std::endl;
  std::cout<<"laser_costmap: "<<argv[3]<<std::endl;
  // create tracker node
  visible_manager visible_region_node(nh, argv[1],argv[2], argv[3]);

  // wait for filter to finish
  visible_region_node.spin();
  // Clean up

  return 0;
}
