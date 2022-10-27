#include "inspection_manager.h"



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
inspection_manager::inspection_manager(ros::NodeHandle nh)
  : nh_(nh),search_map_received(true),
    robot_state_()
{

  //agent_pose_topic = pose_topic_;
  //camera_frame=camera_frame_;
  //cout<<"agent_pose_toic: "<<agent_pose_topic <<std::endl;
  //cout<<"CAMERA_frame: "<<camera_frame<<std::endl;
  viz_pub=nh_.advertise<nav_msgs::OccupancyGrid>("tb2/viz_search_map", 10, true);
  //globalpose_sub=nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>(agent_pose_topic,10,&inspection_manager::global_pose_callback,this);
  inspect_target_pub=nh_.advertise<geometry_msgs::PoseArray>("tb2/inspection_poses", 10, true);
  stfpose_sub=nh_.subscribe<geometry_msgs::PoseArray>("tb2/stf_poses",10,&inspection_manager::posearry_callback,this);
  search_map_sub=nh_.subscribe<nav_msgs::OccupancyGrid>("/search_map",10,&inspection_manager::search_map_callback,this);
  agent_map_sub=nh_.subscribe<nav_msgs::OccupancyGrid>("tb2/fov_map",10,&inspection_manager::agent1_localmap_callback,this);
  global_pose.resize(3,0.0);

  //camera region
  camera_visible_region.info.width=20;
  camera_visible_region.info.height= 20;
  camera_visible_region.info.resolution=0.5;
  camera_visible_region.info.origin.position.x=-5.0;
  camera_visible_region.info.origin.position.y=-5.0;
  camera_visible_region.data.resize(camera_visible_region.info.width*camera_visible_region.info.height,0.0);

  //camera region
  viz_search_map.info.width=100;
  viz_search_map.info.height= 100;
  viz_search_map.info.resolution=0.5;
  viz_search_map.info.origin.position.x=-25.0;
  viz_search_map.info.origin.position.y=-25.0;
  viz_search_map.data.resize(viz_search_map.info.width*viz_search_map.info.height,0.0);



}

// destructor
inspection_manager::~inspection_manager()
{
  // delete sequencer
  // delete all trackers
};



void inspection_manager::publish_cameraregion()
{

   //getCameraregion();
   viz_search_map.header.stamp =  ros::Time::now();
   viz_search_map.header.frame_id = "map"; 
   viz_pub.publish(viz_search_map);
}

void inspection_manager::publish_inspection_poses()
{
    filtered_pose_array.poses.clear();
    bool isnear=false;
    bool issearched=false;
    for(int i(0);i< pose_array.poses.size();i++)
    {
        //if(!near_from_poses(ps) && !checked_with_search_map(ps))
        isnear = near_from_poses(pose_array.poses[i]);
        if(!isnear)
        {
            issearched=checked_with_search_map(pose_array.poses[i]);
        }



        //cout<<i<<" -th pose"<<"near?"<<isnear<<endl;
        if(!isnear && !issearched)
        {
            filtered_pose_array.poses.push_back(pose_array.poses[i]);
            //cout<<"push_back"<<endl;
        }
    }

   //getCameraregion();
   filtered_pose_array.header.stamp =  ros::Time::now();
   filtered_pose_array.header.frame_id = "map"; 
   //filtered_pose_array.header = pose_array.header;
   inspect_target_pub.publish(filtered_pose_array);
   //filtered_pose_array.poses.clear();
}



void inspection_manager::getCameraregion()
{

  double global_robot_x= global_pose[0];
  double global_robot_y= global_pose[1];
  double global_robot_theta = global_pose[2]; //base_yaw+head_yaw

  camera_visible_region.info.origin.position.x=global_robot_x-7.5;
  camera_visible_region.info.origin.position.y=global_robot_y-7.5;



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
      camera_visible_region.data[belief_map_idx]=30;  
      visiblie_idx_set.push_back(belief_map_idx);         //save cell_id 
    }
    else
      camera_visible_region.data[belief_map_idx]=-1.0; 
  }



}

void inspection_manager::agent1_localmap_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    agent1_local_map=*msg;
    update_occ_grid_map(msg);
}

void inspection_manager::update_occ_grid_map(const nav_msgs::OccupancyGridConstPtr& msg)
{
    //origin_point_map_en =local_map.info.origin.position
    double px, py =0.0;
    int map_idx,search_idx =0;

    for(int j(0); j< msg->info.height;j++)
        for(int i(0); i< msg->info.width;i++)
        {
            map_idx = j*msg->info.width+i;
            px = msg->info.origin.position.x+(i+0.5)*msg->info.resolution;
            py = msg->info.origin.position.y+(j+0.5)*msg->info.resolution;
            search_idx = Coord2CellNum(px,py, viz_search_map);         // get search cell idx 
            //Update searchmap according to local measurement
            // if local_map is known (occ or free) and global_map is not occupied by static obstacle 
            if(search_idx<viz_search_map.data.size())
            {
                if(msg->data[map_idx]>0) 
                    viz_search_map.data[search_idx]=30;
            }
        }
}


bool inspection_manager::getlinevalue(int line_type,double input_x, double input_y)
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

void inspection_manager::search_map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    cur_search_map=*msg;
    if(!search_map_received)
    {
        viz_search_map.info = msg->info;
        viz_search_map.header.frame_id = msg->header.frame_id;
        viz_search_map.data.resize(msg->info.width*msg->info.height,0.0);
        search_map_received=true;
    }
}


void inspection_manager::scaled_static_map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
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



bool inspection_manager::check_staticObs(float x_pos,float y_pos)
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


bool inspection_manager::check_cameraregion(float x_pos,float y_pos)
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



int inspection_manager::globalcoord_To_SScaled_map_index(float x_pos,float y_pos)
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


double inspection_manager::getDistance_from_Vec(std::vector<double> origin, double _x, double _y)
{
  double temp=0.0;

  temp=(origin[0]-_x)*(origin[0]-_x);
  temp+=(origin[1]-_y)*(origin[1]-_y);
  temp=sqrt(temp);

  return temp;

}
int inspection_manager::Coord2CellNum(double _x, double _y, const nav_msgs::OccupancyGrid& inputmap_)
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



bool inspection_manager::checked_with_search_map(const geometry_msgs::Pose& input )
{

    // listener.waitForTransform("map_en", "map", ros::Time(0), ros::Duration(2.0));
    geometry_msgs::PoseStamped pose_in;     //added by mk, ryan
    geometry_msgs::PoseStamped pose_out;     //added by mk, ryan
      //pose_in.header.stamp=ros::Time(0);
    pose_in.header.stamp=ros::Time().now();
    pose_in.header.frame_id="map";
    pose_in.pose=input;
    pose_in.pose.orientation.x=0.0;
    pose_in.pose.orientation.y=0.0;
    pose_in.pose.orientation.z=0.0;
    pose_in.pose.orientation.w=1.0;

    pose_out = pose_in;
    // try{
    //     listener.transformPose("map", ros::Time(0), pose_in, pose_in.header.frame_id, pose_out);
    //     pose_out.pose.orientation.w=1;
    //     //return true;
    //   }catch(tf::TransformException ex){
    //     std::cout << ex.what() << std::endl;
    //     sleep(0.1);
    //     cout<<"here-false"<<endl;
    //     return false;
   // }

    if(search_map_received)
    {
        auto search_idx = Coord2CellNum(pose_out.pose.position.x,pose_out.pose.position.y, viz_search_map);         // get search cell idx 
        //cout<<"pos_x:"<<pose_out.pose.position.x <<pose_out.pose.position.y<<endl;
        //cout<<"search_idx"<<search_idx<<endl;
        if(search_idx<viz_search_map.data.size())
        {
            if(viz_search_map.data[search_idx]>0) 
                return true;
            //if already that pose is expored or static obstacles;
            //return false;
        }
        else
            return true;
    }

    return false;

    
   //pose_en ==> pose
   //
   //for(auto epose: pose_array.poses)
    //{
        //filter with existing poses
        //if(sqrt(pow(input.position.x-epose.position.x,2)+pow(input.position.y-epose.position.y))<0.25);
            //return true;
    //}
    //return false;
}

bool inspection_manager::near_from_poses(const geometry_msgs::Pose& input )
{

    double dist_threshold = 0.4;
    //cout<<"filtered_pose_array.size(): "<<filtered_pose_array.poses.size()<<endl;;
    for(auto& epose: filtered_pose_array.poses)
    {
        //filter with existing poses
        double dist = sqrt(pow(input.position.x-epose.position.x,2)+pow(input.position.y-epose.position.y,2));
        //cout<<"dist: "<<dist<<endl;
        if(dist<0.4)
        {
            //cout<<"input_x"<<input.position.x<<", y: "<<input.position.y<<", epose: "<<epose.position.x<<", "<<epose.position.y<<endl;
            return true;
        }
            //cout<<"not here"<<endl;

    }
    //cout<<"return false"<<endl;

    return false;
}


void inspection_manager::posearry_callback(const geometry_msgs::PoseArray::ConstPtr& msg)
{
    //pose_array.header = msg->header;
    pose_array = *msg;
    //bool isnear=false;
    //for(auto& ps : msg->poses)
    //{
        //if(!near_from_poses(ps) && !checked_with_search_map(ps))
        //isnear = near_from_poses(ps);
        //if(!isnear)
            //pose_array.poses.push_back(ps);
    //}
}


void inspection_manager::global_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  //ROS_INFO("pose_callback");
    //std::cout<<"pos_callback"<<std::endl;

   global_pose[0]=msg->pose.pose.position.x;
   global_pose[1]=msg->pose.pose.position.y;

   tf::StampedTransform baselinktransform;
   listener.waitForTransform("map", camera_frame, ros::Time(0), ros::Duration(2.0));
   listener.lookupTransform("map", camera_frame, ros::Time(0), baselinktransform);
   double yaw_tf =   tf::getYaw(baselinktransform.getRotation()); 

    global_pose[2]=yaw_tf;

}


// callback for messages
int inspection_manager::CoordinateTransform_Global2_staticMap(float global_x, float global_y)
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

bool inspection_manager::Comparetwopoistions(std::vector<double> pos,std::vector<double> pos2, double criterion)
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
void inspection_manager::spin()
{
  ROS_INFO("inspection_manager started.");

  while (ros::ok())
  {
    // ------ LOCKED ------
    boost::mutex::scoped_lock lock(filter_mutex_);
    publish_cameraregion();
    publish_inspection_poses();
    lock.unlock();
    // ------ LOCKED ------

    // sleep
    ros::Duration(0.25).sleep();

    ros::spinOnce();
  }
};


// ----------
// -- MAIN --
// ----------
int main(int argc, char **argv)
{
  // Initialize ROS
  ros::init(argc, argv, "inpsection_manager_node");
  ros::NodeHandle(nh);

  //std::cout<<"pose_topic: "<<argv[1]<<std::endl;
  //std::cout<<"camera_frame: "<<argv[2]<<std::endl;
  // create tracker node
  inspection_manager inspection_node(nh);

  // wait for filter to finish
  inspection_node.spin();
  // Clean up

  return 0;
}
