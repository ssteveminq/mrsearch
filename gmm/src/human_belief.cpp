#include "human_belief.h"

using namespace std;
using namespace tf;
using namespace message_filters;

static const double       sequencer_delay            = 0.5; //TODO: this is probably too big, it was 0.8
static const unsigned int sequencer_internal_buffer  = 100;
static const unsigned int sequencer_subscribe_buffer = 10;
static const unsigned int num_particles_tracker      = 1000;
static const double       tracker_init_dist          = 4.0;


constexpr std::size_t SAMPLES = 1500;
bool IsNotInitilized = true;


void emplace_back(std::vector<gauss::gmm::Cluster>& clusters, const double w, const Eigen::VectorXd& mean, const Eigen::MatrixXd& covariance) {
    clusters.emplace_back();
    clusters.back().weight = w;
    clusters.back().distribution = std::make_unique<gauss::GaussianDistribution>(mean, covariance);
}


Eigen::VectorXd make_vector(const std::vector<double> &values) {
  if (values.empty()) {
    throw std::runtime_error("empty buffer");
  }
  Eigen::VectorXd vector(values.size());
  Eigen::Index index = 0;
  for (const auto &value : values) {
    vector(index) = value;
    ++index;
  }
  return vector;
}


// constructor
belief_manager::belief_manager(std::string name): as_(nh_, name, boost::bind(&belief_manager::executeCB, this,_1), false),
    action_name(name), robot_state_(),isCalled(false),
    Is_Searchmap_received(false),MIN_X(-50.0),MAX_X(50.0),MIN_Y(-50.0),MAX_Y(50.0),map_dimension_changed(false)
{
  // initialize
  belief_pub=nh_.advertise<nav_msgs::OccupancyGrid>("/human_belief_map", 10, true);
  searchmap_sub =nh_.subscribe<nav_msgs::OccupancyGrid>("/search_map", 10, &belief_manager::searchmap_callback,this);
  visual_marker_pub= nh_.advertise<visualization_msgs::MarkerArray>("particles", 5);
  mean_pub= nh_.advertise<geometry_msgs::PoseArray>("mean_poses", 5);

  //Initialize human_belief_map
  belief_map.info.width=50;
  belief_map.info.height= 50;
  belief_map.info.resolution=0.5;
  belief_map.info.origin.position.x=-10.0;
  belief_map.info.origin.position.y=-10.0;
  int belief_size=belief_map.info.width*belief_map.info.height;
  belief_map.data.resize(belief_map.info.width*belief_map.info.height);
  for(int k(0);k<belief_size;k++)
      belief_map.data[k]=0.01;


    const std::size_t nClusters = 4;
    Eigen::VectorXd eigs_cov(2);
    eigs_cov << 5.1, 5.2;
    clusters.reserve(nClusters);
    double angle = 0.0;
    double angle_delta = 3.14 / static_cast<double>(nClusters);
    for (std::size_t c = 0; c < nClusters; ++c) {
    emplace_back(clusters, 1.0 / static_cast<double>(nClusters),
                        make_vector({-26.15 , -20.0 }),
                        gauss::make_random_covariance(eigs_cov));
    emplace_back(clusters, 1.0 / static_cast<double>(nClusters),
                        make_vector({-10.0 , -15.0 }),
                        gauss::make_random_covariance(eigs_cov));
    emplace_back(clusters, 1.0 / static_cast<double>(nClusters),
                        make_vector({18.0 , 45.0}),
                        gauss::make_random_covariance(eigs_cov));
    emplace_back(clusters, 1.0 / static_cast<double>(nClusters),
                        make_vector({-25.0 , -47.0 }),
                        gauss::make_random_covariance(eigs_cov));
    }


    mean_poses.header.frame_id="map";
    mean_poses.poses.resize(nClusters);
    mean_poses.poses[0].position.x= -26.15;
    mean_poses.poses[0].position.y= -20;

    mean_poses.poses[1].position.x= -10.15;
    mean_poses.poses[1].position.y= -15;

    mean_poses.poses[2].position.x= 18.15;
    mean_poses.poses[2].position.y= 45;

    mean_poses.poses[3].position.x= -25.15;
    mean_poses.poses[3].position.y= -47;


      //angle += angle_delta;
    //}
  gauss::gmm::GaussianMixtureModel ref_model(clusters);
  
  sample_poses.poses.resize(SAMPLES);
  auto samples = ref_model.drawSamples(SAMPLES);
  for(size_t i(0); i< samples.size();i++)
  {
      std::cout<<i<<" : "<<samples[i][0]<<","<<samples[i][1]<<std::endl;
      sample_poses.poses[i].position.x=samples[i][0];
      sample_poses.poses[i].position.y=samples[i][1];
  }

  as_.start();


}

// destructor
belief_manager::~belief_manager()
{
  // delete sequencer
  // delete all trackers
};

void belief_manager::executeCB(const gmm::PredictionGoalConstPtr &goal)
  {

    //clusters.clear();

   const std::size_t nClusters = goal->mean_xs.size();
   mean_poses.poses.resize(nClusters);

    for (std::size_t c = 0; c < nClusters; ++c){
     mean_poses.poses[c].position.x= goal->mean_xs[c];
     mean_poses.poses[c].position.y= goal->mean_ys[c];
    }



    Eigen::VectorXd eigs_cov(2);
    eigs_cov << 5.1, 5.2;
    clusters.reserve(nClusters);
    for (std::size_t c = 0; c < nClusters; ++c){
        emplace_back(clusters, 1.0 / static_cast<double>(nClusters),
                            make_vector({goal->mean_xs[c],goal->mean_ys[c]}),
                            gauss::make_random_covariance(eigs_cov));
    }

     gauss::gmm::GaussianMixtureModel ref_model(clusters);

      sample_poses.poses.clear();
      sample_poses.poses.resize(SAMPLES);
      auto samples = ref_model.drawSamples(SAMPLES);
      for(size_t i(0); i< samples.size();i++)
      {
          std::cout<<i<<" : "<<samples[i][0]<<","<<samples[i][1]<<std::endl;
          sample_poses.poses[i].position.x=samples[i][0];
          sample_poses.poses[i].position.y=samples[i][1];
      }


      as_.setSucceeded(result_);
      return;
      /*
      double ros_rate = 2;
      ros::Rate r(ros_rate);
      isActionActive=true;
      //reset_search_map();
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

     boost::mutex::scoped_lock lock(filter_mutex_);

      FindUnknowns();
      result_.success = true;
      result_.waypoints = unknown_poses;
      as_.setSucceeded(result_);

      lock.unlock();
      return;
      */
}


void belief_manager::publish_meanpose()
{

    mean_poses.header.stamp=ros::Time::now();
     mean_pub.publish(mean_poses);


}


void belief_manager::publish_clusters()
{

    std_msgs::ColorRGBA blue;
    blue.r = 0; blue.g = 0; blue.b = 1.0; blue.a = 1.0;
    std_msgs::ColorRGBA red;
    red.r = 1.0; red.g = 0; red.b = 0; red.a = 1.0;
    std_msgs::ColorRGBA green;
    green.r = 0; green.g = 1.0; green.b = 0; green.a = 1.0;

    visualization_msgs::MarkerArray markers_msg;
    std::vector<visualization_msgs::Marker>& markers = markers_msg.markers;
    //visualization_msgs::Marker m;

    visualization_msgs::Marker m;
    m.header.frame_id ="map";
    m.header.stamp = ros::Time::now();
    m.action = visualization_msgs::Marker::DELETEALL;

    m.ns = "beliefs";
    //m.action = m.DELETEALL;
    m.scale.x = 1.0;
    m.scale.y = 1.0;
    m.scale.z = 1.0;
    m.color.r = 0;
    m.color.g = 0;
    m.color.b = 255;
    m.color.a = 255;
    // lives forever
    m.lifetime = ros::Duration(5);
    m.frame_locked = true;


    size_t id = 0;
    for(size_t i(0);i<sample_poses.poses.size();i++)
    {

        //m.action = visualization_msgs::Marker::DELETEALL;
        m.action = visualization_msgs::Marker::ADD;
        //size_t id = 0;
        //for (int j(0);j<agent_xs[i].size();j++) {
            m.type = visualization_msgs::Marker::SPHERE;
            m.id = int(id);
            m.pose.position.x = sample_poses.poses[i].position.x;
            m.pose.position.y = sample_poses.poses[i].position.y;
            m.pose.position.z = 0.5;
            m.pose.orientation.w = 1.0;
            m.scale.x = 0.2;
            m.scale.y = 0.2;
            m.scale.z = 0.2;
            //m.points = frontier.points;
            
            m.color=red;
            markers.push_back(m);
            ++id;
        //}
    }
    visual_marker_pub.publish(markers_msg);
}




void belief_manager::searchmap_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    search_map = *msg;
    //Initialize searchmap when this callback function was firstly called
    //
    if(Is_Searchmap_received )
    {
        if(belief_map.info.width!=msg->info.width)
            map_dimension_changed=true;
    }

    if(!Is_Searchmap_received || map_dimension_changed)
    {
       belief_map.info=msg->info;
       int belief_size=belief_map.info.width*belief_map.info.height;
       belief_map.data.resize(belief_map.info.width*belief_map.info.height);
       MIN_X = belief_map.info.origin.position.x;
       MAX_X = belief_map.info.origin.position.x+belief_map.info.width*belief_map.info.resolution;
       MIN_Y = belief_map.info.origin.position.y;
       MAX_Y = belief_map.info.origin.position.y+belief_map.info.height*belief_map.info.resolution;
       Is_Searchmap_received = true;
    }

   //apply prediction_model

    
   //uniform
   int cell_count=0;
   int belief_size=belief_map.info.width*belief_map.info.height;
   for(int k(0);k<belief_size;k++)
   {
       if(search_map.data[k]==0)
       {
           belief_map.data[k]=50.0;
           cell_count++;
       }
       else
           belief_map.data[k]=0.0;
   }
   ROS_INFO("cell_count: %d", cell_count);

   //for(int k(0);k<belief_size;k++)
   //{
       //belief_map.data[k]=1.0/cell_count*(1/3*cell_count);
   //}

   belief_pub.publish(belief_map);

}






void belief_manager::Mapidx2GlobalCoord(int map_idx, std::vector<double>& global_coords, nav_msgs::OccupancyGrid& inputmap_)
{
    global_coords.resize(2,0.0);

    std::vector<int> cell_xy(2,0.0);

    int res = map_idx / inputmap_.info.width;
    int div = map_idx % inputmap_.info.width;

    cell_xy[0]=div;
    cell_xy[1]=res;

    global_coords[0] =  inputmap_.info.origin.position.x+cell_xy[0]*inputmap_.info.resolution;
    global_coords[1] =  inputmap_.info.origin.position.y+cell_xy[1]*inputmap_.info.resolution;
    //ROS_INFO("global coord x: %.lf, div : %.lf", global_coords[0],global_coords[1]);

}

void belief_manager::publish_gmm()
{

    std::vector<gauss::gmm::Cluster> clusters;


}

void belief_manager::publish_human_belief()
{
    typedef std::map<int, float>::iterator map_iter;
    for(map_iter iterator = map_index_of_human_cells_to_prob.begin(); iterator !=  map_index_of_human_cells_to_prob.end(); iterator++) {

        int human_map_idx = iterator->first;
        Human_Belief_Scan_map.data[human_map_idx]=(iterator->second) * 100.0;
    }
    

   Human_Belief_Scan_map.header.stamp =  ros::Time::now();
   Human_Belief_Scan_map.header.frame_id = "map"; 
   belief_pub.publish(Human_Belief_Scan_map);
   human_candidates_poses.poses.clear();

   for(size_t idx=0;idx<Human_Belief_Scan_map.data.size();idx++)
   {
         if(Human_Belief_Scan_map.data[idx]>10)
         {
             std::vector<double> temp_pose(2,0.0);
             Mapidx2GlobalCoord(idx,temp_pose, Human_Belief_Scan_map);

             geometry_msgs::Pose human_pose;
             human_pose.position.x=temp_pose[0];
             human_pose.position.y=temp_pose[1];
             human_pose.position.z=0.5;
             human_pose.orientation.x=0.0;
             human_pose.orientation.y=0.0;
             human_pose.orientation.z=0.0;
             human_pose.orientation.w=1.0;
         
             human_candidates_poses.poses.push_back(human_pose);

         }
   }

   std::vector<std::pair<double,int> > human_pairs;
   std::vector<double> human_candidates_distances;

   size_t candidate_size = human_candidates_poses.poses.size();
   human_candidates_distances.resize(candidate_size,0.0);
   for(size_t i(0);i<candidate_size;i++)
   {
       human_candidates_distances[i]=getDistance_from_Vec(global_pose,
                                                          human_candidates_poses.poses[i].position.x,
                                                          human_candidates_poses.poses[i].position.y);
   
       std::pair<double, int> temp_pair(human_candidates_distances[i],i);
       human_pairs.push_back(temp_pair);
       //human_pairs.push_back(std::pair<double, int>(human_candidates_distances[i],i));
   }

   //print functions
   //ROS_INFO("---------------------before sorting------------------------------");
   for(size_t j(0);j<human_pairs.size();j++)
       ROS_INFO("human pair index: distance = idx: %d, dist:  %.3lf", human_pairs[j].first, human_pairs[j].second);
   //ROS_INFO("------------------------------------------after sorting -------------");
   //This sorting function saves the distance frome nearest one!!
   std::sort(human_pairs.begin(),human_pairs.end(),comparator);

   //for(size_t j(0);j<human_pairs.size();j++)
   //{
       //ROS_INFO("human pair index: distance = idx: %d, dist:  %.3lf", human_pairs[j].first, human_pairs[j].second);
   //}

   geometry_msgs::PoseArray sorted_human_belief;
   for(size_t idx(0);idx<candidate_size;idx++)
   {
        if(human_pairs[idx].second<6.0)
            sorted_human_belief.poses.push_back(human_candidates_poses.poses[human_pairs[idx].first]);
   }

   sorted_human_belief.header.frame_id ="map";
   sorted_human_belief.header.stamp = ros::Time::now();
   human_candidates_pub.publish(sorted_human_belief);
   
}





bool belief_manager::check_staticObs(float x_pos,float y_pos)
{
  
  //return true if it is occupied with obstacles
  if (Scaled_map.data.size()>0)
  {   
      int obs_idx=globalcoord_To_SScaled_map_index(x_pos,y_pos);

    if(Scaled_map.data[obs_idx]>0)
        return true;
    else
      return false;
  }

}




int belief_manager::globalcoord_To_SScaled_map_index(float x_pos,float y_pos)
{
   std::vector<float> cur_coord(2,0.0);
  
   //for case of using static map
  float reference_origin_x =-4;
  float reference_origin_y =-4;
  float Grid_STEP=0.5;
  int num_grid=24;

  //for case of using static map
  // double reference_origin_x =-3.5;
  // double reference_origin_y =-3.5;
  float  temp_x  = x_pos-reference_origin_x;
  float  temp_y = y_pos-reference_origin_y;

  cur_coord[0]= (int) (temp_x/Grid_STEP);
  cur_coord[1]= (int)(temp_y/Grid_STEP);


  int robot_pos_id=num_grid*cur_coord[1]+cur_coord[0];
  //ROS_INFO("Robot pos ID : %d \n", robot_pos_id);

  return robot_pos_id;

}


double belief_manager::getDistance_from_Vec(std::vector<double> origin, double _x, double _y)
{
  double temp=0.0;

  temp=(origin[0]-_x)*(origin[0]-_x);
  temp+=(origin[1]-_y)*(origin[1]-_y);
  temp=sqrt(temp);

  return temp;

}

void belief_manager::global_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{

   global_pose[0]=msg->pose.pose.position.x;
   global_pose[1]=msg->pose.pose.position.y;

   tf::StampedTransform baselinktransform;
   listener.waitForTransform("map", "spot/base_footprint", ros::Time(0), ros::Duration(2.0));
   listener.lookupTransform("map", "spot/base_footprint", ros::Time(0), baselinktransform);
   double yaw_tf =   tf::getYaw(baselinktransform.getRotation()); 

    global_pose[2]=yaw_tf;

}

void belief_manager::update_particles()
{
    boost::random::mt19937 gen;
    boost::random::uniform_real_distribution<> distx(-0.01, 0.01);
    boost::random::uniform_real_distribution<> disty(-0.01, 0.01);


    for(int j(0);j<sample_poses.poses.size();j++)
    {
    
        sample_poses.poses[j].position.x+=distx(gen);
        sample_poses.poses[j].position.y+=disty(gen);


        if(sample_poses.poses[j].position.x<MIN_X)
            sample_poses.poses[j].position.x=MIN_X+1.0;
        if(sample_poses.poses[j].position.x>MAX_X)
            sample_poses.poses[j].position.x=MAX_X-1.0;
        if(sample_poses.poses[j].position.y<MIN_Y)
            sample_poses.poses[j].position.y=MIN_Y+1.0;
        if(sample_poses.poses[j].position.y>MAX_Y)
            sample_poses.poses[j].position.y=MAX_Y-1.0;
    }


}

void belief_manager::people_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    //ROS_INFO("human poses callback");
    index_of_human_occ_cells_updated_recently.clear();

    //check number of detected_humans
    //num_of_detected_human=msg->people.size();
    /*

    if(num_of_detected_human>0)
       cur_people.resize(num_of_detected_human);
    else
    {
      update_human_occ_belief(NO_HUMANS_DETECTED);
      return;
    }


    ROS_INFO("human poses callback---");
    for(int i(0);i<num_of_detected_human;i++)
    {
      double cur_people_x=msg->people[i].position.position.x;
      double cur_people_y=msg->people[i].position.position.y;

      cur_people[i].resize(2,0.0);
      cur_people[i][0]=cur_people_x;
      cur_people[i][1]=cur_people_y;

      ROS_INFO("---human poses callback---");
      int human_mapidx=CoordinateTransform_Global2_beliefMap(cur_people_x,cur_people_y);
      //if((human_mapidx<0) || (human_mapidx>10000))
          //return;

      if((human_mapidx<Human_Belief_Scan_map.data.size())&& (human_mapidx>0))
      {
          index_of_human_occ_cells_updated_recently.push_back(human_mapidx);
          Human_Belief_Scan_map.data[human_mapidx] = 50;
          Human_Belief_type_map.data[human_mapidx] = HUMAN_OCCUPIED;

         if(map_index_of_human_cells_to_prob.count(human_mapidx) >0 ){
          // Encountered the same cells. Update probability:
          // P(H|S) = P(S|H)P(H) / P(S)
          //std::cout<<"human map idx: " <<human_mapidx <<std::endl;
          float prior = map_index_of_human_cells_to_prob[human_mapidx]; // P(H)
          //std::cout<<"prior : " <<prior <<std::endl;
          float P_S = P_S_given_H*prior + P_S_given_Hc*(1-prior);
          //std::cout<<"P_S : " <<P_S <<std::endl;
          float posterior = (P_S_given_H)*prior / P_S;
          //std::cout<<"posterior : " <<posterior <<std::endl;
          //
          //
          ROS_INFO("---------------human poses callback---");
          map_index_of_human_cells_to_prob[human_mapidx] = posterior;

          }else{
            map_index_of_human_cells_to_prob[human_mapidx] = 0.05;
          }		
      }
      //human_occupied_idx.push_back(human_mapidx);
   }

    ROS_INFO("human poses callback--------");
    update_human_occ_belief(HUMANS_DETECTED);
   */
   // printf("size yolo : %d \n",cur_people.size());
}


void belief_manager::human_yolo_callback(const visualization_msgs::MarkerArray::ConstPtr& msg)
{
    //clear human_occ_cells_updated from last callback
    index_of_human_occ_cells_updated_recently.clear();
    //human_occupied_idx.clear();

    //check number of detected_humans
    num_of_detected_human=msg->markers.size();

    if(num_of_detected_human>0)
       cur_people.resize(num_of_detected_human);
    else
    {
        update_human_occ_belief(NO_HUMANS_DETECTED);
      return;
    }

    for(int i(0);i<num_of_detected_human;i++)
    {
      geometry_msgs::Vector3Stamped gV, tV;

      gV.vector.x = msg->markers[i].pose.position.x;
      gV.vector.y = msg->markers[i].pose.position.y;
      gV.vector.z = msg->markers[i].pose.position.z;

      // std::cout<<"x :"<<_x<<"_y:"<<_y<<"_z:"<<_z<<std::endl;
      tf::StampedTransform maptransform;
      listener.waitForTransform("head_rgbd_sensor_rgb_frame", "map", ros::Time(0), ros::Duration(1.0));
              
      gV.header.stamp = ros::Time();
      gV.header.frame_id = "/head_rgbd_sensor_rgb_frame";
      listener.transformVector(std::string("/map"), gV, tV);
              
      double cur_people_x=tV.vector.x+global_pose[0];
      double cur_people_y=tV.vector.y+global_pose[1];

      cur_people[i].resize(2,0.0);
      cur_people[i][0]=cur_people_x;
      cur_people[i][1]=cur_people_y;


      int human_mapidx=CoordinateTransform_Global2_beliefMap(cur_people_x,cur_people_y);
      index_of_human_occ_cells_updated_recently.push_back(human_mapidx);
      Human_Belief_Scan_map.data[human_mapidx] = 75;

      if (map_index_of_human_cells_to_prob.count(human_mapidx) == 1){
          // Encountered the same cells. Update probability:
          // P(H|S) = P(S|H)P(H) / P(S)
          float prior = map_index_of_human_cells_to_prob[human_mapidx]; // P(H)
          float P_S = P_S_given_H*prior + P_S_given_Hc*(1-prior);
          float posterior = (P_S_given_H)*prior / P_S;
          map_index_of_human_cells_to_prob[human_mapidx] = posterior;

      }else{
          map_index_of_human_cells_to_prob[human_mapidx] = 0.1;
      }		
      //human_occupied_idx.push_back(human_mapidx);

   }

    update_human_occ_belief(HUMANS_DETECTED);

   // printf("size yolo : %d \n",cur_people.size());
}

void belief_manager::update_human_occ_belief(int update_type){

    std::map<int, int> map_index_recently_updated;
    if (update_type == (int) HUMANS_DETECTED){
        for(int i = 0; i < index_of_human_occ_cells_updated_recently.size(); i++){
            int index = index_of_human_occ_cells_updated_recently[i];
            map_index_recently_updated[index] = index;
            //std::cout<<"map_index_recently updated set :" <<index <<std::endl;
        }
    }

    // Extract_world_indices_from_visible_camera_region(float depth, float width, float res)
    // For each cell, check if is labeled as human.
    // If labeled as human, update probability of cell regions
    std::vector<int> indices_to_assign_as_free;

    //std::cout<<"camera visiblie_idx_set size : "<<visiblie_idx_set.size()<<std::endl;
    for(int i(0);i< visiblie_idx_set.size();i++)
    {
        int cell_idx= visiblie_idx_set[i];
        if (map_index_recently_updated.count(cell_idx) == 1){
            continue;
            std::cout<<"continue "<<cell_idx<<std::endl;
        }
        /*
        if(Human_Belief_type_map.data[cell_idx]==HUMAN_OCCUPIED)
        {
            float prior = map_index_of_human_cells_to_prob[cell_idx]; // P(H)
            //float P_S = P_Sc_given_H*(prior) + P_Sc_given_Hc*(1-prior);
            float posterior = prior*0.5;
            map_index_of_human_cells_to_prob[cell_idx] =posterior;

            if (posterior < PROB_THRESH){
                indices_to_assign_as_free.push_back(cell_idx);
                //std::cout << "free index : "<<cell_idx<< ", Prob: " << posterior << std::endl;
            }


        }
        */

        //if (Human_Belief_Scan_map.data[cell_idx]>0.05){
             //Update probability			
            //float prior = map_index_of_human_cells_to_prob[cell_idx]; // P(H)
            //float P_S = P_Sc_given_H*(prior) + P_Sc_given_Hc*(1-prior);
            //float posterior = prior*0.7;
            //map_index_of_human_cells_to_prob[cell_idx] =posterior;
            //map_index_of_human_cells_to_prob[cell_idx] =0.05;
            //std::cout << "index : "<<cell_idx<< ", Prob: " << posterior << std::endl;
            //if (posterior < PROB_THRESH){
                //indices_to_assign_as_free.push_back(cell_idx);
                //std::cout << "free index : "<<cell_idx<< ", Prob: " << posterior << std::endl;
            //}

        //}
        //else{
        
            //map_index_of_human_cells_to_prob[cell_idx] =0.05;
        
        //}
    }

    /*
        for(size_t i = 0; i < indices_to_assign_as_free.size(); i++){
            int index_to_erase =  indices_to_assign_as_free[i];
            map_index_of_human_cells_to_prob.erase(index_to_erase);
            Human_Belief_Scan_map.data[index_to_erase]=0.00;
            Human_Belief_type_map.data[index_to_erase]=0;
        }

    */




}


void belief_manager::put_human_occ_map_yolo()
{
	//int num_size = human_occupied_idx.size();
     //std::cout<<"human_occupied_size"<<num_size<<std::endl;
	//if(num_size>0)
	//{
		//if(Human_Belief_Scan_map.data.size()>0){
			//for(int i(0);i<human_occupied_idx.size();i++){
				 //Human_Belief_Scan_map.data[human_occupied_idx[i]]=80.0;
                 //put_human_surrounding_beliefmap(human_occupied_idx[i]);
			//}
		//}
	//}
	//else
	//{


	//}
}



// callback for messages
int belief_manager::CoordinateTransform_Global2_staticMap(float global_x, float global_y)
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


int belief_manager::CoordinateTransform_Global2_beliefMap(double global_x, double global_y)
{

	double reference_origin_x=Human_Belief_Scan_map.info.origin.position.x;
	double reference_origin_y=Human_Belief_Scan_map.info.origin.position.y;

	//Find the coordinate w.r.t map origin
	double  temp_x  = global_x - reference_origin_x;
	double  temp_y  = global_y - reference_origin_y;

	//Find the map cell idx for x, y
	std::vector<int> human_coord(2,0);
 	human_coord[0]= (int) (temp_x/Human_Belief_Scan_map.info.resolution);
 	human_coord[1]= (int) (temp_y/Human_Belief_Scan_map.info.resolution);

 	//Find the map index from cell x, y
 	int static_map_idx= human_coord[0]+Human_Belief_Scan_map.info.width*human_coord[1];

 	return static_map_idx;

}




bool belief_manager::Comparetwopoistions(std::vector<double> pos,std::vector<double> pos2, double criterion)
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
void belief_manager::spin()
{
  //ROS_INFO("People tracking manager started.");

  while (ros::ok())
  {
    //if(isCalled)
    //{
      update_particles();
     publish_clusters();
     publish_meanpose();
    //}
    //publish_human_belief();

    // ------ LOCKED ------
    boost::mutex::scoped_lock lock(filter_mutex_);
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
  ros::init(argc, argv, "human_belief_manager");
  //ros::NodeHandle(nh);

  // create human_tracker node
  belief_manager human_belief_node("Predictions");
  human_belief_node.spin();
  // Clean up

  return 0;
}
