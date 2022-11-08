#include "prediction_server.h"

using namespace std;
using namespace tf;
using namespace message_filters;

static const double       sequencer_delay            = 0.5; //TODO: this is probably too big, it was 0.8
static const unsigned int sequencer_internal_buffer  = 100;
static const unsigned int sequencer_subscribe_buffer = 10;
static const unsigned int num_particles_tracker      = 1000;
static const double       tracker_init_dist          = 4.0;

static const unsigned char  FREE_SPACE = 0;
static const unsigned char  INSCRIBED_INFLATED_OBSTACLE = 253;
static const unsigned char  LETHAL_OBSTACLE = 254;
static const unsigned char  NO_INFORMATION = 255;

bool IsNotInitilized = true;

// constructor
prediction_manager::prediction_manager(std::string name):as_(nh_, name, boost::bind(&prediction_manager::executeCB, this,_1), false), 
   action_name_(name),costmap_(NULL),min_frontier_size_(1),max_frontier_size_(30), potential_scale_(1e-3), gain_scale_(1.0),blacklist_radius_(0.2),map_res(0.5),
   isTargetDetected(false), Is_Searchmap_received(false), isActionActive(false), costmap_size_x(100),costmap_size_y(100)
{
  // Get Parameters
  //
    nh_.param("MAX_X", MAX_X, {16.0});
    nh_.param("MIN_X", MIN_X, {-20.0});
    nh_.param("MAX_Y", MAX_Y, {20.0});
    nh_.param("MIN_Y", MIN_Y, {-30.0});
    nh_.param("AGENT1_POSE_TOPIC", agent1_pose_topic, {"global_pose_a1_950"});
    nh_.param("AGENT2_POSE_TOPIC", agent2_pose_topic, {"hsr/global_pose"});
    nh_.param("AGENT1_MAP_TOPIC", agent1_map_topic, {"costmap_node/costmap/costmap"});
    nh_.param("AGENT2_MAP_TOPIC", agent2_map_topic, {"hsr/costmap"});
    

  // initialize
  tf2_listener = new tf2_ros::TransformListener(tf_buffer);
   
  global_pose.resize(3,0.0);
  global_pose_a1.resize(3,0.0);
  //Head_Pos.resize(2,0.0);

  //Initialize costmap variable
  last_markers_count_=0;
  unsigned int search_size=costmap_size_x * costmap_size_y;
  costmap_ = new unsigned char[search_size];
  memset(costmap_,NO_INFORMATION, search_size * sizeof(unsigned char));
  //when we receive search_map, costmap should be updated

  //parseparameters(nh_);
  unknown_map_pub=nh_.advertise<nav_msgs::OccupancyGrid>("/unknown_map", 10, true);
  unknown_posearray_pub= nh_.advertise<geometry_msgs::PoseArray>("/unknown_poses", 10, true);

  searchmap_sub =nh_.subscribe<nav_msgs::OccupancyGrid>("/search_map", 10, &prediction_manager::searchmap_callback,this);
  globalpose_sub=nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>(agent1_pose_topic,10,&prediction_manager::global_pose_callback,this);


  as_.start();
}

// destructor
prediction_manager::~prediction_manager()
{
    delete []costmap_;
    costmap_ = NULL;
  // delete sequencer
  // delete all trackers
};

void prediction_manager::executeCB(const visual_perception::UnknownSearchGoalConstPtr &goal)
  {
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
}



void prediction_manager::Mapidx2GlobalCoord(int map_idx, std::vector<double>& global_coords)
{
    global_coords.resize(2,0.0);

    std::vector<int> cell_xy(2,0.0);

    int res = map_idx / Target_Search_map.info.width;
    int div = map_idx % Target_Search_map.info.width;
    //ROS_INFO("res: %d, div : %d", res, div);

    cell_xy[0]=div;
    cell_xy[1]=res;

    global_coords[0] =  Target_Search_map.info.origin.position.x+cell_xy[0]*Target_Search_map.info.resolution;
    global_coords[1] =  Target_Search_map.info.origin.position.y+cell_xy[1]*Target_Search_map.info.resolution;
    //ROS_INFO("global coord x: %.lf, div : %.lf", global_coords[0],global_coords[1]);

}

void prediction_manager::publish_searchmap()
{
   Target_Search_map.header.stamp =  ros::Time::now();
   Target_Search_map.header.frame_id = "map"; 
   searchmap_pub.publish(Target_Search_map);
}


void prediction_manager::publish_target_belief()
{
    typedef std::map<int, float>::iterator map_iter;
    for(map_iter iterator = map_index_of_target_cells_to_prob.begin(); iterator !=  map_index_of_target_cells_to_prob.end(); iterator++) {

        int human_map_idx = iterator->first;
        Human_Belief_Scan_map.data[human_map_idx]=(iterator->second) * 100.0;
    }
    

   Human_Belief_Scan_map.header.stamp =  ros::Time::now();
   Human_Belief_Scan_map.header.frame_id = "map"; 
}




void prediction_manager::searchmap_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    // Copy Data;
	// update dynamic map info
   // ROS_INFO("searchmap_callback");
    Target_Search_map = *msg;
    //Initialize searchmap when this callback function was firstly called
    if(Is_Searchmap_received )
    {
        if(costmap_size_x!=msg->info.width)
            map_dimension_changed=true;
    
    }

    if(!Is_Searchmap_received || map_dimension_changed)
    {
        costmap_size_x=msg->info.width;
        costmap_size_y=msg->info.height;
        MIN_X = msg->info.origin.position.x;
        MIN_Y = msg->info.origin.position.y;
        MAX_X = MIN_X+costmap_size_x*msg->info.resolution;
        MAX_Y = MIN_Y+costmap_size_y*msg->info.resolution;
        unsigned int search_size =costmap_size_x*costmap_size_y;
        costmap_ = new unsigned char[search_size];
        memset(costmap_,NO_INFORMATION, search_size * sizeof(unsigned char));
        Is_Searchmap_received = true;
        unknown_map.info=msg->info;
        unknown_map.data.resize(search_size,0.0);
        unknown_poses.header.frame_id="map";
        
    }
    for(size_t i(0);i<msg->data.size();i++)
    {
        if(msg->data[i]>0) //static_obstacles and dynamic_obstacles
        {
            costmap_[i]=INSCRIBED_INFLATED_OBSTACLE ;
        }
        else if (msg->data[i]==0)
        {
            costmap_[i]=NO_INFORMATION;
        }
        else{
        
            costmap_[i]=FREE_SPACE;
        }
    
    }


}


bool prediction_manager::check_staticObs(float x_pos,float y_pos)
{
  //return true if it is occupied with obstacles
  if (Target_Search_map.data.size()>0)
  {   
      int obs_idx=Coord2CellNum(x_pos,y_pos, Target_Search_map);

    if(Target_Search_map.data[obs_idx]>0)
        return true;
    else
      return false;
  }

}
bool prediction_manager::check_staticObs_dist(float x_pos,float y_pos, float radius)
{
  //return true if it is occupied with obstacles
  if (Target_Search_map.data.size()>0)
  {   
      int nums=8;
      int ang_res=2*MATH_PI/nums;
      for(int j(1);j<=2;j++){
      for(int i(0);i<8;i++)
      {
          double theta_=ang_res*i;
          double test_x=x_pos+radius/j*cos(theta_);
          double test_y=y_pos+radius/j*sin(theta_);
          if(check_staticObs(test_x,test_y))
              return true;
      }

      }
      return false;

  }

}




int prediction_manager::Coord2CellNum(double _x, double _y, const nav_msgs::OccupancyGrid& inputmap_)
{	
    std::vector<int> target_Coord;
    target_Coord.resize(2,0);

    double  temp_x  = _x-inputmap_.info.origin.position.x;
    double  temp_y = _y-inputmap_.info.origin.position.y;

    target_Coord[0]= (int) floor(temp_x/inputmap_.info.resolution);
    target_Coord[1]= (int) floor(temp_y/inputmap_.info.resolution);

    int index= target_Coord[0]+inputmap_.info.width*target_Coord[1];
    return index;
}




double prediction_manager::getDistance_from_Vec(std::vector<double> origin, double _x, double _y)
{
  double temp=0.0;

  temp=(origin[0]-_x)*(origin[0]-_x);
  temp+=(origin[1]-_y)*(origin[1]-_y);
  temp=sqrt(temp);

  return temp;
}

void prediction_manager::global_pose_a1_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
   global_pose_a1[0]=msg->pose.position.x;
   global_pose_a1[1]=msg->pose.position.y;
   global_pose_a1[2]=tf2::getYaw(msg->pose.orientation);
   //global_pose[2]=0.0;

}
//
void prediction_manager::global_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
   global_pose[0]=msg->pose.pose.position.x;
   global_pose[1]=msg->pose.pose.position.y;
   global_pose[2]=tf2::getYaw(msg->pose.pose.orientation);
}

void prediction_manager::target_poses_callback(const geometry_msgs::PoseArray::ConstPtr& msg)
{

    //ROS_INFO("taget poses callback---");

    num_of_detected_target=msg->poses.size();
    index_of_target_occ_cells_updated_recently.clear();

    if(num_of_detected_target>0){
    
       cur_target.resize(num_of_detected_target);
       isTargetDetected=true;
    }
    else
    {
      update_human_occ_belief(NO_HUMANS_DETECTED);
      return;
    }

    for(int i(0);i<num_of_detected_target;i++)
    {
      double cur_target_x=msg->poses[i].position.x;
      double cur_target_y=msg->poses[i].position.y;

      cur_target[i].resize(2,0.0);
      cur_target[i][0]=cur_target_x;
      cur_target[i][1]=cur_target_y;

      //ROS_INFO("---human poses callback---");
      int target_mapidx=CoordinateTransform_Global2_beliefMap(cur_target_x,cur_target_y);
      //if((target_mapidx<0) || (target_mapidx>10000))
          //return;

      if((target_mapidx<Human_Belief_Scan_map.data.size())&& (target_mapidx>0))
      {
          index_of_target_occ_cells_updated_recently.push_back(target_mapidx);
          Human_Belief_Scan_map.data[target_mapidx] = 80;
          Human_Belief_type_map.data[target_mapidx] = HUMAN_OCCUPIED;

         if(map_index_of_target_cells_to_prob.count(target_mapidx) >0 ){
          // Encountered the same cells. Update probability:
          // P(H|S) = P(S|H)P(H) / P(S)
          //std::cout<<"human map idx: " <<target_mapidx <<std::endl;
          float prior = map_index_of_target_cells_to_prob[target_mapidx]; // P(H)
          //std::cout<<"prior : " <<prior <<std::endl;
          float P_S = P_S_given_H*prior + P_S_given_Hc*(1-prior);
          //std::cout<<"P_S : " <<P_S <<std::endl;
          float posterior = (P_S_given_H)*prior / P_S;
          //std::cout<<"posterior : " <<posterior <<std::endl;
          //
          //
          //ROS_INFO("----------index: %d", target_mapidx);
          map_index_of_target_cells_to_prob[target_mapidx] = posterior;

          }else{
            map_index_of_target_cells_to_prob[target_mapidx] = 0.05;
          }		
      }
      //human_occupied_idx.push_back(target_mapidx);
   }

    //ROS_INFO("updatd--------");
    update_human_occ_belief(HUMANS_DETECTED);
}

/**
 * @brief Find nearest cell of a specified value
 * @param result Index of located cell
 * @param start Index initial cell to search from
 * @param val Specified value to search for
 * @param costmap Reference to map data
 * @return True if a cell with the requested value was found
 */

std::vector<unsigned int> prediction_manager::nhood4(unsigned int idx){
    //get 4-connected neighbourhood indexes, check for edge of map
    std::vector<unsigned int> out;

    //unsigned int search_size =Target_Search_map.info.width*Target_Search_map.info.height;
    unsigned int size_x_ = costmap_size_x;
    unsigned int size_y_ = costmap_size_y;

    if (idx > size_x_ * size_y_ -1){
        ROS_WARN("Evaluating nhood for offmap point");
        return out;
    }

    if(idx % size_x_ > 0){
        out.push_back(idx - 1);
    }
    if(idx % size_x_ < size_x_ - 1){
        out.push_back(idx + 1);
    }
    if(idx >= size_x_){
        out.push_back(idx - size_x_);
    }
    if(idx < size_x_*(size_y_-1)){
        out.push_back(idx + size_x_);
    }
    return out;

}

std::vector<unsigned int> prediction_manager::nhood8(unsigned int idx){
    //get 8-connected neighbourhood indexes, check for edge of map
    std::vector<unsigned int> out = nhood4(idx);

    unsigned int size_x_ = costmap_size_x;
    unsigned int size_y_ = costmap_size_y;

    if (idx > size_x_ * size_y_ -1){
        return out;
    }

    if(idx % size_x_ > 0 && idx >= size_x_){
        out.push_back(idx - 1 - size_x_);
    }
    if(idx % size_x_ > 0 && idx < size_x_*(size_y_-1)){
        out.push_back(idx - 1 + size_x_);
    }
    if(idx % size_x_ < size_x_ - 1 && idx >= size_x_){
        out.push_back(idx + 1 - size_x_);
    }
    if(idx % size_x_ < size_x_ - 1 && idx < size_x_*(size_y_-1)){
        out.push_back(idx + 1 + size_x_);
    }

    return out;
}

void prediction_manager::parseparameters(ros::NodeHandle n)
  {
      std::string target_frame;
      n.getParam("waypoints/ref_frame", target_frame);
      std::vector<std::string> locations_vector;
      //std::string locations_list;
      XmlRpc::XmlRpcValue location_list;
      ROS_INFO_STREAM("reference frame: " << target_frame);
      n.getParam("waypoints/locations", location_list);
      //ROS_INFO("type %s",location_list.getType());
      ROS_ASSERT(location_list.getType() ==XmlRpc::XmlRpcValue::TypeArray);
      for(size_t j(0);j<location_list.size();j++)
      {
          //ROS_INFO_STREAM("Loded "<< "-" << (std::string)(location_list[j]));
          std::string location_str = static_cast<std::string>(location_list[j]);
          locations_vector.push_back(location_str);
          LoadPositionfromConfig(n,location_str);
      }
      //ROS_ASSERT(ref_frame.getType() == XmlRpc::XmlRpcValue::TypeStruct);
      //print loaded parameters
      //std::map< std::string, std::vector<double> >::iterator map_iter=goal_maps.begin();
      //for(map_iter; map_iter!=goal_maps.end();map_iter++)
      //{
          //std::cout<<map_iter->first <<" : " <<map_iter->second[0]<< ", "<<map_iter->second[1]<<std::endl;
      //}

}

void prediction_manager::LoadPositionfromConfig(ros::NodeHandle n, std::string input_locations)
{
      XmlRpc::XmlRpcValue input_loc;
      std::string param_name = "waypoints/" + input_locations;
      n.getParam(param_name, input_loc);
      std::vector<double> tmp_pos(2,0.0);
      tmp_pos[0]=static_cast<double>(input_loc[0]);
      tmp_pos[1]=static_cast<double>(input_loc[1]);
      goal_maps[input_locations]=tmp_pos;
}






bool prediction_manager::nearestCell(unsigned int &result, unsigned int start, unsigned char val){

    const unsigned char* map_ = getCharMap();
    //const unsigned int size_x = costmap.getSizeInCellsX(), size_y = costmap.getSizeInCellsY();

    unsigned int search_size =Target_Search_map.info.width*Target_Search_map.info.height;

    if(start >= search_size){
        return false;
    }

    //initialize breadth first search
    std::queue<unsigned int> bfs;
    std::vector<bool> visited_flag(search_size, false);

    //push initial cell
    bfs.push(start);
    visited_flag[start] = true;

    //search for neighbouring cell matching value
    while(!bfs.empty()){
        unsigned int idx = bfs.front();
        bfs.pop();

        //return if cell of correct value is found
        if(map_[idx] == val){
            result = idx;
            return true;
        }

        //iterate over all adjacent unvisited cells
        BOOST_FOREACH(unsigned nbr, nhood8(idx)){
            if(!visited_flag[nbr]){
                bfs.push(nbr);
                visited_flag[nbr] = true;
            }
        }
    }

    return false;
}


unsigned char* prediction_manager::getCharMap() const
{
    return costmap_;
}

std::vector<int> prediction_manager::FindCellFrom(geometry_msgs::Point position, int value_)
{

    //ROS_INFO("FineCellFrom");
    //std::vector<frontier_exploration::Frontier> frontier_list;
    //findUnknownLists
    std::vector<int> idx_set;
    std::vector<int> edge_set;

    unsigned int target_mapidx=(unsigned int) CoordinateTransform_Global2_beliefMap(position.x, position.y);
    unsigned int search_size =Target_Search_map.info.width*Target_Search_map.info.height;
    
     //initialize flag arrays to keep track of visited and frontier cells
    std::vector<bool> edge_flag(search_size, false);
    std::vector<bool> visited_flag(search_size, false);

    //initialize breadth first search
    std::queue<unsigned int> bfs;

    unsigned int clear, pos = target_mapidx;
    //put current cell to the dataset
    if(nearestCell(clear, pos, NO_INFORMATION)){
        bfs.push(clear);
    }else{
        bfs.push(pos);
        ROS_WARN("Could not find nearby clear cell to start search");
    }
    visited_flag[bfs.front()] = true;

    while(!bfs.empty()){
        unsigned int idx = bfs.front();
        //ROS_INFO("idx: %d", idx);
        bfs.pop();

        BOOST_FOREACH(unsigned int nbr,nhood4(idx))
        {
            if(costmap_[nbr]==value_ && !visited_flag[nbr])
            {
                idx_set.push_back(nbr);

                if(hasFreenbr(nbr))
                    edge_set.push_back(nbr);

                bfs.push(nbr);
            }

            visited_flag[nbr] = true;
        }
        
    }

    //ROS_INFO("------connected idx size : %d", idx_set.size());

    return idx_set;
}

bool prediction_manager::isNewFrontierCell(unsigned int idx, const std::vector<bool>& frontier_flag){

    //map_ = costmap_.getCharMap();
    //check that cell is unknown and not already marked as frontier
    if(costmap_[idx] != NO_INFORMATION || frontier_flag[idx]){
        return false;
    }

    //frontier cells should have at least one cell in 4-connected neighbourhood that is free
    BOOST_FOREACH(unsigned int nbr, nhood4(idx)){
        if(costmap_[nbr] == FREE_SPACE){
            return true;
        }
    }

    return false;
}

bool prediction_manager::isNewUnknown(unsigned int idx, const std::vector<bool>& frontier_flag){

    if(frontier_flag[idx]==true)
        return false;
    //map_ = costmap_.getCharMap();
    //check that cell is unknown and not already marked as frontier
    if(costmap_[idx] != NO_INFORMATION){
        return false;
    }

    //frontier cells should not have at least one cell in 4-connected neighbourhood that is not free
    BOOST_FOREACH(unsigned int nbr, nhood4(idx)){
        if(costmap_[nbr] != NO_INFORMATION){
            return false;
        }
    }
    return true;
}




bool prediction_manager::isCellValueandFree(unsigned int idx, int value_){

    //map_ = costmap_.getCharMap();
    //check that cell is unknown and not already marked as frontier
    if(costmap_[idx] != value_){
        return false;
    }

    //frontier cells should have at least one cell in 4-connected neighbourhood that is free
    BOOST_FOREACH(unsigned int nbr, nhood4(idx)){
        if(costmap_[nbr] == FREE_SPACE){
            return true;
        }
    }

    return false;
}

bool prediction_manager::hasFreenbr(unsigned int idx){

    //frontier cells should have at least one cell in 4-connected neighbourhood that is free
    BOOST_FOREACH(unsigned int nbr, nhood4(idx)){
        if(costmap_[nbr] == FREE_SPACE){
            return true;
        }
    }

    return false;
}



double prediction_manager::frontierCost(const frontier_exploration::Frontier& frontier)
{
  return (potential_scale_ * frontier.min_distance *map_res) -
         (gain_scale_ * frontier.size * map_res);
}

void prediction_manager::SplitFrontiers(const frontier_exploration::Frontier& frontier, unsigned int reference_,  std::vector<frontier_exploration::Frontier>& newfrontiers)
{
    newfrontiers.clear();
    int num_frontiers = std::ceil((frontier.size-1)/max_frontier_size_)+1;
    //ROS_INFO("size frontiers: %d", frontier.size);

    int maxnum=4;
    int f_index=0;

    std::vector<double> temp_referencepose(2,0.0);
    Mapidx2GlobalCoord(reference_,temp_referencepose);
    double reference_x=temp_referencepose[0];
    double reference_y=temp_referencepose[1];

    for(size_t i(0);i<num_frontiers;i++)
    {
        frontier_exploration::Frontier output;
        output.centroid.x = 0.0;
        output.centroid.y = 0.0;
        output.size = 0;
        output.min_distance = std::numeric_limits<double>::infinity();
        newfrontiers.push_back(output);
    }
    ROS_INFO("size of newfrontiers: %lu", newfrontiers.size());

    for(size_t j(0); j<(frontier.size-1); j++)
    {
        if(j<max_frontier_size_)
            f_index=0;
        else if(j<2*max_frontier_size_)
            f_index=1;
        else if(j<3*max_frontier_size_)
            f_index=2;
        else
            f_index=3;


        //ROS_INFO("here j: %d, f_index: %d", j, f_index);
        newfrontiers[f_index].points.push_back(frontier.points[j]);
        newfrontiers[f_index].size++;
        newfrontiers[f_index].centroid.x += frontier.points[j].x;
        newfrontiers[f_index].centroid.y += frontier.points[j].y;
        double distance = sqrt(pow((double(reference_x)-double(frontier.points[j].x)),2.0) + pow((double(reference_y)-double(frontier.points[j].y)),2.0));
        if(distance < newfrontiers[f_index].min_distance)
            newfrontiers[f_index].min_distance = distance;
    }

    for(size_t i(0);i<newfrontiers.size();i++)
    {
        newfrontiers[i].centroid.x /= newfrontiers[i].size;
        newfrontiers[i].centroid.y /= newfrontiers[i].size;
        newfrontiers[i].travel_point = newfrontiers[i].centroid;
    }

}


frontier_exploration::Frontier prediction_manager::buildNewUnknown(unsigned int initial_cell, unsigned int reference_, std::vector<bool>& visited_flag){

    int max_size = 35; //100
    //int max_size = 100;
    //initialize frontier structure
    frontier_exploration::Frontier output;
    //geometry_msgs::Point centroid, middle;

    output.centroid.x = 0;
    output.centroid.y = 0;
    output.size = 1;
    output.min_distance = std::numeric_limits<double>::infinity();

    //record initial contact point for frontier
    //get glboal coordinate from map coordinate
    std::vector<double> temp_pose(2,0.0);
    Mapidx2GlobalCoord(initial_cell,temp_pose);
    output.travel_point.x=temp_pose[0];
    output.travel_point.y=temp_pose[1];

    //push initial gridcell onto queue
    std::queue<unsigned int> bfs;
    bfs.push(initial_cell);

    std::vector<double> temp_referencepose(2,0.0);
    Mapidx2GlobalCoord(reference_,temp_referencepose);
    double reference_x=temp_referencepose[0];
    double reference_y=temp_referencepose[1];

    while(!bfs.empty()){
        unsigned int idx = bfs.front();
        bfs.pop();

        //try adding cells in 8-connected neighborhood to frontier
        BOOST_FOREACH(unsigned int nbr, nhood8(idx)){

            //check if neighbour is a potential frontier cell
            if(output.size>max_size)
            {
                //among output.points. find the least distace from static obstacles
                output.centroid.x /= (output.size-1);
                output.centroid.y /= (output.size-1);
                output.centroid.x+=0.05;
                output.centroid.y+=0.05;
                output.travel_point = output.centroid;

                //check if the distance between centroid and any static obstacles
                int posidx=0;
                while(check_staticObs_dist(output.centroid.x, output.centroid.y, 0.2))
                {
                    output.centroid.x=output.points[posidx].x;
                    output.centroid.y=output.points[posidx].y;
                    //output.points.push_back(point);
                    posidx++;
                    if(posidx>=output.points.size())
                        break;
                }

                return output;

            }


            if(isNewUnknown(nbr,visited_flag)){

                //mark cell as frontier
                //visited_flag[nbr] = true;
                std::vector<double> temp_position(2,0.0);
                Mapidx2GlobalCoord(nbr,temp_position);
                double wx = temp_position[0];
                double wy = temp_position[1];
                //unsigned int mx,my;
                //double wx,wy;
                //costmap_.indexToCells(nbr,mx,my);
                //costmap_.mapToWorld(mx,my,wx,wy);
                geometry_msgs::Point point;
                point.x = wx;
                point.y = wy;
                output.points.push_back(point);

                //update frontier size
                output.size++;

                //update centroid of frontier
                output.centroid.x += wx;
                output.centroid.y += wy;

                //determine frontier's distance from robot, going by closest gridcell to robot
                double distance = sqrt(pow((double(reference_x)-double(wx)),2.0) + pow((double(reference_y)-double(wy)),2.0));
                if(distance < output.min_distance){
                    output.min_distance = distance;
                    output.middle.x = wx;
                    output.middle.y = wy;
                }

                visited_flag[nbr] = true;

                //add to queue for breadth first search
                bfs.push(nbr);
            }

            visited_flag[nbr] = true;

        }
    }

    //average out frontier centroid
    if(output.size>2)
    {
        output.centroid.x /= (output.size-1);
        output.centroid.y /= (output.size-1);
        
        output.centroid.x+=0.05;
        output.centroid.y+=0.05;


        output.travel_point = output.centroid;
    }


    return output;
}

/*
void prediction_manager::getNextFrontiers(const std::vector<frontier_exploration::Frontier>& frontiers)
{

    frontier_exploration::Frontier selected;
    selected.min_distance = std::numeric_limits<double>::infinity();

    //pointcloud for visualization purposes
    pcl::PointCloud<pcl::PointXYZI> frontier_cloud_viz;
    pcl::PointXYZI frontier_point_viz(50);
    int max_;

    BOOST_FOREACH(frontier_exploration::Frontier frontier, frontiers){
        //load frontier into visualization poitncloud
        frontier_point_viz.x = frontier.travel_point.x;
        frontier_point_viz.y = frontier.travel_point.y;
        frontier_cloud_viz.push_back(frontier_point_viz);

        //ROS_INFO("list travel points:  %.3lf, %.3lf", frontier.travel_point.x, frontier.travel_point.y);
        //ROS_INFO("min_distance : %.3lf", frontier.min_distance);
        //check if this frontier is the nearest to robot
        //if (frontier.min_distance > 3.0 ){
        if (frontier.min_distance < selected.min_distance ){
            
            selected = frontier;
            max_= frontier_cloud_viz.size()-1;
            }
        }
        //ROS_INFO("max: %d", max);
        //ROS_INFO("list travel points - %.3lf, %.3lf", frontier.travel_point.x, frontier,travel_point.y);

        if (std::isinf(selected.min_distance)) {
            ROS_INFO("No valid (non-blacklisted) frontiers found, exploration complete");
            ROS_DEBUG("No valid (non-blacklisted) frontiers found, exploration complete");
            //isActionActive=false;
            //result_.success = true;
            //as_.setSucceeded(result_);
            return;
    }

   //color selected frontier
   frontier_cloud_viz[max_].intensity = 100;

   //publish visualization point cloud-----------------------------
   sensor_msgs::PointCloud2 frontier_viz_output;
   pcl::toROSMsg(frontier_cloud_viz,frontier_viz_output);
   frontier_viz_output.header.frame_id = "map";
   frontier_viz_output.header.stamp = ros::Time::now();
   frontier_cloud_pub.publish(frontier_viz_output);
   //---------------------------------------------------------------

   //set goal pose to next frontier
   geometry_msgs::PoseStamped next_frontier;
   next_frontier.header.frame_id = "map";
   next_frontier.header.stamp = ros::Time::now();

   next_frontier.pose.position = selected.travel_point;
   //next_frontier.pose.orientation = tf::createQuaternionMsgFromYaw( yawOfVector(start_pose.pose.position, next_frontier.pose.position) );
   nextfrontier_pose_pub.publish(next_frontier);
}
*/


void prediction_manager::visualizeFrontiers(const std::vector<frontier_exploration::Frontier>& frontiers)
{
        std_msgs::ColorRGBA blue;
        blue.r = 0; blue.g = 0; blue.b = 1.0; blue.a = 1.0;
        std_msgs::ColorRGBA red;
        red.r = 1.0; red.g = 0; red.b = 0; red.a = 1.0;
        std_msgs::ColorRGBA green;
        green.r = 0; green.g = 1.0; green.b = 0; green.a = 1.0;

        ROS_DEBUG("visualising %lu frontiers", frontiers.size());
        visualization_msgs::MarkerArray markers_msg;
        std::vector<visualization_msgs::Marker>& markers = markers_msg.markers;
        visualization_msgs::Marker m;

        m.header.frame_id ="map";
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
            m.color=red;
            markers.push_back(m);
            ++id;
            m.type = visualization_msgs::Marker::SPHERE;
            m.id = int(id);
            m.pose.position = frontier.travel_point;
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


     std::vector<std::pair<int,double>> frontier_pairs;

     size_t candidate_size = frontiers.size();

     for(size_t i(0);i<candidate_size;i++)
     {
     
          double temp_dist=getDistance_from_Vec(global_pose,
                  frontiers[i].travel_point.x,
                  frontiers[i].travel_point.y)-frontiers[i].size*0.2;
        //ROS_INFO("frontier position x: %.3lf, y: %.3lf, distance: %.2lf: ",
                //frontiers[i].travel_point.x, frontiers[i].travel_point.y,
                 //temp_dist);

           std::pair<int, double> temp_pair(i,temp_dist);
           frontier_pairs.push_back(temp_pair);
     }


     std::sort(frontier_pairs.begin(),frontier_pairs.end(),comparator);

     //pose_array publisher
     geometry_msgs::PoseArray posearray_msg;
     posearray_msg.header.frame_id="map";
     posearray_msg.header.stamp = ros::Time::now();

     std::vector<std::pair<int,double>>::iterator pit=frontier_pairs.begin();
     for(pit;pit!=frontier_pairs.end();pit++) {
         geometry_msgs::Pose temp_pose;
         temp_pose.position=frontiers[pit->first].travel_point;
         temp_pose.orientation.w=1.0;
         posearray_msg.poses.push_back(temp_pose);
    }
    //frontier_posearray_pub.publish(posearray_msg);

}



void prediction_manager::human_poses_callback(const geometry_msgs::PoseArray::ConstPtr& msg)
{

    //ROS_INFO("human poses callback");
    index_of_target_occ_cells_updated_recently.clear();

    //check number of detected_humans
    num_of_detected_human=msg->poses.size();

    if(num_of_detected_human>0)
       cur_target.resize(num_of_detected_human);
    else
    {
      update_human_occ_belief(NO_HUMANS_DETECTED);
      return;
    }


    ROS_INFO("human poses callback---");
    for(int i(0);i<num_of_detected_human;i++)
    {
      double cur_people_x=msg->poses[i].position.x;
      double cur_people_y=msg->poses[i].position.y;

      cur_target[i].resize(2,0.0);
      cur_target[i][0]=cur_people_x;
      cur_target[i][1]=cur_people_y;

      ROS_INFO("---human poses callback---");
      int target_mapidx=CoordinateTransform_Global2_beliefMap(cur_people_x,cur_people_y);
      //if((target_mapidx<0) || (target_mapidx>10000))
          //return;

      if((target_mapidx<Human_Belief_Scan_map.data.size())&& (target_mapidx>0))
      {
          index_of_target_occ_cells_updated_recently.push_back(target_mapidx);
          Human_Belief_Scan_map.data[target_mapidx] = 80;
          Human_Belief_type_map.data[target_mapidx] = HUMAN_OCCUPIED;

         if(map_index_of_target_cells_to_prob.count(target_mapidx) >0 ){
          // Encountered the same cells. Update probability:
          // P(H|S) = P(S|H)P(H) / P(S)
          //std::cout<<"human map idx: " <<target_mapidx <<std::endl;
          float prior = map_index_of_target_cells_to_prob[target_mapidx]; // P(H)
          //std::cout<<"prior : " <<prior <<std::endl;
          float P_S = P_S_given_H*prior + P_S_given_Hc*(1-prior);
          //std::cout<<"P_S : " <<P_S <<std::endl;
          float posterior = (P_S_given_H)*prior / P_S;
          //std::cout<<"posterior : " <<posterior <<std::endl;
          //
          //
          ROS_INFO("---------------human poses callback---");
          map_index_of_target_cells_to_prob[target_mapidx] = posterior;

          }else{
            map_index_of_target_cells_to_prob[target_mapidx] = 0.05;
          }		
      }
      //human_occupied_idx.push_back(target_mapidx);
   }

    ROS_INFO("human poses callback--------");
    update_human_occ_belief(HUMANS_DETECTED);
   // printf("size yolo : %d \n",cur_target.size());
}


void prediction_manager::update_human_occ_belief(int update_type){

    std::map<int, int> map_index_recently_updated;
    if (update_type == (int) HUMANS_DETECTED){
        for(int i = 0; i < index_of_target_occ_cells_updated_recently.size(); i++){
            int index = index_of_target_occ_cells_updated_recently[i];
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
        if(Human_Belief_type_map.data[cell_idx]==HUMAN_OCCUPIED)
        {
            float prior = map_index_of_target_cells_to_prob[cell_idx]; // P(H)
            //float P_S = P_Sc_given_H*(prior) + P_Sc_given_Hc*(1-prior);
            float posterior = prior*0.5;
            map_index_of_target_cells_to_prob[cell_idx] =posterior;

            if (posterior < PROB_THRESH){
                indices_to_assign_as_free.push_back(cell_idx);
                //std::cout << "free index : "<<cell_idx<< ", Prob: " << posterior << std::endl;
            }
        }
        else
        {
        
            indices_to_assign_as_free.push_back(cell_idx);
        
        }

    }
        for(size_t i = 0; i < indices_to_assign_as_free.size(); i++){
            int index_to_erase =  indices_to_assign_as_free[i];
            map_index_of_target_cells_to_prob.erase(index_to_erase);
            Human_Belief_Scan_map.data[index_to_erase]=0.00;
            Human_Belief_type_map.data[index_to_erase]=0;
            Target_Search_map.data[index_to_erase]=70.0;
            costmap_[index_to_erase]=FREE_SPACE;
        }
}



// callback for messages
int prediction_manager::CoordinateTransform_Global2_staticMap(float global_x, float global_y)
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


int prediction_manager::CoordinateTransform_Global2_beliefMap(double global_x, double global_y)
{

	double reference_origin_x=Target_Search_map.info.origin.position.x;
	double reference_origin_y=Target_Search_map.info.origin.position.y;

	//Find the coordinate w.r.t map origin
	double  temp_x  = global_x - reference_origin_x;
	double  temp_y  = global_y - reference_origin_y;

	//Find the map cell idx for x, y
	std::vector<int> coord(2,0);
 	coord[0]= (int) (temp_x/Target_Search_map.info.resolution);
 	coord[1]= (int) (temp_y/Target_Search_map.info.resolution);

 	//Find the map index from cell x, y
 	int static_map_idx= coord[0]+Target_Search_map.info.width*coord[1];

 	return static_map_idx;

}


bool prediction_manager::Comparetwopoistions(std::vector<double> pos,std::vector<double> pos2, double criterion)
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


void prediction_manager::insert_unknownmap(const std::vector<int> idx_set)
{
    if(unknown_map.data.size()>0)
    {
        for(int id(0);id<idx_set.size();id++ )
        {
            unknown_map.data[idx_set[id]]=50;
        }
    }
}

void prediction_manager::find_avg_pose(const std::vector<int> idx_set, geometry_msgs::Pose& avg_point)
{
    double tmp_x=0.0; 
    double tmp_y=0.0; 
    std::vector<double> temp_coord(2,0.0);
    if(unknown_map.data.size()>0)
    {
        for(int id(0);id<idx_set.size();id++ )
        {

            Mapidx2GlobalCoord(idx_set[id], temp_coord);
            tmp_x+=temp_coord[0];
            tmp_y+=temp_coord[1];
        }
        tmp_x=tmp_x/(idx_set.size());
        tmp_y=tmp_y/(idx_set.size());
    }
    avg_point.position.x=tmp_x;
    avg_point.position.y=tmp_y;
}


//output: unknwon_poses
std::vector<frontier_exploration::Frontier> prediction_manager::Unknown_search(geometry_msgs::Point position, std::vector<bool>& visited_flag, int target_value_=0 )
{
    unsigned int target_mapidx=(unsigned int) CoordinateTransform_Global2_beliefMap(position.x, position.y);
    unsigned int search_size =Target_Search_map.info.width*Target_Search_map.info.height;

    std::vector<frontier_exploration::Frontier> unknowns_list;
    std::queue<unsigned int> bfs;
    //cell_idx
     unsigned int clear, pos = target_mapidx;
    //put current cell to the dataset
    if(nearestCell(clear, pos, NO_INFORMATION)){
        bfs.push(clear);
    }else{
        bfs.push(pos);
        ROS_WARN("Could not find nearby clear cell to start search");
        //return unknowns_list;
    }

    while(!bfs.empty()){
        unsigned int idx = bfs.front();
        bfs.pop();

        BOOST_FOREACH(unsigned int nbr,nhood4(idx))
        {
            if(costmap_[nbr]==NO_INFORMATION && !visited_flag[nbr])
            {

                bfs.push(nbr);
                frontier_exploration::Frontier new_frontier = buildNewUnknown(nbr, pos, visited_flag);
                unknowns_list.push_back(new_frontier);
            }

            visited_flag[nbr] = true;
        }
        
    }

    //check if clusters is in occupied
    for(auto it=unknowns_list.begin(); it!=unknowns_list.end();)
    {
        if(check_staticObs(it->centroid.x, it->centroid.y)){
            it=unknowns_list.erase(it);
        }
        else{
            ++it;
        }
    
    }

    return unknowns_list;

}

void prediction_manager::FindUnknowns()
{
    int search_size =Target_Search_map.data.size() ;
    unknown_poses.poses.clear();
    unknown_map.data.clear();
    unknown_map.data.resize(search_size,0);

    //unknown ==search-map -- update static obstacles
    std::vector<bool> visited_flag(search_size, false); // initialize with static obstacle
    double true_ratio=0.0;

    //auto map_iter=goal_maps.begin();
    //for(map_iter; map_iter!=goal_maps.end();map_iter++)
    int max_iter=50;
    int iter_=0;
    boost::random::mt19937 gen;
    boost::random::uniform_real_distribution<> distx(MIN_X, MAX_X);
    boost::random::uniform_real_distribution<> disty(MIN_Y, MAX_Y);

    while((true_ratio<0.975) && (iter_<max_iter))
    {
        geometry_msgs::Point init_pose;

        float xrange = MAX_X- MIN_X;
        float yrange = MAX_Y- MIN_Y;
        //float random = range * ((((float) rand()) / (float) RAND_MAX)) + MIN_RAND ;

        init_pose.x=distx(gen);
        init_pose.y=disty(gen);
        //init_pose.y=map_iter->second[1];
        ROS_INFO("init_pos x: %.2lf, init_pos_y : %.2lf", init_pose.x, init_pose.y);
        auto frontier_list = Unknown_search(init_pose, visited_flag);
        for(auto frontier :frontier_list)  
        {
            geometry_msgs::Pose avg_pose;
            avg_pose.position.x=frontier.centroid.x;
            avg_pose.position.y=frontier.centroid.y;
            avg_pose.orientation.w=1.0;
            ROS_INFO("avg_x: %.2lf, avg_y: %.2lf", avg_pose.position.x, avg_pose.position.y);
            unknown_poses.poses.push_back(avg_pose);
        }

        true_ratio = get_visited_ratio(visited_flag);
        ROS_INFO("true_ratio: %.2lf", true_ratio);
        
        unknown_poses.header.stamp = ros::Time::now();
        unknown_posearray_pub.publish(unknown_poses);
        iter_++;
    }

    unknown_map_pub.publish(unknown_map);
    unknown_poses.header.stamp = ros::Time::now();
    unknown_posearray_pub.publish(unknown_poses);
    //ROS_INFO("publish_unknown");

}


double prediction_manager::get_visited_ratio(std::vector<bool>& visited_flag)
{
    int total = visited_flag.size();

    double ratio_=0.0;
    
    int count_=0;
    for(auto flag: visited_flag)
    {
        if(flag==true)
            count_++;
    }
    ROS_INFO("total: % d, count!! : %d",total,  count_);
    if(total>0)
        ratio_=(double)count_/(double)total;

    return ratio_;
}


// filter loop
void prediction_manager::spin()
{
  while (ros::ok())
  {
    //ROS_INFO("Search started.");
    if(Is_Searchmap_received){
        //FindUnknowns();
	unknown_poses.header.stamp = ros::Time::now();
	unknown_posearray_pub.publish(unknown_poses);
    }

    //searchfrom
    //lock.unlock();
    // ------ LOCKED ------
    ros::Duration(3.0).sleep();
    ros::spinOnce();
  }
};
// ----------
// -- MAIN --
// ----------
int main(int argc, char **argv)
{
  // Initialize ROS
  ros::init(argc, argv, "getunknowns");
  prediction_manager f_manager("getunknowns");
  f_manager.spin();

  return 0;
}
