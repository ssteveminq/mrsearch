#include "state_lattice_planner/state_lattice_planner_ros.h"

StateLatticePlannerROS::StateLatticePlannerROS(void)
:local_nh("~")
{
    local_nh.param("HZ", HZ, {5});// {1.0});
    local_nh.param("ROBOT_FRAME", ROBOT_FRAME, {"base"});
    local_nh.param("N_P", N_P, {16});
    local_nh.param("N_H", N_H, {6});
    local_nh.param("MAX_ALPHA", MAX_ALPHA, {M_PI / 2.0});
    local_nh.param("MAX_PSI", MAX_PSI, {M_PI / 2.0});
    local_nh.param("N_S", N_S, {1000});
    local_nh.param("MAX_ACCELERATION", MAX_ACCELERATION , {0.3});
    local_nh.param("TARGET_VELOCITY", TARGET_VELOCITY, {0.1});
    local_nh.param("LOOKUP_TABLE_FILE_NAME", LOOKUP_TABLE_FILE_NAME, {std::string(std::getenv("HOME")) + "/lookup_table.csv"});
    local_nh.param("MAX_ITERATION", MAX_ITERATION, {100});
    local_nh.param("OPTIMIZATION_TOLERANCE", OPTIMIZATION_TOLERANCE, {0.15});
    local_nh.param("MAX_YAWRATE", MAX_YAWRATE, {0.25});
    local_nh.param("MAX_D_YAWRATE", MAX_D_YAWRATE, {0.2});
    local_nh.param("MAX_WHEEL_ANGULAR_VELOCITY", MAX_WHEEL_ANGULAR_VELOCITY, {8});
    local_nh.param("WHEEL_RADIUS", WHEEL_RADIUS, {0.125});
    local_nh.param("TREAD", TREAD, {0.5});
    local_nh.param("IGNORABLE_OBSTACLE_RANGE", IGNORABLE_OBSTACLE_RANGE, {1.0});
    local_nh.param("VERBOSE", VERBOSE, {false});
    local_nh.param("CONTROL_DELAY", CONTROL_DELAY, {1});
    local_nh.param("TURN_DIRECTION_THRESHOLD", TURN_DIRECTION_THRESHOLD, {M_PI/3.0});
    local_nh.param("ENABLE_SHARP_TRAJECTORY", ENABLE_SHARP_TRAJECTORY, {false});
    local_nh.param("ENABLE_CONTROL_SPACE_SAMPLING", ENABLE_CONTROL_SPACE_SAMPLING, {false});
    local_nh.param("MAP_RANGE", MAP_RANGE, {7.0});
    local_nh.param("SUBGOAL_RANGE", SUBGOAL_RANGE, {0.65});
    local_nh.param("GOAL_RANGE", GOAL_RANGE, {0.2});

    std::cout << "HZ: " << HZ << std::endl;
    std::cout << "ROBOT_FRAME: " << ROBOT_FRAME << std::endl;
    std::cout << "N_P: " << N_P << std::endl;
    std::cout << "N_H: " << N_H << std::endl;
    std::cout << "MAX_ALPHA: " << MAX_ALPHA << std::endl;
    std::cout << "MAX_PSI: " << MAX_PSI << std::endl;
    std::cout << "N_S: " << N_S << std::endl;
    std::cout << "MAX_ACCELERATION: " << MAX_ACCELERATION << std::endl;
    std::cout << "TARGET_VELOCITY: " << TARGET_VELOCITY << std::endl;
    std::cout << "LOOKUP_TABLE_FILE_NAME: " << LOOKUP_TABLE_FILE_NAME << std::endl;
    std::cout << "MAX_ITERATION: " << MAX_ITERATION << std::endl;
    std::cout << "OPTIMIZATION_TOLERANCE: " << OPTIMIZATION_TOLERANCE << std::endl;
    std::cout << "MAX_YAWRATE: " << MAX_YAWRATE << std::endl;
    std::cout << "MAX_D_YAWRATE: " << MAX_D_YAWRATE << std::endl;
    std::cout << "MAX_WHEEL_ANGULAR_VELOCITY: " << MAX_WHEEL_ANGULAR_VELOCITY << std::endl;
    std::cout << "WHEEL_RADIUS: " << WHEEL_RADIUS << std::endl;
    std::cout << "TREAD: " << TREAD << std::endl;
    std::cout << "IGNORABLE_OBSTACLE_RANGE: " << IGNORABLE_OBSTACLE_RANGE << std::endl;
    std::cout << "VERBOSE: " << VERBOSE << std::endl;
    std::cout << "CONTROL_DELAY: " << CONTROL_DELAY << std::endl;
    std::cout << "TURN_DIRECTION_THRESHOLD: " << TURN_DIRECTION_THRESHOLD << std::endl;
    std::cout << "ENABLE_SHARP_TRAJECTORY: " << ENABLE_SHARP_TRAJECTORY << std::endl;
    std::cout << "ENABLE_CONTROL_SPACE_SAMPLING: " << ENABLE_CONTROL_SPACE_SAMPLING << std::endl;
    std::cout << "SUBGOAL_RANGE: " << SUBGOAL_RANGE<< std::endl;
    std::cout << "FINALGOAL_RANGE: " << GOAL_RANGE<< std::endl;

    planner.set_sampling_params(StateLatticePlanner::SamplingParams(N_P, N_H, MAP_RANGE, MAX_ALPHA, MAX_PSI));
    planner.set_optimization_params(MAX_ITERATION, OPTIMIZATION_TOLERANCE);
    planner.set_vehicle_params(WHEEL_RADIUS, TREAD);
    planner.set_motion_params(MAX_ACCELERATION, MAX_YAWRATE, MAX_D_YAWRATE);
    planner.set_target_velocity(TARGET_VELOCITY);

    //tf2_listener= new tf2_ros::TransformListener(tf_buffer);

    velocity_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    candidate_trajectories_pub = local_nh.advertise<visualization_msgs::MarkerArray>("candidate_trajectories", 1);
    candidate_trajectories_pub2 = local_nh.advertise<visualization_msgs::MarkerArray>("candidate_trajectories2", 1);
    candidate_trajectories_no_collision_pub = local_nh.advertise<visualization_msgs::MarkerArray>("candidate_trajectories/no_collision", 1);
    selected_trajectory_pub = local_nh.advertise<visualization_msgs::Marker>("selected_trajectory", 1);
    selected_trajectory_goal_pub = local_nh.advertise<visualization_msgs::Marker>("selected_trajectory_endpoint", 1);
    local_goal_pub= local_nh.advertise<geometry_msgs::PoseStamped>("local_goal_recieved", 1);
    goal_status_pub=local_nh.advertise<std_msgs::Bool>("goal_reached", 1);


    local_goal_sub = nh.subscribe("/local_goal", 1, &StateLatticePlannerROS::local_goal_callback, this);
    //local_map_sub = nh.subscribe("/scaled_static_map", 1, &StateLatticePlannerROS::local_map_callback, this);
    local_map_sub = nh.subscribe("/costmap_node/costmap/costmap", 1, &StateLatticePlannerROS::local_map_callback, this);
    global_map_sub = nh.subscribe("/scaled_static_map", 1, &StateLatticePlannerROS::global_map_callback, this);
    odom_sub = nh.subscribe("/odom", 1, &StateLatticePlannerROS::odom_callback, this);
    pose_sub = nh.subscribe("/global_pose_a1", 1, &StateLatticePlannerROS::pose_callback, this);
    target_velocity_sub = nh.subscribe("/target_velocity", 1, &StateLatticePlannerROS::target_velocity_callback, this);

    local_goal_subscribed = false;
    local_map_updated = false;
    odom_updated = false;
    goal_out_of_map=false;
    goal_reached=false;
    control_mode=false;
    local_map_count=0;
    stack_count=0;
    zero_count=0;
    rot_gain=0.1;
    last_time_trj=0.0;

    planner.load_lookup_table(LOOKUP_TABLE_FILE_NAME);
    std::cout<<"Planner Initialized done!!"<<std::endl;
}

void StateLatticePlannerROS::local_goal_callback(const geometry_msgs::PoseStampedConstPtr& msg)
{
    ROS_INFO("local goal callback");
    local_goal = *msg;
    try{
        //listener.transformPose("/odom", ros::Time(0), local_goal, local_goal.header.frame_id, local_goal);
        listener.transformPose("/map", ros::Time(0), local_goal, local_goal.header.frame_id, local_goal);
        local_goal_subscribed = true;
    }catch(tf::TransformException ex){
        std::cout << ex.what() << std::endl;
    }
}

void StateLatticePlannerROS::pose_callback(const geometry_msgs::PoseStampedConstPtr& msg)
{
    //ROS_INFO("pose_callbackcallback");
    cur_pose= *msg;
    //try{
        //listener.transformPose("/odom", ros::Time(0), local_goal, local_goal.header.frame_id, local_goal);
        //listener.transformPose("map", ros::Time(0), cur_pose, cur_pose.header.frame_id,cur_pose);
    //}catch(tf::TransformException ex){
        //std::cout << ex.what() << std::endl;
    //}
    //odom_updated =true;
}


void StateLatticePlannerROS::local_map_callback(const nav_msgs::OccupancyGridConstPtr& msg)
{

    ROS_INFO("local map callback");
    local_map = *msg;
    local_map_updated = true;
    local_map_count=0;
}

void StateLatticePlannerROS::global_map_callback(const nav_msgs::OccupancyGridConstPtr& msg)
{
    ROS_INFO("global map callback");
    global_map = *msg;
    global_map_updated = true;
}


void StateLatticePlannerROS::odom_callback(const nav_msgs::OdometryConstPtr& msg)
{

    //ROS_INFO("odom callback");
    current_velocity = msg->twist.twist;
    odom_updated = true;
}



void StateLatticePlannerROS::target_velocity_callback(const geometry_msgs::TwistConstPtr& msg)
{
    if(msg->linear.x > 0.0){
        TARGET_VELOCITY = msg->linear.x;
        std::cout << "\033[31mtarget velocity was updated to " << TARGET_VELOCITY << "[m/s]\033[0m" << std::endl;
    }
}


void StateLatticePlannerROS::process(void)
{
    ros::Rate loop_rate(HZ);

    while(ros::ok()){
        bool goal_transformed = false;
        geometry_msgs::PoseStamped local_goal_base_link;
        if(local_goal_subscribed){
            //try{
              //tf2::doTransform(*msg, *msg, map_en_to_map);
              //tf_buffer.transform(local_goal,local_goal_base_link, "base");

              //double temp_dist = 0.0;
                //temp_dist=sqrt(pow(local_goal.pose.position.x,2)+pow(local_goal.pose.position.y,2));
                //ROS_INFO("the distance to the goal : %.2lf", temp_dist);
                //if(temp_dist<0.75)
                //{
                    //ROS_INFO("The robot is near Goal");
                    //geometry_msgs::Twist cmd_vel;
                    //cmd_vel.linear.x = 0.00;
                    //cmd_vel.angular.z = 0.00;
                    //velocity_pub.publish(cmd_vel);
                    //return;
                //}
                //else if(temp_dist<MAP_RANGE/2)
                //{
                    //goal_out_of_map=true;
                    //ROS_INFO("Goal is outside of costmap!");
                //}
                //else
                    //goal_out_of_map=false;
                //goal_transformed = true;

            //}
            //catch (tf2::TransformException &ex)
            //{
                  //ROS_WARN("global_pose: Transform Failed: %s", ex.what());
                  //std::cout << ex.what() << std::endl;
                  //sleep(0.01);
            //}
            try{
                //listener.transformPose(ROBOT_FRAME, ros::Time(0), local_goal, local_goal.header.frame_id, local_goal_base_link);
                listener.transformPose("base", ros::Time(0), local_goal, local_goal.header.frame_id, local_goal_base_link);
                local_goal_pub.publish(local_goal);
                double temp_dist = 0.0;
                temp_dist=sqrt(pow(local_goal_base_link.pose.position.x,2)+pow(local_goal_base_link.pose.position.y,2));
                ROS_INFO("the distance to the goal : %.2lf", temp_dist);
                if(temp_dist<0.75)
                {
                    ROS_INFO("-----The robot is near Goal--------");
                    ROS_INFO("-----------Wait for next goal---------");
                    geometry_msgs::Twist cmd_vel;
                    cmd_vel.linear.x = 0.00;
                    cmd_vel.angular.z = 0.00;
                    velocity_pub.publish(cmd_vel);
                    local_goal_subscribed=false;
                }
                else if(temp_dist<MAP_RANGE/2)
                {
                    goal_out_of_map=false;
                    ROS_INFO("Goal is inside of costmap!");
                }
                else
                    goal_out_of_map=true;
                goal_transformed = true;
            }catch(tf::TransformException ex){
                std::cout << ex.what() << std::endl;
            }
        }
        if(local_goal_subscribed && local_map_updated && odom_updated && goal_transformed){
            std::cout << "=== state lattice planner ===" << std::endl;
            double start = ros::Time::now().toSec();
            static int last_trajectory_num = 0;
            //std::cout << "local goal: \n" << local_goal_base_link << std::endl;
            std::cout << "current_velocity: \n" << current_velocity << std::endl;
            Eigen::Vector3d goal(local_goal_base_link.pose.position.x, local_goal_base_link.pose.position.y, tf::getYaw(local_goal_base_link.pose.orientation));
            //std::cout << "goal w.r.t base: " << "current_velocity << std::endl;
            std::vector<Eigen::Vector3d> states;
            double target_velocity = planner.get_target_velocity(goal);
            //planner.generate_biased_polar_states(N_S, goal, target_velocity, states);
            planner.generate_biased_polar_states_double(N_S, goal, target_velocity, states);
            std::vector<MotionModelDiffDrive::Trajectory> trajectories;
            bool generated = planner.generate_trajectories(states, current_velocity.linear.x, current_velocity.angular.z, target_velocity, trajectories);
            if(ENABLE_SHARP_TRAJECTORY){
                generated |= planner.generate_trajectories(states, current_velocity.linear.x, current_velocity.angular.z + MAX_D_YAWRATE / HZ, target_velocity, trajectories);
                generated |= planner.generate_trajectories(states, current_velocity.linear.x, current_velocity.angular.z - MAX_D_YAWRATE / HZ, target_velocity, trajectories);
            }
            bool turn_flag = false;
            double relative_direction = atan2(local_goal_base_link.pose.position.y, local_goal_base_link.pose.position.x);
            if(goal.segment(0, 2).norm() < 0.15){
                std::cout<<"norm failed"<<std::endl;
                generated = false;
            }else if(fabs(relative_direction) > TURN_DIRECTION_THRESHOLD){
                //if(fabs(goal(2)) > TURN_DIRECTION_THRESHOLD){
                    //std::cout<<"turn direction threshold failed"<<std::endl;
                    generated = false;
                    turn_flag = true;
                //}
            }
            if(generated){
                //std::vector<MotionModelDiffDrive::Trajectory> rot_trjs;
                //rotate_trajectories(trajectories, rot_trjs);
                visualize_trajectories(trajectories, 0, 1, 0, last_trajectory_num, candidate_trajectories_pub);
                //visualize_trajectories(rot_trjs, 0, 0, 1, last_trajectory_num, candidate_trajectories_pub2);


                std::cout << "check candidate trajectories" << std::endl;
                std::vector<MotionModelDiffDrive::Trajectory> candidate_trajectories;
                state_lattice_planner::ObstacleMap<int> obstacle_map;

                //if goal is in local map -- just check local costmap
                /*
                if(!goal_out_of_map)
                {
                    int trj_cont=0;
                    for(const auto& trajectory : trajectories){
                        if(!check_collision_rosmap(local_map, trajectory.trajectory)){
                            candidate_trajectories.push_back(trajectory);
                        }
                        else{
                            trj_cont++;
                        }
                    }
                    std::cout<<trj_cont<<": The number of igonored trjaectories due to collision with LOCAL MAP: "<<std::endl;
                }
                else
                {
                    int trj_cont=0;
                    for(const auto& trajectory : trajectories){
                        if(!check_collision_rosmap(global_map, trajectory.trajectory)){
                            if(!check_collision_rosmap(local_map, trajectory.trajectory))
                                candidate_trajectories.push_back(trajectory);
                        }
                        else{
                            trj_cont++;
                        }
                    }
                    std::cout<<trj_cont<<": The number of igonored trjaectories due to GLOBAL & LOCAL MAP: "<<std::endl;
                    //for(const auto& trajectory : trajectories)
                            //candidate_trajectories.push_back(trajectory);
                }
                //std::cout << "trajectories: " << trajectories.size() << std::endl;
                //std::cout << "candidate_trajectories: " << candidate_trajectories.size() << std::endl;
                
                if(candidate_trajectories.empty()){
                    ROS_INFO("No valid trjaectory candidates- check only local map");
                    // if no candidate trajectories
                    // collision checking with relaxed restrictions
                    int trj_cont=0;
                    for(const auto& trajectory : trajectories){
                        if(!check_collision_rosmap(local_map, trajectory.trajectory)){
                            candidate_trajectories.push_back(trajectory);
                        }
                        else{
                            trj_cont++;
                        }
                    }
                }

                */
                std::cout << "candidate_trajectories(ignore far obstacles): " << candidate_trajectories.size() << std::endl;
                //*/
                // std::cout << "candidate time: " << ros::Time::now().toSec() - start << "[s]" << std::endl;
                if(candidate_trajectories.size() > 0){
                    visualize_trajectories(candidate_trajectories, 0, 0.5, 1, last_trajectory_num, candidate_trajectories_no_collision_pub);

                    std::cout << "pickup a optimal trajectory from candidate trajectories" << std::endl;
                    MotionModelDiffDrive::Trajectory trajectory;
                    planner.pickup_trajectory(candidate_trajectories, goal, trajectory);
                    visualize_trajectory(trajectory, 1, 0, 0, selected_trajectory_pub);
                    std::cout << "pickup time: " << ros::Time::now().toSec() - start << "[s]" << std::endl;
                    // int size = trajectory.trajectory.size();
                    // for(int i=0;i<size;i++){
                    //     std::cout << trajectory.trajectory[i].transpose() << ", " << trajectory.velocities[i] << "[m/s], " << trajectory.angular_velocities[i] << "[rad/s]" << std::endl;
                    // }

                    std::cout << "publish velocity" << std::endl;
                    geometry_msgs::Twist cmd_vel;
                    double calculation_time = ros::Time::now().toSec() - start;
                    int delayed_control_index = std::min(std::ceil(calculation_time * HZ) + CONTROL_DELAY, (double)trajectory.trajectory.size());
                    std::cout<<"delayed_control_index: "<<delayed_control_index <<std::endl;
                    if((int)trajectory.trajectory.size() < CONTROL_DELAY){
                        delayed_control_index = std::ceil(calculation_time * HZ);
                    }

                    std::cout << calculation_time << ", " << delayed_control_index << std::endl;
                    cmd_vel.linear.x = 1.0*trajectory.velocities[delayed_control_index];
                    cmd_vel.angular.z = 0.3*trajectory.angular_velocities[delayed_control_index];
                    velocity_pub.publish(cmd_vel);




                    
                    std::cout << "published velocity: \n" << cmd_vel << std::endl;

                    local_map_updated = false;
                    odom_updated = false;
                    stack_count=0;
                }else{
                    std::cout << "\033[91mERROR: stacking\033[00m" << std::endl;
                    geometry_msgs::Twist cmd_vel;
                    cmd_vel.angular.z = 0.03;
                    velocity_pub.publish(cmd_vel);
                    stack_count++;
                    ROS_INFO("stack count: %d", stack_count);
                    // for clear
                    //std::vector<MotionModelDiffDrive::Trajectory> clear_trajectories;
                    //visualize_trajectories(clear_trajectories, 0, 1, 0, last_trajectory_num, candidate_trajectories_pub);
                    //visualize_trajectories(clear_trajectories, 0, 0.5, 1, last_trajectory_num, candidate_trajectories_no_collision_pub);
                    //visualize_trajectory(MotionModelDiffDrive::Trajectory(), 1, 0, 0, selected_trajectory_pub);
                }
            }else{
                std::cout << "\033[91mERROR: no optimized trajectory was generated\033[00m" << std::endl;
                std::cout << "\033[91mturn for local goal\033[00m" << std::endl;
                geometry_msgs::Twist cmd_vel;
                cmd_vel.linear.x = 0;
                if(!turn_flag){
                    cmd_vel.angular.z = 0.25*std::min(std::max(goal(2), -MAX_YAWRATE), MAX_YAWRATE);
                    std::cout<<"cmd_vel.angular.z "<<cmd_vel.angular.z<<std::endl;
                }else{
                    cmd_vel.angular.z = 0.25*std::min(std::max(relative_direction, -MAX_YAWRATE), MAX_YAWRATE);
                    //cmd_vel.angular.z = -0.05;
                    std::cout<<"cmd_vel.angular.z "<<cmd_vel.angular.z<<std::endl;
                }
                velocity_pub.publish(cmd_vel);
                stack_count=0;
                // for clear
                std::vector<MotionModelDiffDrive::Trajectory> clear_trajectories;
                std::cout<<"clear trjaectories"<<std::endl;
                //try{
                    //visualize_trajectories(clear_trajectories, 0, 1, 0, last_trajectory_num, candidate_trajectories_pub);
                    //visualize_trajectories(clear_trajectories, 0, 0.5, 1, last_trajectory_num, candidate_trajectories_no_collision_pub);
                    //visualize_trajectory(MotionModelDiffDrive::Trajectory(), 1, 0, 0, selected_trajectory_pub);
                //}
                //catch(const char* msg){
                    //std::cout <<"error"<< std::endl;
                //}

                //std::cout<<"clear trjaectories done"<<std::endl;
            }
            last_trajectory_num = trajectories.size();
            std::cout << "final time: " << ros::Time::now().toSec() - start << "[s]" << std::endl;
        }else{
            if(!local_goal_subscribed){
                std::cout << "waiting for local goal" << std::endl;
            }
            if(!local_map_updated){
                std::cout << "waiting for local map" << std::endl;
                local_map_count++;
                if(local_map_count>5)
                {
                    local_map_updated=true;
                    local_map_count=0;
                }
            }
            if(!odom_updated){
                std::cout << "waiting for odom" << std::endl;
            }
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void StateLatticePlannerROS::visualize_trajectories(const std::vector<MotionModelDiffDrive::Trajectory>& trajectories, const double r, const double g, const double b, const int trajectories_size, const ros::Publisher& pub)
{
    visualization_msgs::MarkerArray v_trajectories;
    int count = 0;
    const int size = trajectories.size();
    for(;count<size;count++){
        visualization_msgs::Marker v_trajectory;
        v_trajectory.header.frame_id = ROBOT_FRAME;
        v_trajectory.header.stamp = ros::Time::now();
        v_trajectory.color.r = r;
        v_trajectory.color.g = g;
        v_trajectory.color.b = b;
        v_trajectory.color.a = 0.8;
        v_trajectory.ns = pub.getTopic();
        v_trajectory.type = visualization_msgs::Marker::LINE_STRIP;
        v_trajectory.action = visualization_msgs::Marker::ADD;
        v_trajectory.lifetime = ros::Duration(10.0);
        v_trajectory.id = count;
        v_trajectory.pose.orientation.w = 1.0;
        v_trajectory.scale.x = 0.02;
        geometry_msgs::Point p;
        for(const auto& pose : trajectories[count].trajectory){
            p.x = pose(0);
            p.y = pose(1);
            v_trajectory.points.push_back(p);
        }
        v_trajectories.markers.push_back(v_trajectory);
    }
    for(;count<trajectories_size;){
        visualization_msgs::Marker v_trajectory;
        v_trajectory.header.frame_id = ROBOT_FRAME;
        v_trajectory.header.stamp = ros::Time::now();
        v_trajectory.ns = pub.getTopic();
        v_trajectory.type = visualization_msgs::Marker::LINE_STRIP;
        v_trajectory.action = visualization_msgs::Marker::DELETE;
        v_trajectory.lifetime = ros::Duration(10.0);
        v_trajectory.id = count;
        v_trajectories.markers.push_back(v_trajectory);
        count++;
    }
    pub.publish(v_trajectories);
}

void StateLatticePlannerROS::visualize_trajectory(const MotionModelDiffDrive::Trajectory& trajectory, const double r, const double g, const double b, const ros::Publisher& pub)
{
    visualization_msgs::Marker v_trajectory;
    v_trajectory.header.frame_id = ROBOT_FRAME;
    v_trajectory.header.stamp = ros::Time::now();
    v_trajectory.color.r = r;
    v_trajectory.color.g = g;
    v_trajectory.color.b = b;
    v_trajectory.color.a = 0.8;
    v_trajectory.ns = pub.getTopic();
    v_trajectory.type = visualization_msgs::Marker::LINE_STRIP;
    v_trajectory.action = visualization_msgs::Marker::ADD;
    v_trajectory.lifetime = ros::Duration(10.0);
    v_trajectory.pose.orientation.w = 1.0;
    v_trajectory.pose.position.z = 0.1;
    v_trajectory.scale.x = 0.10;
    geometry_msgs::Point p;
    for(const auto& pose : trajectory.trajectory){
        p.x = pose(0);
        p.y = pose(1);
        v_trajectory.points.push_back(p);
    }
    pub.publish(v_trajectory);

    geometry_msgs::Point lastp;
    lastp.x=trajectory.trajectory.back()[0];
    lastp.y=trajectory.trajectory.back()[1];

    visualization_msgs::Marker p_trajectory;
    p_trajectory.header.frame_id = ROBOT_FRAME;
    p_trajectory.header.stamp = ros::Time::now();
    p_trajectory.color.r = r;
    p_trajectory.color.g = g;
    p_trajectory.color.b = b;
    p_trajectory.color.a = 0.8;
    p_trajectory.ns = "selected_trajectory_endpoint";
    p_trajectory.type = visualization_msgs::Marker::POINTS;
    p_trajectory.action = visualization_msgs::Marker::ADD;
    p_trajectory.lifetime = ros::Duration(10.0);
    p_trajectory.pose.orientation.w = 1.0;
    p_trajectory.pose.position.z = 0.1;
    p_trajectory.scale.x = 0.10;
    p_trajectory.points.push_back(lastp);
    selected_trajectory_goal_pub.publish(p_trajectory);
}

template<typename TYPE>
void StateLatticePlannerROS::get_obstacle_map(const nav_msgs::OccupancyGrid& input_map, state_lattice_planner::ObstacleMap<TYPE>& output_map)
{
    output_map.set_shape(input_map.info.width, input_map.info.height, input_map.info.resolution);
    output_map.data.clear();
    for(const auto& data : input_map.data){
        output_map.data.emplace_back(data);
    }
    //print data
    //std::cout<<"----------------"<<std::endl;
    //std::cout<<"map data: [ ";
    //for(int i(0);i<output_map.data.size();i++)
        //std::cout<<output_map.data[i]<<", ";
    //std::cout<<"----------------"<<std::endl;

}


int StateLatticePlannerROS::get_mapidx_from_xy(const nav_msgs::OccupancyGrid& map_, const double x_, const double y_)
{

    //ROS_INFO("(x: %.3lf ,y: %.3lf): ", x_, y_);
    double  map_res = map_.info.resolution;
    double  temp_x  = x_-map_.info.origin.position.x;
    double  temp_y  = y_-map_.info.origin.position.y;

    int cell_x= (int) (temp_x/map_res);
    int cell_y= (int) (temp_y/map_res);

    if(cell_x>map_.info.width || cell_y>map_.info.height)
    {
        //ROS_INFO("(x,y) is out of local map");
        return -1;
    }
    else{
    
        int index= cell_x+map_.info.width*cell_y;
        return index;
    }
}

bool StateLatticePlannerROS::check_collision_rosmap(const nav_msgs::OccupancyGrid& map_, const std::vector<Eigen::Vector3d>& trajectory, const std::vector<double>& robot_pose)
{
    //check global map 
    int trj_size = trajectory.size();
    int skip_const=3;
    //map checking
    std::vector<double> coord(2,0.0);

    double robot_x=robot_pose[0];
    double robot_y=robot_pose[1];
    double robot_yaw=robot_pose[2];
    if(robot_yaw>M_PI)
        robot_yaw-=M_PI;
    if(robot_yaw<-M_PI)
        robot_yaw+=M_PI;

    double trans_trj_x=0.0;
    double trans_trj_y=0.0;
    for(int i(0);i<trj_size;i++){
        if(i%2==0)
        {
            trans_trj_x=robot_x+cos(robot_yaw)*trajectory[i][0]-sin(robot_yaw)*trajectory[i][1];
            trans_trj_y=robot_y+sin(robot_yaw)*trajectory[i][0]+cos(robot_yaw)*trajectory[i][1];
            //int index = get_mapidx_from_xy(map_, (robot_x+cos(robot_yaw)*trajectory[i][0]),(robot_y+trajectory[i][1]));
            int index = get_mapidx_from_xy(map_, trans_trj_x,trans_trj_y);
            if(index<0)
            {
                std::cout<<"index wrong, x: "<<robot_x+trajectory[i][0]<<", y: "<<robot_x+trajectory[i][1]<<std::endl;
                continue;
            }
            else{
                if(int(map_.data[index])>0)
                {
                    get_xy_from_index(map_, index, coord);
                    //std::cout<<"collision!!- x: "<<coord[0]<<", y: "<<coord[1]<<std::endl;
                    return true;
                }
            }
        }
    }
    return false;

}


void StateLatticePlannerROS::get_xy_from_index(const nav_msgs::OccupancyGrid& map_, int idx_, std::vector<double>& Coord)
{
    //std::vector<double> coord(2,0.0);
    Coord.resize(2,0.0);
    int res=(int)idx_/map_.info.width;
    int div=(int)idx_%map_.info.width;
    //x_=(div+0.5)*resolution;
    //y_=(res+0.5)*resolution;

    Coord[0]=(div+0.5)*map_.info.resolution+map_.info.origin.position.x;
    Coord[1]=(res+0.5)*map_.info.resolution+map_.info.origin.position.y;
    return;
}

void StateLatticePlannerROS::smooth_velocity(geometry_msgs::Twist& cmd_vel)
{
    //check with previous velocity
    if(cmd_vel.linear.x>0.35)
        cmd_vel.linear.x=0.35;

    //if(fabs(-previous_velocity.angular.z+cmd_vel.angular.z)>0.125)
    //{
        //if(cmd_vel.angular.z>0) 
        //{
            //cmd_vel.angular.z=0.125;
        //}
        //else
            //cmd_vel.angular.z=-0.125;
    //}

    //if(fabs(cmd_vel.angular.z)>0.35)
    //{
        //if(cmd_vel.angular.z>0)
            //cmd_vel.angular.z=0.20;
        //else
            //cmd_vel.angular.z=-0.20;
    //}
    //

}
void StateLatticePlannerROS::smoothing_velcommand(geometry_msgs::Twist& cmd_vel_des, geometry_msgs::Twist& cur)
{
    //check with previous velocity
    double  kp=0.3;
    double kd=0.1;
    double kyaw=0.05;
    double x_err=cmd_vel_des.linear.x-cur.linear.x;
    double yaw_err=cmd_vel_des.angular.z-cur.angular.z;

    cmd_vel_des.linear.x = cur.linear.x+x_err*kp;
    std::cout<<"cur_angular.z: " <<cur.angular.z<<", des angular z : " <<cmd_vel_des.angular.z<<std::endl;
    std::cout<<"yaw err: " <<yaw_err<<std::endl;
    //cmd_vel_des.angular.z = cur.angular.z+yaw_err*kyaw;
    //if((cmd_vel_des.linear.x-cur.linear.x)>0.225)
        //cmd_vel.linear.x=0.225;
    //if(fabs(-previous_velocity.angular.z+cmd_vel.angular.z)>0.125)
    //{
        //if(cmd_vel.angular.z>0) 
        //{
            //cmd_vel.angular.z=0.125;
        //}
        //else
            //cmd_vel.angular.z=-0.125;
    //}

    //if(fabs(cmd_vel.angular.z)>0.35)
    //{
        //if(cmd_vel.angular.z>0)
            //cmd_vel.angular.z=0.20;
        //else
            //cmd_vel.angular.z=-0.20;
    //}
    //

}




bool StateLatticePlannerROS::process(const geometry_msgs::PoseStamped& local_goal_base_link, bool is_finalgoal,const std::vector<double>& robot_pose, bool& replan, bool& localtrjupdated, std::vector<Eigen::Vector3d>& local_trj)
{
    //subgoal --- goal pose w.r.t base_link
    geometry_msgs::Twist cmd_vel;

    double temp_dist=sqrt(pow(local_goal_base_link.pose.position.x,2)+pow(local_goal_base_link.pose.position.y,2));
    double goal_range=1.5;
    ROS_INFO("temp_dist: %.2lf", temp_dist);
    if(is_finalgoal)
        goal_range=0.4;

    if(temp_dist<goal_range){
        ROS_INFO("-----The robot is near Goal--------");
        ROS_INFO("-----------Wait for next goal---------");
        //geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = 0.00;
        cmd_vel.linear.y = 0.00;
        cmd_vel.angular.z = 0.00;
        previous_velocity=cmd_vel;
        velocity_pub.publish(cmd_vel);
    
        return true;
    }
    
       std::cout << "=== Calling state lattice planner ===" << std::endl;
       double start = ros::Time::now().toSec();
       bool update_time=false;

        static int last_trajectory_num = 0;
       //std::cout << "local goal: \n" << local_goal_base_link << std::endl;
       
       double goal_yaw = tf::getYaw(local_goal_base_link.pose.orientation);
       if(fabs(goal_yaw)>M_PI/2)
           goal_yaw=M_PI/3;
       //Eigen::Vector3d goal(local_goal_base_link.pose.position.x, local_goal_base_link.pose.position.y, tf::getYaw(local_goal_base_link.pose.orientation));
       Eigen::Vector3d goal(local_goal_base_link.pose.position.x, local_goal_base_link.pose.position.y,goal_yaw);
       //std::cout << "goal w.r.t base: " << "current_velocity << std::endl;
       std::vector<Eigen::Vector3d> states;
       double target_velocity = planner.get_target_velocity(goal);
       double ori_cri=M_PI/5;

       if(stack_count>2)
        {
            target_velocity=0.20;
            //current_velocity.linear.x=0.0;
            //current_velocity.angular.z=0.0;
            ori_cri=M_PI/8;
        }
       else{
           ori_cri=TURN_DIRECTION_THRESHOLD;
       
       }


       planner.generate_biased_polar_states(N_S, goal, target_velocity, states);
       std::vector<MotionModelDiffDrive::Trajectory> trajectories;
       bool generated = planner.generate_trajectories(states, current_velocity.linear.x, current_velocity.angular.z, target_velocity, trajectories);
       if(ENABLE_SHARP_TRAJECTORY){
           generated |= planner.generate_trajectories(states, current_velocity.linear.x, current_velocity.angular.z + MAX_D_YAWRATE / HZ, target_velocity, trajectories);
           generated |= planner.generate_trajectories(states, current_velocity.linear.x, current_velocity.angular.z - MAX_D_YAWRATE / HZ, target_velocity, trajectories);
       }
       bool turn_flag = false; //right turn flag//which direction to turn?
       double relative_direction = atan2(local_goal_base_link.pose.position.y, local_goal_base_link.pose.position.x);
        if(relative_direction >M_PI)
            relative_direction=relative_direction-2*M_PI;
        else if (relative_direction < (-1 * M_PI))
            relative_direction=relative_direction+2*M_PI;

       std::cout<<"relative_direction: "<<relative_direction <<std::endl;
       if(goal.segment(0, 2).norm() < 0.2){
           std::cout<<"norm failed"<<std::endl;
           generated = false;
       }
       else if(fabs(relative_direction) > ori_cri){
            generated = false;
            turn_flag = true;
        }
       if(!turn_flag)
       {
            //cmd_vel.linear.x = 0.00;
            //cmd_vel.linear.y = 0.00;
            //cmd_vel.angular.z = 0.00;
            //previous_velocity=cmd_vel;
            //velocity_pub.publish(cmd_vel);

            double time_diff = start-last_time_trj;
            if((time_diff)<2.75)
            {
                localtrjupdated=true;
                std::cout<<"time: "<<time_diff<<"  - trying to follow current trajectory" <<std::endl;
                return false;
            }
            else
                localtrjupdated=false;

       }
       
        if(generated){

            std::vector<MotionModelDiffDrive::Trajectory> rot_trjs;
            rotate_trajectories(trajectories, rot_trjs);
            visualize_trajectories(trajectories, 0, 1, 0, last_trajectory_num, candidate_trajectories_pub);
            //visualize_trajectories(rot_trjs, 0, 0, 1, last_trajectory_num, candidate_trajectories_pub2);
            //std::cout << "check candidate trajectories" << std::endl;
            std::vector<MotionModelDiffDrive::Trajectory> candidate_trajectories;

            int trj_cont=0;
            for(const auto& trajectory : trajectories)
            {
                if(!check_collision_rosmap(local_map, trajectory.trajectory,robot_pose)){
                    if(trajectory.trajectory.size()>1)
                    {
                        candidate_trajectories.push_back(trajectory);
                    }
                }
                else{
                    trj_cont++;
                }
            }
            //checking for rotated_trjs
            for(const auto& trajectory : rot_trjs)
            {
                if(!check_collision_rosmap(local_map, trajectory.trajectory,robot_pose)){
                    if(trajectory.trajectory.size()>1)
                    {
                        candidate_trajectories.push_back(trajectory);
                    }
                }
                else{
                    trj_cont++;
                }
            }

            //std::cout<<"total: "<<trajectories.size()<<", The number of igonored trjs due to collision with LOCAL MAP: "<<trj_cont<<std::endl;
            if(candidate_trajectories.size() > 0){
                visualize_trajectories(candidate_trajectories, 0, 0.0, 1.0, last_trajectory_num, candidate_trajectories_no_collision_pub);

                //std::cout << "pickup a optimal trajectory from candidate trajectories" << std::endl;
                MotionModelDiffDrive::Trajectory trajectory;
                planner.pickup_trajectory(candidate_trajectories, goal, trajectory);
                local_trj=trajectory.trajectory;
                //std::cout<<"size of local_trj: "<<local_trj.size()<<std::endl;
                visualize_trajectory(trajectory, 1, 0, 0, selected_trajectory_pub);
                last_time_trj=ros::Time::now().toSec();
                //std::cout << "pickup time: " << ros::Time::now().toSec() - start << "[s]" << std::endl;
                double calculation_time = ros::Time::now().toSec() - start;
                //std::cout<<"calculation_time : "<<calculation_time <<std::endl; 
                stack_count=0;
                localtrjupdated=true;

                //check rotataion
                //
                //double robot_yaw=tf::getYaw(cur_pose.pose.orientation);
                //double target_angle_baselink= atan2(local_trj[2][1],local_trj[2][0]);
                //double control_target_yaw = robot_yaw+target_angle_baselink;         //target_yaw/map_frame
                //control_mode=true;
                //process_target
                //navtarget_pose[0]=local_trj[2][0]-cur_pose.position.x;
                //navtarget_pose[1]=local_trj[2][1]-cur_pose.position.y;



            }
            else{
                std::cout << "\033[91mERROR: stacking\033[00m" << std::endl;
                stack_count++;
                std::cout<<"current velocity"<<current_velocity<<std::endl;
                std::cout<<"------------------------------- "<<std::endl;
                std::cout<<"goal_base_link: "<< goal <<std::endl;
                std::cout<<"------------------------------- "<<std::endl;
                ROS_INFO("stack count: %d", stack_count);
                localtrjupdated=false;
                if(stack_count>3)
                {
                    std::vector<Eigen::Vector3d> tmp_trj;
                    tmp_trj.push_back(Eigen::Vector3d(0.1, 0.0, 0.0));
                    tmp_trj.push_back(Eigen::Vector3d(0.3, 0.0, 0.0));
                    tmp_trj.push_back(Eigen::Vector3d(0.5, 0.0, 0.0));
                    local_trj=tmp_trj;
                    last_time_trj=ros::Time::now().toSec();
                    localtrjupdated=true;
                    replan=true;
                    stack_count=0;
                }
            }

                local_map_updated = false;
                odom_updated = false;
            } //generated
           else{
            std::cout << "\033[91mERROR: no optimized trajectory was generated\033[00m" << std::endl;
            std::cout << "\033[91mturn for local goal\033[00m" << std::endl;

            geometry_msgs::Twist cmd_vel;
            cmd_vel.linear.x = 0;
            if(!turn_flag){
                cmd_vel.angular.z = 0.4*std::min(std::max(goal(2), -MAX_YAWRATE), MAX_YAWRATE);
                std::cout<<"cmd_vel.angular.z "<<cmd_vel.angular.z<<std::endl;
            }else{
                cmd_vel.angular.z = 0.4*std::min(std::max(relative_direction, -MAX_YAWRATE), MAX_YAWRATE);
                std::cout<<"cmd_vel.angular.z "<<cmd_vel.angular.z<<std::endl;
            }

            previous_velocity=cmd_vel;
            velocity_pub.publish(cmd_vel);//mk
            localtrjupdated=false;
            std::cout << "published velocity: \n" << cmd_vel << std::endl;

        }

        last_trajectory_num = trajectories.size();
        std::cout << "final time: " << ros::Time::now().toSec() - start << "[s]" << std::endl;
        return false;
}


bool StateLatticePlannerROS::getlocalplan(const geometry_msgs::PoseStamped& local_goal_base_link, bool is_finalgoal,const std::vector<double>& robot_pose, bool& replan)
{
    geometry_msgs::Twist cmd_vel;
    double temp_dist=sqrt(pow(local_goal_base_link.pose.position.x,2)+pow(local_goal_base_link.pose.position.y,2));
    double goal_range=0.5;
    ROS_INFO("temp_dist: %.2lf", temp_dist);
    if(is_finalgoal)
        goal_range=0.35;

    if(temp_dist<goal_range){
        ROS_INFO("-----The robot is near Goal--------");
        ROS_INFO("-----------Wait for next goal---------");
        //geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = 0.00;
        cmd_vel.linear.y = 0.00;
        cmd_vel.angular.z = 0.00;
        previous_velocity=cmd_vel;
        velocity_pub.publish(cmd_vel);
    
        return true;
    }
    else if(temp_dist<MAP_RANGE/2)
    {
        goal_out_of_map=false;
        ROS_INFO("Goal is inside of costmap!");
    }
    else
        goal_out_of_map=true;

       std::cout << "=== Calling state lattice planner ===" << std::endl;
       double start = ros::Time::now().toSec();
       bool update_time=false;
      
        static int last_trajectory_num = 0;
       //std::cout << "local goal: \n" << local_goal_base_link << std::endl;
       Eigen::Vector3d goal(local_goal_base_link.pose.position.x, local_goal_base_link.pose.position.y, tf::getYaw(local_goal_base_link.pose.orientation));
       //std::cout << "goal w.r.t base: " << "current_velocity << std::endl;
       std::vector<Eigen::Vector3d> states;
       double target_velocity = planner.get_target_velocity(goal);

        if(stack_count>3)
        {
            target_velocity=0.0;
            current_velocity.linear.x=0.0;
            current_velocity.angular.z=0.0;
        }

       planner.generate_biased_polar_states(N_S, goal, target_velocity, states);
       std::vector<MotionModelDiffDrive::Trajectory> trajectories;
       bool generated = planner.generate_trajectories(states, current_velocity.linear.x, current_velocity.angular.z, target_velocity, trajectories);
       if(ENABLE_SHARP_TRAJECTORY){
           generated |= planner.generate_trajectories(states, current_velocity.linear.x, current_velocity.angular.z + MAX_D_YAWRATE / HZ, target_velocity, trajectories);
           generated |= planner.generate_trajectories(states, current_velocity.linear.x, current_velocity.angular.z - MAX_D_YAWRATE / HZ, target_velocity, trajectories);
       }
       bool turn_flag = false; //right turn flag//which direction to turn?
       double relative_direction = atan2(local_goal_base_link.pose.position.y, local_goal_base_link.pose.position.x);
        if(relative_direction >M_PI)
            relative_direction=relative_direction-2*M_PI;
        else if (relative_direction < (-1 * M_PI))
            relative_direction=relative_direction+2*M_PI;

       std::cout<<"relative_direction: "<<relative_direction <<std::endl;
       if(goal.segment(0, 2).norm() < 0.2){
           std::cout<<"norm failed"<<std::endl;
           generated = false;
       }
       else if(fabs(relative_direction) > TURN_DIRECTION_THRESHOLD){
            generated = false;
            turn_flag = true;
        }
       
        if(generated){

                //cmd_vel.linear.x =0.1*previous_velocity.linear.x;
            cmd_vel.angular.z=0.0*previous_velocity.angular.z;
            velocity_pub.publish(cmd_vel);//mk


          if(update_time){
            std::vector<MotionModelDiffDrive::Trajectory> rot_trjs;
            rotate_trajectories(trajectories, rot_trjs);
            visualize_trajectories(trajectories, 0, 1, 0, last_trajectory_num, candidate_trajectories_pub);
            visualize_trajectories(rot_trjs, 0, 0, 1, last_trajectory_num, candidate_trajectories_pub2);
            std::cout << "check candidate trajectories" << std::endl;
            std::vector<MotionModelDiffDrive::Trajectory> candidate_trajectories;

            int trj_cont=0;
            //for(const auto& trajectory : trajectories)
            //{
                //if(!check_collision_rosmap(local_map, trajectory.trajectory,robot_pose)){
                    //if(trajectory.trajectory.size()>1)
                    //{
                        //candidate_trajectories.push_back(trajectory);
                    //}
                //}
                //else{
                    //trj_cont++;
                //}
            //}
            //checking for rotated_trjs
            for(const auto& trajectory : rot_trjs)
            {
                if(!check_collision_rosmap(local_map, trajectory.trajectory,robot_pose)){
                    if(trajectory.trajectory.size()>1)
                    {
                        candidate_trajectories.push_back(trajectory);
                    }
                }
                else{
                    trj_cont++;
                }
            }

            std::cout<<"total: "<<trajectories.size()<<", The number of igonored trjs due to collision with LOCAL MAP: "<<trj_cont<<std::endl;
            //if(trajectories.size()==9)
            //{
                //std::cout << "trajectories" << std::endl;
                //for(int k(0);k<trajectories.size();k++)
                //{
                    //std::cout<<k<<"-th trajectory "<<std::endl;
                    //for (int l(0);l<trajectories[k].trajectory.size();l++)
                    //{
                        //std::cout<<"x: "<< trajectories[k].trajectory[l][0]<< ", y: "<<trajectories[k].trajectory[l][1]<<std::endl;
                        //std::cout<<"vel: "<< trajectories[k].velocities[l]<<std::endl;
                        //std::cout<<"omega: "<< trajectories[k].angular_velocities[l]<<std::endl;
                    //}
                //}
              //}
             //}
            if(candidate_trajectories.size() > 0){
                visualize_trajectories(candidate_trajectories, 0, 0.0, 1.0, last_trajectory_num, candidate_trajectories_no_collision_pub);

                std::cout << "pickup a optimal trajectory from candidate trajectories" << std::endl;
                MotionModelDiffDrive::Trajectory trajectory;
                planner.pickup_trajectory(candidate_trajectories, goal, trajectory);
                cur_trj=trajectory;
                visualize_trajectory(trajectory, 1, 0, 0, selected_trajectory_pub);
                last_time_trj=ros::Time::now().toSec();
                std::cout << "pickup time: " << ros::Time::now().toSec() - start << "[s]" << std::endl;

                double calculation_time = ros::Time::now().toSec() - start;
                int delayed_control_index = std::min(std::ceil(calculation_time * HZ) + CONTROL_DELAY, (double)trajectory.trajectory.size());
                std::cout<<"delayed_control_index: "<<delayed_control_index<<std::endl; 
                if((int)trajectory.trajectory.size() < CONTROL_DELAY){
                    delayed_control_index = std::ceil(calculation_time * HZ);
                }

                for(int l(0);l<trajectory.velocities.size();l++)
                    std::cout<<"index: "<<l<<", vel: " << trajectory.velocities[l]<<", angular: "<< trajectory.angular_velocities[l]<<std::endl;
                        
                        //cmd_vel.linear.x = 1.0*trajectory.velocities[delayed_control_index];
                std::cout << calculation_time << ", " << delayed_control_index << std::endl;
                cmd_vel.linear.x = 0.85*trajectory.velocities[delayed_control_index];

                if(trajectory.angular_velocities[delayed_control_index]>0.20)
                    cmd_vel.angular.z = rot_gain*trajectory.angular_velocities[delayed_control_index];
                else
                    cmd_vel.angular.z = trajectory.angular_velocities[delayed_control_index];

                stack_count=0;
            }
            else{
                std::cout << "\033[91mERROR: stacking\033[00m" << std::endl;
                std::cout<<"goal_base_link: "<< goal <<std::endl;
                std::cout<<"------------------------------- "<< goal <<std::endl;
                geometry_msgs::Twist cmd_vel;
                cmd_vel.angular.z = 0.25*std::min(std::max(relative_direction, -MAX_YAWRATE), MAX_YAWRATE);
                smooth_velocity(cmd_vel);
                //velocity_pub.publish(cmd_vel);//mk
                stack_count++;
                ROS_INFO("stack count: %d", stack_count);
                if(stack_count>6)
                {
                    replan=true;
                    stack_count=0;
                }
            }
          }
        else{//updated time
            double calculation_time = ros::Time::now().toSec() -last_time_trj;
            int delayed_control_index = std::min(std::ceil(calculation_time * HZ) + CONTROL_DELAY, (double)cur_trj.trajectory.size());
            if((int)cur_trj.trajectory.size() < CONTROL_DELAY){
                delayed_control_index = std::ceil(calculation_time * HZ);
            }

            std::cout << calculation_time << ", " << delayed_control_index << std::endl;
            cmd_vel.linear.x = 0.85*cur_trj.velocities[delayed_control_index];

                if(cur_trj.angular_velocities[delayed_control_index]>0.15)
                    cmd_vel.angular.z = rot_gain*cur_trj.angular_velocities[delayed_control_index];
                else
                    cmd_vel.angular.z = cur_trj.angular_velocities[delayed_control_index];
         }
            //check lienar x
            if(fabs(cmd_vel.linear.x)>0.02)
            {
                previous_velocity=cmd_vel;
                smoothing_velcommand(cmd_vel, current_velocity);
                smooth_velocity(cmd_vel);
                //update_linear_y(local_goal_base_link.pose, cmd_vel);
                velocity_pub.publish(cmd_vel);//mk
                std::cout << "published velocity: \n" << cmd_vel << std::endl;
                replan=false;
             }
            else{
                std::cout << "published temp velocity" <<std::endl;
                zero_count++;
                cmd_vel.linear.x =-0.00;
                if(zero_count>10)
                {
                    replan=true;
                    zero_count=0;
                }
                update_linear_y(local_goal_base_link.pose, cmd_vel);

                previous_velocity=cmd_vel;
                smoothing_velcommand(cmd_vel, current_velocity);
                velocity_pub.publish(cmd_vel);//mk
                //std::cout << "published velocity: \n" << cmd_vel << std::endl;
             }
                    //std::cout << "published velocity: \n" << cmd_vel << std::endl;
                local_map_updated = false;
                odom_updated = false;
            } //generated
           else{
            std::cout << "\033[91mERROR: no optimized trajectory was generated\033[00m" << std::endl;
            std::cout << "\033[91mturn for local goal\033[00m" << std::endl;
            geometry_msgs::Twist cmd_vel;
            cmd_vel.linear.x = 0;
            if(!turn_flag){
                cmd_vel.angular.z = 0.35*std::min(std::max(goal(2), -MAX_YAWRATE), MAX_YAWRATE);
                std::cout<<"cmd_vel.angular.z "<<cmd_vel.angular.z<<std::endl;
            }else{
                cmd_vel.angular.z = 0.35*std::min(std::max(relative_direction, -MAX_YAWRATE), MAX_YAWRATE);
                std::cout<<"cmd_vel.angular.z "<<cmd_vel.angular.z<<std::endl;
            }

            previous_velocity=cmd_vel;
            velocity_pub.publish(cmd_vel);//mk
            std::cout << "published velocity: \n" << cmd_vel << std::endl;
        }

        last_trajectory_num = trajectories.size();
        std::cout << "final time: " << ros::Time::now().toSec() - start << "[s]" << std::endl;
}






void StateLatticePlannerROS::update_linear_y(geometry_msgs::Pose base_pose, geometry_msgs::Twist& cmd_vel)
{
    if(base_pose.position.y>0)
    {
        if(base_pose.position.y>0.3)
            cmd_vel.linear.y=0.1;
    }
    else
    {
        if(base_pose.position.y<-0.3)
            cmd_vel.linear.y=-0.1;
    }
}

void StateLatticePlannerROS::rotate_trajectories(const std::vector<MotionModelDiffDrive::Trajectory> originals,   std::vector<MotionModelDiffDrive::Trajectory>& rotated)
{

    //roate trjaectories for the angles in yawset( currently M_PI/3 and -M_PI/3)
    std::vector<double> yawset(2,0.0);
    yawset[0]=M_PI/3;
    //yawset[1]=M_PI;
    yawset[1]=-M_PI/3;
    //double rot_yaw=M_PI;

    std::cout << "trajectories" << std::endl;
    //rotated.resize(originals.size());
    rotated.clear();

    for( int i(0); i< yawset.size();i++ ){
        for(int k(0);k<originals.size();k++)
        {
            MotionModelDiffDrive::Trajectory rotated_trj;
            //std::cout<<k<<"-th trajectory "<<std::endl;
            for (int l(0);l<originals[k].trajectory.size();l++)
            {
                Eigen::Vector3d temp_trj(0.0,0.0,0.0);
                temp_trj[0]=cos(yawset[i])*originals[k].trajectory[l][0]-sin(yawset[i])*originals[k].trajectory[l][1];
                temp_trj[1]=sin(yawset[i])*originals[k].trajectory[l][0]+cos(yawset[i])*originals[k].trajectory[l][1];
                temp_trj[2]=originals[k].trajectory[l][2];
                rotated_trj.trajectory.push_back(temp_trj);
            }

            if(i==0)
            {
                for(int r(0); r<3;r++)
                {
                    rotated_trj.velocities.push_back(0.05);
                    rotated_trj.angular_velocities.push_back(-0.10-r*0.05);
                }
            }
            else{
                for(int r(0); r<3;r++)
                {
                    rotated_trj.velocities.push_back(0.05);
                    rotated_trj.angular_velocities.push_back(0.10+r*0.05);
                }
            }
            for(int j(0); j<originals[k].velocities.size();j++ )
            {
                rotated_trj.velocities.push_back(originals[k].velocities[j]);
                rotated_trj.angular_velocities.push_back(originals[k].angular_velocities[j]);
            }
            //rotated_trj.angular_velocities= originals[k].angular_velocities;
            rotated.push_back(rotated_trj);
        }
    }
}






