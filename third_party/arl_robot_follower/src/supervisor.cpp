///////////////////////////////////////////////////////////////////////////////
//      Title     : supervisor.cpp
//      Platforms : Ubuntu 64-bit
//      Copyright : Copyright© The University of Texas at Austin, 2021. All rights reserved.
//
//          All files within this directory are subject to the following, unless an alternative
//          license is explicitly included within the text of each file.
//
//          This software and documentation constitute an unpublished work
//          and contain valuable trade secrets and proprietary information
//          belonging to the University. None of the foregoing material may be
//          copied or duplicated or disclosed without the express, written
//          permission of the University. THE UNIVERSITY EXPRESSLY DISCLAIMS ANY
//          AND ALL WARRANTIES CONCERNING THIS SOFTWARE AND DOCUMENTATION,
//          INCLUDING ANY WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
//          PARTICULAR PURPOSE, AND WARRANTIES OF PERFORMANCE, AND ANY WARRANTY
//          THAT MIGHT OTHERWISE ARISE FROM COURSE OF DEALING OR USAGE OF TRADE.
//          NO WARRANTY IS EITHER EXPRESS OR IMPLIED WITH RESPECT TO THE USE OF
//          THE SOFTWARE OR DOCUMENTATION. Under no circumstances shall the
//          University be liable for incidental, special, indirect, direct or
//          consequential damages or loss of profits, interruption of business,
//          or related expenses which may arise from use of software or documentation,
//          including but not limited to those resulting from defects in software
//          and/or documentation, or loss or inaccuracy of data of any kind.
//
///////////////////////////////////////////////////////////////////////////////

#include <arl_robot_follower/supervisor.h>


namespace arl_robot_follower {


  Supervisor::Supervisor() :
      private_nh_("~")
    , paused_(true)
    , inspect_client_("inspect_target", true)
    , follow_client_("follow_target", true)
    , gotoregion_client_("goto_region", true)
  {
    pause_server_ = private_nh_.advertiseService("pause",
                                                 &Supervisor::pauseCb, this);

    waypoints_sub_ = private_nh_.subscribe("waypoint_plan", 1,
                                            &Supervisor::waypointsCb, this);

    detected_objects_sub_ = private_nh_.subscribe("detected_objects", 10,
                                                  &Supervisor::detectedObjectsCb, this);

    bool debug_detected_objects;
    private_nh_.param("debug_detected_objects", debug_detected_objects, true);
    if (debug_detected_objects)
    {
      debug_detected_objects_pub = private_nh_.advertise<vision_msgs::ObjectHypothesisWithPose>("debug_detected_objects", 1);
    }

    total_tgts_ = 0;
  }


  Supervisor::~Supervisor()
  {
    inspect_client_.cancelGoal();
    follow_client_.cancelGoal();
    gotoregion_client_.cancelGoal();
  }


  bool Supervisor::runDemo(const std::string lead_robot_name,
                           const double follow_distance)
  {
    lead_robot_name_ = lead_robot_name;
    follow_distance_ = follow_distance;

    paused_ = true;

    ROS_INFO("Demo ready and paused. Un-pause to begin.");

    // wait until we have received the demo waypoints
    while (waypoint_queue_.empty() && ros::ok())
    {
      ROS_INFO_THROTTLE(10.0, "Waiting to load waypoints for survey demo");
      ros::spinOnce();
    }

    while (paused_) {
      ros::spinOnce();
      ROS_WARN_THROTTLE(10.0, "Survey demo paused. Unpause to navigate to first waypoint");
    }

    // send the first waypoint goal
    nav_attempts_made_ = 1;
    sendGoToRegionGoal();

    // start the following behavior
    sendFollowGoal(lead_robot_name_, follow_distance_);

    // spin until the waypoint queue and task queue are empty
    while ( (!waypoint_queue_.empty() || !inspection_task_queue_.empty()) && ros::ok() )
    {
      ros::spinOnce();
    }

    return true;
  }


  void Supervisor::waypointsCb(const nav_msgs::PathConstPtr msg)
  {
    if (!waypoint_queue_.empty()) {
      ROS_ERROR("Waypoints have already been commanded for the survey demo.  Rerun node to reset demo.");
      return;
    }

    for (const geometry_msgs::PoseStamped &pose : msg->poses)
    {
      waypoint_queue_.push(pose);
    }
  }


  void Supervisor::detectedObjectsCb(const vision_msgs::Detection3DArrayConstPtr msg)
  {
    // check if the demo is running
    if (paused_)
    {
      return;
    }
    ROS_WARN_STREAM("DETS SIZE: " << msg->detections.size() << " TGTS: " << total_tgts_);
    if (msg->detections.size() == total_tgts_) {
      ROS_WARN_STREAM("NOTHING NEW TO INSPECT.  Returning");
      return;
    }

    while (total_tgts_ < msg->detections.size()) {
      ROS_WARN_STREAM("ATTEMPTING TO ADD NEW TGT");
      total_tgts_++;
      vision_msgs::Detection3D detection = msg->detections[total_tgts_ - 1];

      vision_msgs::ObjectHypothesisWithPose detected_object;
      detected_object = detection.results.front();

      ROS_WARN_STREAM("Adding new object to inpsection queue:\n" << detected_object);
      inspection_task_queue_.push_back(detected_object);
      debug_detected_objects_pub.publish(detected_object);
    }

    // for (vision_msgs::Detection3D detection : msg->detections)
    // {
    //   if (detection.results.empty()) {
    //     return;
    //   }

    //   // use the highest-scoring object class for this detection
    //   vision_msgs::ObjectHypothesisWithPose detected_object;
    //   detected_object = detection.results.front();
    //   // detected_object.score = -1;
    //   // for (const vision_msgs::ObjectHypothesisWithPose &result : detection.results) {
    //   //   if (result.score > detected_object.score) {
    //   //     detected_object = result;
    //   //   }
    //   // }
    //   if (detected_object.score < 0.9) {
    //     break;
    //   }


    //   // have we seen this object before?
    //   bool object_is_new = true;

    //   // check if the detected objects are already in the task queue
    //   for (vision_msgs::ObjectHypothesisWithPose tasked_object : inspection_task_queue_)
    //   {
    //     if (compareObjects(tasked_object, detected_object, 10.0))
    //     {
    //       object_is_new = false;
    //       break;
    //     }
    //   }

    //   // check if the object is in the completed tasks
    //   if (object_is_new)
    //   {
    //     for (vision_msgs::ObjectHypothesisWithPose completed_object : completed_tasks_)
    //     {
    //       if (compareObjects(completed_object, detected_object, 1.0))
    //       {
    //         object_is_new = false;
    //         break;
    //       }
    //     }
    //   }

    //   // if object is new, add a new task to the queue
    //   if (object_is_new)
    //   {
    //     ROS_WARN_STREAM("Adding new object to inpsection queue:\n" << detected_object);
    //     inspection_task_queue_.push_back(detected_object);
    //     debug_detected_objects_pub.publish(detected_object);
    //   }
    // }


    // if there is not already an active inspction task,
    // send an inspection goal
    const actionlib::SimpleClientGoalState state = inspect_client_.getState();
    if (state.isDone())
    {
      ROS_ERROR_STREAM("DETECTED OBJ CB and Inspect state is done.  Calling inspection goal");
      sendInspectionGoal();
    }
  }


  bool Supervisor::pauseCb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
  {
    paused_ = req.data;

    if (paused_)
    {
      // stop all our active actions
      gotoregion_client_.cancelGoal();
      inspect_client_.cancelGoal();
      follow_client_.cancelGoal();

      ROS_INFO("Demo paused.");
    }
    else
    {
      ROS_INFO("Demo unpaused.");
      sendGoToRegionGoal();
      sendFollowGoal(lead_robot_name_, follow_distance_);
      sendInspectionGoal();
    }
  }


  void Supervisor::inspectionDoneCb(const actionlib::SimpleClientGoalState& state,
                                    const InspectTargetResultConstPtr& result)
  {
    ROS_ERROR("INSPECTION DONE CB");
    // are we paused?
    if (paused_ && ros::ok())
    {
      // this will not count towards our number of inspection attempts
      return;
    }


    int max_attempts;
    private_nh_.param<int>("max_inspect_tries", max_attempts, 3);

    // see if the action was successful, or if it's time to give up
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED ||
       (max_attempts > 0 && inspect_attempts_made_ >= max_attempts))
    {
      // move the completed task to the completion queue
      if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
      {
        ROS_WARN("Follower succeeded at object inspection.");
        completed_tasks_.push_back(inspection_task_queue_.front());
      }
      else {
        ROS_WARN("Follower reached the attempt limit for its inspection goal.");
      }
      
      ROS_WARN("Popping inspection queue front.");
      // remove the element that we just completed
      inspection_task_queue_.pop_front();

      // are there any more waypoints?
      if (!inspection_task_queue_.empty())
      {
        ROS_WARN("Follower proceeding to next inspection task in queue.");

        inspect_attempts_made_ = 1;
        // get the next waypoint from the queue and produce a new goal from it
        sendInspectionGoal();
      }
      else {
        ROS_WARN("Inspection queue empty. Follower resuming follow behavior.");
        sendFollowGoal(lead_robot_name_, follow_distance_);
      }
    }
    else {
      ROS_WARN("Follower re-attempting previous inspection goal.");

      // retry the previous goal
      sendInspectionGoal();

      inspect_attempts_made_++;
      ROS_WARN_STREAM("INPECT ATTEMPT NUM: " << inspect_attempts_made_);
    }
  }


  void Supervisor::waypointDoneCb(const actionlib::SimpleClientGoalState& state,
                                  const arl_nav_msgs::GotoRegionResultConstPtr& result)
  {
    // are we paused?
    if (paused_)
    {
      // this will not count towards our number of waypoint attempts
      return;
    }

    int max_attempts;
    private_nh_.param<int>("max_nav_tries", max_attempts, 3);

    // see if the action was successful, or if it's time to give up
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED ||
       (max_attempts > 0 && nav_attempts_made_ >= max_attempts))
    {
      if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_ERROR("Leader reached waypoint.");
      }
      else {
        ROS_WARN("Leader reached attempt limit for its GoToRegion goal.");
      }

      // remove the element that we just completed
      waypoint_queue_.pop();

      // are there any more waypoints?
      if (!waypoint_queue_.empty())
      {
        ROS_WARN("Leader proceeding to next waypoint");

        // get the next waypoint from the queue and produce a new goal from it
        nav_attempts_made_ = 1;
        sendGoToRegionGoal();
      }
      else {
        ROS_WARN("Leader completed its waypoint queue.");
      }
    }
    else
    {
      ROS_WARN("Leader retying previous GoToRegion goal.");
      // retry the previous goal
      nav_attempts_made_++;
      sendGoToRegionGoal();
    }
  }


  void Supervisor::sendGoToRegionGoal()
  {
    if (waypoint_queue_.empty())
    {
      ROS_WARN("Waypoints queue is empty! Cannot send region goal.");
      return;
    }

    arl_nav_msgs::GotoRegionGoal goal;
    goal.region_center = waypoint_queue_.front();

    goal.radius = 1 + (nav_attempts_made_ / 5.0);
    goal.angle_threshold = (30.0 / 180.0) * M_PI * (1 + nav_attempts_made_ / 5.0);

    ROS_WARN("UTDD supervisor sending GotoRegion goal to leader.");

    typedef actionlib::SimpleActionClient<arl_nav_msgs::GotoRegionAction> Client;
    gotoregion_client_.sendGoal(goal,
                                boost::bind(&Supervisor::waypointDoneCb, this, _1, _2),
                                Client::SimpleActiveCallback(),
                                Client::SimpleFeedbackCallback());
  }


  void Supervisor::sendInspectionGoal()
  {
    if (inspection_task_queue_.empty())
    {
      ROS_WARN("Inspection task queue is empty! Cannot send inspection goal.");
      return;
    }

    ROS_WARN("Supervisor commanding follower to inspect object.");

    // stop the following behavior, if it's running
    // if (!follow_client_.getState().isDone()) {
      ROS_WARN("Supervisor cancelling follow goal.");
      follow_client_.cancelGoal();
    // }

    const vision_msgs::ObjectHypothesisWithPose object = inspection_task_queue_.front();

    ROS_WARN("Init inspection goal.");

    arl_robot_follower::InspectTargetGoal goal;
    // private_nh_.param("camera_frame", goal.camera_frame);
    goal.target_pose.pose = object.pose.pose;
    goal.min_distance = 1.0;
    goal.max_distance = 2.0;
    goal.max_angle = 5.0 / 180.0 * M_PI;

    // TODO remove the hard coded frames here and pass in frames as appropriate
    goal.target_pose.header.frame_id = "walrus/map";
    goal.camera_frame = "jackal/base_link";

    ROS_WARN("UTDD supervisor sending inspection goal.");

    typedef actionlib::SimpleActionClient<arl_robot_follower::InspectTargetAction> Client;
    inspect_client_.sendGoal(goal,
                             boost::bind(&Supervisor::inspectionDoneCb, this, _1, _2),
                             Client::SimpleActiveCallback(),
                             Client::SimpleFeedbackCallback());
  }


  void Supervisor::sendFollowGoal(const std::string lead_robot_name,
                                  const double follow_distance)
  {
    ROS_WARN("Supervisor commanding follower to follow.");

    arl_robot_follower::FollowTargetGoal goal;
    goal.target_frame = lead_robot_name + "/base_link";

    // 1m leeway
    goal.min_distance = follow_distance - 0.5;
    goal.max_distance = follow_distance + 0.5;

    ROS_WARN("UTDD supervisor sending follow goal.");

    follow_client_.sendGoal(goal);
  }


  bool Supervisor::compareObjects(const vision_msgs::ObjectHypothesisWithPose &first,
                                  const vision_msgs::ObjectHypothesisWithPose &second,
                                  const double distance_threshold) const
  {
    // compute the distance between the two objects
    if (first.id != second.id)
    {
      return false;
    }

    // get distance between positions. we don't care about orientations.
    const double delta_x = first.pose.pose.position.x - second.pose.pose.position.x;
    const double delta_y = first.pose.pose.position.y - second.pose.pose.position.y;
    const double delta_z = first.pose.pose.position.z - second.pose.pose.position.z;
    const double distance_sqr = delta_x*delta_x + delta_y*delta_y + delta_z*delta_z;

    // check if the two detections are within a given distance of each other
    return distance_sqr < distance_threshold*distance_threshold;
  }

}  // namespace arl_robot_follower

int main(int argc, char **argv)
{
  ros::init(argc, argv, "demo_supervisor");

  if (argc != 3)
  {
    ROS_FATAL("usage: rosrun arl_robot_follower supervisor lead_robot_name follow_distance");
    return 1;
  }

  const std::string lead_robot_name(argv[1]);
  const double follow_distance = std::atof(argv[2]);

  if (follow_distance <= 0)
  {
    ROS_FATAL("Argument follow_distance must be positive!");
    return 2;
  }

  arl_robot_follower::Supervisor node;
  node.runDemo(lead_robot_name, follow_distance);

  return 0;
}
