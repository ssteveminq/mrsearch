/*
 * Copyright (C) 2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <functional>

#include <algorithm>
#include <ignition/math.hh>
#include "gazebo/physics/physics.hh"
#include "ActorPlugin_origin.hh"
#include <ros/console.h>
#include <ros/console.h> //roslogging

#define PI 3.14159265359

using namespace gazebo;
using namespace physics;
using namespace ignition;
GZ_REGISTER_MODEL_PLUGIN(ActorPlugin)

#define WALKING_ANIMATION "walking"

/////////////////////////////////////////////////
ActorPlugin::ActorPlugin()
{
    maxAngleUpdate=0.1;
    waypoint_length=0;
    waypoint_iter=0;
}

/////////////////////////////////////////////////
void ActorPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  this->sdf = _sdf;
  this->actor = boost::dynamic_pointer_cast<physics::Actor>(_model);
  this->world = this->actor->GetWorld();
  this->start_location = this->actor->WorldPose().Pos();

  this->connections.push_back(event::Events::ConnectWorldUpdateBegin(
          std::bind(&ActorPlugin::OnUpdate, this, std::placeholders::_1)));

  this->Reset();

  // Read in the target weight
  //if (_sdf->HasElement("target_weight"))
    //this->targetWeight = _sdf->Get<double>("target_weight");
  //else
    //this->targetWeight = 1.15;

  // Read in the speed
  if (_sdf->HasElement("speed"))
    this->vMax = _sdf->Get<double>("speed");
  else
    this->vMax = 1.5;

  // Read in the obstacle weight
  if (_sdf->HasElement("obstacle_weight"))
    this->obstacleWeight = _sdf->Get<double>("obstacle_weight");
  else
    this->obstacleWeight = 1.5;

  // Read in the animation factor (applied in the OnUpdate function).
  if (_sdf->HasElement("animation_factor"))
    this->animationFactor = _sdf->Get<double>("animation_factor");
  else
    this->animationFactor = 4.5;

  // Add our own name to models we should ignore when avoiding obstacles.
  this->ignoreModels.push_back(this->actor->GetName());

  // Read in the other obstacles to ignore
  if (_sdf->HasElement("ignore_obstacles"))
  {
    sdf::ElementPtr modelElem =
      _sdf->GetElement("ignore_obstacles")->GetElement("model");
    while (modelElem)
    {
      this->ignoreModels.push_back(modelElem->Get<std::string>());
      modelElem = modelElem->GetNextElement("model");
    }
  }

  //ROS_INFO("before loading Script");
  Load_trajectory(_sdf);

  //collision parts
  // Get a pointer to the actor
  auto actor = boost::dynamic_pointer_cast<physics::Actor>(_model);

  // Map of collision scaling factors
  std::map<std::string, ignition::math::Vector3d> scaling;
  std::map<std::string, ignition::math::Pose3d> offsets;

  // Read in the collision scaling factors, if present
  if (_sdf->HasElement("scaling"))
  {

    auto elem = _sdf->GetElement("scaling");
    while (elem)
    {
      if (!elem->HasAttribute("collision"))
      {
        gzwarn << "Skipping element without collision attribute" << std::endl;
        elem = elem->GetNextElement("scaling");
        continue;
      }
      auto name = elem->Get<std::string>("collision");

      if (elem->HasAttribute("scale"))
      {
        auto scale = elem->Get<ignition::math::Vector3d>("scale");
        scaling[name] = scale;
        //std::cout<<"scale names:"<<name<<std::endl;
      }

      if (elem->HasAttribute("pose"))
      {
        auto pose = elem->Get<ignition::math::Pose3d>("pose");
        offsets[name] = pose;
      }
      elem = elem->GetNextElement("scaling");
    }
  }

  for (const auto &link : actor->GetLinks())
  {
    
    //std::cout<<"initilize link for making collisions"<<std::endl;
    // Init the links, which in turn enables collisions
    link->Init();

    if (scaling.empty())
    {
        std::cout<<"scaling is empty"<<std::endl;
        continue;
    }

    // Process all the collisions in all the links
    for (const auto &collision : link->GetCollisions())
    {
      auto name = collision->GetName();
      //std::cout<<"collisions:"<<name<<std::endl;
      //std::cout<<"collisions shape:"<<collision->GetShape()<<std::endl;
      if (scaling.find(name) != scaling.end())
      {

          //gazebo::physics::BoxShape haha;
        //auto boundingBox = collision->GetBoundingBox();
        //std::cout<<"boundingbox:"<<boundingBox <<std::endl;
        //collision->SetShape(gazebo::physics::BoxShapePtr);
        //auto boxShape = boost::dynamic_pointer_cast<gazebo::physics::BoxShape>(
        //auto boxShape = boost::dynamic_pointer_cast<gazebo::physics::SphereShape*>(
        //
        //auto boxShape = boost::dynamic_pointer_cast<gazebo::physics::BoxShape>(
        auto boxShape = boost::dynamic_pointer_cast<gazebo::physics::SphereShape>(
            collision->GetShape());

        //std::cout<<"boxshape:"<<boxShape<<std::endl;
        if (boxShape)
        {
          //std::cout<<"size of box :"<<boxShape->GetRadius() <<std::endl;
          //boxShape->SetSize(boxShape->GetRadius() * scaling[name][0]);
          boxShape->SetRadius(boxShape->GetRadius() * 10);
          boxShape->Update();
        }
        else
        {
            std::cout<<"box shape does not exists"<<std::endl;
        }
      }

      if (offsets.find(name) != offsets.end())
      {
        collision->SetInitialRelativePose(
            offsets[name] + collision->InitialRelativePose());
      }
    }
  }



}

void ActorPlugin::Load_trajectory(sdf::ElementPtr _sdf)
{

 //this->actor->loop = _sdf->Get<bool>("loop");
 //this->actor->autostart= _sdf->Get<bool>("auto_start");
 //this->actor->active = this->actor->autostart;
    if (_sdf->HasElement("trajectory"))
    {
        sdf::ElementPtr trajSdf = _sdf->GetElement("trajectory");
        while (trajSdf)
        {
           //points.clear();
           if(trajSdf->HasElement("waypoint"))
           {
               sdf::ElementPtr wayptSdf = trajSdf->GetElement("waypoint");
               while (wayptSdf)
               {
                   auto pose = wayptSdf->Get<ignition::math::Pose3d>("pose");
                   points.insert(std::make_pair(wayptSdf->Get<double>("time"), wayptSdf->Get<ignition::math::Pose3d>("pose")));
                   //points[wayptSdf->Get<double>("time")] =wayptSdf->Get<ignition::math::Pose3d>("pose");
                   //ROS_INFO("points x: %.2lf, y: %.2lf", pose.Pos().X(), pose.Pos().Y());
                   wayptSdf = wayptSdf->GetNextElement("waypoint");
                   waypoint_length++;
               }

               trajSdf = trajSdf->GetNextElement("trajectory");
           }
        }
        std::map<double, ignition::math::Pose3d>::iterator map_iter= points.begin(); 
        this->target = ignition::math::Vector3d(map_iter->second.Pos().X(), map_iter->second.Pos().Y(),map_iter->second.Pos().Z()); //should be the first waypoint
        point_iter = points.begin();
        std::cout<<"first target"<<point_iter->second.Pos().X()<<", "
               <<point_iter->second.Pos().Y()<<", "<<
               point_iter->second.Pos().Z()<<std::endl;
    }
    else
    {
        //set Target as in the sdf if there is no given trajectory
        if (this->sdf && this->sdf->HasElement("target"))
            this->target = this->sdf->Get<ignition::math::Vector3d>("target"); 
        else
            this->target = ignition::math::Vector3d(0, -5, 1.2138);
    }
}

/////////////////////////////////////////////////
void ActorPlugin::Reset()
{
  //set target & trajectory, skeleton animations
  this->velocity = 0.8;
  this->lastUpdate = 0;

  auto skelAnims = this->actor->SkeletonAnimations();
  if (skelAnims.find(WALKING_ANIMATION) == skelAnims.end())
  {
    gzerr << "Skeleton animation " << WALKING_ANIMATION << " not found.\n";
  }
  else
  {
    // Create custom trajectory
    this->trajectoryInfo.reset(new physics::TrajectoryInfo());
    this->trajectoryInfo->type = WALKING_ANIMATION;
    this->trajectoryInfo->duration = 1.0;

    this->actor->SetCustomTrajectory(this->trajectoryInfo);
  }
}

/////////////////////////////////////////////////
void ActorPlugin::ChooseNewTarget()
{

  //ROS_INFO("choose new target");
  ignition::math::Vector3d newTarget(this->target);
  while ((newTarget - this->target).Length() < 2.0)
  {
    newTarget.X(ignition::math::Rand::DblUniform(-3, 3.5));
    newTarget.Y(ignition::math::Rand::DblUniform(-10, 2));

    for (unsigned int i = 0; i < this->world->ModelCount(); ++i)
    {
      double dist = (this->world->ModelByIndex(i)->WorldPose().Pos()
          - newTarget).Length();
      if (dist < 2.0)
      {
        newTarget = this->target;
        break;
      }
    }
  }
  this->target = newTarget;
}

void ActorPlugin::ChooseNextTarget()
{

  //ROS_INFO("choose next target from waypoint");
  //check current waypoint and time
  //check current waypoint with a distance 
  if(points.size()>0)
  {
      if(point_iter==points.end())
      {
          point_iter=points.begin();
          //ROS_INFO("end reached");
      }
      else{

          waypoint_iter++;
          if(waypoint_iter==waypoint_length){
          
              point_iter=points.begin();
              waypoint_iter=0;
          }
          else{
                  point_iter++;
          }
              
      }

      this->target = ignition::math::Vector3d(point_iter->second.Pos().X(), point_iter->second.Pos().Y(),point_iter->second.Pos().Z()); //should be the first waypoint
      //ROS_INFO("waytime %.2lf, next target %.2lf, %.2lf", point_iter->first,point_iter->second.Pos().X(), point_iter->second.Pos().Y());

  }
  else
  {
        this->target = ignition::math::Vector3d(1.0, 1.0, 1.02); 
  }

  //check current 
  //get next waypoint if the actor is close enough to the current target
  //ignition::math::Vector3d newTarget(this->target);
  //while ((newTarget - this->target).Length() < 2.0)
  //{
    //newTarget.X(ignition::math::Rand::DblUniform(-3, 3.5));
    //newTarget.Y(ignition::math::Rand::DblUniform(-10, 2));

    //for (unsigned int i = 0; i < this->world->ModelCount(); ++i)
    //{
      //double dist = (this->world->ModelByIndex(i)->WorldPose().Pos()
          //- newTarget).Length();
      //if (dist < 2.0)
      //{
        //newTarget = this->target;
        //break;
      //}
    //}
  //}
  //this->target = newTarget;
}



/////////////////////////////////////////////////
void ActorPlugin::HandleObstacles(ignition::math::Vector3d &_pos)
{
  for (unsigned int i = 0; i < this->world->ModelCount(); ++i)
  {
    physics::ModelPtr model = this->world->ModelByIndex(i);
    if (std::find(this->ignoreModels.begin(), this->ignoreModels.end(),
          model->GetName()) == this->ignoreModels.end())
    {
      ignition::math::Vector3d offset = model->WorldPose().Pos() -
        this->actor->WorldPose().Pos();
      double modelDist = offset.Length();
      if (modelDist < 4.0)
      {
        double invModelDist = this->obstacleWeight / modelDist;
        offset.Normalize();
        offset *= invModelDist;
        _pos -= offset;
      }
    }
  }
}

/////////////////////////////////////////////////
void ActorPlugin::OnUpdate(const common::UpdateInfo &_info)
{
  // Time delta
  double dt = (_info.simTime - this->lastUpdate).Double();
  if(dt>10)
      dt=0.01;

  ignition::math::Pose3d pose = this->actor->WorldPose();
  ignition::math::Vector3d pos = this->target - pose.Pos();
  //ROS_INFO(" %s actor target pos x: %.2lf , y: %.2lf ",this->actor->GetName().c_str(), this->target.X(), this->target.Y());
  ignition::math::Vector3d rpy = pose.Rot().Euler();

  ignition::math::Vector3d desiredForce = pos.Normalize() * this->vMax;
  ignition::math::Vector3d a = (10 * desiredForce);             //10= desired force gain
  this->velocity = this->velocity + a * dt;
  double speed = this->velocity.Length();
  if (speed > this->vMax) {
    this->velocity = this->velocity.Normalize() * this->vMax;
  }


  //check distance to the target 
  //double distance = pos.Length();
  //ROS_INFO("actor target to distance: %.2lf ",distance);

  // Choose a new target position if the actor has reached its current
  // target.
  //if (distance < 0.3)
  //{
    //this->ChooseNextTarget();
    //pos = this->target - pose.Pos();
  //}

  // Normalize the direction vector, and apply the target weight
  pos = pos.Normalize() * this->targetWeight;

  // Adjust the direction vector by avoiding obstacles
  this->HandleObstacles(pos);

  // Compute the yaw orientation
  //ignition::math::Angle yaw = atan2(pos.Y(), pos.X()) + 1.5707 - rpy.Z();
  //yaw.Normalize();

  ignition::math::Angle yaw_update = atan2(this->velocity.Y(), this->velocity.X()) + 0.5*PI - rpy.Z();
  yaw_update.Normalize();

  double temp_vel = this->velocity.Length();

if (std::fabs(yaw_update.Radian()) > (this->maxAngleUpdate/180.0 * PI)){
    double yaw_update_sign = yaw_update.Radian()/std::fabs(yaw_update.Radian());
    yaw_update = this->maxAngleUpdate/180 * PI*yaw_update_sign;  
  }
  ignition::math::Angle new_yaw = rpy.Z()+yaw_update.Radian()-0.5*PI;
  //this->velocity.X() = temp_vel * cos(yaw_update.Radian()) * cos(new_yaw.Radian());
  //this->velocity.Y() = temp_vel * cos(yaw_update.Radian()) * sin(new_yaw.Radian());

  pose.Rot() = ignition::math::Quaterniond(0.5*PI, 0, new_yaw.Radian()+0.5*PI);
  //yaw_vel = yaw_update.Radian()/dt;



  // Rotate in place, instead of jumping.
  //if (std::abs(yaw.Radian()) > IGN_DTOR(10))
  //{
    //pose.Rot() = ignition::math::Quaterniond(1.5707, 0, rpy.Z()+yaw.Radian()*0.001);
  //}
  //else
  //{

  pose.Pos() = pose.Pos() + this->velocity * dt;
  // Make sure the actor stays within bounds
  pose.Pos().Z(1.015);

  //ROS_INFO("Updated POS %.2lf, %.2lf", pose.Pos().X(), pose.Pos().Y());

  // Distance traveled is used to coordinate motion with the walking
  // animation
  double distanceTraveled = (pose.Pos() -
      this->actor->WorldPose().Pos()).Length();

  this->actor->SetWorldPose(pose, false, false);

  this->actor->SetScriptTime(this->actor->ScriptTime() +
    (distanceTraveled * this->animationFactor));
  this->lastUpdate = _info.simTime;

  //double distance = pos.Length();
  double distance = pose.Pos().Distance(this->target);

  // Choose a new target position if the actor has reached its current
  // target.
  if (distance < 0.25)
  {
    this->ChooseNextTarget();
    pos = this->target - pose.Pos();
  }




}
