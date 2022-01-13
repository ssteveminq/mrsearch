/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

#include <ignition/math/Vector3.hh>

#include <gazebo/physics/Actor.hh>
#include <gazebo/physics/BoxShape.hh>
#include <gazebo/physics/CylinderShape.hh>
#include <gazebo/physics/SphereShape.hh>
#include <gazebo/physics/Collision.hh>
#include <gazebo/physics/Link.hh>
#include "ActorCollisionsPlugin.hh"

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(ActorCollisionsPlugin)


/////////////////////////////////////////////////
ActorCollisionsPlugin::ActorCollisionsPlugin()
{
}

/////////////////////////////////////////////////
void ActorCollisionsPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
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
        std::cout<<"scale names:"<<name<<std::endl;
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
      std::cout<<"collisions:"<<name<<std::endl;
      std::cout<<"collisions shape:"<<collision->GetShape()<<std::endl;

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

        std::cout<<"boxshape:"<<boxShape<<std::endl;

        if (boxShape)
        {
          std::cout<<"size of box :"<<boxShape->GetRadius() <<std::endl;
          //std::cout<<"size of box :"<<boxShape->Size() <<std::endl;
          //boxShape->SetSize(boxShape->GetRadius() * scaling[name][0]);
          boxShape->SetRadius(boxShape->GetRadius() * 7);
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
