#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Assert.hh>
#include <gazebo/common/Events.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

#include "mikant_gazebo_plugins/buoyancy_force_plugin.hh"
#include <string>

namespace buoyancy_force_plugin
{
  BuoyancyForcePlugin::BuoyancyForcePlugin()
  : fluid_density(998),
    fluid_level(0.0),
    link_volume(0.042556812)
  {
  }

  void BuoyancyForcePlugin::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {
    // Store pointers
    this->model = _model;
    this->world = model->GetWorld();
    this->link_name = _sdf->GetElement("link_name")->Get<std::string>();
    this->link = model->GetLink(link_name);
    
    // Store elements
    if(_sdf->HasElement("fluid_density"))
    {
      this->fluid_density = _sdf->Get<double>("fluid_density");
    }
    if(_sdf->HasElement("fluid_level"))
    {
      this->fluid_level = _sdf->Get<double>("fluid_level");
    }
    if(_sdf->HasElement("link_volume"))
    {
      this->link_volume = _sdf->Get<double>("link_volume");
    }
    
    // Update the event
    this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&BuoyancyForcePlugin::OnUpdate, this));
  }

  // Called by the world update start event
  void BuoyancyForcePlugin::OnUpdate()
  {
    // Calculate the displaced volume
    ignition::math::Pose3d link_pose = link->WorldPose();
    ignition::math::Vector3d link_position = link_pose.Pos();

    // Calculate the bottom displaced level
    double bottom_displaced_level = fluid_level - link_position.Z();
    double bottom_displaced_volume = 0.1895489009*bottom_displaced_level;

    // Limit the bottom displaced volume
    if(bottom_displaced_volume < 0.0)
    {
      bottom_displaced_volume = 0.0;
    }
    else if(bottom_displaced_volume > 0.2548287)
    {
      bottom_displaced_volume = 0.2548287;
    }

    // Calculate the top displaced level
    double top_displaced_level = (fluid_level - link_position.Z() - 0.036666);
    double top_displaced_volume = 0.218*top_displaced_level;

    // Limit the top displaced volume
    if(top_displaced_volume < 0.0)
    {
      top_displaced_volume = 0.0;
    }

    // Calculate the total displaced_volume
    double total_displaced_volume = bottom_displaced_volume + top_displaced_volume;
    
    // Check the tolerance volume
    double tolerance = 0.1;
    if(total_displaced_volume < tolerance)
    {
      total_displaced_volume = 1e-6;
    }

    // Limit the maximum volume
    else if(total_displaced_volume > link_volume)
    {
      total_displaced_volume = link_volume;
    }

    // Calculate the buoyancy force
    ignition::math::Vector3d gravity = world->Gravity();
    ignition::math::Vector3d buoyancy_force = -fluid_density*total_displaced_volume*gravity;
    
    // Calculate the CG of the model
    double total_link_mass = 0.0;
    ignition::math::Vector3d model_cg_position = ignition::math::Vector3d(0, 0, 0);
    for(unsigned int i = 0; i < model->GetChildCount(); i++)
    {
      gazebo::physics::BasePtr temp_base = model->GetChild(i);
      if(temp_base->HasType(gazebo::physics::Base::LINK))
      {
        gazebo::physics::LinkPtr temp_link = model->GetLink(temp_base->GetName());
        double temp_link_mass = temp_link->GetInertial()->Mass();
        model_cg_position.operator+=(temp_link->WorldCoGPose().Pos() * temp_link_mass);
        total_link_mass = total_link_mass + temp_link_mass;
      }
    }
    model_cg_position = model_cg_position/total_link_mass;

    // Apply the buoyancy force to the model at its CG
    link->AddForceAtWorldPosition(buoyancy_force, model_cg_position);
  }
  
  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(BuoyancyForcePlugin)
}
