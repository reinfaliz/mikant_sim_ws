#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <gazebo/transport/transport.hh>
#include "mikant_gazebo_plugins/yaw_damping_torque_plugin.hh"

namespace yaw_damping_torque_plugin
{
  YawDampingTorquePlugin::YawDampingTorquePlugin()
    : damping_coef(0.0)
  {
  }

    void YawDampingTorquePlugin::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      // Store the pointer to the model
      this->model = _model;
      if(_sdf->HasElement("link_name")){
      	this->link_name = _sdf->GetElement("link_name")->Get<std::string>();
    	this->link = model->GetLink(this->link_name);
      }
      if(_sdf->HasElement("damping_coef")){
      	this->damping_coef=_sdf->Get<double>("damping_coef");
      }
      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
          std::bind(&YawDampingTorquePlugin::OnUpdate, this));
    }

    // Called by the world update start event
    void YawDampingTorquePlugin::OnUpdate()
    {
      // Apply a small linear velocity to the model.
      world_frame = model->WorldAngularVel(); //WorldAngularVel() is function.
      phi_dot = (-1)*world_frame.Z(); //get the member of z in world_frame matrix.
      this->torque = damping_coef*phi_dot*abs(phi_dot); //calculate torque
      this->link->AddRelativeTorque(ignition::math::Vector3d(0,0,(-1)*torque)); //change to original axis.

    }

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(YawDampingTorquePlugin)
}
