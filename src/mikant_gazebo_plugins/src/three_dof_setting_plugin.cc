#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Assert.hh>
#include <gazebo/common/Events.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>

#include "mikant_gazebo_plugins/three_dof_setting_plugin.hh"

namespace three_dof_setting_plugin
{
  ThreeDOFSettingPlugin::ThreeDOFSettingPlugin()
  : enable(true)
  {
  }

  void ThreeDOFSettingPlugin::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {
    // Store pointers
    this->model = _model;
    
    // Store elements
    if(_sdf->HasElement("enable"))
    {
      this->enable = _sdf->Get<bool>("enable");
    }

    // Update the event
    this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&ThreeDOFSettingPlugin::OnUpdate, this));
  }

  // Called by the world update start event
  void ThreeDOFSettingPlugin::OnUpdate()
  {
    if(enable)
    {
      // Set the model velocities where roll, and pitch motions are eliminated
      model->SetAngularVel(ignition::math::Vector3d(0,0,model->WorldAngularVel().Z()));

      // Set the model pose where roll and pitch angles are set as 0
      ignition::math::Vector3d model_pos_world = this->model->WorldPose().Pos();
      ignition::math::Vector3d model_orientation_world = this->model->WorldPose().Rot().Euler();
      model_orientation_world.X(0);
      model_orientation_world.Y(0);
      model->SetWorldPose(ignition::math::Pose3d(model_pos_world,ignition::math::Quaternion(model_orientation_world)));
    }
  }
  
  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ThreeDOFSettingPlugin)
}
