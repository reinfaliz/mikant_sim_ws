include <gazebo/common/common.hh>
#include <gazebo/common/Event.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

#include <string>

namespace yaw_damping_torque_plugin
{
  /// \brief A class for storing buoyancy object properties
  class YawDampingTorquePlugin : public gazebo::ModelPlugin
  {
    public:
      /// \brief Default constructor

      YawDampingTorquePlugin();
      
      void Load(const gazebo::physics::ModelPtr _model, const sdf::ElementPtr _sdf);
      
      /// \brief Associated link name            
      
    private:
      void OnUpdate();
      
      // Pointer to the model
      gazebo::physics::ModelPtr model;
      
      //Retrieved when the model is loaded.
      physics::ModelPtr model;
    	physics::linkPtr link;
	double damping_coef;
	double world_frame;
	double phi_dot;
	double torque;

      // Pointer to the update event connection
      gazebo::event::ConnectionPtr updateConnection;
  };
}
