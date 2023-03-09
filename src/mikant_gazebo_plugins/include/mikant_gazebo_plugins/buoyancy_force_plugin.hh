#include <gazebo/common/common.hh>
#include <gazebo/common/Event.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

#include <string>

namespace buoyancy_force_plugin
{
  /// \brief A class for storing buoyancy object properties
  class BuoyancyForcePlugin : public gazebo::ModelPlugin
  {
    public:
      /// \brief Default constructor

      BuoyancyForcePlugin();
      
      void Load(const gazebo::physics::ModelPtr _model, const sdf::ElementPtr _sdf);
      
      /// \brief Associated link name            
      
    private:
      void OnUpdate();
      
      // Pointer to the model
      gazebo::physics::ModelPtr model;
      
      //Retrieved when the model is loaded.
      gazebo::physics::WorldPtr world;

      std::string link_name;

      gazebo::physics::LinkPtr link;

      int link_id;

      double linear_drag;

      double link_volume;

      double bottom_link_level;
      
      double fluid_density;
      
      double fluid_level;

      double displaced_volume;

      double level;

      double dif_level;

      double bottom_level;

      double top_level;

      // Pointer to the update event connection
      gazebo::event::ConnectionPtr updateConnection;
  };
}