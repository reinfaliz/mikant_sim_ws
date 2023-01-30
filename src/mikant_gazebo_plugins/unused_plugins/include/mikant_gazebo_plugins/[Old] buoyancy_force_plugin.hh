#include <gazebo/common/common.hh>
#include <gazebo/common/Event.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

#include <map>
#include <string>
#include <vector>

namespace buoyancy_force_plugin
{
  class BuoyancyForcePlugin : public gazebo::ModelPlugin
  {
    public:
      BuoyancyForcePlugin();
      void Load(const gazebo::physics::ModelPtr _model, const sdf::ElementPtr _sdf);

    private:
      gazebo::physics::ModelPtr model;
      gazebo::physics::WorldPtr world;
      std::string link_name;
      gazebo::physics::LinkPtr link;
      double fluid_density;
      double fluid_level;
      double link_volume;
      gazebo::event::ConnectionPtr updateConnection;
      void OnUpdate();
  };
}
