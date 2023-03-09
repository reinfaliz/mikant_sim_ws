#include <gazebo/common/common.hh>
#include <gazebo/common/Event.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>

#include <map>
#include <string>
#include <vector>

namespace three_dof_setting_plugin
{
  class ThreeDOFSettingPlugin : public gazebo::ModelPlugin
  {
    public:
      ThreeDOFSettingPlugin();
      void Load(const gazebo::physics::ModelPtr _model, const sdf::ElementPtr _sdf);

    private:
      gazebo::physics::ModelPtr model;
      bool enable;
      gazebo::event::ConnectionPtr updateConnection;
      void OnUpdate();
  };
}
