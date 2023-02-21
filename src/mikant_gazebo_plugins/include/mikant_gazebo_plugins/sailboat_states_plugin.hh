#include <gazebo/common/common.hh>
#include <gazebo/common/Event.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>

#include <map>
#include <string>
#include <vector>

namespace sailboat_states_plugin
{
  class SailboatStatesPlugin : public gazebo::ModelPlugin
  {
    public:
      SailboatStatesPlugin();
      void Load(const gazebo::physics::ModelPtr _model, const sdf::ElementPtr _sdf);       
      
    private:
      gazebo::physics::ModelPtr model;
      std::string model_name;
      gazebo::physics::WorldPtr world;
      std::string world_name;
      std::string rudder_joint_name;
      gazebo::physics::JointPtr rudder_joint;
      std::string sail_joint_name;
      gazebo::physics::JointPtr sail_joint;
      std::string tail_joint_name;
      gazebo::physics::JointPtr tail_joint;
      std::string left_propeller_joint_name;
      gazebo::physics::JointPtr left_propeller_joint;
      std::string right_propeller_joint_name;
      gazebo::physics::JointPtr right_propeller_joint;
      bool convert_to_deg;
      gazebo_ros::Node::SharedPtr ros_node;
      std::map<std::string, rclcpp::PublisherBase::SharedPtr> ros_node_publisher;
      gazebo::event::ConnectionPtr updateConnection;
      void OnUpdate();
      ignition::math::Vector3d ToNED(const ignition::math::Vector3d gazebo_vector);
  };
}