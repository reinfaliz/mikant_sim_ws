#include <gazebo/common/common.hh>
#include <gazebo/common/Event.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

#include <map>
#include <string>
#include <vector>

namespace revolute_joint_setting_plugin
{
  class RevoluteJointSettingPlugin : public gazebo::ModelPlugin
  {
    public:
      RevoluteJointSettingPlugin();
      void Load(const gazebo::physics::ModelPtr _model, const sdf::ElementPtr _sdf);

    private:
      gazebo::physics::ModelPtr model;
      std::string model_name;
      gazebo::physics::WorldPtr world;
      std::string world_name;
      std::string joint_name;
      gazebo::physics::JointPtr joint;
      double joint_setting_lower_limit;
      double joint_setting_upper_limit;
      int joint_mode;
      std::string sub_joint_topic;
      gazebo_ros::Node::SharedPtr ros_node;
      std::map<std::string, rclcpp::SubscriptionBase::SharedPtr> ros_node_subscriber;
      gazebo::event::ConnectionPtr updateConnection;
      void TopicCallBack(const std_msgs::msg::Float64::SharedPtr msg);
      double joint_setting;
      void OnUpdate();
  };
}
