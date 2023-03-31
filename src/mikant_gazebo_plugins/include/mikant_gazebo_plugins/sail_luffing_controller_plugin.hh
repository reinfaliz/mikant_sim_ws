#include <gazebo/common/common.hh>
#include <gazebo/common/Event.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <rclcpp/rclcpp.hpp>
#include <gazebo_ros/node.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <string>

namespace sail_luffing_controller_plugin
{
  class SailLuffingControllerPlugin : public gazebo::ModelPlugin
  {
    public:
      SailLuffingControllerPlugin();
      
      void Load(const gazebo::physics::ModelPtr _model, const sdf::ElementPtr _sdf);

      void callbacksubmodel(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

      void callbacksubworld(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
      
    private:
      gazebo::physics::ModelPtr model;
      gazebo::physics::WorldPtr world;

      std::string joint_name;
      double max_sail_angle;
      double min_sail_angle;
      ignition::math::Vector3d center_of_pressure;
      std::string sub_model_topic;
      std::string sub_world_topic;
      bool model_in_deg;
      bool world_in_deg;

      rclcpp::Node::SharedPtr node_;
      rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_model;
      rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_world;
      rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;       

      gazebo::event::ConnectionPtr updateConnection;

      double phi;
      double psi;
      double u_b;
      double v_b;
      double p_b;
      double r_b;

      double true_wind_speed;
      double true_wind_angle;

      void OnUpdate();
  };
}