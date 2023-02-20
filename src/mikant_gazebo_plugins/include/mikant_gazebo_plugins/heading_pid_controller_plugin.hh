#include <gazebo/common/common.hh>
#include <gazebo/common/Event.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>

#include <rclcpp/rclcpp.hpp>
#include <gazebo_ros/node.hpp>
#include <string>

namespace heading_pid_controller_plugin
{
  /// \brief A class for storing buoyancy object properties
  class HeadingPIDControllerPlugin : public gazebo::ModelPlugin
  {
    public:
      /// \brief Default constructor

      HeadingPIDControllerPlugin();
      
      void Load(const gazebo::physics::ModelPtr _model, const sdf::ElementPtr _sdf);

      void callbacksub(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
      
      /// \brief Associated link name            
      
    private:
      void OnUpdate();
      
      // Pointer to the model
      gazebo::physics::ModelPtr model;
      
      //Retrieved when the model is loaded.
      gazebo::physics::WorldPtr world;

      std::string link_name;

      std::string model_name;

      std::string joint_name;

      gazebo::physics::LinkPtr link;

      double p_gain;

      double i_gain;

      double d_gain;

      double x;

      double y;

      double phi;

      double psi;

      double u;

      double v;

      double p;

      double r;

      double error;
      
      rclcpp::Node::SharedPtr node_;

      rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;

      rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_;

      // Pointer to the update event connection
      gazebo::event::ConnectionPtr updateConnection;
  };
}