#include <gazebo/common/common.hh>
#include <gazebo/common/Event.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <rclcpp/rclcpp.hpp>
#include <gazebo_ros/node.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <string>

namespace lift_drag_force_plugin
{
  class LiftDragForcePlugin : public gazebo::ModelPlugin
  {
    public:
      /// \brief Default constructor

      LiftDragForcePlugin();
      
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

      rclcpp::Node::SharedPtr node_;

      rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_;

      double fluid_density;

      double foil_area;

      int link_id;

      double cla;
      
      double cda0;

      double cda;

      ignition::math::Vector3d center_of_pressure;

      double init_angle_of_attack;

      double span;

      // Pointer to the update event connection
      gazebo::event::ConnectionPtr updateConnection;
  };
}