#include <gazebo/common/common.hh>
#include <gazebo/common/Event.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>

#include <rclcpp/rclcpp.hpp>
#include <gazebo_ros/node.hpp>
#include <string>

namespace wind_world_plugin
{
  /// \brief A class for storing buoyancy object properties
  class WindWorldPlugin : public gazebo::WorldPlugin
  {
    public:
      /// \brief Default constructor

      WindWorldPlugin();
      
      void Load(const gazebo::physics::WorldPtr _parent, const sdf::ElementPtr _sdf);
      
      /// \brief Associated link name            
      
    private:
      void OnUpdate();
      
      // Pointer to the model
      gazebo::physics::ModelPtr model;
      
      //Retrieved when the model is loaded.
      gazebo::physics::WorldPtr world;

      std::string link_name;

      gazebo::physics::LinkPtr link;

      double true_wind_speed;

      double true_wind_angle;
      
      rclcpp::Node::SharedPtr node_;

      rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;      

      // Pointer to the update event connection
      gazebo::event::ConnectionPtr updateConnection;
  };
}