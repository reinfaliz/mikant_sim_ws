#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Assert.hh>
#include <gazebo/common/Events.hh>
#include <rclcpp/rclcpp.hpp>
#include <gazebo_ros/node.hpp>

#include <std_msgs/msg/float64_multi_array.hpp>
#include "mikant_gazebo_plugins/wind_world_plugin.hh"
#include <string>

namespace wind_world_plugin
{

  WindWorldPlugin::WindWorldPlugin()
    : true_wind_speed(3),
      true_wind_angle(0)
  {   
  }

  void WindWorldPlugin::Load(gazebo::physics::WorldPtr _parent, sdf::ElementPtr _sdf)
  {
    std::string world_name = _parent->Name();
    std::string topic_name = world_name + "/true_wind" ;

    if(_sdf->HasElement("true_wind_speed"))
    {
      this->true_wind_speed = _sdf->Get<double>("true_wind_speed");
    }
    if(_sdf->HasElement("true_wind_angle"))
    {
      this->true_wind_angle = _sdf->Get<double>("true_wind_angle");
    }

    node_ = gazebo_ros::Node::CreateWithArgs("wind_world_plugin_" + world_name);
    RCLCPP_INFO(node_->get_logger(), "Loading World Wind Plugin");
    
    // Set up a update event callback
    this->publisher_ = this->node_->create_publisher<std_msgs::msg::Float64MultiArray>(topic_name, 10);

    
    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&WindWorldPlugin::OnUpdate, this));
  }

  // Called by the world update start event
  void WindWorldPlugin::OnUpdate()
  {    
    auto true_wind = std_msgs::msg::Float64MultiArray();

    // Set the data for the message
    true_wind.data.resize(2);
    true_wind.data[0] = true_wind_speed;
    true_wind.data[1] = true_wind_angle;

    // Publish the message
    publisher_->publish(true_wind);
    
  }
    
  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(WindWorldPlugin)
}

