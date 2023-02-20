#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Assert.hh>
#include <gazebo/common/Events.hh>
#include <rclcpp/rclcpp.hpp>
#include <gazebo_ros/node.hpp>

#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include "mikant_gazebo_plugins/heading_pid_controller_plugin.hh"
#include <string>

namespace heading_pid_controller_plugin
{

  void HeadingPIDControllerPlugin::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {

    this->model = _model;
    this->world = this->model->GetWorld();
    this->link_name = _sdf->GetElement("link_name")->Get<std::string>();
    this->joint_name = _sdf->GetElement("joint_name")->Get<std::string>();

    std::string world_name = this->world->Name();
    std::string model_name = this->model->GetName();
    std::string sub_topic_name = world_name + "/" + model_name + "/sub_topic";
    std::string pub_topic_name = world_name + "/" + model_name + "/" + joint_name + "/heading_pid_controller_plugin";

    if(_sdf->HasElement("p_gain"))
    {
      this->p_gain = _sdf->Get<double>("p_gain");
    }
    if(_sdf->HasElement("i_gain"))
    {
      this->i_gain = _sdf->Get<double>("i_gain");
    }
    if(_sdf->HasElement("d_gain"))
    {
      this->d_gain = _sdf->Get<double>("d_gain");
    }

    node_ = gazebo_ros::Node::make_shared(world_name+"_"+model_name+"_"+joint_name+"_heading_pid_controller_plugin");
    
    // Set up a update event callback
    this->publisher_ = this->node_->create_publisher<std_msgs::msg::Float64>(pub_topic_name, 10);

    this->sub_ = this->node_->create_subscription<std_msgs::msg::Float64MultiArray>(sub_topic_name, 10,
    std::bind(&HeadingPIDControllerPlugin::callbacksub, this, std::placeholders::_1));
    
    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&HeadingPIDControllerPlugin::OnUpdate, this));
  }

  void HeadingPIDControllerPlugin::callbacksub(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
  x = msg->data[0];
  y = msg->data[1];
  phi = msg->data[2];
  psi = msg->data[3];
  u = msg->data[4];
  v = msg->data[5];
  p = msg->data[6];
  r = msg->data[7];
  }

  // Called by the world update start event
  void HeadingPIDControllerPlugin::OnUpdate()
  {    
    
    error = psi;

    auto Rudder_Angle = std_msgs::msg::Float64();

    // Set the data for the message
    Rudder_Angle.data = p_gain*error;
    // Publish the message
    publisher_->publish(Rudder_Angle);
    
  }
    
  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(HeadingPIDControllerPlugin)
}