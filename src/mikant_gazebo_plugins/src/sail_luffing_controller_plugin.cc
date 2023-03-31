#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Assert.hh>
#include <gazebo/common/Events.hh>
#include <ignition/math/Vector3.hh>
#include <gazebo_ros/node.hpp>
#include <ignition/math/Matrix3.hh>

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/float64_multi_array.hpp"
#include "mikant_gazebo_plugins/sail_luffing_controller_plugin.hh"
#include <string>

namespace sail_luffing_controller_plugin
{
  SailLuffingControllerPlugin::SailLuffingControllerPlugin()
    : max_sail_angle(M_PI/2),
      min_sail_angle(-M_PI/2),
      center_of_pressure(ignition::math::Vector3d(0, 0, 0)),
      model_in_deg(false),
      world_in_deg(false),
      phi(0.0),
      psi(0.0),
      u_b(0.0),
      v_b(0.0),
      p_b(0.0),
      r_b(0.0),
      true_wind_speed(0.0),
      true_wind_angle(0.0)
  {
  }

  void SailLuffingControllerPlugin::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {
    // Store the pointer to the model
    this->model = _model;
    this->world = this->model->GetWorld();

    if(_sdf->HasElement("joint_name"))
    {
      this->joint_name = _sdf->GetElement("joint_name")->Get<std::string>();
    }
    if(_sdf->HasElement("max_sail_angle"))
    {
      this->max_sail_angle = _sdf->Get<double>("max_sail_angle");
    }
    if(_sdf->HasElement("min_sail_angle"))
    {
      this->min_sail_angle = _sdf->Get<double>("min_sail_angle");
    }
    if(_sdf->HasElement("center_of_pressure"))
    {
      this->center_of_pressure = _sdf->Get<ignition::math::Vector3d>("center_of_pressure");
    }
    if(_sdf->HasElement("sub_model_topic"))
    {
      this->sub_model_topic = _sdf->GetElement("sub_model_topic")->Get<std::string>();
    }
    if(_sdf->HasElement("sub_world_topic"))
    {
      this->sub_world_topic = _sdf->GetElement("sub_world_topic")->Get<std::string>();
    }
    if(_sdf->HasElement("model_in_deg"))
    {
      this->model_in_deg = _sdf->Get<bool>("model_in_deg");
    }
    if(_sdf->HasElement("world_in_deg"))
    {
      this->world_in_deg = _sdf->Get<bool>("world_in_deg");
    }

    std::string world_name = this->world->Name();
    std::string model_name = this->model->GetName();
    std::string sub_model_topic_name = world_name + "/" + model_name + "/" + this->sub_model_topic;
    std::string sub_world_topic_name = world_name + "/" + this->sub_world_topic;
    std::string pub_topic_name = world_name + "/" + model_name + "/" + this->joint_name + "/" + "sail_luffing_controller_plugin";

    this->node_ = gazebo_ros::Node::CreateWithArgs(world_name + "_" + model_name + "_" + joint_name + "_sail_luffing_controller_plugin");
    this->sub_model = this->node_->create_subscription<std_msgs::msg::Float64MultiArray>(sub_model_topic_name, 10,
    std::bind(&SailLuffingControllerPlugin::callbacksubmodel, this, std::placeholders::_1));
    this->sub_world = this->node_->create_subscription<std_msgs::msg::Float64MultiArray>(sub_world_topic_name, 10,
    std::bind(&SailLuffingControllerPlugin::callbacksubworld, this, std::placeholders::_1));
    this->publisher_ = this->node_->create_publisher<std_msgs::msg::Float64>(pub_topic_name, 10);

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&SailLuffingControllerPlugin::OnUpdate, this));
  }

  void SailLuffingControllerPlugin::callbacksubmodel(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    this->phi = msg->data[2];
    this->psi = msg->data[3];
    this->u_b = msg->data[4];
    this->v_b = msg->data[5];
    this->p_b = msg->data[6];
    this->r_b = msg->data[7];

    if(this->model_in_deg)
    {
      this->phi = this->phi*M_PI/180;
      this->psi = this->psi*M_PI/180;
      this->p_b = this->p_b*M_PI/180;
      this->r_b = this->r_b*M_PI/180;
    }
  }

  void SailLuffingControllerPlugin::callbacksubworld(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    this->true_wind_speed = msg->data[0];
    this->true_wind_angle = msg->data[1];

    if(this->world_in_deg)
    {
      this->true_wind_angle = this->true_wind_angle*M_PI/180;
    }
  }

  // Called by the world update start event
  void SailLuffingControllerPlugin::OnUpdate()
  {
    double v_awu = true_wind_speed*cos(true_wind_angle - psi) - u_b + (r_b * center_of_pressure.Y());
    double v_awv = true_wind_speed*sin(true_wind_angle - psi)*cos(phi) - v_b - (r_b * center_of_pressure.X()) + (p_b * center_of_pressure.Z()); 

    double signal =  atan2(v_awv ,-v_awu); // Based on alpha_aw

    if(signal > this->max_sail_angle)
    {
      signal = this->max_sail_angle;
    }
    else if(signal < this->min_sail_angle)
    {
      signal = this->min_sail_angle;
    }
    // signal = M_PI/6;

    // Set the data for the message
    auto sail_angle = std_msgs::msg::Float64();
    sail_angle.data = signal;

    // Publish the message
    publisher_->publish(sail_angle);
  }

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(SailLuffingControllerPlugin)
}