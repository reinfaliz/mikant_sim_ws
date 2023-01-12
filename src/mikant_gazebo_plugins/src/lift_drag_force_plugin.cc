#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Assert.hh>
#include <gazebo/common/Events.hh>
#include <ignition/math/Vector3.hh>
#include <gazebo_ros/node.hpp>
#include <math.h>
#include <ignition/math/Matrix3.hh>

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/float64_multi_array.hpp"
#include "mikant_gazebo_plugins/lift_drag_force_plugin.hh"
#include <string>

namespace lift_drag_force_plugin
{
  LiftDragForcePlugin::LiftDragForcePlugin()
    : fluid_density(1.225),
      center_of_pressure(0, 0, 0)
  {   
  }

  void LiftDragForcePlugin::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {
    // Store the pointer to the model
    this->model = _model;
    this->world = this->model->GetWorld();
    this->link_name = _sdf->GetElement("link_name")->Get<std::string>();
    this->link = model->GetLink(link_name);
    this->link_id = link->GetId();

    node_ = gazebo_ros::Node::Get(_sdf);

    this->sub_ = this->node_->create_subscription<std_msgs::msg::Float64MultiArray>("rimsa_world_true_wind", 10,
    std::bind(&LiftDragForcePlugin::callbacksub, this, std::placeholders::_1));

    if(_sdf->HasElement("fluid_density"))
    {
      this->fluid_density = _sdf->Get<double>("fluid_density");
    }
    if(_sdf->HasElement("foil_area"))
    {
      this->foil_area = _sdf->Get<double>("foil_area");
    }
    if(_sdf->HasElement("cla"))
    {
      this->cla = _sdf->Get<double>("cla");
    }
    if(_sdf->HasElement("cda0"))
    {
      this->cda0 = _sdf->Get<double>("cda0");
    }
    if(_sdf->HasElement("cda"))
    {
      this->cda = _sdf->Get<double>("cda");
    }
    if(_sdf->HasElement("center_of_pressure"))
    {
      this->center_of_pressure = _sdf->Get<double>("center_of_pressure");
    }
    if(_sdf->HasElement("init_angle_of_attack"))
    {
      this->init_angle_of_attack = _sdf->Get<double>("init_angle_of_attack");
    }
    if(_sdf->HasElement("span"))
    {
      this->span = _sdf->Get<double>("span");
    }

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&LiftDragForcePlugin::OnUpdate, this));
  }

  void LiftDragForcePlugin::callbacksub(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
  this->true_fluid_speed = msg->data[0];
  this->true_fluid_angle = msg->data[1];
  }



  // Called by the world update start event
  void LiftDragForcePlugin::OnUpdate()
  {    
    ignition::math::Pose3d Pose = this->link->WorldPose();
    ignition::math::Vector3d VelLinearB = this->link->RelativeLinearVel();
    ignition::math::Vector3d VelAngularB = this->link->RelativeAngularVel();

    double u_b = VelLinearB.X();
    double v_b = VelLinearB.Y();

    double phi = Pose.Roll();
    double psi = Pose.Yaw();

    double p_b = VelAngularB.X();
    double r_b = VelAngularB.Z();

    double v_awu = true_fluid_speed*cos(true_fluid_angle - psi) - u_b + (r_b * center_of_pressure.Y());
    double v_awv =true_fluid_speed*sin(true_fluid_angle - psi)*cos(phi) - v_b - (r_b * center_of_pressure.X()) + (p_b * center_of_pressure.Z()); 

  }
  

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(LiftDragForcePlugin)
}