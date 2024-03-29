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
#include "mikant_gazebo_plugins/lift_drag_force_plugin.hh"
#include <string>

namespace lift_drag_force_plugin
{
  LiftDragForcePlugin::LiftDragForcePlugin()
    : fluid_density(1.225),
      center_of_pressure(ignition::math::Vector3d(0, 0, 0)),
      true_fluid_speed(0.0),
      true_fluid_angle(0.0),
      delta_s(0.0)
  {   
  }

  void LiftDragForcePlugin::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {
    // Store the pointer to the model
    this->model = _model;
    this->world = this->model->GetWorld();
    this->link_name = _sdf->GetElement("link_name")->Get<std::string>();

    this->link = model->GetLink(link_name);

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
      this->center_of_pressure = _sdf->Get<ignition::math::Vector3d>("center_of_pressure");
    }
    if(_sdf->HasElement("init_actuator_angle"))
    {
      this->init_actuator_angle = _sdf->Get<double>("init_actuator_angle");
    }
    if(_sdf->HasElement("span"))
    {
      this->span = _sdf->Get<double>("span");
    }
    if(_sdf->HasElement("joint_name"))
    {
      this->joint_name = _sdf->GetElement("joint_name")->Get<std::string>();
      this->joint = model->GetJoint(joint_name);
    }
    if(_sdf->HasElement("sub_world_topic"))
    {
      this->sub_world_topic = _sdf->GetElement("sub_world_topic")->Get<std::string>();
    }

    if(this->sub_world_topic != "")
    {
      std::string world_name = this->world->Name();
      std::string topic_name = world_name + "/" + this->sub_world_topic;
      std::string model_name = this->model->GetName();
      
      this->node_ = gazebo_ros::Node::CreateWithArgs(world_name+"_"+model_name+"_"+joint_name+"_lift_drag_force_plugin");
      this->sub_ = this->node_->create_subscription<std_msgs::msg::Float64MultiArray>(topic_name, 10,
    std::bind(&LiftDragForcePlugin::callbacksub, this, std::placeholders::_1));
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

  ignition::math::Vector3d LiftDragForcePlugin::RotateX(const ignition::math::Vector3d Ro)
  {
    ignition::math::Vector3d V = ignition::math::Vector3d(1, -1, -1);
    ignition::math::Vector3d Result = V * Ro;
    return Result;
  }

  // Called by the world update start event
  void LiftDragForcePlugin::OnUpdate()
  {
    ignition::math::Vector3d PoseOrien = RotateX(this->model->WorldPose().Rot().Euler());
    ignition::math::Vector3d VelLinearB = RotateX(this->model->RelativeLinearVel());
    ignition::math::Vector3d VelAngularB = RotateX(this->model->RelativeAngularVel());

    double u_b = VelLinearB.X();
    double v_b = VelLinearB.Y();

    double phi = PoseOrien.X();
    double psi = PoseOrien.Z();

    double p_b = VelAngularB.X();
    double r_b = VelAngularB.Z();

    double v_awu = true_fluid_speed*cos(true_fluid_angle - psi) - u_b + (r_b * center_of_pressure.Y());
    double v_awv = true_fluid_speed*sin(true_fluid_angle - psi)*cos(phi) - v_b - (r_b * center_of_pressure.X()) + (p_b * center_of_pressure.Z());

    double alpha_aw =  atan2(v_awv ,-v_awu);
    double v_aw = sqrt((v_awu*v_awu) + (v_awv*v_awv));

    if(this->joint_name != "")
    {
      this->delta_s = this->joint->Position(0);
    }

    double angle_of_attack = alpha_aw - delta_s + init_actuator_angle;

    double cl = cla *  sin(2 * angle_of_attack);
    double lift_force = 0.5 * fluid_density * foil_area * v_aw * v_aw * cl;

    double cdi = 0;
    if(this->span != 0)
    {
      cdi = (cl * cl * foil_area)/(M_PI * span * span);
    }
    double cd = cda0 + (cda * sin(angle_of_attack) * sin(angle_of_attack)) + cdi;
    double drag_force = 0.5 * fluid_density * foil_area * v_aw  * v_aw * cd;

    double lift_drag_world_x = lift_force *(cos(-psi)*sin(alpha_aw) + sin(-psi)*cos(alpha_aw)) + drag_force * (sin(-psi) * sin(alpha_aw) - cos(alpha_aw) * cos(-psi));
    double lift_drag_world_y = -(lift_force * (cos(alpha_aw) * cos(-psi) - sin(alpha_aw) * sin(-psi)) + drag_force * (cos(alpha_aw) * sin(-psi) + sin(alpha_aw) * cos(-psi)));

    ignition::math::Vector3d lift_drag_world = ignition::math::Vector3d(lift_drag_world_x, lift_drag_world_y, 0);

    double center_of_pressure_xy_radius = sqrt(this->center_of_pressure.X()*this->center_of_pressure.X() + this->center_of_pressure.Y()*this->center_of_pressure.Y());
    double center_of_pressure_xy_angle = atan2(this->center_of_pressure.Y(), this->center_of_pressure.X());
    ignition::math::Vector3d center_of_pressure_world = RotateX(ignition::math::Vector3d(center_of_pressure_xy_radius*cos(center_of_pressure_xy_angle + psi), center_of_pressure_xy_radius*sin(center_of_pressure_xy_angle + psi), 0)).operator+(this->model->WorldPose().Pos());

    link->AddForceAtWorldPosition(lift_drag_world,center_of_pressure_world);
  }

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(LiftDragForcePlugin)
}