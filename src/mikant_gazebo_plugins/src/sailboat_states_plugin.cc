#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Assert.hh>
#include <gazebo/common/Events.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

#include "mikant_gazebo_plugins/sailboat_states_plugin.hh"

namespace sailboat_states_plugin
{
  SailboatStatesPlugin::SailboatStatesPlugin()
  : convert_to_deg(false)
  {
  }

  void SailboatStatesPlugin::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {
    // Store pointers
    this->model = _model;
    this->model_name = model->GetName();
    this->world = this->model->GetWorld();
    this->world_name = this->world->Name();
    
    // Store elements
    if(_sdf->HasElement("rudder_joint_name"))
    {
      this->rudder_joint_name = _sdf->GetElement("rudder_joint_name")->Get<std::string>();
      this->rudder_joint = model->GetJoint(rudder_joint_name);
    }
    if(_sdf->HasElement("sail_joint_name"))
    {
      this->sail_joint_name = _sdf->GetElement("sail_joint_name")->Get<std::string>();
      this->sail_joint = model->GetJoint(sail_joint_name);
    }
    if(_sdf->HasElement("tail_joint_name"))
    {
      this->tail_joint_name = _sdf->GetElement("tail_joint_name")->Get<std::string>();
      this->tail_joint = model->GetJoint(tail_joint_name);
    }
    if(_sdf->HasElement("left_propeller_joint_name"))
    {
      this->left_propeller_joint_name = _sdf->GetElement("left_propeller_joint_name")->Get<std::string>();
      this->left_propeller_joint = model->GetJoint(left_propeller_joint_name);
    }
    if(_sdf->HasElement("right_propeller_joint_name"))
    {
      this->right_propeller_joint_name = _sdf->GetElement("right_propeller_joint_name")->Get<std::string>();
      this->right_propeller_joint = model->GetJoint(right_propeller_joint_name);
    }
    if(_sdf->HasElement("convert_to_deg"))
    {
      this->convert_to_deg = _sdf->GetElement("convert_to_deg")->Get<bool>();
    }

    // Check the rclcpp
    if (!rclcpp::ok())
    {
      gzerr << "Not loading plugin since ROS has not been "
        << "properly initialized.  Try starting gazebo with ros plugin:\n"
        << "  gazebo -s libgazebo_ros_api_plugin.so\n";
      return;
    }

    // Create the ROS node
    ros_node =  gazebo_ros::Node::CreateWithArgs("sailboat_states_plugin_" + model_name);

    // Create topics of publisher
    ros_node_publisher["states"] = ros_node->create_publisher<std_msgs::msg::Float64MultiArray>(world_name + "/" + model_name + "/states", 0);
    ros_node_publisher["joints"] = ros_node->create_publisher<std_msgs::msg::Float64MultiArray>(world_name + "/" + model_name + "/joints", 0);
    ros_node_publisher["sim_time"] = ros_node->create_publisher<std_msgs::msg::Float64>(world_name + "/sim_time", 0);

    // Update the event
    this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&SailboatStatesPlugin::OnUpdate, this));
  }

  // This function rotate the x-axis of the gazebo coordinate system by pi
  ignition::math::Vector3d SailboatStatesPlugin::ToNED(const ignition::math::Vector3d gazebo_vector)
  {
    ignition::math::Vector3d rotation = ignition::math::Vector3d(1,-1,-1);
    ignition::math::Vector3d ned_vector = gazebo_vector.operator*(rotation);
    return ned_vector;
  }

  // Called by the world update start event
  void SailboatStatesPlugin::OnUpdate()
  {
    // Calculate the CG of the model in the world frame
    double total_link_mass = 0.0;
    ignition::math::Vector3d model_cg_world_position = ignition::math::Vector3d(0, 0, 0);
    for(unsigned int i = 0; i < model->GetChildCount(); i++)
    {
      gazebo::physics::BasePtr temp_base = model->GetChild(i);
      if(temp_base->HasType(gazebo::physics::Base::LINK))
      {
        gazebo::physics::LinkPtr temp_link = model->GetLink(temp_base->GetName());
        double temp_link_mass = temp_link->GetInertial()->Mass();
        model_cg_world_position.operator+=(temp_link->WorldCoGPose().Pos() * temp_link_mass);
        total_link_mass = total_link_mass + temp_link_mass;
      }
    }
    model_cg_world_position = model_cg_world_position/total_link_mass;

    // Get the model states
    ignition::math::Vector3d model_n_position = SailboatStatesPlugin::ToNED(model_cg_world_position);
    ignition::math::Vector3d model_n_orientation = SailboatStatesPlugin::ToNED(model->WorldPose().Rot().Euler());
    ignition::math::Vector3d model_b_linearvel = SailboatStatesPlugin::ToNED(model->RelativeLinearVel());
    ignition::math::Vector3d model_b_angularvel = SailboatStatesPlugin::ToNED(model->RelativeAngularVel());
    double x = model_n_position.X(); //x-position in the n-frame
    double y = model_n_position.Y(); //y-position in the n-frame
    double phi = model_n_orientation.X(); //roll angle in the n-frame
    double psi = model_n_orientation.Z(); //yaw angle in the n-frame
    double u = model_b_linearvel.X(); //surge speed in the b-frame
    double v = model_b_linearvel.Y(); //sway speed in the b-frame
    double p = model_b_angularvel.X(); //roll speed in the b-frame
    double r = model_b_angularvel.Z(); //pitch speed in the b-frame

    // Get the joint states
    double rudder_joint_position = rudder_joint->Position(0); //rudder angle in the b-frame
    double sail_joint_position = sail_joint->Position(0); //sail angle in the b-frame
    double tail_joint_position = tail_joint->Position(0); //tail angle in the b-frame
    double left_propeller_joint_rpm = 9.549296585513721*left_propeller_joint->GetVelocity(0); //RPM of the left propeller
    double right_propeller_joint_rpm = 9.549296585513721*right_propeller_joint->GetVelocity(0); //RPM of the right propeller

    // Convert the angular from rad to deg if convert_to_deg == true
    if(convert_to_deg)
    {
      phi = phi*180/M_PI;
      psi = psi*180/M_PI;
      p = p*180/M_PI;
      r = r*180/M_PI;
      rudder_joint_position = rudder_joint_position*180/M_PI;
      sail_joint_position = sail_joint_position*180/M_PI;
      tail_joint_position = tail_joint_position*180/M_PI;
    }

    // Get the simulation time
    double sim_time = this->world->SimTime().Double();

    // Assign the model states to be published
    std_msgs::msg::Float64MultiArray msg_array;
    msg_array.data = {x, y, phi, psi, u, v, p, r};
    
    // Publish the model states to the topic
    std::static_pointer_cast<
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>>(ros_node_publisher["states"])->publish(msg_array);

    // Assign the joint states to be published
    msg_array.data = {rudder_joint_position, sail_joint_position, tail_joint_position, left_propeller_joint_rpm, right_propeller_joint_rpm};
        
    // Publish the joint states to the topic
    std::static_pointer_cast<
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>>(ros_node_publisher["joints"])->publish(msg_array);

    // Assign the simulation time to be published
    std_msgs::msg::Float64 msg;
    msg.data = sim_time;
        
    // Publish the joint states to the topic
    std::static_pointer_cast<
        rclcpp::Publisher<std_msgs::msg::Float64>>(ros_node_publisher["sim_time"])->publish(msg);
  }
  
  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(SailboatStatesPlugin)
}
