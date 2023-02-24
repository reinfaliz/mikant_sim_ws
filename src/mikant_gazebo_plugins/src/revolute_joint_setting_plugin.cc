#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Assert.hh>
#include <gazebo/common/Events.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

#include "mikant_gazebo_plugins/revolute_joint_setting_plugin.hh"

namespace revolute_joint_setting_plugin
{
  RevoluteJointSettingPlugin::RevoluteJointSettingPlugin()
  : joint_mode(0),
    joint_setting(0.0)
  {
  }

  void RevoluteJointSettingPlugin::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {
    // Store pointers
    this->model = _model;
    this->model_name = this->model->GetName();
    this->world = this->model->GetWorld();
    this->world_name = this->world->Name();
    this->joint_name = _sdf->Get<std::string>("joint_name");
    this->joint = this->model->GetJoint(this->joint_name);
    
    // Store elements
    if(_sdf->HasElement("joint_setting_lower_limit"))
    {
      this->joint_setting_lower_limit = _sdf->Get<double>("joint_setting_lower_limit");
    }
    if(_sdf->HasElement("joint_setting_upper_limit"))
    {
      this->joint_setting_upper_limit = _sdf->Get<double>("joint_setting_upper_limit");
    }
    if(_sdf->HasElement("joint_mode"))
    {
      this->joint_mode = _sdf->Get<int>("joint_mode");
    }
    if(_sdf->HasElement("sub_joint_topic"))
    {
      this->sub_joint_topic = _sdf->Get<std::string>("sub_joint_topic");
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
    this->ros_node =  gazebo_ros::Node::CreateWithArgs(this->world_name + "_" + this->model_name + "_" + this->joint_name + "_revolute_joint_setting_plugin");

    // Create topics of subscriber
    this->ros_node_subscriber[this->sub_joint_topic] = this->ros_node->create_subscription<std_msgs::msg::Float64>(this->world_name + "/" + this->model_name + "/" + this->joint_name + "/"+ this->sub_joint_topic, 0, std::bind(&RevoluteJointSettingPlugin::TopicCallBack, this, std::placeholders::_1));
    
    // Update the event
    this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&RevoluteJointSettingPlugin::OnUpdate, this));
  }

  void RevoluteJointSettingPlugin::TopicCallBack(const std_msgs::msg::Float64::SharedPtr msg)
  {
    this->joint_setting = msg->data;
  }

  // Called by the world update start event
  void RevoluteJointSettingPlugin::OnUpdate()
  {
    // Bound the joint setting
    // double final_joint_setting = this->joint_setting;
    double final_joint_setting = 120;
    if(final_joint_setting > this->joint_setting_upper_limit)
    {
      final_joint_setting = this->joint_setting_upper_limit;
    }
    else if(final_joint_setting < this->joint_setting_lower_limit)
    {
      final_joint_setting = this->joint_setting_lower_limit;
    }

    // Set the joint setting
    if(this->joint_mode == 0)
    {
      this->joint->SetPosition(0,final_joint_setting,true);
    }
    else if(this->joint_mode == 1)
    {
      this->joint->SetVelocity(0,final_joint_setting*0.10471975511965977); // Receive final_joint_setting as rpm and convert to rad/s
    }
  }
  
  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(RevoluteJointSettingPlugin)
}
