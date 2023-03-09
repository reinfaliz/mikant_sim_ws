#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Assert.hh>
#include <gazebo/common/Events.hh>
#include <ignition/math/Vector3.hh>
#include <gazebo_ros/node.hpp>
#include <ignition/math/Matrix3.hh>

#include <rclcpp/rclcpp.hpp>
#include <string>
#include "std_msgs/msg/float64_multi_array.hpp"

#include "mikant_gazebo_plugins/sail_luffing_controller_plugin.hh"

namespace sail_luffing_controller_plugin
{
  SailLuffingControllerPlugin::SailLuffingControllerPlugin()
  :
  {
  }

	void SailLuffingControllerPlugin::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
    	{
      	// Store the model pointer for convenience.
      	this->model = _model;
      	
      	// Initialize controller parameters.
      	if(_sdf->HasElement("max_sail_angle")){
      		this->max_sail_angle = _sdf->Get<double>("max_sail_angle");
      	}
      	
      	
      	if(_sdf->HasElement("min_sail_angle")){
      		this->min_sail_angle = _sdf->Get<double>("min_sail_angle");
      	}
      	
      	
      	if(_sdf->HasElement("joint_name"))
    	{
      		this->joint_name = _sdf->GetElement("joint_name")->Get<std::string>();
      		this->joint = model->GetJoint(joint_name);
    	}
    	
    	
    	if(_sdf->HasElement("sub_model_topic"))
    	{
      		this->sub_model_topic = _sdf->GetElement("sub_model_topic")->Get<std::string>();
    	}
    	
    	
    	if(_sdf->HasElement("sub_world_topic"))
    	{
      		this->sub_world_topic = _sdf->GetElement("sub_world_topic")->Get<std::string>();
    	}
      	
      	
      	std::string world_name = this->world->Name();
      	std::string model_name = this->model->GetName();
      	std::string topic_name = world_name+'/'+model_name+'/'+joint_name+'/'+"sail_luffing_controller_plugin";
      	std::string topic_name1 = world_name+'/'+model_name+'/'+sub_model_topic;
      	std::string topic_name2 = world_name+'/'+sub_world_topic;
      	
      	
	
      	// Create a node handle for communication.
      	node_ = gazebo_ros::Node::CreateWithArgs(<world_name>+'_'+<model_name>+'_'+<joint_name>+'_'+"sail_luffing_controller_plugin");

      	// Subscribe to the necessary topics.
      	this->sub_ = this->node_->create_subscription<std_msgs::msg::Float64MultiArray>(topic_name1, 10,
    	std::bind(&SailLuffingControllerPlugin::callbacksub, this, std::placeholders::_1));
      	this->sub_ = this->node_->create_subscription<std_msgs::msg::Float64MultiArray>(topic_name2, 10,
    	std::bind(&SailLuffingControllerPlugin::callbacksub, this, std::placeholders::_1));

      	// Advertise the output topic.
      	this->pub = this->node_->create_publisher<std_msgs::msg::Float64MultiArray>(topic_name, 10);

      	
    	this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&SailLuffingControllerPlugin::OnUpdate, this));
  	}

    	void SailLuffingControllerPlugin::OnUpdate()
    	{
    	
      	// Store the velocity components.
      	ignition::math::Vector3d PoseOrien = RotateX(this->model->WorldPose().Rot().Euler());
    	ignition::math::Vector3d VelLinearB = RotateX(this->model->RelativeLinearVel());
    	ignition::math::Vector3d VelAngularB = RotateX(this->model->RelativeAngularVel());
    	
    	double u = VelLinearB.X();
    	double v = VelLinearB.Y();

    	double p = VelAngularB.X();
    	double r = VelAngularB.Z();

      	// Calculate the alpha_aw value.
      	double vawu = v*cos(alpha_tw)-u*sin(alpha_tw);
      	double vawv = v*sin(alpha_tw)+u*cos(alpha_tw);
      	double alpha_aw = arctan2(vawv,-vawu);

      	// Check the saturation limits.
      	if (alpha_aw > max_sail_angle) {
        	sail_angle = max_sail_angle;
      	} 
      	if (alpha_aw < min_sail_angle) {
        	sail_angle = min_sail_angle;
      	}
	//what happen when alpha_aw == min_sail_angle?
	
      	// Publish the result to the output topic.
      	publisher_->publish(sail_angle);
    	}



// Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(SailLuffingControllerPlugin)
}
  
  
