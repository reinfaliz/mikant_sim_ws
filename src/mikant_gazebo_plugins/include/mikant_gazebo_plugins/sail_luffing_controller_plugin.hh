#include <gazebo/common/common.hh>
#include <gazebo/common/Event.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>

#include <rclcpp/rclcpp.hpp>
#include <gazebo_ros/node.hpp>
#include <string>

namespace sail_luffing_controller_plugin
{
	class SailLuffingControllerPlugin : public gazebo::ModelPlugin
	{
		public: SailLuffingControllerPlugin();
		void Load(const gazebo::physics::ModelPtr _model, const sdf::ElementPtr _sdf);
		void callbacksub(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

      		ignition::math::Vector3d RotateX(const ignition::math::Vector3d Ro);
		
		private: gazebo::physics::ModelPtr model;
		
			 std::string joint_name;
			 gazebo::physics::JointPtr joint;
			 gazebo::physics::WorldPtr world;
			 ignition::math::Vector3d center_of_pressure;
			 double max_sail_angle,min_sail_angle;
			 std::string sub_model_topic,sub_world_topic;
			 std::string world_name,model_name,topic_name,topic_name1,topic_name2;
			 double u,v,p,r,vawu,vawv,alpha_aw;
			 double sail_angle;
			 double alpha_tw;
			 
			 rclcpp::Node::SharedPtr node_;
			 rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub;
			 rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;  
			 rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_1;
			 rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_2;
			 
			 double true_wind_speed;
			 double true_wind_angle;
		
			gazebo::event::ConnectionPtr updateConnection;
			void OnUpdate();
	
	};

}
