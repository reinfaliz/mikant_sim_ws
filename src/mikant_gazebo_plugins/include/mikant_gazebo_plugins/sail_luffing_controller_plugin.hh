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
		
		private: gazebo::physics::ModelPtr model;
		
			 std::string joint_name;
			 double max_sail_angle,min_sail_angle;
			 std::string sub_model_topic,sub_world_topic;
			 std::string world_name,model_name,topic_name,topic_name1,topic_name2;
			 double u,v,p,r,vawu,vawv,alpha_aw;
			 double min_sail_angle,max_sail_angle;
			 double sail_angle;
			 double alpha_tw;
		
			gazebo::event::ConnectionPtr updateConnection;
			void OnUpdate();
	
	};

}
