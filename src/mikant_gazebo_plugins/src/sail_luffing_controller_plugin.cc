#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Assert.hh>
#include <gazebo/common/Events.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

#include "mikant_gazebo_plugins/sail_luffing_controller_plugin.hh"

namespace gazebo
{
  class sail_luffing_controller_plugin : public ModelPlugin
  {
    public: SailLuffingControllerPlugin() {}

    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      // Store the model pointer for convenience.
      this->model = _model;

      // Create a node handle for communication.
      this->node = new ros::NodeHandle();

      // Subscribe to the necessary topics.
      this->node->subscribe("/sailboat_state_plugin/vel", 1, &SailLuffingControllerPlugin::OnVelUpdate, this);
      this->node->subscribe("/wind_world_plugin/wind", 1, &SailLuffingControllerPlugin::OnWindUpdate, this);

      // Advertise the output topic.
      this->pub = this->node->advertise<geometry_msgs::Twist>("/revolute_joint_setting_plugin/input", 1);

      // Initialize controller parameters.
      if (_sdf->HasElement("max_alpha_aw")) {
        this->max_alpha_aw = _sdf->Get<double>("max_alpha_aw");
      } 
      else {
        this->max_alpha_aw = 10.0;
      }
      if (_sdf->HasElement("min_alpha_aw")) {
        this->min_alpha_aw = _sdf->Get<double>("min_alpha_aw");
      } 
      else {
        this->min_alpha_aw = -10.0;
      }
    }

    public: void OnVelUpdate(const geometry_msgs::Twist::ConstPtr& msg)
    {
      // Store the velocity components.
      this->u = msg->linear.x;
      this->v = msg->linear.y;
      this->p = msg->angular.x;
      this->r = msg->angular.z;

      // Calculate the alpha_aw value.
      double vawa = v*cos(alpha_tw)-u*sin(alpha_tw);
      double vawv = v*sin(alpha_tw)+u*cos(alpha_tw);
      alpha_aw = sqrt(pow(vawa, 2) + pow(vawv, 2));

      // Check the saturation limits.
      if (alpha_aw > max_alpha_aw) { //if
        alpha_aw = max_alpha_aw;
      } else if (alpha_aw < min_alpha_aw) {
        alpha_aw = min_alpha_aw;
      }

      // Publish the result to the output topic.
      geometry_msgs::Twist output_msg;
      output_msg.angular.z = alpha_aw;
      this->pub.publish(output_msg);
    }
