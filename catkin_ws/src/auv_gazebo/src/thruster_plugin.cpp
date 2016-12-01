#include <boost/bind.hpp>
#include <thread>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <auv_msgs/MotorCommands.h>

class ThrusterController : public gazebo::ModelPlugin
{
public:
  ThrusterController();
  void Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf);
  void OnUpdate(const gazebo::common::UpdateInfo & info);

private:
  // ros::NodeHandle nh_;
  ros::Subscriber thrust_cmds_sub_;

  void thrustCommandCallback(const auv_msgs::MotorCommands::ConstPtr& msg);
  void queueThread();

  gazebo::physics::ModelPtr model_;
  gazebo::event::ConnectionPtr updateConnection_;
  std::string robot_namespace_;

  auv_msgs::MotorCommands current_commands_;

  /// \brief A node use for ROS transport
  std::unique_ptr<ros::NodeHandle> nh_;

/// \brief A ROS callbackqueue that helps process messages
  ros::CallbackQueue rosQueue;

/// \brief A thread the keeps running the rosQueue
  std::thread rosQueueThread;
};

ThrusterController::ThrusterController()
{
  auv_msgs::MotorCommands zeros;
  current_commands_ = zeros;
  ROS_INFO("Controller");
}

void ThrusterController::Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  model_ = _parent;
  ROS_INFO("Loading");

  if (_sdf->HasElement("robotNamespace"))
  {
    robot_namespace_ = _sdf->Get<std::string>("robotNamespace") + "/";
  }
  else
  {
    ROS_INFO("AUV thruster plugin missing <robotNameSpace>, defaults to /bradbury");
    robot_namespace_ = "bradbury";
  }

  // thrust_cmds_sub_ = nh_.subscribe<auv_msgs::MotorCommands>("electrical_interface/motor", 10, &ThrusterController::thrustCommandCallback, this);

  // Initialize ros, if it has not already been initialized.
  if (!ros::isInitialized())
  {
    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "thrust_gazebo_client", ros::init_options::NoSigintHandler);
  }

  // Create ROS node.
  nh_.reset(new ros::NodeHandle("thrust_gazebo_client"));

  // Create a named topic, and subscribe to it.
  ros::SubscribeOptions so =
    ros::SubscribeOptions::create<auv_msgs::MotorCommands>(
        "/electrical_interface/motor",
        1,
        boost::bind(&ThrusterController::thrustCommandCallback, this, _1),
        ros::VoidPtr(), &rosQueue);
  thrust_cmds_sub_ = nh_->subscribe(so);

  // Spin up the queue helper thread.
  rosQueueThread = std::thread(std::bind(&ThrusterController::queueThread, this));

  updateConnection_ = gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&ThrusterController::OnUpdate, this, _1));
}

void ThrusterController::OnUpdate(const gazebo::common::UpdateInfo & info)
{
  // ROS_INFO("A");
  // starboard_surge
  model_->GetLink("base_link")->AddForceAtRelativePosition(gazebo::math::Vector3(current_commands_.starboard_surge, 0, 0), gazebo::math::Vector3(0, 0, 0));
  // ROS_INFO("B");
  // model_->GetJoint("starboard_surge")->SetForce(0, (double)current_commands_.starboard_surge);
  // model_->GetJoint("bow_sway")->SetForce(0, (double)current_commands_.bow_sway);
  // model_->GetJoint("stern_sway")->SetForce(0, (double)current_commands_.stern_sway);
  // model_->GetJoint("port_bow_heave")->SetForce(0, (double)current_commands_.port_bow_heave);
  // model_->GetJoint("starboard_bow_heave")->SetForce(0, (double)current_commands_.starboard_bow_heave);
  // model_->GetJoint("port_stern_heave")->SetForce(0, (double)current_commands_.port_stern_heave);
  // model_->GetJoint("starboard_stern_heave")->SetForce(0, (double)current_commands_.starboard_stern_heave);
  // ROS_INFO("C");
}

void ThrusterController::thrustCommandCallback(const auv_msgs::MotorCommands::ConstPtr& msg)
{
  current_commands_ = *msg;
  ROS_INFO("Received message yay");
}

void ThrusterController::queueThread()
{
  static const double timeout = 0.01;
  while (nh_->ok())
  {
    rosQueue.callAvailable(ros::WallDuration(timeout));
  }
}

GZ_REGISTER_MODEL_PLUGIN(ThrusterController);
