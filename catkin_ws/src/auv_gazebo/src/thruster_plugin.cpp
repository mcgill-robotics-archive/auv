#include <boost/bind.hpp>
#include <thread>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <geometry_msgs/Wrench.h>
#include <gazebo_msgs/ApplyBodyWrench.h>

class ThrusterController : public gazebo::ModelPlugin
{
public:
  ThrusterController();

  /**
   * Function from base class, which runs once when the plugin loads.
   * @param _parent Parent model.
   * @param _sdf    Robot description.
   */
  void Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf);

  /**
   * Function from base class, which runs on every Gazebo update loop.
   * @param info Update info.
   */
  void OnUpdate(const gazebo::common::UpdateInfo & info);

private:
  ros::Subscriber thrust_cmds_sub_;

  /**
   * Callback for the thrust command on the /controls/wrench topic. Applies the
   * published wrench to the robot model.
   * @param msg Wrench message.
   */
  void thrustCommandCallback(const geometry_msgs::Wrench::ConstPtr& msg);
  void queueThread();

  gazebo::physics::ModelPtr model_;
  gazebo::event::ConnectionPtr updateConnection_;
  std::string robot_namespace_;
  bool new_cmd_;  // True if a new command has been received.

  geometry_msgs::Wrench current_commands_;
  ros::ServiceClient control_client_;

  // A node use for ROS transport
  std::unique_ptr<ros::NodeHandle> nh_;

  // A ROS callbackqueue that helps process messages
  ros::CallbackQueue rosQueue;

  // A thread the keeps running the rosQueue
  std::thread rosQueueThread;
};

ThrusterController::ThrusterController() :
  new_cmd_(false)
{
  geometry_msgs::Wrench zeros;
  current_commands_ = zeros;
}

void ThrusterController::Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  model_ = _parent;

  if (_sdf->HasElement("robotNamespace"))
  {
    robot_namespace_ = _sdf->Get<std::string>("robotNamespace") + "/";
  }
  else
  {
    ROS_INFO("AUV thruster plugin missing <robotNameSpace>, defaults to /bradbury");
    robot_namespace_ = "bradbury";
  }

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
    ros::SubscribeOptions::create<geometry_msgs::Wrench>(
        "/controls/wrench",
        1,
        boost::bind(&ThrusterController::thrustCommandCallback, this, _1),
        ros::VoidPtr(), &rosQueue);
  thrust_cmds_sub_ = nh_->subscribe(so);

  control_client_ = nh_->serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench");
  control_client_.waitForExistence();

  // Spin up the queue helper thread.
  rosQueueThread = std::thread(std::bind(&ThrusterController::queueThread, this));

  updateConnection_ = gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&ThrusterController::OnUpdate, this, _1));
}

void ThrusterController::OnUpdate(const gazebo::common::UpdateInfo& info)
{
  // Only send a cmd if there's a new one.
  if (new_cmd_)
  {
    gazebo_msgs::ApplyBodyWrench wrench;
    wrench.request.body_name = "base_link";
    wrench.request.reference_frame = "base_link";
    wrench.request.duration = ros::Duration(0.2);

    wrench.request.wrench = current_commands_;

    // Our coordinate system inverts z, y, yaw and pitch.
    wrench.request.wrench.force.z *= -1;
    wrench.request.wrench.force.y *= -1;
    wrench.request.wrench.torque.z *= -1;
    wrench.request.wrench.torque.y *= -1;

    // Call the apply wrench service.
    if (!control_client_.call(wrench))
    {
      ROS_ERROR("Service call failed");
    }

    // Reset the commands to zero.
    geometry_msgs::Wrench zeros;
    current_commands_ = zeros;
    new_cmd_ = false;
  }
}

void ThrusterController::thrustCommandCallback(const geometry_msgs::Wrench::ConstPtr& msg)
{
  // Save the published command and indicate that a new command has arrived.
  current_commands_ = *msg;
  new_cmd_ = true;
}

void ThrusterController::queueThread()
{
  static const double timeout = 0.01;
  while (nh_->ok())
  {
    rosQueue.callAvailable(ros::WallDuration(timeout));
  }
}

// Necessary to register this as a Gazebo plugin.
GZ_REGISTER_MODEL_PLUGIN(ThrusterController);
