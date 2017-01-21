#include <boost/bind.hpp>
#include <thread>

#include <gazebo/gazebo.hh> //include core gazebo functions
#include <gazebo/physics/physics.hh> //include gazebo physics functions
#include <gazebo/common/common.hh> //classes and functions used by mutliple modules

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>

#include <std_msgs/Float64.h>
#include <gazebo_msgs/GetModelState.h>

class DepthPlugin : public gazebo :: ModelPlugin
{
public:
    DepthPlugin();

    void Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    void OnUpdate(const gazebo::common::UpdateInfo & info);

private:

    void queueThread();
    ros::Publisher depth_pub_;

  // A node use for ROS transport
    std::unique_ptr<ros::NodeHandle> nh_;

  // A ROS callbackqueue that helps process messages
    ros::CallbackQueue rosQueue;

  // A thread the keeps running the rosQueue
    std::thread rosQueueThread;

    gazebo::physics::ModelPtr model_;
    gazebo::event::ConnectionPtr updateConnection_;
    std::string robot_namespace_;

    ros::ServiceClient depth_client_;

};//close class DepthPlugin

DepthPlugin::DepthPlugin()
    {}

void DepthPlugin::Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  model_ = _parent;

  if (_sdf->HasElement("robotNamespace"))
  {
    robot_namespace_ = _sdf->Get<std::string>("robotNamespace") + "/";
  }
  else
  {
    ROS_INFO("AUV depth plugin missing <robotNameSpace>, defaults to /bradbury");
    robot_namespace_ = "bradbury";
  }

  // Initialize ros, if it has not already been initialized.
  if (!ros::isInitialized())
  {
    int argc = 0;
    char **argv = NULL;
    //name of node: depth_gazebo_client
    ros::init(argc, argv, "depth_gazebo_client", ros::init_options::NoSigintHandler);
  }

  // Create ROS node.
  nh_.reset(new ros::NodeHandle("depth_gazebo_client"));

  depth_pub_ = nh_->advertise<std_msgs::Float64>("/state_estimation/depth", 1);

  depth_client_ = nh_->serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

  depth_client_.waitForExistence();

  // Spin up the queue helper thread.
  rosQueueThread = std::thread(std::bind(&DepthPlugin::queueThread, this));

  updateConnection_ = gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&DepthPlugin::OnUpdate, this, _1));

}

void DepthPlugin::OnUpdate(const gazebo::common::UpdateInfo & info)
{
    gazebo_msgs::GetModelState state;
    state.request.model_name = "bradbury";

    if(!depth_client_.call(state))
    {
        ROS_ERROR("Service call failed");
    }

    std_msgs::Float64 depth;
    depth.data = state.response.pose.position.z;

    depth_pub_.publish(depth);
}

void DepthPlugin::queueThread()
{
  static const double timeout = 0.01;
  while (nh_->ok())
  {
    rosQueue.callAvailable(ros::WallDuration(timeout));
  }
}

//Register Plugin at the end
GZ_REGISTER_MODEL_PLUGIN(DepthPlugin);
