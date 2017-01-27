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

std::random_device rd;
std::mt19937 gen(rd());

class DepthPlugin : public gazebo::ModelPlugin
{
public:
    DepthPlugin();

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
    std::normal_distribution<> gaussian_noise; //declare noise function

    ros::ServiceClient depth_client_;

    int count;
    double depth_noise;

};//close class DepthPlugin

DepthPlugin::DepthPlugin() :
  count(1)
  {
  }

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
    ros::init(argc, argv, "depth_gazebo_client", ros::init_options::NoSigintHandler);
  }

  //set up gaussian noise distribution
  if(_sdf->HasElement("depthNoise"))
  {
    depth_noise = _sdf->Get<double>("depthNoise");
  }
  else
  {
    depth_noise = 0.0;
    ROS_INFO("depth plugin missing <depthNoise>, defaults to %f", depth_noise);
  }
  gaussian_noise = std::normal_distribution<>(0,depth_noise);

  // Create ROS node.
  nh_.reset(new ros::NodeHandle("depth_gazebo_client"));
  // Create the publisher
  depth_pub_ = nh_->advertise<std_msgs::Float64>("/state_estimation/depth", 1);
  // Create client for gazebo service Get Model State
  depth_client_ = nh_->serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

  depth_client_.waitForExistence();

  // Spin up the queue helper thread.
  rosQueueThread = std::thread(std::bind(&DepthPlugin::queueThread, this));

  updateConnection_ = gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&DepthPlugin::OnUpdate, this, _1));

}

void DepthPlugin::OnUpdate(const gazebo::common::UpdateInfo & info)
{
  count++;

  // enforeces a lower publish frequency
  if (count % 5 != 0)
  {
    return;
  }

  // Fill in parameters for service
  gazebo_msgs::GetModelState state;
  state.request.model_name = "bradbury";

  // Call Get Model State service
  if(!depth_client_.call(state))
  {
      ROS_ERROR("Service call failed");
  }

  std_msgs::Float64 depth;
  //get z-coordinate from gazebo and publish
  depth.data = state.response.pose.position.z * -1;
  //add noise
  depth.data = depth.data + gaussian_noise(gen);

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

//Register as Gazebo plugin
GZ_REGISTER_MODEL_PLUGIN(DepthPlugin);
