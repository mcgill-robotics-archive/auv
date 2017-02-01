#include <string>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>

#define PI 3.14159265358979323846

tf::Vector3 zero(0,0,0);

tf::Quaternion imuMountToRobotFrame;
tf::Quaternion imuInternalHorizonToMountPoint;
tf::Quaternion rawHorizonToRobot;
tf::Quaternion rotationCorrection;

void imuCallBack(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  ros::Time time(ros::Time::now());
  tf::TransformBroadcaster broadcaster;

  tf::Quaternion orientation(
    msg->pose.orientation.x,
    msg->pose.orientation.y,
    msg->pose.orientation.z,
    msg->pose.orientation.w
  );

  rawHorizonToRobot = imuMountToRobotFrame.inverse()
                    * imuInternalHorizonToMountPoint.inverse()
                    * orientation.inverse()
                    * rotationCorrection.inverse()
                    * imuMountToRobotFrame;

  broadcaster.sendTransform(
    tf::StampedTransform(
      tf::Transform(rawHorizonToRobot, zero),
      time,
      "/raw_horizon",
      "/robot"
    )
  );

  double roll, pitch, yaw;
  tf::Matrix3x3(rawHorizonToRobot).getRPY(roll, pitch, yaw);

  tf::Quaternion yawQuat;
  yawQuat.setRPY(0, 0, yaw);
  broadcaster.sendTransform(tf::StampedTransform(
    tf::Transform(yawQuat, zero),
    time,
    "/raw_horizon",
    "/horizon"
  ));
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "tf_broadcaster");
  ros::NodeHandle node;
  imuMountToRobotFrame.setRPY(0, 0, 0);
  imuInternalHorizonToMountPoint.setRPY(PI, 0, 0);
  // TODO: try to only use the 2 rotations above
  // Correction from /robot frame to axes on IMU:
  rotationCorrection.setRPY(0, 0, -PI/2);

  // TODO: Figure out why nothing gets broadcast without this line
  tf::TransformBroadcaster broadcaster;
  ros::Subscriber imuSub = node.subscribe(
    "state_estimation/pose",
    1000,
    imuCallBack
  );
  ros::spin();

  return 0;
}
