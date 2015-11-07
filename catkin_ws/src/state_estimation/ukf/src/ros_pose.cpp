#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "sensor_msgs/Imu.h"

#include "ukf_pose.h"

ros::Publisher pub;
ros::Subscriber sub;
ukf_pose estimator;
Vector3d acc, gyro, pose;

void msgVectorToEigenVector(Ref<Vector3d> vector3d, geometry_msgs::Vector3 vector) {
    vector3d << vector.x, vector.y, vector.z;
}

// angle*axis to quaternion
Quaterniond quaternion(Vector3d pose) {
    if (pose.norm() > 0) {
        return Quaterniond(AngleAxisd(pose.norm(), pose.normalized()));
    } else {
        return Quaterniond(1, 0, 0, 0);
    }
}

void poseToQuaternion(geometry_msgs::Quaternion& msgQ, Vector3d angleAxis) {
    Quaterniond q(quaternion(angleAxis));
    msgQ.w = q.w();
    msgQ.x = q.x();
    msgQ.y = q.y();
    msgQ.z = q.z();
}

void dataCallback(const sensor_msgs::Imu::ConstPtr& imu) {
    msgVectorToEigenVector(acc, imu->linear_acceleration);
    msgVectorToEigenVector(gyro, imu->angular_velocity);

    acc = -9.8 * acc.normalized(); //TODO(max) possible aliasing
    gyro *= 0.026;

    //Max: testing only
    //acc *= 0.;
    //gyro *= 0.;

    estimator.update(acc, gyro, pose);
    geometry_msgs::Quaternion quat = geometry_msgs::Quaternion();
    //printf("%10f\n", pose(0));
    poseToQuaternion(quat, pose);

    geometry_msgs::PoseStamped posStamped = geometry_msgs::PoseStamped();
    posStamped.header = imu->header;
    posStamped.header.frame_id = "base_footprint";
    posStamped.pose.orientation = quat;

    pub.publish(posStamped);
}


int main (int argc, char **argv) {
    ros::init(argc, argv, "pose_ukf");
    ros::NodeHandle node;

    pub = node.advertise<geometry_msgs::PoseStamped>("ukf", 100);
    sub = node.subscribe("imu_data", 100, dataCallback);

    ros::spin();

    return 0;
}
