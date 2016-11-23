/*
 *==============================================================================
 * Name: Double Integration Pose
 * Creator: Jeremy Mallette
 * Last Updated: 30/10/2016
 * Updated By:
 *
 * Notes:
 *      - This is a fairly inaccurate method of calculating the position from a
 *          set of acceleration data-points, given by the IMU.
 *      - It makes use of the trapazoid rule to provide increased accuracy and
 *          in consideration of special cases.
 *
 * TODO:
 *      - Use the Euler Method instead of the trapazoid method.
 *      - Use timestamps instead of a fixed rate.
 *
 *==============================================================================
 */

//For TEST NODE
//Note: to test, make sure to change subscriber
//#define RATE 10
//#define DELTA_T 0.1

//For IMU
#define RATE 8.35               //In Hz
#define DELTA_T 0.119760479     //Must be 1/RATE

//Libraries etc. ---------------------------------------------------------------
#include "doubleIntPose.h"
#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"

//Definitions ------------------------------------------------------------------
timedVector currAcc, lastAcc, currVel, lastVel, position;

//ROS --------------------------------------------------------------------------
ros::Publisher pub, pub2;
ros::Subscriber sub;

//Publish Results --------------------------------------------------------------
void publish() {
    //Velocity
    geometry_msgs::TwistStamped velocity = geometry_msgs::TwistStamped();
    velocity.twist.linear.x = currVel.getX();
    velocity.twist.linear.y = currVel.getY();
    velocity.twist.linear.z = currVel.getZ();
    pub.publish(velocity);
    //Position
    geometry_msgs::PoseStamped pose = geometry_msgs::PoseStamped();
    pose.pose.position.x = position.getX();
    pose.pose.position.y = position.getY();
    pose.pose.position.z = position.getZ();
    pub2.publish(pose);
}

//Print Data -------------------------------------------------------------------
void printData() {
    ROS_INFO("DEBUG: Acceleration from IMU: %lf\n", currAcc.getX());
    ROS_INFO("Speed in x direction: %lf", currVel.getX());
    ROS_INFO("Speed in y direction: %lf", currVel.getY());
    ROS_INFO("Position in x direction: %lf", position.getX());
    ROS_INFO("Position in y direction: %lf", position.getY());
    printf("--------------------------------------\n");
}

//Callback ---------------------------------------------------------------------
void dataCallback(const geometry_msgs::Vector3ConstPtr& data) {
    //Update Accelerations
    lastAcc = currAcc;
    currAcc.setVector(data->x, data->y, data->z);
    //Update Velocities
    lastVel = currVel;
    currVel.integrate(currAcc, lastAcc, DELTA_T);
    //Update Position
    position.integrate(currVel, lastVel, DELTA_T);

    printData();
    publish();
}

//Main -------------------------------------------------------------------------
int main (int argc, char **argv) {
    unsigned int count = 0;

    ros::init(argc, argv, "double_int_pose");
    ros::NodeHandle node;

    sub = node.subscribe("/state_estimation/acc", 100, &dataCallback);

    pub = node.advertise<geometry_msgs::TwistStamped>("imu/vel", 100);
    pub2 = node.advertise<geometry_msgs::PoseStamped>("imu/pose", 100);

    ros::Rate r(RATE);
    while(ros::ok()) {
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}

/*
 *==============================================================================
 */
