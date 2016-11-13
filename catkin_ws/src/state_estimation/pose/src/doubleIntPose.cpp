/*
 *==============================================================================
 *Name: Double Integration Pose
 *Creator: Jeremy Mallette
 *Last Updated: 30/10/2016
 *Updated By:
 *
 *Notes:
 *  This is a fairly inaccurate method of calculating the position from a set
 *      of acceleration data-points, given by the IMU.
 *  Uses the "Trapazoid Rule" for integration and performs a simple error
 *      calculation.
 *
 *==============================================================================
 */

//Libraries etc. ---------------------------------------------------------------
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"

//Definitions ------------------------------------------------------------------
#define COMPARISON_THRESHOLD 0.00001

class acceleration {
    public:
        float x;
        float y;
        float z;
        float t;
};

acceleration accIn;

/*  Structure of Acceleration Matrix
 *
 *  (x) {acc, timestamp1},{acc, timestamp2},{acc, timestamp3},
 *  (y) {acc, timestamp1},{acc, timestamp2},{acc, timestamp3},
 *  (z) {acc, timestamp1},{acc, timestamp2},{acc, timestamp3}
 */
float accArr[3][2][3] = {0.0};

/*  Structure of Velocity Matrix
 *
 *  (x) {vel, timestamp1},{vel, timestamp2},
 *  (y) {vel, timestamp1},{vel, timestamp2},
 *  (z) {vel, timestamp1},{vel, timestamp2}
 */
float velArr[3][2][2] = {0.0};

/*  Structure of Position Matrix
 *
 *  (x) {pos, timestamp},
 *  (y) {pos, timestamp},
 *  (z) {pos, timestamp}
 */
float posArr[3][2] = {0.0};

//ROS --------------------------------------------------------------------------
ros::Publisher pub, pub2;
ros::Subscriber sub;

//Update Array -----------------------------------------------------------------
void updateArray(int i) {
    accArr[0][0][i] = accIn.x;
    accArr[1][0][i] = accIn.y;
    accArr[2][0][i] = accIn.z;

    accArr[0][1][i] = accIn.t;
    accArr[1][1][i] = accIn.t;
    accArr[2][1][i] = accIn.t;
}

//Take First Derivative --------------------------------------------------------
void firstIntegral() {
    velArr[0][0][0] = (accArr[0][0][1] - accArr[0][0][0]) / (accArr[0][1][1] - accArr[0][1][0]);
    velArr[1][0][0] = (accArr[1][0][1] - accArr[1][0][0]) / (accArr[1][1][1] - accArr[1][1][0]);
    velArr[2][0][0] = (accArr[2][0][1] - accArr[2][0][0]) / (accArr[2][1][1] - accArr[2][1][0]);

    velArr[0][1][0] = accArr[1][1][1];
    velArr[1][1][0] = accArr[1][1][1];
    velArr[2][1][0] = accArr[1][1][1];

    velArr[0][0][1] = (accArr[0][0][2] - accArr[0][0][1]) / (accArr[0][1][2] - accArr[0][1][1]);
    velArr[1][0][1] = (accArr[1][0][2] - accArr[1][0][1]) / (accArr[1][1][2] - accArr[1][1][1]);
    velArr[2][0][1] = (accArr[2][0][2] - accArr[2][0][1]) / (accArr[2][1][2] - accArr[2][1][1]);

    velArr[0][1][1] = accArr[1][1][2];
    velArr[1][1][1] = accArr[1][1][2];
    velArr[2][1][1] = accArr[1][1][2];
}

//Take Second Derivative -------------------------------------------------------
void secondIntegral() {
    posArr[0][0] += ((velArr[0][0][1] - velArr[0][0][0]) / (velArr[0][1][1] - velArr[0][1][0]));
    posArr[1][0] += ((velArr[1][0][1] - velArr[1][0][0]) / (velArr[1][1][1] - velArr[1][1][0]));
    posArr[2][0] += ((velArr[2][0][1] - velArr[2][0][0]) / (velArr[2][1][1] - velArr[2][1][0]));

    posArr[0][1] = velArr[1][1][1];
    posArr[1][1] = velArr[1][1][1];
    posArr[2][1] = velArr[1][1][1];
}

//Publish Results --------------------------------------------------------------
void publish() {
    //Velocity
    ROS_INFO("Speed in x direction: %f\n", velArr[0][0][1]);
    ROS_INFO("Speed in y direction: %f\n", velArr[1][0][1]);

    geometry_msgs::TwistStamped velocity = geometry_msgs::TwistStamped();
    velocity.twist.linear.x = velArr[0][0][1];
    velocity.twist.linear.y = velArr[1][0][1];
    velocity.twist.linear.z = velArr[2][0][1];
    pub.publish(velocity);

    //Position
    ROS_INFO("Position in x direction: %f\n", posArr[0][0]);
    ROS_INFO("Position in y direction: %f\n", posArr[1][0]);

    geometry_msgs::PoseStamped doubleIntPose = geometry_msgs::PoseStamped();
    doubleIntPose.pose.position.x = posArr[0][0];
    doubleIntPose.pose.position.y = posArr[1][0];
    doubleIntPose.pose.position.z = posArr[2][0];
    pub2.publish(doubleIntPose);
}

//Shift Data -------------------------------------------------------------------
void dataShift() {
    float tempArr[3][2];
    tempArr[0][0] = accArr[0][0][2];
    tempArr[0][1] = accArr[0][1][2];
    tempArr[1][0] = accArr[1][0][2];
    tempArr[1][1] = accArr[1][1][2];
    tempArr[2][0] = accArr[2][0][2];
    tempArr[2][1] = accArr[2][1][2];

    //Needs different version of c++ accArr = {0.0};
    //Needs different version of c++ velArr = {0.0};

    accArr[0][0][0] = tempArr[0][0];
    accArr[0][1][0] = tempArr[0][1];
    accArr[1][0][0] = tempArr[1][0];
    accArr[1][1][0] = tempArr[1][1];
    accArr[2][0][0] = tempArr[2][0];
    accArr[2][1][0] = tempArr[2][1];
}

//AccCallback ----------------------------------------------------------------
void dataCallback(const sensor_msgs::Imu::ConstPtr& data) {
    accIn.x = data->linear_acceleration.x;
    accIn.y = data->linear_acceleration.y;
    accIn.z = data->linear_acceleration.z;

    ros::Time time = ros::Time::now();
}

//Main -------------------------------------------------------------------------
int main (int argc, char **argv) {
    ros::init(argc, argv, "doubleIntPose");
    ros::NodeHandle node;

    sub = node.subscribe("state_estimation/acc", 1000, &dataCallback);

    pub = node.advertise<geometry_msgs::TwistStamped>("imu/vel", 1000);
    pub2 = node.advertise<geometry_msgs::PoseStamped>("imu/pose", 1000);

    //Fill Array For First Time
    ros::spinOnce();
    updateArray(0);

    //Continuous Filling of Array
    while(ros::ok()) {
        for(int i = 1; i < 3; i++) {
            ros::spinOnce();
            updateArray(i);
        }

        //Calculate Integrals
        firstIntegral();
        secondIntegral();

        //Publish Results
        publish();
        dataShift();
    }

    return 0;
}

/*
 *==============================================================================
 */
