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
 *
 *==============================================================================
 */

//Libraries etc. ---------------------------------------------------------------
#include "ros/ros.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"

//Definitions ------------------------------------------------------------------
#define DELTA_TIME 0.01 //For debugging

class acceleration {
    public:
        float x;
        float y;
        float z;
        float t;
} accIn, accArr[3];

class velocity {
    public:
        float x;
        float y;
        float z;
        float t;
} velArr[2];


class position {
    public:
        float x;
        float y;
        float z;
} position;

//ROS --------------------------------------------------------------------------
ros::Publisher pub, pub2;
ros::Subscriber sub;

//Callback ---------------------------------------------------------------------
void dataCallback(const geometry_msgs::Vector3Stamped::ConstPtr& data) {
    accIn.x = data->vector.x;
    accIn.y = data->vector.y;
    accIn.z = data->vector.z;
    accIn.t = (data->header.stamp).toSec();
}

//Update Array -----------------------------------------------------------------
void update(int i) {
    accArr[i].x = accIn.x;
    accArr[i].y = accIn.y;
    accArr[i].z = accIn.z;
    accArr[i].t = accIn.t;
}

//Take First Derivative --------------------------------------------------------
void firstIntegral() {
    velArr[0].x = (accArr[1].x - accArr[0].x) / (accArr[1].t - accArr[0].t);
    velArr[0].y = (accArr[1].y - accArr[0].y) / (accArr[1].t - accArr[0].t);
    velArr[0].z = (accArr[1].z - accArr[0].z) / (accArr[1].t - accArr[0].t);
    velArr[0].t = accArr[1].t;

    velArr[1].x = (accArr[2].x - accArr[1].x) / (accArr[2].t - accArr[1].t);
    velArr[1].y = (accArr[2].y - accArr[1].y) / (accArr[2].t - accArr[1].t);
    velArr[1].z = (accArr[2].z - accArr[1].z) / (accArr[2].t - accArr[1].t);
    velArr[1].t = accArr[2].t;
}
//Take Second Derivative -------------------------------------------------------
void secondIntegral() {
    position.x += (velArr[1].x - velArr[0].x) / (velArr[1].t - velArr[0].t);
    position.y += (velArr[1].y - velArr[0].y) / (velArr[1].t - velArr[0].t);
    position.z += (velArr[1].z - velArr[0].z) / (velArr[1].t - velArr[0].t);
}

//Publish Results --------------------------------------------------------------
void publish() {
    //Raw Data : For Debugging
    ROS_INFO("Acceleration from IMU: %lf", accIn.x);
    ROS_INFO("ROS Time: %lf", accIn.t);
    ROS_INFO("Time difference sample: %lf", (accArr[1].t - accArr[0].t));

    //Velocity
    ROS_INFO("Speed in x direction: %lf", velArr[1].x);
    ROS_INFO("Speed in y direction: %lf", velArr[1].y);

    geometry_msgs::TwistStamped intVelocity = geometry_msgs::TwistStamped();
    intVelocity.twist.linear.x = velArr[1].x;
    intVelocity.twist.linear.y = velArr[1].y;
    intVelocity.twist.linear.z = velArr[1].z;
    pub.publish(intVelocity);

    //Position
    ROS_INFO("Position in x direction: %f", position.x);
    ROS_INFO("Position in y direction: %f\n", position.y);

    geometry_msgs::PoseStamped doubleIntPose = geometry_msgs::PoseStamped();
    doubleIntPose.pose.position.x = position.x;
    doubleIntPose.pose.position.y = position.y;
    doubleIntPose.pose.position.z = position.z;
    pub2.publish(doubleIntPose);
}

//Shift Data -------------------------------------------------------------------
void dataShift() {
    for(int j = 0; j < 2; j++) {
        accArr[j].x = accArr[j+1].x;
        accArr[j].y = accArr[j+1].y;
        accArr[j].z = accArr[j+1].z;
        accArr[j].t = accArr[j+1].t;
    }
}

//Main -------------------------------------------------------------------------
int main (int argc, char **argv) {
    ros::init(argc, argv, "double_int_pose");
    ros::NodeHandle node;

    sub = node.subscribe("state_estimation/acc", 1000, &dataCallback);

    pub = node.advertise<geometry_msgs::TwistStamped>("imu/vel", 100);
    pub2 = node.advertise<geometry_msgs::PoseStamped>("imu/pose", 100);

    //Fill First Data Point
    ros::spinOnce();
    update(0);

    //Fill Second Data Point
    ros::spinOnce();
    update(1);

    //Continuous
    while(ros::ok()) {
        //Updating Third Datapoint
        ros::spinOnce();
        update(2);

        //Calculate Integrals
        firstIntegral();
        secondIntegral();

        //Publish Results and shift data
        publish();
        dataShift();
    }

    return 0;
}

/*
 *==============================================================================
 */
