
#include "sonar_proc/scan_preprocessor.h"
#include "boost/date_time/gregorian/gregorian.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sonar_node");
    ros::NodeHandle nh("sonar_node");
    ScanPreprocessor process(nh);

    ros::spin();
    return 0;
}
