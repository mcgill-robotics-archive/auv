
#include "sonar_proc/scan_preprocessor.h"
#include "boost/date_time/gregorian/gregorian.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sonar_node");
    ros::NodeHandle nh("sonar_node");
    // ScanPreprocessor process(nh);

    std::string s("2001-10-9"); //2001-October-09
    boost::gregorian::date d(boost::gregorian::from_simple_string(s));

    ros::spin();
    return 0;
}
