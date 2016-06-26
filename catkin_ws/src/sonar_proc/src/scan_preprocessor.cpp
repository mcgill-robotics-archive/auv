#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <math.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include "sonar_proc/scan_preprocessor.h"


ScanPreprocessor::ScanPreprocessor(ros::NodeHandle& nh)
{
  // ROSPARAMS
  ros::param::param<double>("~intensity_threshold", INTENSITY_THRESHOLD_, 90.0);
  ros::param::param<double>("~radius_threshold", RADIUS_THRESHOLD_, 1.5);

  // ROS PUB/SUB
  full_scan_sub_ = nh.subscribe<sensor_msgs::PointCloud>("/full_scan", 10, &ScanPreprocessor::fullScanCallback, this);
  filtered_scan_pub_ = nh.advertise<sensor_msgs::PointCloud>("pcl_filtered", 1);
}

void ScanPreprocessor::fullScanCallback(const sensor_msgs::PointCloud::ConstPtr& msg)
{
  // Create holder for filtered message and run basic filter.
  sensor_msgs::PointCloud cloud_filtered(radiusThresholdFilter(*msg));

  // Filtering example from PCL website

  //Assemble a local 3D grid over a given PointCloud, and downsamples + filters the data.
  //cloud_filtered = voxelGrid(cloud_filtered);

  //Uses point neighborhood statistics to filter outlier data
  //cloud_filtered = statisticalOutlierRemoval(cloud_filtered);
  
  //Filter out points outside a specified range in one dimension.
  //cloud_filtered = passThrough(cloud_filtered);

  //Removes outliers if the number of neighbors in a certain search radius is smaller than a given K
  cloud_filtered = radiusOutlierRemoval(cloud_filtered);

  // Publish the data.
  filtered_scan_pub_.publish(cloud_filtered);
}

sensor_msgs::PointCloud ScanPreprocessor::radiusThresholdFilter(const sensor_msgs::PointCloud& cloud)
{
  // Make holder for a new filtered cloud.
  sensor_msgs::PointCloud* filtered = new sensor_msgs::PointCloud;
  filtered->header = cloud.header;

  // Create a channel object for the intensities.
  sensor_msgs::ChannelFloat32* channel = new sensor_msgs::ChannelFloat32;
  channel->name = "intensity";

  // Create vector holders for points and values to go into the filtered cloud.
  std::vector<float> values;  // contains the data array where all the values of type float are stored
  std::vector<geometry_msgs::Point32> points; // contains the data array where all the points of type geometry_msgs::point32 are stored

  // Iterate through all points in the cloud.
  for (int i = 0; i < cloud.points.size(); ++i)
  {
    // Get the intensity and radius of the given point.
    double radius = sqrt(pow(cloud.points.at(i).x, 2.0) + pow(cloud.points.at(i).y, 2.0));
    double intensity = cloud.channels.at(0).values.at(i);

    // Only add the point to the new cloud if it is under both thresholds.
    if (radius >= RADIUS_THRESHOLD_ && intensity >= INTENSITY_THRESHOLD_)
    {
      points.push_back(cloud.points.at(i));
      values.push_back(cloud.channels.at(0).values.at(i));
    }
  }

  // Populate message.
  channel->values = values;
  filtered->points = points;
  filtered->channels.push_back(*channel);

  return *filtered;
}

// PointCloudLibrary filtering algorithm use PCLPointCloud2
// We convert PointCloud -> PointCloud2 -> PCLPointCloud2 (all from sensor_msgs)
// FOLOWING: Filtering algorithm
//
//
sensor_msgs::PointCloud ScanPreprocessor::voxelGrid(const sensor_msgs::PointCloud& cloud)
{
  
  // Convert to PointCloud2.
  sensor_msgs::PointCloud2 input;
  sensor_msgs::convertPointCloudToPointCloud2(cloud, input);

  // Convert to PCLPointCloud2.
  pcl::PCLPointCloud2* cloud2 = new pcl::PCLPointCloud2;
  pcl_conversions::toPCL(input, *cloud2);
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud2);

  // Create filter output storage.
  pcl::PCLPointCloud2 cloud_filtered;

  // Filter.
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud(cloudPtr);
  sor.setLeafSize(0.3, 0.3, 0.3); //Set the voxel grid leaf size
  sor.filter(cloud_filtered);

  // Convert to ROS data type
  sensor_msgs::PointCloud2 output_pcl;
  pcl_conversions::fromPCL(cloud_filtered, output_pcl);

  // Convert back to PointCloud for backwards compatibility.
  sensor_msgs::PointCloud output;
  sensor_msgs::convertPointCloud2ToPointCloud(output_pcl, output);
  // END CONVERSION

  return output;
}


sensor_msgs::PointCloud ScanPreprocessor::statisticalOutlierRemoval(const sensor_msgs::PointCloud& cloud)
{
  
  // Convert to PointCloud2
  sensor_msgs::PointCloud2 input;
  sensor_msgs::convertPointCloudToPointCloud2(cloud, input);	

  // Convert to PCLPointCloud2
  pcl::PCLPointCloud2* cloud2 = new pcl::PCLPointCloud2;
  pcl_conversions::toPCL(input, *cloud2);
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud2);

  // Create filter output storage.
  pcl::PCLPointCloud2 cloud_filtered;

  // Create the filtering object
  pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2> sor;
  sor.setInputCloud(cloudPtr);
  sor.setMeanK(60.0); //Set the number of points (k) to use for mean distance estimation. 
  sor.setStddevMulThresh(1.0); //Set the standard deviation multiplier threshold.
  sor.filter(cloud_filtered);

  // Convert to ROS data type
  sensor_msgs::PointCloud2 output_pcl;
  pcl_conversions::fromPCL(cloud_filtered, output_pcl);

  // Convert back to PointCloud for backwards compatibility.
  sensor_msgs::PointCloud output;
  sensor_msgs::convertPointCloud2ToPointCloud(output_pcl, output);
  // END CONVERSION

  return output;
}


sensor_msgs::PointCloud ScanPreprocessor::passThrough(const sensor_msgs::PointCloud& cloud)
{
  // Convert to PointCloud2
  sensor_msgs::PointCloud2 input;
  sensor_msgs::convertPointCloudToPointCloud2(cloud, input);

  // Convert to PCLPointCloud2
  pcl::PCLPointCloud2* cloud2 = new pcl::PCLPointCloud2;
  pcl_conversions::toPCL(input, *cloud2);
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud2);

  // Create filter output storage
  pcl::PCLPointCloud2 cloud_filtered;
  
  // Create the filtering object
  pcl::PassThrough<pcl::PCLPointCloud2> sor;
  sor.setInputCloud(cloudPtr);
  sor.setFilterLimits(0.0, 5.0); //(min, max) limits for the field for filtering data
  sor.filter(cloud_filtered);

  // Convert to ROS data type
  sensor_msgs::PointCloud2 output_pcl;
  pcl_conversions::fromPCL(cloud_filtered, output_pcl);

  // Convert back to PointCloud for backwards compatibility.
  sensor_msgs::PointCloud output;
  sensor_msgs::convertPointCloud2ToPointCloud(output_pcl, output);
  // END CONVERSION

  return output;
}


sensor_msgs::PointCloud ScanPreprocessor::radiusOutlierRemoval(const sensor_msgs::PointCloud& cloud)
{

  // Convert to PointCloud2
  sensor_msgs::PointCloud2 input;
  sensor_msgs::convertPointCloudToPointCloud2(cloud, input);

  // Convert to PCLPointCloud2
  pcl::PCLPointCloud2* cloud2 = new pcl::PCLPointCloud2;
  pcl_conversions::toPCL(input, *cloud2);
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud2);

  // Create flter output storage
  pcl::PCLPointCloud2 cloud_filtered;

  // Create the filtering object  
  pcl::RadiusOutlierRemoval<pcl::PCLPointCloud2> sor;
  sor.setInputCloud(cloudPtr);
  sor.setRadiusSearch(0.5); //Sphere radius that is to be used for determining the k-nearest neighbors for filtering
  sor.setMinNeighborsInRadius (40.0); //Set the minimum number of neighbors that a point needs to have in the given search radius in order to be considered an inlier
  // apply filter
  sor.filter(cloud_filtered);

  // Convert to ROS data type
  sensor_msgs::PointCloud2 output_pcl;
  pcl_conversions::fromPCL(cloud_filtered, output_pcl);

  // Convert back to PointCloud for backwards compatibility.
  sensor_msgs::PointCloud output;
  sensor_msgs::convertPointCloud2ToPointCloud(output_pcl, output);
 
  // END CONVERSION

  return output;

}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "scan_preprocessor");
    ros::NodeHandle nh("filtered_scan");
    ScanPreprocessor process(nh);

    ros::spin();
    return 0;
}
