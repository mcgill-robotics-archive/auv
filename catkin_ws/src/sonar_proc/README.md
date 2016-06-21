# Sonar Proc Package

This package processes sonar data in order to determine objects in the viscinity 
of the Robot. But also select the most meaningful object: the one that represents
a task.

The (unimplemented) goal is to do this in Four stages:

1. Scan Stitcher: Stitches PointCloud slices together and publishes to /full_scan 
2. Scan prepocessor : Filters the /full_scan sonar PointCloud and published to /filtered_scan
3. Meanshift Clustering : Cluster the filtered PointCloud data and publishes to /visualization_marker_array
4. Select the meaningful cluster

## Utilities

Available utilities include:

* PointCloud to LaserScan: Converts to LaserScan (pretty lossy)


## Running Meanshift clustering

1. roscore
2. rosbag play <path to your rosbag> --loop 
3. rosrun sonar_proc scan_stitcher.py
4. rosrun sonar_proc scan_preprocessor_fourth.py
5. rosrun sonar_proc meanshiftfifth.py
6. rosrun rviz rviz

## Important

1. meanshiftfifth.py uses class Clustering which can be found within Clustering.py. 
   Therefore, both meanshiftfifth.py and Clustreing.py must remain in the same location.

