# Sonar Processing

This package processes sonar data. The (unimplemented) goal is to do this in
two stages:

1. Cluster the PointCloud data,
2. Analyze clusters to recognize objects.

## Utilities

Available utilities include:

* PointCloud to LaserScan: Converts to LaserScan (pretty lossy)
* Scan stitcher: Stitches PointCloud slices together and publishes full scans
