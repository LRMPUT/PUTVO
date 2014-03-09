#ifndef _SHO
#define _SHO

// C++
#include <cstring>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "pcl/visualization/pcl_visualizer.h"
#include <pcl/visualization/cloud_viewer.h>

// OpenCV
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

// initial setup of camera
void setViewerPose (pcl::visualization::PCLVisualizer& viewer);

// Show Cloud XYZ
void showCloudXYZ(pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloud, std::string name =
		"Point cloud without color");

// Show Cloud XYZRGBA
void showCloudXYZRGBA(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PointCloud,
		std::string name = "Chmura punktow RGB");

// Show Image
void showImage(cv::Mat & img, std::string name = "Image");


// Show 2 clouds with different colors
void show2Clouds(pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloud,
		pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloud2);

#endif
