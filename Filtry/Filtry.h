#ifndef _FIL
#define _FIL

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>


// AntyNaN
pcl::PointCloud<pcl::PointXYZ>::Ptr AntyNaNFilter(
		pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud);

// VoxelGrid
pcl::PointCloud<pcl::PointXYZ>::Ptr VoxelGridFilter(
		pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud, float LeafSize = 0.01);

// AntySzum
pcl::PointCloud<pcl::PointXYZ>::Ptr AntySzumFilter(
		pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud, float Mean = 50.0,
		float Stddev = 1.0);

// AntyNaN Color Point Cloud
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr AntyNaNFilter(
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr Cloud);

// VoxelGrid Color Point Cloud
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr VoxelGridFilter(
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr Cloud, float LeafSize = 0.01);

#endif
