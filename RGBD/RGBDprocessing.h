#ifndef _RGBD
#define _RGBD

// Ogolne + STL
#include <iostream>
#include <vector>
#include <cstring>

// Podst PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "pcl/io/pcd_io.h"

// OpenCV
#include "opencv/cv.h"
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>


#include "../Includes/includes.h"

class RGBDclass
{
public:
	
	// Konstruktor
	RGBDclass();
	
	// Building a point cloud
	static Eigen::Vector3f point2Dto3D(cv::Point2f p, float z, cv::Mat cameraMatrix, cv::Mat distCoeffs);
	static Eigen::Vector3f simplePoint2Dto3D(cv::Point2f p, float z, CalibrationParameters cameraParams);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr BuildPointCloudFromRGBD(
			cv::Mat rgbImage, cv::Mat dImage, double depthInvScale, CalibrationParameters cameraParameters);

	// Operations on pointCloud
	void TransformSelf(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,
			Eigen::Matrix4f &transformation);
	void TransformSelf(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
			Eigen::Matrix4f &transformation);
	pcl::PointCloud<pcl::PointXYZ>::Ptr Transform(
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
			Eigen::Matrix4f &transformation);

	// Loading
	void LoadRGB(std::string rgbName, cv::Mat &rgbImage);
	void LoadD(std::string dName, cv::Mat &dImage);
	void LoadRGBD(std::string rgbName, std::string dName, cv::Mat &rgbImage,
			cv::Mat &dImage);
	void LoadCloudXYZ(std::string cloudName,
				pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
	void LoadCloudXYZRGBA(std::string cloudName,
			pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud);

	// Saving
	void SaveRGBD(std::string name, cv::Mat rgbImage, cv::Mat dImage);
	void SaveCloudXYZ(std::string cloudName,
				pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
	void SaveCloudXYZRGBA(std::string cloudName,
			pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud);

};

#endif
