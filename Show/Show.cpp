#include "Show.h"

void setViewerPose (pcl::visualization::PCLVisualizer& viewer)
{
	Eigen::Vector3f pos_vector =  Eigen::Vector3f (0, 0, 0);
	Eigen::Vector3f look_at_vector = Eigen::Vector3f (0, 0, 1) + pos_vector;
	Eigen::Vector3f up_vector =  Eigen::Vector3f (0, -1, 0);

	viewer.setCameraPosition(pos_vector[0], pos_vector[1], pos_vector[2],look_at_vector[0],look_at_vector[1],look_at_vector[2],up_vector[0],up_vector[1],up_vector[2]);
	viewer.updateCamera ();
}



void showCloudXYZ(pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloud, std::string name)
{
	pcl::visualization::PCLVisualizer viewer ("3D Viewer");
	viewer.setBackgroundColor (1, 1, 1);

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> point_cloud_color_handler (PointCloud, 255, 0, 0);
	viewer.addPointCloud (PointCloud, point_cloud_color_handler, "original point cloud");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "original point cloud");
		  
	viewer.initCameraParameters ();
		
	setViewerPose(viewer);
	while (!viewer.wasStopped ())
	{
		viewer.spinOnce();
		pcl_sleep(0.01);
	}	
}


void showCloudXYZRGBA(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PointCloud, std::string name)
{
	pcl::visualization::CloudViewer viewer(name.c_str());
	viewer.showCloud (PointCloud);
	while (!viewer.wasStopped ())
	{
	}	
}

void showImage(cv::Mat & img, std::string name)
{
	IplImage ImgShow = img;
	cvNamedWindow(name.c_str(), 1);
	cvShowImage(name.c_str(), &ImgShow);
	cvWaitKey();
}


void show2Clouds(pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloud2 )
{
	// Wizualizacja nalozonych chmur punktow
	pcl::visualization::PCLVisualizer viewer ("3D Viewer");
	viewer.setBackgroundColor (1, 1, 1);

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> point_cloud_color_handler (PointCloud, 255, 0, 0);
	viewer.addPointCloud (PointCloud, point_cloud_color_handler, "original point cloud");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "original point cloud");
		  
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> point_cloud_color_handler2 (PointCloud2, 0, 255, 0);
	viewer.addPointCloud (PointCloud2, point_cloud_color_handler2, "original point cloud2");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "original point cloud2");
	viewer.initCameraParameters ();
		
	setViewerPose(viewer);  
	while (!viewer.wasStopped ())
	{
		viewer.spinOnce ();
		pcl_sleep(0.01);
	}	
}
