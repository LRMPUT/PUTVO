#include "RGBDprocessing.h"

RGBDclass::RGBDclass()
{
	

}

Eigen::Vector3f RGBDclass::point2Dto3D(cv::Point2f p, float z,
		cv::Mat cameraMatrix, cv::Mat distCoeffs) {
	cv::Mat src(1,1,CV_32FC2), dst(1,1,CV_32FC2);

	src.at < cv::Vec2f > (0, 0)[0] = p.x;
	src.at < cv::Vec2f > (0, 0)[1] = p.y;

	cv::undistortPoints(src, dst, cameraMatrix, distCoeffs);

	float X = dst.at < cv::Vec2f > (0, 0)[0] * z;
	float Y = dst.at < cv::Vec2f > (0, 0)[1] * z;
	return Eigen::Vector3f(X,Y,z);
}

Eigen::Vector3f RGBDclass::simplePoint2Dto3D(cv::Point2f p, float z,
		CalibrationParameters cameraParams) {
	Eigen::Vector3f point3D;

	point3D(0) = (p.x - cameraParams.u0) * z / cameraParams.fu;
	point3D(1) = (p.y - cameraParams.v0) * z / cameraParams.fv;
	point3D(2) = z;

	return point3D;
}

// Building a point cloud
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr RGBDclass::BuildPointCloudFromRGBD(
		cv::Mat rgbImage, cv::Mat dImage, double depthInvScale, CalibrationParameters cameraParameters) {

	float data[5] = { cameraParameters.k1, cameraParameters.k2,
			cameraParameters.p1, cameraParameters.p2, cameraParameters.k3 };
	cameraParameters.distCoeffs = cv::Mat(1, 5, CV_32FC1, &data);
	float cm[3][3] = { { cameraParameters.fu, 0, cameraParameters.u0 }, { 0,
			cameraParameters.fv, cameraParameters.v0 }, { 0, 0, 1 } };
	cameraParameters.cameraMatrix = cv::Mat(3, 3, CV_32FC1, &cm);

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr BuildCloud(
			new pcl::PointCloud<pcl::PointXYZRGBA>);

	for (int i = 0; i < 480; i++) {
		for (int j = 0; j < 640; j++) {
			if (dImage.at<uint16_t>(i, j) != 0) {
				pcl::PointXYZRGBA p;
				Eigen::Vector3f XYZ = point2Dto3D(cv::Point2f(i, j),
						dImage.at<uint16_t>(i, j) / depthInvScale, cameraParameters.cameraMatrix,
						cameraParameters.distCoeffs);

				p.x = XYZ[0];
				p.y = XYZ[1];
				p.z = XYZ[2];

				cv::Vec3b Color = rgbImage.at < cv::Vec3b > (i, j);
				p.r = Color.val[2];
				p.g = Color.val[1];
				p.b = Color.val[0];

				BuildCloud->points.push_back(p);
			}
		}
	}

	BuildCloud->width = BuildCloud->points.size();
	BuildCloud->height = 1;
	BuildCloud->is_dense = false;


	return BuildCloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr RGBDclass::Transform(
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
		Eigen::Matrix4f &transformation) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr tym(new pcl::PointCloud<pcl::PointXYZ>);


	tym->width = cloud->width;
	tym->height = cloud->height;
	tym->is_dense = false;
	tym->points.resize(cloud->width * cloud->height);

	// Transform every point
	for(int j=0;j<cloud->points.size();j++)
	{
		Eigen::Vector4f point(cloud->points[j].x, cloud->points[j].y, cloud->points[j].z, 1.0);

		// Zastosowanie optymalnej transformacji
		point = transformation * point;

		// Przepisanie nowej wartości pointu do chmury
		tym->points[j].x = point[0];
		tym->points[j].y = point[1];
		tym->points[j].z = point[2];
	}
	return tym;
}
void RGBDclass::TransformSelf(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Eigen::Matrix4f &transformation)
{
	// Przekształcenie chmury 1 przez zadany obrót i translacje
	for(int j=0;j<cloud->points.size();j++)
	{
		// Przepisanie pointu chmury i przesunięcie pointu do miejsca, dla którego liczony był 1 obrót
		Eigen::Vector4f point(cloud->points[j].x, cloud->points[j].y,
						cloud->points[j].z, 1.0);

		// Zastosowanie optymalnej transformacji
		point = transformation * point;

		// Przepisanie nowej wartości pointu do chmury
		cloud->points[j].x = point[0];
		cloud->points[j].y = point[1];
		cloud->points[j].z = point[2];
	}
}

void RGBDclass::TransformSelf(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, Eigen::Matrix4f &transformation)
{
	// Przekształcenie chmury 1 przez zadany obrót i translacje
	for(int j=0;j<cloud->points.size();j++)
	{
		// Przepisanie pointu chmury i przesunięcie pointu do miejsca, dla którego liczony był 1 obrót
		Eigen::Vector4f point(cloud->points[j].x, cloud->points[j].y,
				cloud->points[j].z, 1.0);

		// Zastosowanie optymalnej transformacji
		point = transformation * point;

		// Przepisanie nowej wartości pointu do chmury
		cloud->points[j].x = point[0];
		cloud->points[j].y = point[1];
		cloud->points[j].z = point[2];
	}
}




void RGBDclass::LoadRGB(std::string rgbName, cv::Mat &rgbImage)
{	
	rgbImage = cv::imread(rgbName);
}

void RGBDclass::LoadD(std::string dName, cv::Mat &dImage)
{
	dImage = cv::imread(dName, CV_LOAD_IMAGE_ANYDEPTH);
}

void RGBDclass::LoadRGBD(std::string rgbName, std::string dName, cv::Mat &rgbImage,
		cv::Mat &dImage) {

	LoadRGB(rgbName, rgbImage);
	LoadD(dName, dImage);
}

void RGBDclass::LoadCloudXYZ(std::string cloudName,
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
	pcl::io::loadPCDFile < pcl::PointXYZ > (cloudName, *cloud);
}

void RGBDclass::LoadCloudXYZRGBA(std::string cloudName,
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud) {

	pcl::io::loadPCDFile < pcl::PointXYZRGBA > (cloudName, *cloud);
}

///
/// Saving data
///

void RGBDclass::SaveRGBD(std::string name, cv::Mat rgbImage, cv::Mat dImage)
{

}

void RGBDclass::SaveCloudXYZRGBA(std::string cloudName, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud)
{
	pcl::io::savePCDFileASCII(cloudName, *cloud);
}

void RGBDclass::SaveCloudXYZ(std::string cloudName, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	pcl::io::savePCDFileASCII(cloudName, *cloud);
}
