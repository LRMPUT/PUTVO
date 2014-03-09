#ifndef _TRA
#define _TRA

#include "../Includes/includes.h"
#include "../RGBD/RGBDprocessing.h"
#include "../DBscan/dbscan.h"
#include "../TransformationEstimation/TransformationEstimation.h"

// Ogolne + STL
#include <math.h>
#include <cstring>
#include <fstream>
#include <iostream>
#include <vector>
#include <queue>

// Podst PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "pcl/io/pcd_io.h"

#include "../RGBD/RGBDprocessing.h"

struct kabschVertex {
	int vertexId;
	std::vector< cv::Point2f > keypoints;
	std::vector<int> keypointsId;
	std::vector< Eigen::Vector3f > keypoints3d;
	std::vector< bool > inGraph;
};

class Track
{
public:
	// Zmienne Mat do odbioru danych Depth
	cv::Mat frameD[3];
	
	// Zmienne Mat do odbioru danych RGB
	cv::Mat frameBGR[3];
	
	//
	bool performedTrack;



private:
	// My rounding
	int roundX(double X);
	int roundY(double Y);



public:
	Track();

	// Data for bundle adjustment
	std::vector< kabschVertex > vertexHistory;

	// Tracked points:
	// 0 - begin
	// 1 - last
	// 2 - most up-to-date
	std::vector<cv::Point2f> points[3];
	std::vector<int> pointsID;
	std::vector<int> pointDepth;

	int featureCounter;
	int vertexCounter;

	std::vector<bool> wasInlierSecond, wasInlierFirst;

	// FM
	double FMmodifiedLaplacian(const cv::Mat& src);

	// Clare before next tracking
	void clearTrack();
	
	// Detection at start
	void newDetection(ProgramParameters programParameters, CalibrationParameters calibrationParams, double depthInvScale, int frameId);


	void createVertex(ProgramParameters programParameters, CalibrationParameters calibrationParams, double depthInvScale, int frameId);

	// Detection using new detector
	cv::Point3f normalize_vector(cv::Point3f point);
	cv::Point3f normalize_point(int x, int y);
	std::vector<cv::KeyPoint> detect_rgbd_features(cv::Mat rgb_image,
			cv::Mat depth_image, int fast_threshold, float flatness_threshold, int kickThreshold);

	// Tracking after detection
	void doTracking(ProgramParameters programParameters, double depthInvScale);

	// Show tracked image
	void trackShow();

	// RANSAC and Kabsch
	Eigen::Matrix4f estimateTransformation(kabschVertex firstFrame, kabschVertex secondFrame, int pair_number,
			Constraints constraints, CalibrationParameters cameraParameters,
			double inliner_threshold, double ransac_break_percent,
			double depthInvScale, Eigen::Matrix4f &res, std::ofstream &g2o);

};

#endif
