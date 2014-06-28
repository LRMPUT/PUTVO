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


#include <Eigen/StdVector>

struct kabschVertex {
	cv::Mat imageRGB, imageD;
	long int vertexId;
	std::vector< cv::Point2f > keypoints;
	std::vector<int> keypointsId;
	std::vector< Eigen::Vector3f > keypoints3d;
	std::vector< bool > inGraph;

	std::vector< Eigen::Vector3f > matchingKeypoints3d;
	cv::Mat descriptors;
	std::vector< Eigen::Matrix3f, Eigen::aligned_allocator<Eigen::Matrix3f> > informationMatrix;
	Eigen::Matrix4f absolutePosition;
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

	// Time measurements;
	double detectionTime, descriptionTime, trackingTime, matchingTime;
	int measurementCounter;

	double matrixError(Eigen::Matrix4f x, Eigen::Matrix4f y);

private:
	// My rounding
	int roundX(double X);
	int roundY(double Y);

	int computeRANSACIterationCount(double successRate, double inlierRate, int numberOfPairs = 3);

	bool robustTransformationEstimation(
			const std::vector<std::pair<int, int> >& matches, int pair_number,
			const std::vector< Eigen::Vector3f > firstKeypoints3d, double inliner_threshold,
			const std::vector< Eigen::Vector3f > secondKeypoints3d, const Constraints& constraints,
			double ransac_break_percent, Eigen::Matrix4f &OptimalTransformation, bool saveInliers, bool turnOffConstrains, std::vector<bool> &isBestInlier);

	// Focus measure
	double FMmodifiedLaplacian(const cv::Mat& src);

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
	
	// Detection at start
	void newDetection(ProgramParameters programParameters, CalibrationParameters calibrationParams, double depthInvScale, long int frameId);
	void newDetectionMatching(ProgramParameters programParameters, CalibrationParameters calibrationParams, double depthInvScale, long int frameId);


	void createVertex(ProgramParameters programParameters,
			CalibrationParameters calibrationParams, double depthInvScale,
			long int frameId, cv::Mat imageRGB);

	// Detection using new detector
	cv::Point3f normalize_vector(cv::Point3f point);
	cv::Point3f normalize_point(int x, int y);
	std::vector<cv::KeyPoint> detect_rgbd_features(cv::Mat rgb_image,
			cv::Mat depth_image, int fast_threshold, float flatness_threshold, int kickThreshold);

	// Tracking after detection
	void doTracking(ProgramParameters programParameters, double depthInvScale);

	// Show tracked image
	void trackShow();

	// RANSAC
	bool estimateTransformation(kabschVertex *firstFrame, kabschVertex *secondFrame, ProgramParameters programParameters,
			Constraints constraints, CalibrationParameters cameraParameters,
			double depthInvScale, Eigen::Matrix4f &res, bool matching = false, int numberOfFeatures = 500, bool saveInliers = false, bool turnOffConstrains = false);

	// Information matrix
	Eigen::Matrix3f getInformationMatrix(double u, double v, double z);

};

#endif
