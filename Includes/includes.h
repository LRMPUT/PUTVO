#ifndef _INC
#define _INC

#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

#include "opencv/cv.h"
#include <Eigen/Eigen>

struct Constraints
{
	double minObrX, maxObrX, minObrY, maxObrY, minObrZ, maxObrZ, minTx, maxTx, minTy, maxTy, minTz, maxTz;
};

struct ProgramParameters
{
	//
	// Tracking parameters
	//

	// Tracking length
	int TrackingLength;

	// Tracking feature number
	int TrackingFeatureNumber;

	// Tracking Step Size
	int TrackingStepSize;

	// Tracking features threshold
	int TrackingFeaturesThreshold;

	// Tracking winSize
	int winSize;

	// Tracking max levels
	int maxLevels;

	//
	// Detection
	//

	// Detection in stripes
	int detectionInStripes;

	// New detector on/off
	int newDetector;

	// New detector kickThreshold
	int newDetectorKickThreshold;

	// New detector fast threshold
	int newDetectorFastThreshold;

	// New detector depth test threshold
	float newDetectorDepthTestThreshold;

	//
	//	DBScan
	//
	// DBScan on/off
	int DBScan;

	// Minimal number of points to form a cluster
	int DBScanMinPts;
		
	// DBScan eps		
	double DBScanEps;
	
	//
	// Transformation estimation
	//

	// Number of used pairs compute transformation
	int transformationPairNumber;

	// Threshold - when acknowledged as inlier
	double inlierThreshold;

	// Minimal ratio of inliers to treat transformation as the best
	int wantedInlierRatio;
	
	// Wanted success rate of RANSAC
	double RANSACsccessrate;
	
	//
	// Translation/Rotation constraints
	//

	Constraints constrain;
	
	//
	// 	Filters
	//

	// LeafSize in VoxelGrid
	double LeafSize;

	//
	//
	//

	// Visualization on/off
	int visualization;

	// Showing tracking
	int showTracking;

	// historySize
	int historySize;

	// g2o features
	int g2oWithFeatures;

};

struct CalibrationParameters
{
	double fu, fv, u0, v0;
	double k1, k2, p1, p2, k3, k4, k5, k6;
	cv::Mat cameraMatrix;
	cv::Mat distCoeffs;
};


// calculate transform from Euler
void calculateTransformation(Eigen::Matrix4f &transformation,float alpha, float beta, float gamma, float tx, float ty, float tz);

// Transform to euler
void TransformationToEuler(Eigen::Matrix4f transformation, float *Orient);

#endif

