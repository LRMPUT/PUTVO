#include "kinect.h"
#include <pcl/common/time.h>
#include <float.h>

bool geometricDistanceCompare(geometricDistance i, geometricDistance j) {
	return i.distance < j.distance;
}

Kinect::Kinect() {
	this->ReadParameters();
	this->ReadCalibrationParameters();
	this->ReadInitialTransformation();
	zapis.open("Kinect.log");
}

Kinect::~Kinect() {
	zapis.close();
}

void Kinect::ReadParameters() {
	std::string linia;
	ifstream o;
	o.open("../parameters.cfg");

	while (!o.eof()) {
		getline(o, linia);

		if (linia == "# Verbose") {
			o >> linia;
			programParameters.verbose = atoi(linia.c_str());
		}
		else if (linia == "# Tracking (0), Matching (1)") {
			o >> linia;
			programParameters.trackingMatching = atoi(linia.c_str());
		}
		else if (linia == "# Matching Loop Closure") {
			o >> linia;
			programParameters.matchingLoopClosure = atoi(linia.c_str());
		}
		else if (linia
				== "# Matching Loop Closure window from i-x (define x)") {
			o >> linia;
			programParameters.matchingLoopClosureWindowFrom = atoi(linia.c_str());
		} else if (linia
				== "# Matching Loop Closure window to i-y (define y, y<x)") {
			o >> linia;
			programParameters.matchingLoopClosureWindowTo = atoi(linia.c_str());
		} else if (linia == "# Matching type (0 - FAST/BRIEF | 1 - SURF/SURF | 2 - ORB)")
		{
			o >> linia;
			programParameters.matchingType = atoi(linia.c_str());
		} else if (linia == "# Temporally saved history size") {
			o >> linia;
			programParameters.historySize = atoi(linia.c_str());
		} else if (linia == "# Bundle Adjustment") {
			o >> linia;
			programParameters.bundleAdjustment = atoi(linia.c_str());
		} else if (linia == "# Bundle Adjustment window size") {
			o >> linia;
			programParameters.bundleAdjustmentWindowSize = atoi(linia.c_str());
		}

		//
		// Tracking parameters
		//
		else if (linia == "# Number of tracked frames") {
			o >> linia;
			programParameters.TrackingLength = atoi(linia.c_str());
		} else if (linia == "# Tracking feature number") {
			o >> linia;
			programParameters.TrackingFeatureNumber = atoi(linia.c_str());
		} else if (linia == "# Maximal tracking step") {
			o >> linia;
			programParameters.TrackingStepSize = atoi(linia.c_str());
		} else if (linia == "# Tracking features threshold") {
			o >> linia;
			programParameters.TrackingFeaturesThreshold = atoi(linia.c_str());
		} else if (linia == "# Tracking winSize") {
			o >> linia;
			programParameters.winSize = atoi(linia.c_str());
		} else if (linia == "# Tracking max levels") {
			o >> linia;
			programParameters.maxLevels = atoi(linia.c_str());
		}
		//
		// Detection
		//
		else if (linia == "# Detection in stripes") {
			o >> linia;
			programParameters.detectionInStripes = atoi(linia.c_str());
		} else if (linia == "# New detector on/off") {
			o >> linia;
			programParameters.newDetector = atoi(linia.c_str());
		} else if (linia == "# New detector kickThreshold") {
			o >> linia;
			programParameters.newDetectorKickThreshold = atoi(linia.c_str());
		} else if (linia == "# New detector fast threshold") {
			o >> linia;
			programParameters.newDetectorFastThreshold = atoi(linia.c_str());
		} else if (linia == "# New detector depth test threshold") {
			o >> linia;
			programParameters.newDetectorDepthTestThreshold = atof(
					linia.c_str());
		}
		//
		// DB Scan
		//
		else if (linia == "# DBScan on/off") {
			o >> linia;
			programParameters.DBScan = atoi(linia.c_str());
		} else if (linia == "# DBScan MinPts") {
			o >> linia;
			programParameters.DBScanMinPts = atoi(linia.c_str());
		} else if (linia == "# DBScan eps") {
			o >> linia;
			programParameters.DBScanEps = atoi(linia.c_str());
		}
		//
		// Quadtree
		//
		else if (linia == "# Quadtree on/off") {
			o >> linia;
			programParameters.Quadtree = atoi(linia.c_str());
		}
		//
		// Transformation estimation
		//
		else if (linia == "# Number of pairs used for estimation") {
			o >> linia;
			programParameters.transformationPairNumber = atoi(linia.c_str());
		} else if (linia == "# Inlier threshold") {
			o >> linia;
			programParameters.inlierThreshold = atof(linia.c_str());
		} else if (linia
				== "# Wanted percentage of correct matches in RANSAC") {
			o >> linia;
			programParameters.wantedInlierRatio = atof(linia.c_str());
		}

		else if (linia == "# RANSAC success rate") {
			o >> linia;
			programParameters.RANSACsccessrate = atof(linia.c_str());
		}
		//
		// Translation/Rotation constraints
		//
		else if (linia == "# min rotation X") {
			o >> linia;
			programParameters.constrain.minObrX = atof(linia.c_str());
		} else if (linia == "# max rotation X") {
			o >> linia;
			programParameters.constrain.maxObrX = atof(linia.c_str());
		} else if (linia == "# min rotation Y") {
			o >> linia;
			programParameters.constrain.minObrY = atof(linia.c_str());
		} else if (linia == "# max rotation Y") {
			o >> linia;
			programParameters.constrain.maxObrY = atof(linia.c_str());
		} else if (linia == "# min rotation Z") {
			o >> linia;
			programParameters.constrain.minObrZ = atof(linia.c_str());
		} else if (linia == "# max rotation Z") {
			o >> linia;
			programParameters.constrain.maxObrZ = atof(linia.c_str());
		} else if (linia == "# min translation X") {
			o >> linia;
			programParameters.constrain.minTx = atof(linia.c_str());
		} else if (linia == "# max translation X") {
			o >> linia;
			programParameters.constrain.maxTx = atof(linia.c_str());
		} else if (linia == "# min translation Y") {
			o >> linia;
			programParameters.constrain.minTy = atof(linia.c_str());
		} else if (linia == "# max translation Y") {
			o >> linia;
			programParameters.constrain.maxTy = atof(linia.c_str());
		} else if (linia == "# min translation Z") {
			o >> linia;
			programParameters.constrain.minTz = atof(linia.c_str());
		} else if (linia == "# max translation Z") {
			o >> linia;
			programParameters.constrain.maxTz = atof(linia.c_str());
		}
		//
		// 	Filters
		//
		else if (linia == "# VoxelGrid (Leaf size)") {
			o >> linia;
			programParameters.LeafSize = atof(linia.c_str());
		}
		//
		// Running parameters
		//
		else if (linia == "# Visualize a sum point cloud") {
			o >> linia;
			programParameters.visualization = atoi(linia.c_str());
		} else if (linia == "# Showing tracking") {
			o >> linia;
			programParameters.showTracking = atoi(linia.c_str());
		} else if (linia == "# Transformation Uncertainty") {
			o >> linia;
			programParameters.transformationUncertainty = atoi(linia.c_str());
		} else if (linia == "# g2o with features") {
			o >> linia;
			programParameters.g2oWithFeatures = atoi(linia.c_str());
		} else if (linia == "# g2o with TRACKXYZ") {
			o >> linia;
			programParameters.g2oWithTRACKXYZ = atoi(linia.c_str());
		} else if (linia == "# g2o vs sba save file") {
			o >> linia;
			programParameters.g2o_vs_sba = atoi(linia.c_str());
		}
		else if (linia.find("###") == -1 && linia.size() > 1)
		{
			std::cout<<linia<<std::endl;
			std::cout<< linia.size() <<std::endl;
			std::cout<<"Error reading parameters!" << std::endl;
			exit(0);
		}

	}

	o.close();
}

void Kinect::ReadCalibrationParameters() {
	std::string linia;
	ifstream o;
	o.open("../camera.cfg");

	while (!o.eof()) {
		getline(o, linia);

		if (linia == "# fu") {
			o >> linia;
			calibrationParams.fu = atof(linia.c_str());
		} else if (linia == "# fv") {
			o >> linia;
			calibrationParams.fv = atof(linia.c_str());
		} else if (linia == "# u0") {
			o >> linia;
			calibrationParams.u0 = atof(linia.c_str());
		} else if (linia == "# v0") {
			o >> linia;
			calibrationParams.v0 = atof(linia.c_str());
		} else if (linia == "# k1") {
			o >> linia;
			calibrationParams.k1 = atof(linia.c_str());
			;
		} else if (linia == "# k2") {
			o >> linia;
			calibrationParams.k2 = atof(linia.c_str());
		} else if (linia == "# p1") {
			o >> linia;
			calibrationParams.p1 = atof(linia.c_str());
		} else if (linia == "# p2") {
			o >> linia;
			calibrationParams.p2 = atof(linia.c_str());
		} else if (linia == "# k3") {
			o >> linia;
			calibrationParams.k3 = atof(linia.c_str());
		}

		else if (linia == "# k4") {
			o >> linia;
			programParameters.RANSACsccessrate = atof(linia.c_str());
			calibrationParams.k4 = atof(linia.c_str());
		} else if (linia == "# k5") {
			o >> linia;
			calibrationParams.k5 = atof(linia.c_str());
		} else if (linia == "# k6") {
			o >> linia;
			calibrationParams.k6 = atof(linia.c_str());
		}

	}
	o.close();

	// Calibration matrices
	/*	float data[5] = { calibrationParams.k1, calibrationParams.k2,
	 calibrationParams.p1, calibrationParams.p2, calibrationParams.k3 };
	 calibrationParams.distCoeffs = cv::Mat(1, 5, CV_32FC1, &data);
	 float cm[3][3] = { { calibrationParams.fu, 0, calibrationParams.u0 }, { 0,
	 calibrationParams.fv, calibrationParams.v0 }, { 0, 0, 1 } };
	 calibrationParams.cameraMatrix = cv::Mat(3, 3, CV_32FC1, &cm);
	 */
	/*std::cout << calibrationParams.fu << " " << calibrationParams.fv << " "
	 << calibrationParams.u0 << " " << calibrationParams.v0 << " "
	 << calibrationParams.k1 << " " << calibrationParams.k2 << " "
	 << calibrationParams.p1 << " " << calibrationParams.p2 << " "
	 << calibrationParams.k3 << std::endl;
	 std::cout<<calibrationParams.cameraMatrix<<std::endl;*/
}

void Kinect::ReadInitialTransformation() {
	ifstream o;
	o.open("initialPosition");

	double x, y, z, qx, qy, qz, qw;
	o >> x >> y >> z >> qx >> qy >> qz >> qw;

	Eigen::Quaternion<float> quat(qw, qx, qy, qz);
	KinectInitial.block<3, 3>(0, 0) = quat.toRotationMatrix();
	KinectInitial(0, 3) = x;
	KinectInitial(1, 3) = y;
	KinectInitial(2, 3) = z;
	KinectInitial(3, 3) = 1.0;
}

void Kinect::saveTrajectory(Eigen::Matrix4f transformation,
		std::ofstream & estTrajectory, const std::string& timestamp) {
	// Saving estimate in Freiburg format
	Eigen::Quaternion<float> Q(transformation.block<3, 3>(0, 0));
	estTrajectory << timestamp << " " << transformation(0, 3) << " "
			<< transformation(1, 3) << " " << transformation(2, 3) << " "
			<< Q.coeffs().x() << " " << Q.coeffs().y() << " " << Q.coeffs().z()
			<< " " << Q.coeffs().w() << endl;
}

void Kinect::saveG20Vertex(Eigen::Matrix4f transformation, std::ofstream& g2o,
		const kabschVertex& tmpVertex) {

	saveTrajectory(transformation, g2o,
			"VERTEX_SE3:QUAT " + std::to_string(tmpVertex.vertexId));
}

void Kinect::saveG20Vertex(Eigen::Matrix4f transformation, std::ofstream& g2o,
		const int id) {

	saveTrajectory(transformation, g2o,
			"VERTEX_SE3:QUAT " + std::to_string(id));
}

void Kinect::saveG20Edge(std::ofstream& g2o, const kabschVertex& tmpVertex,
		int i) {
	g2o << "EDGE_SE3:QUAT " << tmpVertex.vertexId << " "
			<< tmpVertex.keypointsId[i] << " " << tmpVertex.keypoints3d[i][0]
			<< " " << tmpVertex.keypoints3d[i][1] << " "
			<< tmpVertex.keypoints3d[i][2] << " 0 0 0 1 "
			<< tmpVertex.informationMatrix[i](0, 0) << " "
			<< tmpVertex.informationMatrix[i](0, 1) << " "
			<< tmpVertex.informationMatrix[i](0, 2) << " 0 0 0 "
			<< tmpVertex.informationMatrix[i](1, 1) << " "
			<< tmpVertex.informationMatrix[i](1, 2) << " 0 0 0 "
			<< tmpVertex.informationMatrix[i](2, 2)
			<< " 0 0 0 0.00000001 0 0 0.00000001 0 0.00000001" << std::endl;
}

void Kinect::saveG20Edge(std::ofstream& g2o, const int vertex1Id,
		const int vertex2Id, Eigen::Matrix4f transformation) {
	Eigen::Quaternion<float> Q(transformation.block<3, 3>(0, 0));

	if (std::isfinite(transformation(0, 3))
			&& std::isfinite(transformation(1, 3))
			&& std::isfinite(transformation(2, 3))
			&& std::isfinite(transformation(3, 3))
			&& std::isfinite(Q.coeffs().x()) && std::isfinite(Q.coeffs().y())
			&& std::isfinite(Q.coeffs().z()) && std::isfinite(Q.coeffs().w())
			&& transformation(0, 3) < 100 && transformation(1, 3) < 100
			&& transformation(2, 3) < 100) {
		g2o << "EDGE_SE3:QUAT " << vertex1Id << " " << vertex2Id << " "
				<< transformation(0, 3) << " " << transformation(1, 3) << " "
				<< transformation(2, 3) << " " << Q.coeffs().x() << " "
				<< Q.coeffs().y() << " " << Q.coeffs().z() << " "
				<< Q.coeffs().w()
				<< " 1 0 0 0 0 0 1 0 0 0 0 1 0 0 0 1 0 0 1 0 1" << endl;
	}
}

void Kinect::saveG20Edge(std::ofstream& g2o, const int vertex1Id,
		const int vertex2Id, Eigen::Matrix4f transformation, Eigen::Matrix<double, 6, 6> transInfMatrix) {
	Eigen::Quaternion<float> Q(transformation.block<3, 3>(0, 0));

	if (std::isfinite(transformation(0, 3))
			&& std::isfinite(transformation(1, 3))
			&& std::isfinite(transformation(2, 3))
			&& std::isfinite(transformation(3, 3))
			&& std::isfinite(Q.coeffs().x()) && std::isfinite(Q.coeffs().y())
			&& std::isfinite(Q.coeffs().z()) && std::isfinite(Q.coeffs().w())
			&& transformation(0, 3) < 100 && transformation(1, 3) < 100
			&& transformation(2, 3) < 100) {
		g2o << "EDGE_SE3:QUAT " << vertex1Id << " " << vertex2Id << " "
				<< transformation(0, 3) << " " << transformation(1, 3) << " "
				<< transformation(2, 3) << " " << Q.coeffs().x() << " "
				<< Q.coeffs().y() << " " << Q.coeffs().z() << " "
				<< Q.coeffs().w() << " "
				<< transInfMatrix(0, 0) << " "
				<< transInfMatrix(0, 1) << " "
				<< transInfMatrix(0, 2) << " "
				<< transInfMatrix(0, 3) << " "
				<< transInfMatrix(0, 4) << " "
				<< transInfMatrix(0, 5) << " "
				<< transInfMatrix(1, 1) << " "
				<< transInfMatrix(1, 2) << " "
				<< transInfMatrix(1, 3) << " "
				<< transInfMatrix(1, 4) << " "
				<< transInfMatrix(1, 5) << " "
				<< transInfMatrix(2, 2) << " "
				<< transInfMatrix(2, 3) << " "
				<< transInfMatrix(2, 4) << " "
				<< transInfMatrix(2, 5) << " "
				<< transInfMatrix(3, 3) << " "
				<< transInfMatrix(3, 4) << " "
				<< transInfMatrix(3, 5) << " "
				<< transInfMatrix(4, 4) << " "
				<< transInfMatrix(4, 5) << " "
				<< transInfMatrix(5, 5) << endl;
	}
}


void Kinect::saveG20EdgeTRACKXYZ(std::ofstream& g2o, const kabschVertex& tmpVertex,
		int i) {
	g2o << "EDGE_SE3_TRACKXYZ " << tmpVertex.vertexId << " "
			<< tmpVertex.keypointsId[i] << " 0 " << tmpVertex.keypoints3d[i][0]
			<< " " << tmpVertex.keypoints3d[i][1] << " "
			<< tmpVertex.keypoints3d[i][2] << " "
			<< tmpVertex.informationMatrix[i](0, 0) << " "
			<< tmpVertex.informationMatrix[i](0, 1) << " "
			<< tmpVertex.informationMatrix[i](0, 2) << " "
			<< tmpVertex.informationMatrix[i](1, 1) << " "
			<< tmpVertex.informationMatrix[i](1, 2) << " "
			<< tmpVertex.informationMatrix[i](2, 2) << std::endl;
}

void Kinect::saveG2OnonExistingFeatures(std::ofstream& g2o, const int vertex1Id,
		const int vertex2Id, int nonExistingFeatureCounter) {

	for (int i=0;i<2;i++)
	{
		for (int j=0;j<2;j++)
		{
			for (int k = 0; k < 2; k++) {
				g2o << "EDGE_SE3_TRACKXYZ " << vertex1Id << " "
						<< nonExistingFeatureCounter + i * 4 + j * 2 + k
						<< " 0 " << -1.0 + i * 2.0 << " " << -1.0 + j * 2.0 << " " << -1.0 + k * 2.0 << " "
						<< FLT_MAX << " 0 0 " << FLT_MAX << " 0 " << FLT_MAX
						<< std::endl;
				g2o << "EDGE_SE3_TRACKXYZ " << vertex2Id << " "
										<< nonExistingFeatureCounter + i * 4 + j * 2 + k
										<< " 0 " << -1.0 + i * 2.0 << " " << -1.0 + j * 2.0 << " " << -1.0 + k * 2.0 << " "
										<< FLT_MAX << " 0 0 " << FLT_MAX << " 0 " << FLT_MAX
										<< std::endl;
			}
		}
	}


}

void Kinect::saveG2OcameraPos(std::ofstream& g2o) {
	g2o << "PARAMS_SE3OFFSET 0 0 0 0 0 0 0 1 " << endl;
}

void Kinect::saveG20Fix(std::ofstream& g2o, const int id) {
	g2o << "FIX " << id << endl;
}

void Kinect::findAndSaveG20Features(kabschVertex& frame, std::ofstream& g2o,
		const int firstOrSecond, const int g2oWithTRACKXYZ) {


	if (firstOrSecond == 1) {
		for (int i = 0; i < tracking->wasInlierFirst.size(); i++) {
			if (tracking->wasInlierFirst[i] == true
					&& frame.inGraph[i] == false) {
				frame.inGraph[i] = true;

				if (g2oWithTRACKXYZ > 0)
					saveG20EdgeTRACKXYZ(g2o, frame, i);
				else
					saveG20Edge(g2o, frame, i);
			}
		}
		tracking->wasInlierFirst.clear();
	} else if (firstOrSecond == 2) {
		for (int i = 0; i < tracking->wasInlierSecond.size(); i++) {
			if (tracking->wasInlierSecond[i] == true
					&& frame.inGraph[i] == false) {
				frame.inGraph[i] = true;
				if (g2oWithTRACKXYZ > 0)
					saveG20EdgeTRACKXYZ(g2o, frame, i);
				else
					saveG20Edge(g2o, frame, i);
			}
		}
		// To jest kwestia BA -> z aktualnej klatki zapisujemy inlier z
		// jakiejkolwiek transformacji, wiec nie zerujemy licznika, chociaz powinnismy
		tracking->wasInlierFirst.clear();
		//tracking->wasInlierSecond.clear();
	}
}


void Kinect::saveG2OVertexPosition(Eigen::Matrix4f transformation,
		std::ofstream & estTrajectory, const int vertexId) {
	// Saving estimate in Freiburg format

	//VERTEX_SE3:QUAT 1 1.32118 0.840088 1.5281 0.891676 0.211772 -0.0878279 -0.390324
	Eigen::Quaternion<float> Q(transformation.block<3, 3>(0, 0));
	estTrajectory << "VERTEX_SE3:QUAT " << vertexId << " " << transformation(0, 3) << " "
			<< transformation(1, 3) << " " << transformation(2, 3) << " "
			<< Q.coeffs().x() << " " << Q.coeffs().y() << " " << Q.coeffs().z()
			<< " " << Q.coeffs().w() << endl;
}
void Kinect::TrackingRun() {

	// Locally used variables
	char RGBname[30], Dname[30];
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr partial_cloud[2500];
	Eigen::Matrix4f transformation[2500];
	char relativeTransformationName[30];
	int j = 0, saved_number = 1;
	transformation[0] = KinectInitial;
	cv::Mat rgbImg[3], dImg[3];
	std::ifstream gtNumberStream;
	std::string timestamp;
	std::ofstream estTrajectory, g2o, g2oIndices;

	gtNumberStream.open("matchedIndices");
	estTrajectory.open("../results/result");
	g2o.open("../results/graphFile.g2o");
	g2oIndices.open("../results/g2oIndices");

	// Using TRACKXYZ we need to set up the camera position
	if (programParameters.g2oWithTRACKXYZ > 0)
	{
		saveG2OcameraPos(g2o);
	}

	// Creating the core
	tracking = new Track();

	double startTime = pcl::getTime();

	// Reading 1st image
	rgbImg[0] = cv::Mat();
	RGBD->LoadRGBD("rgb_00001.png", "depth_00001.png", rgbImg[0], dImg[0]);
	tracking->frameBGR[0] = rgbImg[0];
	tracking->frameD[0] = dImg[0];

	// Visualization
	if (programParameters.visualization == 1) {
		// Saving point clouds
		sprintf(relativeTransformationName, "1.pcd");
		partial_cloud[0] = RGBD->BuildPointCloudFromRGBD(tracking->frameBGR[0],
				tracking->frameD[0], depthInvScale, calibrationParams);
		RGBD->SaveCloudXYZRGBA(relativeTransformationName, partial_cloud[0]);
	}

	// First position in gt
	getline(gtNumberStream, timestamp);
	timestamp = timestamp.substr(0, timestamp.find("\t"));

	// Detection on the first set of images
	float data[5] = { calibrationParams.k1, calibrationParams.k2,
			calibrationParams.p1, calibrationParams.p2, calibrationParams.k3 };
	calibrationParams.distCoeffs = cv::Mat(1, 5, CV_32FC1, &data);
	float cm[3][3] = { { calibrationParams.fu, 0, calibrationParams.u0 }, { 0,
			calibrationParams.fv, calibrationParams.v0 }, { 0, 0, 1 } };
	calibrationParams.cameraMatrix = cv::Mat(3, 3, CV_32FC1, &cm);

	if (programParameters.trackingMatching == 0) {
		tracking->newDetection(programParameters, calibrationParams,
				depthInvScale, 0);
	} else {
		tracking->newDetectionMatching(programParameters, calibrationParams,
				depthInvScale, 0);
	}
	// Saving 1st estimate
	saveTrajectory(transformation[0], estTrajectory, timestamp);

	// Saving to .g2o
	g2oIndices << timestamp << std::endl;
	kabschVertex &tmpVertex = tracking->vertexHistory[0];
	tmpVertex.absolutePosition = transformation[0];

	saveG20Vertex(transformation[0], g2o, tmpVertex);
	saveG20Fix(g2o, tmpVertex.vertexId);

	std::cout<<" programParameters.g2oWithTRACKXYZ " << programParameters.g2oWithTRACKXYZ<< std::endl;
	std::cout<<" programParameters.g2oWithFeatures " << programParameters.g2oWithFeatures<< std::endl;
	std::cout<<" programParameters.transformationUncertainty " << programParameters.transformationUncertainty<< std::endl;

	if (programParameters.g2oWithFeatures > 0) {
		if (programParameters.g2oWithTRACKXYZ > 0) {
			for (int i = 0; i < tmpVertex.keypointsId.size(); i++) {
//				saveG20Vertex(Eigen::Matrix4f::Identity(), g2o,
//						tmpVertex.keypointsId[i]);
				saveG20EdgeTRACKXYZ(g2o,tmpVertex, i);
			}

		} else {
			for (int i = 0; i < tmpVertex.keypointsId.size(); i++) {
				saveG20Vertex(Eigen::Matrix4f::Identity(), g2o,
						tmpVertex.keypointsId[i]);
				saveG20Edge(g2o, tmpVertex, i);
			}
		}
	}

	// Saving absolute transformation
	sprintf(relativeTransformationName, "1.abs");
	saveTransformation(relativeTransformationName, transformation[0]);

	// MAIN TRACKING LOOP
	for (int i = 2; i < programParameters.TrackingLength; i++) {
		// Gt timstamp
		getline(gtNumberStream, timestamp);
		timestamp = timestamp.substr(0, timestamp.find("\t"));

		// Loading the image to track features
		sprintf(RGBname, "rgb_%05d.png", i);
		sprintf(Dname, "depth_%05d.png", i);

		RGBD->LoadRGBD(RGBname, Dname, rgbImg[2], dImg[2]);
		tracking->frameBGR[2] = rgbImg[2];
		tracking->frameD[2] = dImg[2];

		// Doing the tracking procedure
		if (programParameters.trackingMatching == 0)
		{
			tracking->doTracking(programParameters, depthInvScale);
		}
		else
		{
			swap(tracking->frameBGR[1], tracking->frameBGR[2]);
		}

//		// Preview of the tracking
//		if (programParameters.showTracking) {
//			tracking->trackShow();
//		}

		// If we need to start new detection
		if (i % programParameters.TrackingStepSize == j
				|| (tracking->points[1].size()
						< programParameters.TrackingFeaturesThreshold && programParameters.trackingMatching == 0)) {

			// Let's make a a history vertex out of the currently tracked features
			g2oIndices << timestamp << std::endl;
			tracking->createVertex(programParameters, calibrationParams,
								depthInvScale, saved_number, tracking->frameBGR[1]);

			// Current frame is secondFrame in matching
			Eigen::Matrix4f relativeTransformation, oldTransformation;
			kabschVertex *secondFrame =
					&tracking->vertexHistory[tracking->vertexHistory.size() - 1];

			// Matching frames
			int historySavedSize = tracking->vertexHistory.size();
			int bundleAdjustmentStart, bundleAdjustmentStop =  historySavedSize - 1;

			// If there is bundle adjustment activated
			if (programParameters.bundleAdjustment == 1)
				bundleAdjustmentStart =  std::max(0, historySavedSize - 1 - programParameters.bundleAdjustmentWindowSize);
			else
				bundleAdjustmentStart = std::max(0, historySavedSize - 2);

			// Main matching loop
			for (int y = bundleAdjustmentStart ; y < bundleAdjustmentStop; y++) {
				kabschVertex *firstFrame = &tracking->vertexHistory[y];

				if (programParameters.verbose != 0)
				{
					cout << "Estimating transformation between "
							<< firstFrame->vertexId << " & " << secondFrame->vertexId
							<< endl;
				}
				bool saveInliers =  !programParameters.trackingMatching;//( y == bundleAdjustmentStop - 1);
				// Estimating transformation
				Eigen::Matrix<double, 6, 6> transUncertainty;
				// Returns the uncertainty of the INVERSE of the transformation !!!!
				bool transformationFound = tracking->estimateTransformation(
						firstFrame, secondFrame, programParameters,
						programParameters.constrain, calibrationParams,
						depthInvScale, relativeTransformation, transUncertainty,
						programParameters.trackingMatching,
						programParameters.TrackingFeatureNumber,
						saveInliers);

				// Adding inlier features to graph
				if (programParameters.g2oWithFeatures > 0) {

					// Wont work with BA
					saveG2OVertexPosition(transformation[saved_number - 1]
					 									* relativeTransformation.inverse(), g2o, secondFrame->vertexId);

					if (transformationFound)
						findAndSaveG20Features(*firstFrame, g2o, 1, programParameters.g2oWithTRACKXYZ);
					else if (y == bundleAdjustmentStop - 1)
					{
						saveG20Edge(g2o, firstFrame->vertexId,
														secondFrame->vertexId,
														Eigen::Matrix4f::Identity());
						// CHANGE IT
//						saveG2OnonExistingFeatures(g2o, firstFrame->vertexId,
//								secondFrame->vertexId, tracking->nonExistingFeatureCounter);
//						tracking->nonExistingFeatureCounter += 8;
//						tracking->vertexCounter -= 1;
//						secondFrame->vertexId -= 1;
					}
				}
				// G20 w/o features
				else if (transformationFound) {
					saveG20Edge(g2o, firstFrame->vertexId, secondFrame->vertexId,
							relativeTransformation.inverse(), transUncertainty.inverse());
				}
				// No transformation found to the last frame -- adding identity to have continuous graph
				else if (y == bundleAdjustmentStop - 1) {
					saveG20Edge(g2o, firstFrame->vertexId, secondFrame->vertexId,
							Eigen::Matrix4f::Identity());
					relativeTransformation = Eigen::Matrix4f::Identity();
				}

			}

			// Adding current frame inlier features to graph
			if (programParameters.g2oWithFeatures > 0) {
				findAndSaveG20Features(*secondFrame, g2o, 2, programParameters.g2oWithTRACKXYZ);
			}

			// HACK for reading KW's results
			/*		ifstream icp;
			 sprintf(relativeTransformationName, "../icp/%d.icp", i);
			 icp.open(relativeTransformationName);
			 std::string linia;
			 for (int hh1=0;hh1<4;hh1++)
			 {
			 for (int jj1=0;jj1<4;jj1++)
			 {
			 icp>>linia;
			 relativeTransformation(hh1,jj1) = atof(linia.c_str());
			 }
			 getline(icp, linia);
			 }
			 std::cout<<"i="<<i<<std::endl<<relativeTransformation<<std::endl;
			 icp.close();
			 */
//			if (std::isnan(relativeTransformation(0,0)))
//			{
//				relativeTransformation = Eigen::Matrix4f::Identity();
//			}


			// Saving relative transformation
			sprintf(relativeTransformationName, "%d.rel", i);
			saveTransformation(relativeTransformationName,
					relativeTransformation);

//			printf("DET: %f\n", (relativeTransformation).determinant() );

			// Saving absolute transformation
			transformation[saved_number] = transformation[saved_number - 1]
					* relativeTransformation.inverse();
			sprintf(relativeTransformationName, "%d.abs", i);
			saveTransformation(relativeTransformationName,
					transformation[saved_number]);
			tracking->vertexHistory[tracking->vertexHistory.size() - 1].absolutePosition =
					transformation[saved_number];

			// Saving in freiburg
			saveTrajectory(transformation[saved_number], estTrajectory,
					timestamp);


			// Attemp to perform loop closure
//			if (programParameters.matchingLoopClosure == 1) {
//
//				int historySavedSize = tracking->vertexHistory.size();
//				int loopClosureStart = std::max(0, historySavedSize - 1 - programParameters.matchingLoopClosureWindowFrom);
//				int loopClosureStop = std::max(0, historySavedSize - 1 - programParameters.matchingLoopClosureWindowTo);
//
//				std::cout<< loopClosureStart << " to " << loopClosureStop << std::endl;
//
//				double minimalDistance = -1;
//				int minimalIndex = -1;
//				for (int y = loopClosureStart; y < loopClosureStop ; y++) {
//					kabschVertex *firstFrame = &tracking->vertexHistory[y];
//
//					double matrixErr = tracking->matrixError(
//							firstFrame->absolutePosition,
//							secondFrame->absolutePosition);
//
//					cout << "Errors for loop closure: " << matrixErr << " --- "
//							<< firstFrame->vertexId << " "
//							<< secondFrame->vertexId << std::endl;
//					if (minimalDistance == -1 || minimalDistance > matrixErr) {
//						minimalDistance = matrixErr;
//						minimalIndex = y;
//					}
//				}
//
//				if (minimalIndex != -1) {
//					// Estimating transformation
//					kabschVertex *firstFrame =
//							&tracking->vertexHistory[minimalIndex];
//					cout << "Index: " << firstFrame->vertexId << " "
//							<< secondFrame->vertexId << std::endl;
//
//					bool transformationFound = tracking->estimateTransformation(
//							firstFrame, secondFrame, programParameters,
//							programParameters.constrain, calibrationParams,
//							depthInvScale, relativeTransformation,
//							0, programParameters.TrackingFeatureNumber, true);
//
//					std::cout << "Loop closure success : "
//							<< transformationFound << std::endl;
//
//					if (transformationFound) {
//						saveG20Edge(g2o, firstFrame->vertexId,
//								secondFrame->vertexId,
//								relativeTransformation.inverse());
//					}
//				}
//
//			}

			// Saving point clouds
			if (programParameters.visualization == 1) {
				sprintf(relativeTransformationName, "%d.pcd", i);
				partial_cloud[0] = RGBD->BuildPointCloudFromRGBD(
						tracking->frameBGR[2], tracking->frameD[2],
						depthInvScale, calibrationParams);
				RGBD->SaveCloudXYZRGBA(relativeTransformationName,
						partial_cloud[0]);

				sprintf(relativeTransformationName, "%d.pcd", i);
				partial_cloud[saved_number] = RGBD->BuildPointCloudFromRGBD(
						tracking->frameBGR[2], tracking->frameD[2],
						depthInvScale, calibrationParams);
				RGBD->SaveCloudXYZRGBA(relativeTransformationName,
						partial_cloud[saved_number]);
			}

			// If we need to detect new features
			if (programParameters.verbose != 0) {
				cout << "Starting new detection" << endl;
			}
			RGBD->LoadRGBD(RGBname, Dname, rgbImg[0], dImg[0]);
			tracking->frameBGR[0] = rgbImg[0];
			tracking->frameD[0] = dImg[0];

			if (programParameters.trackingMatching == 0) {
				tracking->newDetection(programParameters, calibrationParams,
						depthInvScale, saved_number);
			} else {
				tracking->newDetectionMatching(programParameters,
						calibrationParams, depthInvScale, saved_number);
			}
			saved_number++;
		}
	}


	double ti = pcl::getTime() - startTime;
	if (programParameters.verbose != 0) {
		printf("\n\n\nEnded algorithm calculation: %f\n", ti);
	}

	ofstream t;
	t.open("timeAndLC");
	t << ti << std::endl;



	// LC
	int ileLC = 0;
	if (programParameters.matchingLoopClosure == 1) {
		Eigen::Matrix4f relativeTransformation;
		double startLC = pcl::getTime();
		int historySavedSize = tracking->vertexHistory.size();

		std::vector<geometricDistance> loopClosureDistances;
		for (int i=0;i<historySavedSize;i++)
		{
			for(int j=i+2*programParameters.bundleAdjustmentWindowSize;j<historySavedSize;j++)
			{
				kabschVertex *firstFrame = &tracking->vertexHistory[i];
				kabschVertex *secondFrame = &tracking->vertexHistory[j];

				geometricDistance *tmp = new geometricDistance();
				tmp->i = i;
				tmp->j = j;
				tmp->distance = tracking->matrixError(
						firstFrame->absolutePosition,
						secondFrame->absolutePosition);
				loopClosureDistances.push_back(*tmp);
			}
		}

		// Sort by distances
		std::sort(loopClosureDistances.begin(), loopClosureDistances.end(), geometricDistanceCompare);

		int i=0;
		while (pcl::getTime() - startLC < ti) {

			if (i < loopClosureDistances.size()) {
				geometricDistance tmp = loopClosureDistances[i];
				if (programParameters.verbose != 0) {
					std::cout << "LC distance : " << tmp.distance << std::endl;
				}
				// Estimating transformation
				kabschVertex *firstFrame =
						&tracking->vertexHistory[tmp.i];
				kabschVertex *secondFrame =
						&tracking->vertexHistory[tmp.j];

				Eigen::Matrix<double, 6, 6> transUncertainty;
				// Returns the uncertainty of the INVERSE of returned transformation!!!
				bool transformationFound = tracking->estimateTransformation(
						firstFrame, secondFrame, programParameters,
						programParameters.constrain, calibrationParams,
						depthInvScale, relativeTransformation, transUncertainty,  1,
						programParameters.TrackingFeatureNumber, false, true);

				if (programParameters.verbose != 0) {
					std::cout << "Loop closure success : "
							<< transformationFound << std::endl;
				}

				if (transformationFound) {
					saveG20Edge(g2o, firstFrame->vertexId,
							secondFrame->vertexId,
							relativeTransformation.inverse(), transUncertainty.inverse());
					ileLC++;
				}
				i++;
			}
			else
				break;

		}

	}
	t << "LC : " << ileLC << std::endl;
	t.close();

	if (programParameters.verbose != 0) {
		printf("Number of LC : %d \n", ileLC);

		printf("Comparison after %d measurements\n",
				tracking->measurementCounter);
		printf("Detection time : %.4f ms\n",
				tracking->detectionTime / tracking->measurementCounter * 1000);
		printf("Description time : %.4f ms\n",
				tracking->descriptionTime / tracking->measurementCounter
						* 1000);
		printf("Tracking time : %.4f ms\n",
				tracking->trackingTime / tracking->measurementCounter * 1000);
		printf("Matching time : %.4f ms\n",
				tracking->matchingTime / tracking->measurementCounter * 1000);

		printf("DBScan time : %.4f ms\n",
						tracking->dbscanOrQuadtreeTime / tracking->dbscanOrQuadtreeCounter * 1000);
	}
	estTrajectory.close();
	g2o.close();
	g2oIndices.close();

	//showCloudXYZRGBA(partial_cloud[0]);
	// Sum point cloud
	if (programParameters.visualization != 0) {
		cout << "Number of partial point clouds: " << saved_number << endl;

		for (int i = 2; i < saved_number; i++) {

			if (i % 5 == 2) {
				//	Eigen::Matrix4f tmp = KinectInitial.inverse() * transformation[i];
				//	RGBD->TransformSelf(partial_cloud[i], tmp );
				RGBD->TransformSelf(partial_cloud[i], transformation[i]);
				cout << "SUM" << endl;
				*partial_cloud[0] = *partial_cloud[0] + *partial_cloud[i];
			}
		}

		cout << "VOXEL" << endl;
		partial_cloud[0] = VoxelGridFilter(partial_cloud[0],
				programParameters.LeafSize);
		RGBD->SaveCloudXYZRGBA("results.pcd", partial_cloud[0]);
		cout << "Saving the sum point cloud" << endl;

		showCloudXYZRGBA(partial_cloud[0]);
	}

	delete tracking;
}

void Kinect::saveTransformation(char *wzg, Eigen::Matrix4f transformation) {
	ofstream relativeStream;
	relativeStream.open(wzg);
	relativeStream << transformation;
	relativeStream.close();

}
