#include "kinect.h"
#include <pcl/common/time.h>

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

		//
		// Tracking parameters
		//
		if (linia == "# Number of tracked frames") {
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
		}
		else if (linia == "# New detector on/off") {
			o >> linia;
			programParameters.newDetector = atoi(linia.c_str());
		}
		else if (linia == "# New detector kickThreshold") {
			o >> linia;
			programParameters.newDetectorKickThreshold = atoi(linia.c_str());
		}
		else if (linia == "# New detector fast threshold") {
			o >> linia;
			programParameters.newDetectorFastThreshold = atoi(linia.c_str());
		}
		else if  (linia == "# New detector depth test threshold") {
			o >> linia;
			programParameters.newDetectorDepthTestThreshold = atof(linia.c_str());
		}
		//
		// DB Scan
		//
		else if (linia == "# DBScan on/off") {
			o >> linia;
			programParameters.DBScan = atoi(linia.c_str());
		}
		else if (linia == "# DBScan MinPts") {
			o >> linia;
			programParameters.DBScanMinPts = atoi(linia.c_str());
		} else if (linia == "# DBScan eps") {
			o >> linia;
			programParameters.DBScanEps = atoi(linia.c_str());
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
		} else if (linia == "# historySize - additional edges") {
			o >> linia;
			programParameters.historySize = atoi(linia.c_str());
		} else if (linia == "# g2o with features") {
			o >> linia;
			programParameters.g2oWithFeatures = atoi(linia.c_str());
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

	Eigen::Quaternion<float> quat( qw, qx, qy, qz );
	KinectInitial.block<3,3>(0,0) = quat.toRotationMatrix();
	KinectInitial(0,3) = x;
	KinectInitial(1,3) =  y;
	KinectInitial(2,3) =  z;
	KinectInitial(3,3) = 1.0;
}

void Kinect::TrackingRun() {
	tracking = new Track();



	// Saving trajectory in Freiburg format
	std::ifstream gtNumberStream;
	gtNumberStream.open("matchedIndices");
	std::string timestamp;
	ofstream estTrajectory;
	estTrajectory.open("../results/result");

	// Locally used variables
	char RGBname[30], Dname[30];
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr partial_cloud[2500];
	Eigen::Matrix4f transformation[2500];
	char relativeTransformationName[30];
	int j = 0, saved_number = 1;
	transformation[0] = KinectInitial;
	//transformation[0] = Eigen::Matrix4f::Identity();
	cv::Mat rgbImg[3], dImg[3];

	/*for (int i = 1 ; i < programParameters.TrackingLength; i++) {
			cv::Mat xxx;
			sprintf(RGBname, "rgb_%04d.png", i);
			RGBD->LoadRGB(RGBname, xxx);
			double start = pcl::getTime();
			double val = tracking->FMmodifiedLaplacian(xxx);
			start = pcl::getTime() - start;
			cout<<val << " " << start <<endl;
	}
	int a;
	cin>>a;*/

	// First detection
	cout << "Reading first RGB i D" << endl;


	long startTime = pcl::getTime();

	rgbImg[0] = cv::Mat();
	RGBD->LoadRGBD("rgb_0001.png", "depth_0001.png", rgbImg[0], dImg[0]);
	tracking->frameBGR[0] = rgbImg[0];
	tracking->frameD[0] = dImg[0];

	if (programParameters.visualization == 1)
	{
		// Saving point clouds
		sprintf(relativeTransformationName, "1.pcd");
		partial_cloud[0] = RGBD->BuildPointCloudFromRGBD(
						tracking->frameBGR[0], tracking->frameD[0], depthInvScale, calibrationParams);
		RGBD->SaveCloudXYZRGBA(relativeTransformationName, partial_cloud[0]);
	}

	// Detection on the first set of images
	cout << "New detection" << endl;
	tracking->newDetection(programParameters, calibrationParams, depthInvScale, 0);
	cout<<"Ended detection" << endl;

	// First position
	getline(gtNumberStream,timestamp);
	timestamp = timestamp.substr(0,timestamp.find("\t"));

	cout << "Saving in freiburg format" << endl;
	Eigen::Quaternion<float> Q(transformation[0].block<3, 3>(0, 0));
	estTrajectory << timestamp << " " << transformation[0](0, 3)
			<< " " << transformation[0](1, 3) << " "
			<< transformation[0](2, 3) << " " << Q.coeffs().x()
			<< " " << Q.coeffs().y() << " " << Q.coeffs().z() << " "
			<< Q.coeffs().w() << endl;

	// zapis g2o
	std::ofstream g2o;
	g2o.open("../results/tracking.g2o");
	kabschVertex tmpFrame =
						tracking->vertexHistory[0];
	g2o << "VERTEX_SE3:QUAT " << tmpFrame.vertexId <<  " "
			<< transformation[0](0, 3) << " " << transformation[0](1, 3) << " "
			<< transformation[0](2, 3) << " " << Q.coeffs().x() << " "
			<< Q.coeffs().y() << " " << Q.coeffs().z() << " " << Q.coeffs().w()
			<< endl;
	g2o << "FIX " << tmpFrame.vertexId << endl;
//	kabschVertex tmpVertex = tracking->vertexHistory[tracking->vertexHistory.size() - 1];
//	for (int i =0; i< tmpVertex.keypointsId.size(); i++) {
//		g2o << "VERTEX_SE3:QUAT " << tmpVertex.keypointsId[i] << " 0 0 0 0 0 0 1"<<endl;
//		g2o << "EDGE_SE3:QUAT 0 " << tmpVertex.keypointsId[i] << " "
//				<< tmpVertex.keypoints3d[i][0] << " "
//				<< tmpVertex.keypoints3d[i][1] << " "
//				<< tmpVertex.keypoints3d[i][2] << " 0 0 0 1"
//				<< " 0.0001 0 0 0 0 0 0.0001 0 0 0 0 0.0001 0 0 0 0.0001 0 0 0.0001 0 0.0001" << endl;
//				//<< " 1 0 0 0 0 0 1 0 0 0 0 1 0 0 0 1 0 0 1 0 1" << endl;
//	}



	cout << "Saving absolute" << endl;
	// Saving absolute transformation
	sprintf(relativeTransformationName, "1.abs");
	saveTransformation(relativeTransformationName,
			transformation[0]);


	// tracking loop
	cout<<"Tracking length: " << programParameters.TrackingLength<<endl;
	for (int i = 2 ; i < programParameters.TrackingLength; i++) {
		// Gt timstamp
		getline(gtNumberStream,timestamp);
		timestamp = timestamp.substr(0,timestamp.find("\t"));

		// Loading the image to track features
		cout << "Loading tracking: " << i << endl;
		sprintf(RGBname, "rgb_%04d.png", i);
		sprintf(Dname, "depth_%04d.png", i);
		cout<<RGBname<<endl;
		RGBD->LoadRGBD(RGBname, Dname, rgbImg[2], dImg[2]);
		tracking->frameBGR[2] = rgbImg[2];
		tracking->frameD[2] = dImg[2];
		std::cout<<"s : " << tracking->frameBGR[2].cols<<" "<<tracking->frameBGR[2].rows<<endl;

		// Doing the tracking procedure
		tracking->doTracking(programParameters, depthInvScale);

		// Preview of the tracking
		if (programParameters.showTracking)
		{
			tracking->trackShow();
		}
		// If we need to start new detection
		cout << "Checking stop condition" << endl;
		if ( i % programParameters.TrackingStepSize == j ) {


			// Let's make a a history vertex out of the currently tracked features
			tracking->createVertex(programParameters, calibrationParams, depthInvScale, atoi(timestamp.c_str()));

			Eigen::Matrix4f relativeTransformation, oldTransformation;
			kabschVertex &secondFrame =
					tracking->vertexHistory[tracking->vertexHistory.size() - 1];

			// Matching frames
			for (int y = 0; y < tracking->vertexHistory.size() - 1; y++) {
				kabschVertex &firstFrame = tracking->vertexHistory[y];
				cout << "Estimating transformation between " << firstFrame.vertexId << " & " << secondFrame.vertexId << endl;

				// Estimating transformation
				KinectInitial = tracking->estimateTransformation(firstFrame,
						secondFrame, programParameters.transformationPairNumber,
						programParameters.constrain, calibrationParams,
						programParameters.inlierThreshold,
						programParameters.wantedInlierRatio, depthInvScale,
						relativeTransformation, g2o);

				// Adding features to graph
				if (programParameters.g2oWithFeatures > 0) {
					// First frame
					std::cout<<"SIZES: " << tracking->wasInlierFirst.size() << " " << firstFrame.inGraph.size() << std::endl;
					for (int i = 0; i < tracking->wasInlierFirst.size(); i++) {
						if ( tracking->wasInlierFirst[i] == true && firstFrame.inGraph[i] == false ) {
							firstFrame.inGraph[i] = true;
							g2o << "EDGE_SE3:QUAT " << firstFrame.vertexId
									<< " " << 5000 + firstFrame.keypointsId[i] << " " // TODO: 5000 is hardcoded !!!
									<< firstFrame.keypoints3d[i][0] << " "
									<< firstFrame.keypoints3d[i][1] << " "
									<< firstFrame.keypoints3d[i][2]
									<< " 0 0 0 1"
									<< " 0.0001 0 0 0 0 0 0.0001 0 0 0 0 0.0001 0 0 0 0.00000001 0 0 0.00000001 0 0.00000001"
									<< std::endl;
						}
					}
					tracking->wasInlierFirst.clear();
				}


				if ( ( relativeTransformation - Eigen::Matrix4f::Identity() ).norm() > 0.1
						|| y + 1 == tracking->vertexHistory.size() - 1) {
					// Adding transformation to graph
					Eigen::Matrix4f xxx = relativeTransformation.inverse();
					Eigen::Quaternion<float> Qg(xxx.block<3, 3>(0, 0));
					g2o << "EDGE_SE3:QUAT " << firstFrame.vertexId << " "
							<< secondFrame.vertexId << " " << xxx(0, 3) << " "
							<< xxx(1, 3) << " " << xxx(2, 3) << " "
							<< Qg.coeffs().x() << " " << Qg.coeffs().y() << " "
							<< Qg.coeffs().z() << " " << Qg.coeffs().w()
							<< " 1 0 0 0 0 0 1 0 0 0 0 1 0 0 0 1 0 0 1 0 1"
							<< endl;
				}

				// Saving for feature
				if (y + 1 == tracking->vertexHistory.size() - 1) {
					oldTransformation = relativeTransformation;
				}

			}
			// Adding features to graph
			if (programParameters.g2oWithFeatures > 0) {
				// First frame
				std::cout<<"SIZES: " << tracking->wasInlierSecond.size() << " " << secondFrame.inGraph.size() << std::endl;
				for (int i = 0; i < tracking->wasInlierSecond.size(); i++) {
					if (tracking->wasInlierSecond[i] == true
							&& secondFrame.inGraph[i] == false) {
						secondFrame.inGraph[i] = true;
						g2o << "EDGE_SE3:QUAT " << secondFrame.vertexId << " "
								<< 5000 + secondFrame.keypointsId[i]
								<< " " // TODO: 5000 is hardcoded !!!
								<< secondFrame.keypoints3d[i][0] << " "
								<< secondFrame.keypoints3d[i][1] << " "
								<< secondFrame.keypoints3d[i][2] << " 0 0 0 1"
								<< " 0.0001 0 0 0 0 0 0.0001 0 0 0 0 0.0001 0 0 0 0.00000001 0 0 0.00000001 0 0.00000001"
								<< std::endl;
					}
				}
				tracking->wasInlierFirst.clear();
			}


			relativeTransformation = oldTransformation;
//			cout << "Estimating transformation " << endl;
//			Eigen::Matrix4f relativeTransformation;
//			KinectInitial = tracking->estimateTransformation(firstFrame, secondFrame,
//					programParameters.transformationPairNumber,
//					programParameters.constrain, calibrationParams,
//					programParameters.inlierThreshold,
//					programParameters.wantedInlierRatio,
//					depthInvScale,
//					relativeTransformation, g2o);

			// Reading KW's results
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
			cout << "Saving relative" << endl;
			// Saving relative transformation
			sprintf(relativeTransformationName, "%d.rel", i);
			saveTransformation(relativeTransformationName, relativeTransformation);


			cout<<"Saving absolute"<<endl;
			// Saving absolute transformation
			transformation[saved_number] = transformation[saved_number-1] * relativeTransformation.inverse();
			sprintf(relativeTransformationName, "%d.abs", i);
			saveTransformation(relativeTransformationName, transformation[saved_number]);

			cout<<"Saving in freiburg format" << endl;
			Eigen::Quaternion<float> Q(
					transformation[saved_number].block<3, 3>(0, 0));
			estTrajectory << timestamp << " " << transformation[saved_number](0, 3) << " "
					<< transformation[saved_number](1, 3) << " " << transformation[saved_number](2, 3) << " "
					<< Q.coeffs().x() << " " << Q.coeffs().y() << " "
					<< Q.coeffs().z() << " " << Q.coeffs().w() << endl;


			sprintf(relativeTransformationName, "%d.pcd", i);
			partial_cloud[0] = RGBD->BuildPointCloudFromRGBD(
					tracking->frameBGR[2], tracking->frameD[2], depthInvScale,
					calibrationParams);
			RGBD->SaveCloudXYZRGBA(relativeTransformationName,
					partial_cloud[0]);


			// Saving point clouds
			if (programParameters.visualization == 1)
			{
				sprintf(relativeTransformationName, "%d.pcd", i);
				partial_cloud[saved_number] = RGBD->BuildPointCloudFromRGBD(
						tracking->frameBGR[2], tracking->frameD[2], depthInvScale, calibrationParams);
				RGBD->SaveCloudXYZRGBA(relativeTransformationName, partial_cloud[saved_number]);
			}



			//if (tracking->points[1].size()
			//		< programParameters.TrackingFeaturesThreshold)
			//{
			// Starting new detection
			cout << "Starting new detection" << endl;
			RGBD->LoadRGBD(RGBname, Dname, rgbImg[0], dImg[0]);
			tracking->frameBGR[0] = rgbImg[0];
			tracking->frameD[0] = dImg[0];


			tracking->newDetection(programParameters, calibrationParams,
					depthInvScale, atoi(timestamp.c_str()));

			//}

//			g2o << "VERTEX_SE3:QUAT " << tracking->vertexCounter-5 << " "
//					<< transformation[saved_number](0, 3) << " "
//					<< transformation[saved_number](1, 3) << " "
//					<< transformation[saved_number](2, 3) << " "
//					<< Q.coeffs().x() << " " << Q.coeffs().y() << " "
//					<< Q.coeffs().z() << " " << Q.coeffs().w() << endl;

//			Eigen::Quaternion<float> Qg(
//					relativeTransformation.block<3, 3>(0, 0));
//
//			g2o << "EDGE_SE3:QUAT " << tracking->vertexCounter - 10 << " "
//					<< tracking->vertexCounter - 5 << " "
//					<< relativeTransformation(0, 3) << " "
//					<< relativeTransformation(1, 3) << " "
//					<< relativeTransformation(2, 3) << " " << Qg.coeffs().x()
//					<< " " << Qg.coeffs().y() << " " << Qg.coeffs().z() << " "
//					<< Qg.coeffs().w()
//					<< " 1 0 0 0 0 0 1 0 0 0 0 1 0 0 0 1 0 0 1 0 1" << endl;

//			tmpVertex = tracking->vertexHistory[tracking->vertexHistory.size() - 1];
//				for (int i =0; i< tmpVertex.keypointsId.size(); i++) {
//					g2o << "VERTEX_SE3:QUAT " << tmpVertex.keypointsId[i] << " 0 0 0 0 0 0 1"<<endl;
//					g2o << "EDGE_SE3:QUAT "<<tracking->vertexHistory.size()*5 - 5<<" " << tmpVertex.keypointsId[i] << " "
//							<< tmpVertex.keypoints3d[i][0] << " "
//							<< tmpVertex.keypoints3d[i][1] << " "
//							<< tmpVertex.keypoints3d[i][2] << " 0 0 0 1"
//							<< " 0.0001 0 0 0 0 0 0.0001 0 0 0 0 0.0001 0 0 0 0.0001 0 0 0.0001 0 0.0001" << endl;
//							//<< " 1 0 0 0 0 0 1 0 0 0 0 1 0 0 0 1 0 0 1 0 1" << endl;
//				}

			saved_number++;
		}
	}

	cout<<"Ended algorithm calculation:"<<endl<< pcl::getTime() - startTime << endl;

	estTrajectory.close();
	g2o.close();


	//showCloudXYZRGBA(partial_cloud[0]);
	// Sum point cloud
	if ( programParameters.visualization != 0)
	{
		cout << "Number of partial point clouds: " << saved_number << endl;

		for (int i = 2; i < saved_number; i++) {

			if ( i%5 == 2)
			{
			//	Eigen::Matrix4f tmp = KinectInitial.inverse() * transformation[i];
			//	RGBD->TransformSelf(partial_cloud[i], tmp );
				RGBD->TransformSelf(partial_cloud[i], transformation[i] );
				cout<<"SUM"<<endl;
				*partial_cloud[0] = *partial_cloud[0] + *partial_cloud[i];
			}
		}

		cout<<"VOXEL"<<endl;
		partial_cloud[0] = VoxelGridFilter(partial_cloud[0],programParameters.LeafSize);
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
