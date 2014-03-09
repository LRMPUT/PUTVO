#include "track.h"
#include <pcl/common/time.h>

Track::Track() {
	for (int i = 0; i < 3; i++) {
		frameD[i] = cv::Mat(480, 640, CV_32F);
	}
	featureCounter = 150;
	vertexCounter = 0;
}

void Track::clearTrack() {
	for (int i = 0; i < 3; i++)
		points[i].clear();
}

bool keypointClassResponseCompare(std::pair<cv::KeyPoint, int> i,
		std::pair<cv::KeyPoint, int> j) {
	if (i.second != j.second)
		return i.second > j.second;
	return (i.first.response > j.first.response);
}

bool keypointResponseCompare(cv::KeyPoint i, cv::KeyPoint j) {
	return i.response > j.response;
}

cv::Point3f Track::normalize_vector(cv::Point3f point) {
	cv::Point3f result;

	float norm = sqrtf(
			point.x * point.x + point.y * point.y + point.z * point.z);
	result.x = point.x / norm;
	result.y = point.y / norm;
	result.z = point.z / norm;

	return result;
}

cv::Point3f Track::normalize_point(int x, int y) {
	//cv::Point3f temp_pt( (x - 319.796)/574.124, (y - 243.961)/573.976, 1 );
	cv::Point3f temp_pt((x - 319.5) / 525.0, (y - 239.5) / 525.0, 1);
	return temp_pt;
}

std::vector<cv::KeyPoint> Track::detect_rgbd_features(cv::Mat rgb_image,
		cv::Mat depth_image, int fast_threshold, float flatness_threshold, int kickThreshold) {
	//std::cout<<"! " << rgb_image.cols<<" "<<rgb_image.rows<<" "<<depth_image.cols<<" "<<depth_image.rows<<std::endl;
	//cv::imshow("x",depth_image);
	//cvWaitKey(0);
	//shift coefficients for FAST Bresenham circle point coordinates
	int circle_coeffs[16][2] = { { 0, 3 }, { 1, 3 }, { 2, 2 }, { 3, 1 },
			{ 3, 0 }, { 3, -1 }, { 2, -2 }, { 1, -3 }, { 0, -3 }, { -1, -3 }, {
					-2, -2 }, { -3, -1 }, { -3, 0 }, { -3, 1 }, { -2, 2 }, { -1,
					3 } };
	//raw keypoints detected by masked FAST
	std::vector<cv::KeyPoint> raw_keypoints;
	//function result - filtered keypoints
	std::vector<cv::KeyPoint> filtered_keypoints;
	//create temportary Point3f storage
	cv::Point3f temp_pt;
	//create temporary keypoint storage
	cv::KeyPoint kpt;
	//create fast detector with defined threshold
	cv::FastFeatureDetector fast_detector(fast_threshold);
	//create storage for the detector mask
	cv::Mat depth_image_mask;
	//normalized depths of points on the Bresenham circle
	std::vector<cv::Point3f> normalized_bresenham(16);

	//create detector mask
	convertScaleAbs(depth_image, depth_image_mask, 5000, 0);

	//create fast features using mask (features not detected if depth=0)
	fast_detector.detect(rgb_image, raw_keypoints, depth_image_mask);
	//convert for 'at' compatibility
	depth_image.convertTo(depth_image, CV_32F);

	struct timeval startT, endT;
	gettimeofday( &startT, 0 );

	//for each detected keypoint do
	for (unsigned int j = 0; j < raw_keypoints.size(); j++) {
		//std::cout << "Count : " << j << std::endl;
		kpt = raw_keypoints[j];

		float lambdas[16];

		float depth_center = depth_image.at<float>(kpt.pt.y, kpt.pt.x);
	//	std::cout << "depth center : " << depth_center << std::endl;

		// divide the depths of the points on the Bresenham circle by the depth of the central point -> normalized depth
		for (unsigned int i = 0; i < 16; i++) {
			lambdas[i] = depth_image.at<float>(kpt.pt.y + circle_coeffs[i][0],
					kpt.pt.x + circle_coeffs[i][1]) / depth_center;
		}

		// normalize the coordinates of the central point
		cv::Point3f pt_c = normalize_point(kpt.pt.y, kpt.pt.x);

		// multiply by the computed scaling coefficients
		for (unsigned int i = 0; i < 16; i++) {
			temp_pt = normalize_point(kpt.pt.y + circle_coeffs[i][0],
					kpt.pt.x + circle_coeffs[i][1]) * lambdas[i];
			normalized_bresenham[i] = temp_pt;
		}

		// subtract the central point vector -> results are vectors connecting the central point to bresenham points
		for (unsigned int i = 0; i < 16; i++) {   //diff
			temp_pt = normalized_bresenham[i] - pt_c;
			normalized_bresenham[i] = temp_pt;
		}

		// normalize the vectors to unit length
		for (unsigned int i = 0; i < 16; i++) {
			temp_pt = normalize_vector(normalized_bresenham[i]);
			normalized_bresenham[i] = temp_pt;
		}

		unsigned int kickout = 0;
		// decide whether or not keep the keypoint based on the flatness established by the dot product of opposing points
		for (unsigned int i = 0; i < 8; i++) {

			if (fabs(normalized_bresenham[i].dot(normalized_bresenham[i + 8]))
					< flatness_threshold) {
				kickout++; // figure out how to delete keypoints not meeting criteria from the list
			}

		}

		if (kickout < kickThreshold) {
			filtered_keypoints.push_back(kpt);
		}

		//cvWaitKey(0);
	}
	gettimeofday( &endT, 0 );
	std::cout << "Czas testu: " << 1000000 * (endT.tv_sec - startT.tv_sec) + (endT.tv_usec - startT.tv_usec) <<std::endl;
	std::cout << "Filtered keypoints: " << filtered_keypoints.size()
			<< std::endl;
	return filtered_keypoints;

}


void Track::newDetection(ProgramParameters programParameters, CalibrationParameters calibrationParams, double depthInvScale, int frameId) {

	std::vector<cv::KeyPoint> keypoints;
	std::cout<<"Parameter test: " << programParameters.newDetector<< " " << programParameters.detectionInStripes << std::endl;
	if ( programParameters.newDetector == 0 )
	{

		if (programParameters.detectionInStripes == 1)
		{
			cv::Ptr<cv::FeatureDetector> detector(
							new cv::DynamicAdaptedFeatureDetector(
									new cv::FastAdjuster(2, true),
									(programParameters.TrackingFeatureNumber - 100) / 6,
									programParameters.TrackingFeatureNumber / 6));
			for (int i = 0; i < 6; i++) {
				std::vector<cv::KeyPoint> keypointsInROI;
				cv::Mat roi(frameBGR[0], cv::Rect(0, i * 80, 640, 80));
				detector->detect(roi, keypointsInROI);

				sort(keypointsInROI.begin(), keypointsInROI.end(),
						keypointResponseCompare);

				for (int j = 0;
						j < programParameters.TrackingFeatureNumber / 6
								&& j < keypointsInROI.size(); j++) {
					keypointsInROI[j].pt.y += i * 80;
					keypoints.push_back(keypointsInROI[j]);
				}
			}
		}
		else
		{
			cv::Ptr<cv::FeatureDetector> detector(
										new cv::DynamicAdaptedFeatureDetector(
												new cv::FastAdjuster(5, true),
												programParameters.TrackingFeatureNumber - 50,
												programParameters.TrackingFeatureNumber + 50));
			detector->detect(frameBGR[0], keypoints);
		}
	}
	// New detector
	else
	{
		if ( programParameters.detectionInStripes == 0)
		{
			keypoints = detect_rgbd_features(frameBGR[0], frameD[0], programParameters.newDetectorFastThreshold,
					programParameters.newDetectorDepthTestThreshold, programParameters.newDetectorKickThreshold);
		}
		else {
			for (int i = 0; i < 6; i++) {
				std::vector<cv::KeyPoint> keypointsInROI;
				cv::Mat roiBGR(frameBGR[0], cv::Rect(0, i * 80, 640, 80));
				cv::Mat roiD(frameD[0], cv::Rect(0, i * 80, 640, 80));
				keypointsInROI = detect_rgbd_features(roiBGR, roiD,
						programParameters.newDetectorFastThreshold,
						programParameters.newDetectorDepthTestThreshold,
						programParameters.newDetectorKickThreshold);

				sort(keypointsInROI.begin(), keypointsInROI.end(), keypointResponseCompare);
				for (int j = 0; j < keypointsInROI.size(); j++) {
					if ( j > programParameters.TrackingFeatureNumber / 6)
						break;
					keypointsInROI[j].pt.y += i * 80;
					keypoints.push_back(keypointsInROI[j]);
				}
			}
		}

	}
//	cv::Mat x;
//	cv::drawKeypoints(frameBGR[0],keypoints,x,cv::Scalar( 0, 255, 0), cv::DrawMatchesFlags::DEFAULT );
//	cv::imshow("Keypoints 1", x );
//	cv::imwrite("../../../Desktop/1.png",x);
//	cv::waitKey(0);
	//
	//std::cout << "Typical detection just ended" << std::endl;

	// If we can use DBScan
	if (programParameters.DBScan == 1)
	{
		std::vector<int> clusters;
		DBScan dbscan(programParameters.DBScanEps, programParameters.DBScanMinPts);
		dbscan.run(keypoints, clusters);

		// Choose only the best candidate from each group
		std::vector<std::pair<cv::KeyPoint, int> > keypointClusterSet;
		for (int i = 0; i < keypoints.size(); i++)
			keypointClusterSet.push_back(std::make_pair(keypoints[i], clusters[i]));
		sort(keypointClusterSet.begin(), keypointClusterSet.end(),
				keypointClassResponseCompare);
		keypoints.clear();

		// Number of detected clusters
		int clustersNumber = 0;
		for (int i = 0; i < clusters.size(); i++) {
			clustersNumber = std::max(clustersNumber, clusters[i]);
		}
		clustersNumber++;
		std::cout << "Cluster number: " << clustersNumber << std::endl;
//		int withoutCluster = expectedNumberOfFeatures - clustersNumber;

		std::cout << "Number of features BEFORE dbscan: "
				<< keypointClusterSet.size() << std::endl;
		cv::Point2f chosen;
		for (int i = 0, w_grupie = 0, last = -1, keypoints_index = 0;
				i < keypointClusterSet.size(); i++) {
			// Different labels - starting new group
			if (last != keypointClusterSet[i].second)
				w_grupie = 0;

			// It has been labeled
			if (keypointClusterSet[i].second != 0) {

				if (w_grupie == 0) {
					keypoints.push_back(keypointClusterSet[i].first);
					last = keypointClusterSet[i].second;
					w_grupie++;
				}
				i++;

				// The group ended
				while (keypointClusterSet[i].second
						== keypointClusterSet[i - 1].second) {
					i--;
					break;
				}
			} else {
				keypoints.push_back(keypointClusterSet[i].first);
			}
		}
		std::cout << "Number of features AFTER dbscan:" << keypoints.size()
				<< std::endl;
	}

//	cv::drawKeypoints(frameBGR[0],keypoints,x,cv::Scalar( 0, 255, 0) , cv::DrawMatchesFlags::DEFAULT );
//	cv::imshow("Keypoints 1", x );
//	cv::imwrite("../../../Desktop/2.png",x);
//	cv::waitKey(0);


	points[0].clear();
	points[2].clear();

	std::vector<cv::Point2f>::iterator it = points[1].begin();
	std::vector<int>::iterator itDepth = pointDepth.begin(), itID = pointsID.begin();
	for (int i=0;i<wasInlierSecond.size();i++)
	{
		if (!wasInlierSecond[i])
		{
			it = points[1].erase(it);
			itDepth = pointDepth.erase(itDepth);
			itID = pointsID.erase(itID);
		}
		else
		{
			++it, ++itDepth, ++itID;
		}
	}
	wasInlierSecond.clear();

//	points[1].clear();
//	pointDepth.clear();
//	pointsID.clear();

	// Sorting and choosing the strongest
	sort(keypoints.begin(), keypoints.end(), keypointResponseCompare);
	for (int i = 0; i < keypoints.size() && i < programParameters.TrackingFeatureNumber; i++)
		points[0].push_back(keypoints[i].pt);

	std::cout << "Starting subpixel detection" << std::endl;
	cv::Mat grayImg;
	cvtColor(frameBGR[0], grayImg, CV_RGB2GRAY);
	cornerSubPix(grayImg, points[0], cvSize(7, 7), cvSize(-1, -1),
			cvTermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 4, 0.01));

	float data[5] = { calibrationParams.k1, calibrationParams.k2,
			calibrationParams.p1, calibrationParams.p2, calibrationParams.k3 };
	calibrationParams.distCoeffs = cv::Mat(1, 5, CV_32FC1, &data);
	float cm[3][3] = { { calibrationParams.fu, 0, calibrationParams.u0 }, { 0,
			calibrationParams.fv, calibrationParams.v0 }, { 0, 0, 1 } };
	calibrationParams.cameraMatrix = cv::Mat(3, 3, CV_32FC1, &cm);



	kabschVertex kbVertex;
	//kbVertex.vertexId = vertexCounter++;
	kbVertex.vertexId = frameId;
	int ii = 0;
	for (std::vector<cv::Point2f>::iterator it = points[1].begin();
				it != points[1].end();it++, ii++) {
		kbVertex.keypointsId.push_back(pointsID[ii]);
		kbVertex.keypoints.push_back(*it);
		double Z = pointDepth[ii];
		kbVertex.keypoints3d.push_back(
				RGBDclass::point2Dto3D(*it, Z, calibrationParams.cameraMatrix,
						calibrationParams.distCoeffs));
	}


//	int ii = 0;

	for (std::vector<cv::Point2f>::iterator it = points[0].begin();
			it != points[0].end(); ii++) {
		double Z = double(frameD[0].at<uint16_t>(roundY(it->y),
				roundX(it->x)))
						/ depthInvScale;
		if ( Z != 0 )
		{
			pointsID.push_back(featureCounter);

			pointDepth.push_back(Z);

			kbVertex.keypointsId.push_back( featureCounter );
			kbVertex.keypoints.push_back(keypoints[ii].pt);
			kbVertex.keypoints3d.push_back( RGBDclass::point2Dto3D(*it, Z, calibrationParams.cameraMatrix,
							calibrationParams.distCoeffs) );

			featureCounter++;
			points[1].push_back(*it);
			it++;
		}
		else
		{
			it++;// = points[0].erase(it);
		}
	}

	kbVertex.inGraph.resize(kbVertex.keypoints.size(), 0);

	// Indirect copy
	//points[1] = points[0];
	frameBGR[0].copyTo(frameBGR[1]);
	std::cout << "Ended subpixel detection " << std::endl;




	//for (int i=0;i<points[0].size();i++)
	//{
		//kbVertex.keypointsId.push_back( pointsID[i] );

	//	cv::Point2f a = keypoints[i].pt;
	//	double Z = double(frameD[0].at<uint16_t>(roundY(a.y), roundX(a.x)))
	//			/ depthInvScale;

	//	kbVertex.keypoints3d.push_back( RGBDclass::point2Dto3D(a, Z, calibrationParams.cameraMatrix,
	//			calibrationParams.distCoeffs) );
	//}

	// Temporal vertex out
	if (vertexHistory.size() > 0 ) 	vertexHistory.pop_back();
	vertexHistory.push_back(kbVertex);

	if ( vertexHistory.size() > programParameters.historySize ) {
		vertexHistory.erase(vertexHistory.begin());
	}

}

/// TRACKing
void Track::doTracking(ProgramParameters programParameters, double depthInvScale) {

	std::vector<uchar> status;
	std::vector<float> err;
	cv::TermCriteria termcrit(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 15, 0.05);

	performedTrack = true;

	// Calculating movement of features
	cv::calcOpticalFlowPyrLK(frameBGR[1], frameBGR[2], points[1], points[2],
			status, err, cvSize(programParameters.winSize,programParameters.winSize), programParameters.maxLevels, termcrit);

	//std::cout << " Ended calc Optimal Flow : " << points[1].size() << " "
	//		<< points[2].size() << std::endl;
	int i = 0;


	std::vector<int>::iterator depthIt = pointDepth.begin(), IdIt = pointsID.begin();
	std::vector<cv::Point2f>::iterator it = points[2].begin();
	for (; it != points[2].end(); i++) {
		if (status[i] == 0) {
			depthIt = pointDepth.erase(depthIt);
			IdIt = pointsID.erase(IdIt);
			it = points[2].erase(it);
			std::cout<<"Status["<<i<<"] = 0" << std::endl;
		} else {
			++it;
			++depthIt;
			++IdIt;
		}
	}

	std::cout<<"Tracking sizes: " << points[1].size()<<" " <<  points[2].size() << " " << pointsID.size() << " " << pointDepth.size()<< std::endl;


	std::swap(points[1], points[2]);
	swap(frameBGR[1], frameBGR[2]);

	points[2].clear();
}

void Track::trackShow() {
	for (int i = 0; i < points[1].size(); i++) {
		cv::circle(frameBGR[1], points[1][i], 3, cv::Scalar(0, 255, 0), -1, 8);
	}
	cv::imshow("LK Demo", frameBGR[1]);
	cv::waitKey(2000);
}

int Track::roundX(double X) {
	if (X < 0)
		X = 0;
	else if (X > 639)
		X = 639;
	return round(X);
}

int Track::roundY(double Y) {
	if (Y < 0)
		Y = 0;
	else if (Y > 479)
		Y = 479;
	return round(Y);
}

double Track::FMmodifiedLaplacian(const cv::Mat& src) {
	double data[3] = { -1.0, 2.0, -1.0 };
	cv::Mat M = cv::Mat(3, 1, CV_32FC1, &data);
	cv::Mat G = cv::getGaussianKernel(3, -1, CV_64F);

	cv::Mat Lx;
	cv::sepFilter2D(src, Lx, CV_64F, M, G);

	cv::Mat Ly;
	cv::sepFilter2D(src, Ly, CV_64F, G, M);

	cv::Mat FM = cv::abs(Lx) + cv::abs(Ly);

	double focusMeasure = cv::mean(FM).val[0];
	return focusMeasure;
}

Eigen::Matrix4f Track::estimateTransformation(kabschVertex firstFrame, kabschVertex secondFrame, int pair_number,
		Constraints constraints, CalibrationParameters cameraParameters,
		double inliner_threshold, double ransac_break_percent,
		double depthInvScale, Eigen::Matrix4f &res, std::ofstream &g2o) {

	float data[5] = { cameraParameters.k1, cameraParameters.k2,
			cameraParameters.p1, cameraParameters.p2, cameraParameters.k3 };
	cameraParameters.distCoeffs = cv::Mat(1, 5, CV_32FC1, &data);
	float cm[3][3] = { { cameraParameters.fu, 0, cameraParameters.u0 }, { 0,
			cameraParameters.fv, cameraParameters.v0 }, { 0, 0, 1 } };
	cameraParameters.cameraMatrix = cv::Mat(3, 3, CV_32FC1, &cm);

	std::cout << "Poczatkowa liczba cech: " << firstFrame.keypoints.size() << " vs "
			<< secondFrame.keypoints.size()//points[1].size()
			<< std::endl;


	wasInlierFirst.resize( firstFrame.keypoints.size(), 0);

	// TODO: MOVE IT
	// Removing features without depth
//	std::vector<cv::Point2f>::iterator it2;
//	std::vector<int>::iterator itId;
//	for (it2 = points[1].begin(), itId = pointsID.begin(); it2 != points[1].end();
//			) {
//		uint16_t depthEnd = frameD[2].at<uint16_t>(roundY(it2->y),
//				roundX(it2->x));
//		if (depthEnd == 0) {
//			it2 = points[1].erase(it2);
//			itId = pointsID.erase(itId);
//		} else {
//			++it2;
//			++itId;
//		}
//	}
//	std::cout << "Bez NaN liczba cech wynosi: " << lastFrame.keypoints.size() << " "
//			<< points[1].size() << std::endl;


//	cv::Mat grayImg;
//	cvtColor(frameBGR[0], grayImg, CV_RGB2GRAY);
//	cornerSubPix(grayImg, points[1], cvSize(7, 7), cvSize(-1, -1),
//				cvTermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 4, 0.01));
//	std::cout<< "Po cornerSubPix" << std::endl;

	// Finding matches
	std::vector< std::pair<int,int> > matches;
	for (int i=0;i< firstFrame.keypoints.size(); i++)
	{
		//for (int j=0;j<points[1].size();j++)
		for (int j=0;j<secondFrame.keypoints.size();j++)
		{
			if ( firstFrame.keypointsId[i] == secondFrame.keypointsId[j])
			{
				matches.push_back(std::make_pair(i,j));

				//std::cout<<"Matched : " << points[1][j].x << " " << points[1][j].y << " vs " << lastFrame.keypoints[i].pt.x << " " << lastFrame.keypoints[i].pt.y << std::endl;
			}
		}
	}

	std::cout<< "Liczba wykrytych matchy: " <<matches.size() << std::endl;


	if ( matches.size() < 50 )
	{
		res = Eigen::Matrix4f::Identity();
		return Eigen::Matrix4f::Identity();
	}

//	for (int i=233;i<243;i++)
//	{
//		std::cout<<matches[i].first << " " << matches[i].second << std::endl;
//	}

	// 2D to 3D
//	double X, Y, Z;
//	std::vector<Eigen::Vector3f> point3d;
//	for (int i = 0; i < points[1].size(); i++) {
//
//		cv::Point2f a = points[1][i];
//		Z = double(frameD[2].at<uint16_t>(roundY(a.y), roundX(a.x)))
//				/ depthInvScale;
//		//std::cout<<"Z value : " << Z<< std::endl;
//		// 2d to 3d with removing distortion
//		//std::cout<<cameraParameters.cameraMatrix<<std::endl;
//		point3d.push_back(
//				RGBDclass::point2Dto3D(a, Z, cameraParameters.cameraMatrix,
//						cameraParameters.distCoeffs));
//		// 2d to 3d without removing distortion
//		//point3d[j].push_back ( RGBDclass::simplePoint2Dto3D(a, Z, cameraParameters) );
//
//	}
//	std::cout << "Ended removing distortion" << std::endl;







	// Glowna petla RANSACa
	int iterationNumber = 3000;
	int matchesSize = matches.size();



	// RANSAC preparation
	Eigen::Matrix4f BestTransformation;
	double BestInlierPercentage = -1;
	int BestInlierCount = -1;
	std::vector<int> currentModelIndices, BestModelIndices;
	std::vector<bool> isInlier(matchesSize), isBestInlier;

	std::cout << "Inlier: " << inliner_threshold << std::endl;
	// RANSAC
	for (int i = 0; i < iterationNumber; i++) {
		//std::cout<<"Iteration number : " << i <<std::endl;
		currentModelIndices.clear();
		std::fill(isInlier.begin(), isInlier.end(), 0);

		// subsampling a model
		std::vector<int> validIndex(matchesSize, 1);
		while (currentModelIndices.size() < pair_number) {
			int a = rand() % matchesSize;
			//int a = currentModelIndices.size();
			std::pair<int,int> chosenMatch = matches[a];


			if (validIndex[a] == true) {
				for (int k = 0; k < currentModelIndices.size(); k++) {
					int p = currentModelIndices[k];


					if (((firstFrame.keypoints3d[chosenMatch.first] - firstFrame.keypoints3d[matches[p].first]).norm()
							< inliner_threshold)
							|| ((secondFrame.keypoints3d[chosenMatch.second] - secondFrame.keypoints3d[matches[p].second]).norm()
									< inliner_threshold)) {
						validIndex[a] = false;
						break;
					}


					/*if (((point3d[0][a] - point3d[0][p]).norm()
							< inliner_threshold)
							|| ((point3d[1][a] - point3d[1][p]).norm()
									< inliner_threshold)) {
						validIndex[a] = false;
						break;
					}*/
				}

				if (validIndex[a])
					currentModelIndices.push_back(a);
				validIndex[a] = false;
			}
		}

		//std::cout<<"Chosen matches: " << currentModelIndices[0] <<" " << currentModelIndices[1] << " " << currentModelIndices[2]<<std::endl;

		//std::cout<<"Evaluating model : " << i <<std::endl;
		// Evaluating model: calculate a transformation
		Eigen::MatrixXf P(pair_number, 3), Q(pair_number, 3);

		for (int j = 0; j < pair_number; j++) {
			int p = currentModelIndices[j];
			P.block<1, 3>(j, 0) = firstFrame.keypoints3d[matches[p].first];
			Q.block<1, 3>(j, 0) = secondFrame.keypoints3d[matches[p].second];

//			std::cout << lastFrame.keypoints[matches[p].first].x << " "
//					<< lastFrame.keypoints[matches[p].first].y << " vs "
//					<< currentFrame.keypoints[matches[p].second].x << " "
//					<< currentFrame.keypoints[matches[p].second].y << std::endl;
//
//			std::cout << lastFrame.keypoints3d[matches[p].first][0] << " "
//								<< lastFrame.keypoints3d[matches[p].first][1] << " " << lastFrame.keypoints3d[matches[p].first][2] << " vs "
//								<< currentFrame.keypoints3d[matches[p].second][0] << " "
//								<< currentFrame.keypoints3d[matches[p].second][1] << " " << currentFrame.keypoints3d[matches[p].second][2] << std::endl;

		}

		Eigen::Matrix4f Optimal;
		Kabsch(P, Q, Optimal);

		//std::cout<<"P:\n"<<P<<std::endl;
		//std::cout<<"Q:\n"<<Q<<std::endl;
		//std::cout<<"Optimal:\n"<<Optimal<<std::endl;

		//std::cout<<"counting the quality : " << i <<std::endl;

		// Evaluating the model: counting the quality
		int inlierCount = 0;
		for (int w = 0; w < matchesSize; w++) {
			Eigen::Vector4f punkt;
			punkt.head(3) = firstFrame.keypoints3d[matches[w].first];
			punkt[3] = 1.0;

			// Optimal transformation on points from 1 set
			punkt = Optimal * punkt;

			// Checking if it is inlier

		//	if ( w == currentModelIndices[0] || w == currentModelIndices[1] || w == currentModelIndices[2])
			//	std::cout<< punkt.head(3) << " " <<  point3d[matches[w].second] << std::endl;
		//	std::cout<<"Inlier error :" << (punkt.head(3) - currentFrame.keypoints3d[matches[w].second]).norm()<<std::endl;
			if ((punkt.head(3) - secondFrame.keypoints3d[matches[w].second]).norm() < inliner_threshold) {
				inlierCount++;
				isInlier[w] = true;
			}
		}
		float inlierPercentage = inlierCount * 100.0 / matchesSize;

		// From transfomation to euler/translation
		float eulerAngleCheck[3];
		TransformationToEuler(Optimal, eulerAngleCheck);
		Eigen::Vector3f translationCheck(0, 0, 0);
		translationCheck.head<3>() = Optimal.block<3, 1>(0, 3);


//		std::cout << "Model inlier per: " << inlierPercentage<< std::endl;
//		std::cout<<translationCheck[0]<<" " << translationCheck[1] <<" " <<translationCheck[2] << std::endl;
//		std::cout<<(translationCheck[0] <= constraints.maxTx)
//				<<" " << (translationCheck[0] >= constraints.minTx)
//				<<" " << (translationCheck[1] <= constraints.maxTy)
//				<<" " << (translationCheck[1] >= constraints.minTy)
//				<<" " << (translationCheck[2] <= constraints.maxTz)
//				<<" " << (translationCheck[2] >= constraints.minTz) << std::endl;
		// Check if it is ok
		if (eulerAngleCheck[0] <= constraints.maxObrX
				&& eulerAngleCheck[0] >= constraints.minObrX
				&& eulerAngleCheck[1] <= constraints.maxObrY
				&& eulerAngleCheck[1] >= constraints.minObrY
				&& eulerAngleCheck[2] <= constraints.maxObrZ
				&& eulerAngleCheck[2] >= constraints.minObrZ
				&& translationCheck[0] <= constraints.maxTx
				&& translationCheck[0] >= constraints.minTx
				&& translationCheck[1] <= constraints.maxTy
				&& translationCheck[1] >= constraints.minTy
				&& translationCheck[2] <= constraints.maxTz
				&& translationCheck[2] >= constraints.minTz) {

			// Save the model if it is the best one
			if (inlierPercentage > BestInlierPercentage) {
				BestTransformation = Optimal;
				BestInlierPercentage = inlierPercentage;
				BestInlierCount = inlierCount;
				BestModelIndices.clear();
				isBestInlier = isInlier;
				BestModelIndices = currentModelIndices;
			}
		}

		if (BestInlierPercentage > ransac_break_percent) {
			break;
		}
	}

	std::cout << std::endl << "Liczba Dopasowanych [%]: "
			<< BestInlierPercentage << std::endl;
	Eigen::Matrix4f Optimal;
	if ( BestInlierPercentage < 5 )
	{
		Optimal = Eigen::Matrix4f::Identity();
	}
	else
	{
//		for (int i = 0; i < pointsTrackedSize; i++) {
//			if (isBestInlier[i] == true) {
//				g2o << "EDGE_SE3:QUAT " << vertexCounter << " " << pointsID[matches[i].second]
//						<< " " << point3d[matches[i].second][0] << " "
//						<< point3d[matches[i].second][1] << " "
//						<< point3d[matches[i].second][2] << " 0 0 0 1"
//						<< " 0.01 0 0 0 0 0 0.01 0 0 0 0 0.01 0 0 0 0.000 0 0 0.000 0 0.000"
//						<< std::endl;
//			}
//
//		}


		int prevIle;
		for (int k = 0; k < 10; k++) {
			// Again, reestimation from inliers
			Eigen::MatrixXf P(BestInlierCount, 3), Q(BestInlierCount, 3);
			for (int i = 0, k = 0; i < matchesSize; i++) {
				if (isBestInlier[i] == true) {
					P.block<1, 3>(k, 0) = firstFrame.keypoints3d[matches[i].first];
					Q.block<1, 3>(k, 0) = secondFrame.keypoints3d[matches[i].second];

					wasInlierFirst[matches[i].first] = true;
					wasInlierSecond[matches[i].second] = true;
					k++;
				}
			}
			std::cout << "Reestimation" << std::endl;
			Kabsch(P, Q, Optimal);

			///!!!!!
			int ile = 0;
			for (int i = 0, k = 0; i < matchesSize; i++) {
				if (isBestInlier[i] == true) {

					Eigen::Vector4f punkt;
					punkt.head(3) = firstFrame.keypoints3d[matches[i].first];
					punkt[3] = 1.0;

					// Optimal transformation on points from 1 set
					punkt = Optimal * punkt;

					// Checking if it is inlier
					//std::cout<<"Inlier error :" << (punkt.head(3) - point3d[1][w]).norm()<<std::endl;
					if ((punkt.head(3) - secondFrame.keypoints3d[matches[i].second]).norm()
							> inliner_threshold) {
						isBestInlier[i] = false;
						ile++;
					}
				}
			}
			if (prevIle == ile) {
				inliner_threshold = 0.9 * inliner_threshold;
			}
			prevIle = ile;
			if (ile < 30)
				break;
		}
	}

	std::cout << std::endl << "TRANSFORMACJA z inlierow :" << std::endl
			<< Optimal << std::endl;

	res = Optimal;
	return Optimal;
}


void Track::createVertex(ProgramParameters programParameters, CalibrationParameters calibrationParams, double depthInvScale, int frameId)
{
//		std::cout << "Starting subpixel detection" << std::endl;
//		cv::Mat grayImg;
//		cvtColor(frameBGR[0], grayImg, CV_RGB2GRAY);
//		cornerSubPix(grayImg, points[0], cvSize(7, 7), cvSize(-1, -1),
//				cvTermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 4, 0.01));

		float data[5] = { calibrationParams.k1, calibrationParams.k2,
				calibrationParams.p1, calibrationParams.p2, calibrationParams.k3 };
		calibrationParams.distCoeffs = cv::Mat(1, 5, CV_32FC1, &data);
		float cm[3][3] = { { calibrationParams.fu, 0, calibrationParams.u0 }, { 0,
				calibrationParams.fv, calibrationParams.v0 }, { 0, 0, 1 } };
		calibrationParams.cameraMatrix = cv::Mat(3, 3, CV_32FC1, &cm);


		kabschVertex kbVertex;
		//kbVertex.vertexId = vertexCounter;
		kbVertex.vertexId = frameId;
		int ii = 0;
		for (std::vector<cv::Point2f>::iterator it = points[1].begin();
				it != points[1].end(); it++, ii++) {

			double Z =  double(frameD[2].at<uint16_t>(roundY(it->y), roundX(it->x)))
						/ depthInvScale;;
			if ( Z != 0 )
			{
				kbVertex.keypointsId.push_back( pointsID[ii] );
				kbVertex.keypoints.push_back( *it);
				kbVertex.keypoints3d.push_back( RGBDclass::point2Dto3D(*it, Z, calibrationParams.cameraMatrix,
								calibrationParams.distCoeffs) );
			}
		}
		kbVertex.inGraph.resize(kbVertex.keypoints.size());

		vertexHistory.push_back(kbVertex);
		if ( vertexHistory.size() > 5 ) {
			vertexHistory.erase(vertexHistory.begin());
		}

		wasInlierSecond.clear();
		wasInlierSecond.resize(kbVertex.keypoints.size(), 0);
}

