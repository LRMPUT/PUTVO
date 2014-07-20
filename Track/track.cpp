#include "track.h"
#include <pcl/common/time.h>

Track::Track() {
//	for (int i = 0; i < 3; i++) {
//		frameD[i] = cv::Mat(480, 640, CV_32F);
//	}
	featureCounter = 5000;
	vertexCounter = 0;

	detectionTime = 0;
	descriptionTime = 0;
	trackingTime = 0;
	matchingTime = 0;
	measurementCounter = 0;
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
		cv::Mat depth_image, int fast_threshold, float flatness_threshold,
		int kickThreshold) {
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
	gettimeofday(&startT, 0);

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
	gettimeofday(&endT, 0);
	//std::cout << "Czas testu: " << 1000000 * (endT.tv_sec - startT.tv_sec) + (endT.tv_usec - startT.tv_usec) <<std::endl;
	//std::cout << "Filtered keypoints: " << filtered_keypoints.size()
	//		<< std::endl;
	return filtered_keypoints;

}

void Track::newDetection(ProgramParameters programParameters,
		CalibrationParameters calibrationParams, double depthInvScale,
		long int frameId) {

	std::vector<cv::KeyPoint> keypoints;

	// OLD detector
	if (programParameters.newDetector == 0) {

		// STRIPES
		if (programParameters.detectionInStripes == 1) {

			cv::Ptr<cv::FeatureDetector> detector(
					new cv::DynamicAdaptedFeatureDetector(
							new cv::FastAdjuster(2, true),
							(programParameters.TrackingFeatureNumber + 100) / 6,
							(programParameters.TrackingFeatureNumber + 200)
									/ 6));
//			cv::FastFeatureDetector detector(1);
			for (int i = 0; i < 6; i++) {
				std::vector<cv::KeyPoint> keypointsInROI;
				cv::Mat roi(frameBGR[0], cv::Rect(0, i * 80, 640, 80));
				detector->detect(roi, keypointsInROI);

				sort(keypointsInROI.begin(), keypointsInROI.end(),
						keypointResponseCompare);

				for (int j = 0;
						j < programParameters.TrackingFeatureNumber * 2.0 / 6
								&& j < keypointsInROI.size(); j++) {
					keypointsInROI[j].pt.y += i * 80;
					keypoints.push_back(keypointsInROI[j]);
				}
			}
		}
		// NO STRIPES
		else {
			cv::FastFeatureDetector fastDetector;
			fastDetector.detect(frameBGR[0], keypoints);
		}
	}
	// New detector
	else {
		if (programParameters.detectionInStripes == 0) {
			keypoints = detect_rgbd_features(frameBGR[0], frameD[0],
					programParameters.newDetectorFastThreshold,
					programParameters.newDetectorDepthTestThreshold,
					programParameters.newDetectorKickThreshold);
		} else {
			for (int i = 0; i < 6; i++) {
				std::vector<cv::KeyPoint> keypointsInROI;
				cv::Mat roiBGR(frameBGR[0], cv::Rect(0, i * 80, 640, 80));
				cv::Mat roiD(frameD[0], cv::Rect(0, i * 80, 640, 80));
				keypointsInROI = detect_rgbd_features(roiBGR, roiD,
						programParameters.newDetectorFastThreshold,
						programParameters.newDetectorDepthTestThreshold,
						programParameters.newDetectorKickThreshold);

				sort(keypointsInROI.begin(), keypointsInROI.end(),
						keypointResponseCompare);
				for (int j = 0; j < keypointsInROI.size(); j++) {
					if (j > programParameters.TrackingFeatureNumber / 6)
						break;
					keypointsInROI[j].pt.y += i * 80;
					keypoints.push_back(keypointsInROI[j]);
				}
			}
		}

	}

	// If we can use DBScan
	if (programParameters.DBScan == 1) {
		std::vector<int> clusters;
		DBScan dbscan(programParameters.DBScanEps,
				programParameters.DBScanMinPts);
		dbscan.run(keypoints, clusters);

		// Choose only the best candidate from each group
		std::vector<std::pair<cv::KeyPoint, int> > keypointClusterSet;
		for (int i = 0; i < keypoints.size(); i++)
			keypointClusterSet.push_back(
					std::make_pair(keypoints[i], clusters[i]));
		sort(keypointClusterSet.begin(), keypointClusterSet.end(),
				keypointClassResponseCompare);
		keypoints.clear();

		// Number of detected clusters
		int clustersNumber = 0;
		for (int i = 0; i < clusters.size(); i++) {
			clustersNumber = std::max(clustersNumber, clusters[i]);
		}
		clustersNumber++;

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
		//	std::cout << "Number of features AFTER dbscan:" << keypoints.size()
	}
	if (programParameters.verbose != 0) {
		printf("Detected %lu points\n", keypoints.size());
	}
	/// Copying previous inliers
	points[0].clear();
	points[2].clear();

	std::vector<cv::Point2f>::iterator it = points[1].begin();
	std::vector<int>::iterator itDepth = pointDepth.begin(), itID =
			pointsID.begin();
	if (programParameters.verbose != 0) {
		printf("There was %lu (%lu)\n", wasInlierSecond.size(),
				points[1].size());
	}
	int featuresToCpy = 0;
	for (int i = 0; i < wasInlierSecond.size(); i++) {
		if (!wasInlierSecond[i]
				|| featuresToCpy
						> 0.5 * programParameters.TrackingFeatureNumber) {
			it = points[1].erase(it);
			itDepth = pointDepth.erase(itDepth);
			itID = pointsID.erase(itID);
		} else {
			++it, ++itDepth, ++itID;
			featuresToCpy++;
		}
	}
	if (programParameters.verbose != 0) {
		printf("There is %lu \n", points[1].size());
	}
	wasInlierSecond.clear();

	/// Sorting and choosing the strongest
	sort(keypoints.begin(), keypoints.end(), keypointResponseCompare);
	for (int i = 0;
			i < keypoints.size() && i < programParameters.TrackingFeatureNumber;
			i++)
		points[0].push_back(keypoints[i].pt);

	if (points[0].size() > 0) {
		cv::Mat grayImg;
		cvtColor(frameBGR[0], grayImg, CV_RGB2GRAY);
		cornerSubPix(grayImg, points[0], cvSize(7, 7), cvSize(-1, -1),
				cvTermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 4, 0.01));
	} else {
		std::cout << "No points in starting image?" << std::endl;
	}

	/// Creating a vertex out of data
	float data[5] = { calibrationParams.k1, calibrationParams.k2,
			calibrationParams.p1, calibrationParams.p2, calibrationParams.k3 };
	calibrationParams.distCoeffs = cv::Mat(1, 5, CV_32FC1, &data);
	float cm[3][3] = { { calibrationParams.fu, 0, calibrationParams.u0 }, { 0,
			calibrationParams.fv, calibrationParams.v0 }, { 0, 0, 1 } };
	calibrationParams.cameraMatrix = cv::Mat(3, 3, CV_32FC1, &cm);

	kabschVertex kbVertex;
	kbVertex.vertexId = frameId;
	kbVertex.imageRGB = frameBGR[0].clone();
	kbVertex.imageD = frameD[0].clone();
	int ii = 0;
	for (std::vector<cv::Point2f>::iterator it = points[1].begin();
			it != points[1].end(); it++, ii++) {
		kbVertex.keypointsId.push_back(pointsID[ii]);
		kbVertex.keypoints.push_back(*it);
		double Z = pointDepth[ii];
		kbVertex.keypoints3d.push_back(
				RGBDclass::point2Dto3D(*it, Z, calibrationParams.cameraMatrix,
						calibrationParams.distCoeffs));
		kbVertex.informationMatrix.push_back(
				getInformationMatrix(it->x, it->y, Z));
	}

	int ile = 0;
	for (std::vector<cv::Point2f>::iterator it = points[0].begin();
			it != points[0].end(); ii++) {

		double Z = double(frameD[0].at<uint16_t>(roundY(it->y), roundX(it->x)))
				/ depthInvScale;
		if (fabs(Z) > 0.001) {
			pointsID.push_back(featureCounter);

			pointDepth.push_back(Z);

			kbVertex.keypointsId.push_back(featureCounter);
			kbVertex.keypoints.push_back(keypoints[ii].pt);
			kbVertex.keypoints3d.push_back(
					RGBDclass::point2Dto3D(*it, Z,
							calibrationParams.cameraMatrix,
							calibrationParams.distCoeffs));
			kbVertex.informationMatrix.push_back(
					getInformationMatrix(it->x, it->y, Z));

			featureCounter++;
			points[1].push_back(*it);
			it++;
		} else {
			ile++;
			it++;	// = points[0].erase(it);
//			std::cout<<"Depth issue " << roundY(it->y)<< " " << roundX(it->x)<< " " << Z << "  " << fabs(Z) << std::endl;
		}
	}
	if (programParameters.verbose != 0) {
		printf("Point after combining %lu | wo depth: %d\n", points[1].size(),
				ile);
	}
	kbVertex.inGraph.resize(kbVertex.keypoints.size(), 0);

	// Indirect copy
	frameBGR[0].copyTo(frameBGR[1]);

	// Temporal vertex out
	if (vertexHistory.size() > 0) {
		kbVertex.matchingKeypoints3d =
				vertexHistory[vertexHistory.size() - 1].matchingKeypoints3d;
		kbVertex.absolutePosition =
				vertexHistory[vertexHistory.size() - 1].absolutePosition;
		vertexHistory.pop_back();
	}
	vertexHistory.push_back(kbVertex);

	if (vertexHistory.size() > programParameters.historySize) {
		vertexHistory.erase(vertexHistory.begin());
	}

}

void Track::newDetectionMatching(ProgramParameters programParameters,
		CalibrationParameters calibrationParams, double depthInvScale,
		long int frameId) {

	kabschVertex kbVertex;
	kbVertex.vertexId = frameId;
	kbVertex.imageRGB = frameBGR[0].clone();
	kbVertex.imageD = frameD[0].clone();

	// Indirect copy
	frameBGR[0].copyTo(frameBGR[1]);

	// Temporal vertex out
	if (vertexHistory.size() > 0) {
		kbVertex.matchingKeypoints3d =
				vertexHistory[vertexHistory.size() - 1].matchingKeypoints3d;
		kbVertex.descriptors =
				vertexHistory[vertexHistory.size() - 1].descriptors.clone();
		kbVertex.absolutePosition =
				vertexHistory[vertexHistory.size() - 1].absolutePosition;
		vertexHistory.pop_back();
	}
	vertexHistory.push_back(kbVertex);

	if (vertexHistory.size() > programParameters.historySize) {
		vertexHistory.erase(vertexHistory.begin());
	}

}
/// TRACKing
void Track::doTracking(ProgramParameters programParameters,
		double depthInvScale) {

	std::vector<uchar> status;
	std::vector<float> err;
	cv::TermCriteria termcrit(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 15, 0.05);

	performedTrack = true;

	// Calculating movement of features
	double startTime = pcl::getTime();
	cv::calcOpticalFlowPyrLK(frameBGR[1], frameBGR[2], points[1], points[2],
			status, err,
			cvSize(programParameters.winSize, programParameters.winSize),
			programParameters.maxLevels, termcrit);
	double endTime = pcl::getTime();
	trackingTime += (endTime - startTime);

	measurementCounter++;

	int i = 0;

//	std::cout<<"Tracking sizes pre removal: " << points[1].size()<<" " <<  points[2].size() << " " << pointsID.size() << " " << pointDepth.size()<< std::endl;

	std::vector<int>::iterator depthIt = pointDepth.begin(), IdIt =
			pointsID.begin();
	std::vector<cv::Point2f>::iterator it = points[2].begin();
	for (; it != points[2].end(); i++) {
		if (status[i] == 0) {
			depthIt = pointDepth.erase(depthIt);
			IdIt = pointsID.erase(IdIt);
			it = points[2].erase(it);
			//	std::cout<<"Status["<<i<<"] = 0" << std::endl;
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

int Track::roundSize(double Y, int size) {
	if (Y < 0)
		Y = 0;
	else if (Y > size - 1)
		Y = size;
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

bool Track::robustTransformationEstimation(
		const std::vector<std::pair<int, int> >& matches, int pair_number,
		const std::vector<Eigen::Vector3f> firstKeypoints3d,
		double inliner_threshold,
		const std::vector<Eigen::Vector3f> secondKeypoints3d,
		const Constraints& constraints, double ransac_break_percent,
		Eigen::Matrix4f &Optimal, bool saveInliers, bool turnOffConstrains, std::vector<bool> &isBestInlier) {

	// RANSAC preparation
	int iterationNumber = computeRANSACIterationCount(0.99, 0.10, pair_number);
	int maxIterationNumber = iterationNumber;
	int matchesSize = matches.size();
	int BestInlierCount = -1;
	double BestInlierPercentage = -1;
	Eigen::Matrix4f BestTransformation;

	//std::vector<int> currentModelIndices, BestModelIndices;
	std::vector<bool> isInlier(matchesSize);
	std::vector<int> currentModelIndices, BestModelIndices;

	// RANSAC loop
	for (int i = 0; i < iterationNumber; i++) {

		currentModelIndices.clear();
		std::fill(isInlier.begin(), isInlier.end(), 0);
		// subsampling a model
		std::vector<int> validIndex(matchesSize, 1);

		// Choosing needed pairs to compute transformation
		int deadlockCounter = 0;
		while (currentModelIndices.size() < pair_number) {
			int a = rand() % matchesSize;
			//int a = currentModelIndices.size();
			std::pair<int, int> chosenMatch = matches[a];
			if (validIndex[a] == true) {
				for (int k = 0; k < currentModelIndices.size(); k++) {
					int p = currentModelIndices[k];
					if (((firstKeypoints3d[chosenMatch.first]
							- firstKeypoints3d[matches[p].first]).norm()
							< inliner_threshold)
							|| ((secondKeypoints3d[chosenMatch.second]
									- secondKeypoints3d[matches[p].second]).norm()
									< inliner_threshold)) {
						validIndex[a] = false;
						break;
					}
				}
				if (validIndex[a])
					currentModelIndices.push_back(a);

				validIndex[a] = false;
			}
			deadlockCounter++;

			if (deadlockCounter > pair_number * 25) {
				Optimal = Eigen::Matrix4f::Identity();
				return false;
			}
		}

		// Evaluating model: calculate a transformation
		Eigen::MatrixXf P(pair_number, 3), Q(pair_number, 3);
		for (int j = 0; j < pair_number; j++) {
			int p = currentModelIndices[j];
			P.block<1, 3>(j, 0) = firstKeypoints3d[matches[p].first];
			Q.block<1, 3>(j, 0) = secondKeypoints3d[matches[p].second];
		}
		Eigen::Matrix4f Optimal;
//		Kabsch(P, Q, Optimal);
		Umeyama(P, Q, Optimal);

		// Evaluating the model: counting the quality
		int inlierCount = 0;
		for (int w = 0; w < matchesSize; w++) {
			Eigen::Vector4f punkt;
			punkt.head(3) = firstKeypoints3d[matches[w].first];
			punkt[3] = 1.0;
			// Optimal transformation on points from 1 set
			punkt = Optimal * punkt;
			// Checking if it is inlier
			if ((punkt.head(3) - secondKeypoints3d[matches[w].second]).norm()
					< inliner_threshold) {
				inlierCount++;
				isInlier[w] = true;
			}
		}
		float inlierPercentage = inlierCount * 100.0 / matchesSize;

//		std::cout << "Liczba Dopasowanych [%]: "
//						<< inlierPercentage << std::endl;

		// From transfomation to euler/translation
		float eulerAngleCheck[3];
		TransformationToEuler(Optimal, eulerAngleCheck);
		Eigen::Vector3f translationCheck(0, 0, 0);
		translationCheck.head<3>() = Optimal.block<3, 1>(0, 3);

		// Check if it is ok
		if ((eulerAngleCheck[0] <= constraints.maxObrX
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
				&& translationCheck[2] >= constraints.minTz)
				|| turnOffConstrains) {
			// Save the model if it is the best one
			if (inlierPercentage > BestInlierPercentage
					&& (Optimal.block<3, 3>(0, 0)).determinant() > 0.1) {
				BestTransformation = Optimal;
				BestInlierPercentage = inlierPercentage;
				BestInlierCount = inlierCount;
				BestModelIndices.clear();
				isBestInlier = isInlier;
				BestModelIndices = currentModelIndices;
				iterationNumber = std::min(
						computeRANSACIterationCount(0.99,
								BestInlierPercentage / 100, pair_number),
						maxIterationNumber);
			}
		}
		if (BestInlierPercentage > ransac_break_percent) {
			break;
		}
	}
	//if (programParameters.verbose != 0) {
		std::cout << std::endl << "Best Liczba Dopasowanych [%]: "
				<< BestInlierPercentage << std::endl;
	//}
	if (BestInlierPercentage < 15) {
		Optimal = Eigen::Matrix4f::Identity();
		return false;
	} else if (turnOffConstrains && BestInlierPercentage < 25) {
		Optimal = Eigen::Matrix4f::Identity();
		return false;
	} else {

		int prevIle;
		for (int k = 0; k < 10; k++) {
			// Again, reestimation from inliers
			Eigen::MatrixXf P(BestInlierCount, 3), Q(BestInlierCount, 3);
			int w = 0;
			for (int i = 0; i < matchesSize; i++) {
				if (isBestInlier[i] == true) {
					P.block<1, 3>(w, 0) = firstKeypoints3d[matches[i].first];
					Q.block<1, 3>(w, 0) = secondKeypoints3d[matches[i].second];

					if (saveInliers) {
						wasInlierFirst[matches[i].first] = true;
						wasInlierSecond[matches[i].second] = true;
					}
					w++;
				}
			}
//			if (programParameters.verbose != 0) {
//				std::cout << "Reestimation" << std::endl;
//			}
//			Kabsch(P, Q, Optimal);
			double st = pcl::getTime();
			Umeyama(P, Q, Optimal);
			double en = pcl::getTime();
			std::cout<<"Reestymacja zajela " << (en - st)*1224 << " dla " << w << " inlierów " << std::endl;
			///!!!!!
			int ile = 0;
			for (int i = 0; i < matchesSize; i++) {
				if (isBestInlier[i] == true) {
					Eigen::Vector4f punkt;
					punkt.head(3) = firstKeypoints3d[matches[i].first];
					punkt[3] = 1.0;
					punkt = Optimal * punkt;
					// Checking if it is inlier
					//std::cout<<"Inlier error :" << (punkt.head(3) - point3d[1][w]).norm()<<std::endl;
					if ((punkt.head(3) - secondKeypoints3d[matches[i].second]).norm()
							> inliner_threshold) {
						isBestInlier[i] = false;
					}
					else
					{
						ile++;
					}
				}
			}
			if (prevIle == ile) {
				inliner_threshold = 0.9 * inliner_threshold;
			}
			prevIle = ile;
			if (ile < 50)
				break;
		}
	}
	return true;
}

bool Track::estimateTransformation(kabschVertex *firstFrame,
		kabschVertex *secondFrame, ProgramParameters programParameters,
		Constraints constraints, CalibrationParameters cameraParameters,
		double depthInvScale, Eigen::Matrix4f &res, bool matching,
		int numberOfFeatures, bool saveInliers, bool turnOffConstrains) {

	float data[5] = { cameraParameters.k1, cameraParameters.k2,
			cameraParameters.p1, cameraParameters.p2, cameraParameters.k3 };
	cameraParameters.distCoeffs = cv::Mat(1, 5, CV_32FC1, &data);
	float cm[3][3] = { { cameraParameters.fu, 0, cameraParameters.u0 }, { 0,
			cameraParameters.fv, cameraParameters.v0 }, { 0, 0, 1 } };
	cameraParameters.cameraMatrix = cv::Mat(3, 3, CV_32FC1, &cm);

	wasInlierFirst.resize(firstFrame->keypoints.size(), 0);

	// Parameters to fill for track/match
	std::vector<cv::KeyPoint> keypoints[2];
	std::vector<Eigen::Vector3f> keypointsToPass[2];
	std::vector<std::pair<int, int> > matches;

	if (matching == false) {
		// Finding matches
		std::vector<cv::DMatch> matches2Draw;
		for (int i = 0; i < firstFrame->keypoints.size(); i++) {
			for (int j = 0; j < secondFrame->keypoints.size(); j++) {
				if (firstFrame->keypointsId[i] == secondFrame->keypointsId[j]) {
					matches.push_back(std::make_pair(i, j));
					cv::DMatch *x = new cv::DMatch(i, j, 0);
					matches2Draw.push_back(*x);
				}
			}
		}
		if (programParameters.verbose != 0) {
			std::cout << "Number of matches: " << matches2Draw.size()
					<< std::endl;
		}
		if (programParameters.showTracking == 1) {
			std::vector<cv::KeyPoint> x1, x2;
			cv::KeyPoint::convert(firstFrame->keypoints, x1);
			cv::KeyPoint::convert(secondFrame->keypoints, x2);

			cv::Mat img_matches;
			cv::drawMatches(firstFrame->imageRGB, x1, secondFrame->imageRGB, x2,
					matches2Draw, img_matches);
			cv::Mat imageShow;
			cv::resize(img_matches, imageShow, cv::Size(1280, 480));

			cv::Mat img_matches2,tmpx,tmpy;
			firstFrame->imageD.copyTo(tmpx);
			secondFrame->imageD.copyTo(tmpy);
			tmpx = tmpx * 50; tmpy = tmpy * 50;
			tmpx.convertTo(tmpx, CV_8U);
			tmpy.convertTo(tmpy, CV_8U);
			cv::drawMatches(tmpx, x1, tmpy, x2,
				matches2Draw, img_matches2);
			cv::Mat imageShow2;
			cv::resize(img_matches2, imageShow2, cv::Size(1280, 480));

			cv::imshow("matches", imageShow);
			cv::imshow("matches2", imageShow2);
			cv::waitKey(0);
		}
	} else {

		if (programParameters.verbose != 0) {
			std::cout << "Matching activated with feature count :"
					<< numberOfFeatures << std::endl;
		}
		// PARAMETERS
		enum detectorType {
			FAST, SURF, ORB
		};

		cv::FeatureDetector *featureDetector;
		cv::DescriptorExtractor * extractor;
		if (programParameters.matchingType == FAST) {
			featureDetector = new cv::FastFeatureDetector();
			extractor = new cv::BriefDescriptorExtractor();
		} else if (programParameters.matchingType == SURF) {
			featureDetector = new cv::SurfFeatureDetector();
			extractor = new cv::SurfDescriptorExtractor();
		} else if (programParameters.matchingType == ORB) {
			featureDetector = new cv::ORB();
			extractor = new cv::ORB;
		} else {
			printf("ERROR ! \n");
			exit(0);
		}

		cv::Mat objectDescriptors[2];

		for (int k = 0; k < 2; k++) {

			cv::Mat rgbImg, depthImg;
			if (k == 0) {

				//printf("First frame size: %lu %d %d\n",firstFrame->matchingKeypoints3d.size(), firstFrame->descriptors.rows, firstFrame->descriptors.cols  );

			   if (programParameters.g2o_vs_sba == false && firstFrame->matchingKeypoints3d.size() > 0)
					continue;
				rgbImg = firstFrame->imageRGB;
				depthImg = firstFrame->imageD;
			} else {
//				printf("Second frame size: %lu %d %d\n",secondFrame->matchingKeypoints3d.size(), secondFrame->descriptors.rows, secondFrame->descriptors.cols    );
				if (programParameters.g2o_vs_sba == false && secondFrame->matchingKeypoints3d.size() > 0)
					continue;
				rgbImg = secondFrame->imageRGB;
				depthImg = secondFrame->imageD;
			}

			featureDetector->detect(rgbImg, keypoints[k]);
//			std::cout<<"Features found " << keypoints[k].size() << std::endl;
			if (keypoints[k].size() == 0) {
				res = Eigen::Matrix4f::Identity();
				return false;
			}

			keypoints[k].resize(numberOfFeatures);

			cv::Mat grayImg;
			cvtColor(rgbImg, grayImg, CV_RGB2GRAY);
			std::vector<cv::Point2f> points2d;
			cv::KeyPoint::convert(keypoints[k], points2d);

//			cornerSubPix(grayImg, points2d, cvSize(7, 7), cvSize(-1, -1),
//					cvTermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 4,
//							0.01));

			int jj = 0;
			for (std::vector<cv::KeyPoint>::iterator it = keypoints[k].begin();
					it != keypoints[k].end(); jj++) {
//				double Z = double(
//						depthImg.at<uint16_t>(roundY(it->pt.y),
//								roundX(it->pt.x))) / depthInvScale;

				double Z = double(depthImg.at<uint16_t>(roundSize(it->pt.y, depthImg.rows),
												roundSize(it->pt.x, depthImg.cols))) / depthInvScale;

				if (fabs(Z) > 0.001) {
					Eigen::Vector3f x = RGBDclass::point2Dto3D(points2d[jj], Z,
							cameraParameters.cameraMatrix,
							cameraParameters.distCoeffs);

					keypointsToPass[k].push_back(x);
					++it;
				} else {
					it = keypoints[k].erase(it);
				}
			}
			extractor->compute(rgbImg, keypoints[k], objectDescriptors[k]);

			if (k == 0) {
				firstFrame->matchingKeypoints3d.swap(keypointsToPass[0]);
				firstFrame->descriptors = objectDescriptors[0].clone();
			} else {
				secondFrame->matchingKeypoints3d.swap(keypointsToPass[1]);
				secondFrame->descriptors = objectDescriptors[1].clone();
			}
		}

		delete featureDetector;
		delete extractor;

		cv::BFMatcher *matcher;
		if (programParameters.matchingType == FAST
				|| programParameters.matchingType == ORB) {
			matcher = new cv::BFMatcher(cv::NORM_HAMMING, true);
		} else {
			matcher = new cv::BFMatcher(cv::NORM_L2, true);
		}

		if (firstFrame->matchingKeypoints3d.size() == 0
				|| secondFrame->matchingKeypoints3d.size() == 0) {
			res = Eigen::Matrix4f::Identity();
			return false;
		}
		std::vector<cv::DMatch> matchingMatch;
		if (programParameters.verbose != 0) {
			std::cout << "Matching : " << firstFrame->descriptors.rows << " "
					<< firstFrame->descriptors.cols << " "
					<< secondFrame->descriptors.rows << " "
					<< secondFrame->descriptors.cols << std::endl;
			std::cout << "Matching keypoints: " << keypoints[0].size() << " "
								<< keypoints[1].size() << std::endl;
		}
		matcher->match(firstFrame->descriptors, secondFrame->descriptors,
				matchingMatch);

		delete matcher;
		//std::cout<<"Matching : " << matchingMatch.size() << std::endl;

//		cv::namedWindow("matches", 1);
//		cv::Mat img_matches;
//		cv::drawMatches(firstFrame.imageRGB, keypoints[0], secondFrame.imageRGB, keypoints[1], matchingMatch, img_matches);
//		imshow("matches", img_matches);
//		cv::waitKey(0);

		for (int i = 0; i < matchingMatch.size(); i++) {
			matches.push_back(
					std::make_pair(matchingMatch[i].queryIdx,
							matchingMatch[i].trainIdx));
		}

	}

	if (matches.size() < 10) {
		res = Eigen::Matrix4f::Identity();
		return false;
	}

	// Glowna petla RANSACa
	bool transformationFound = false;
	std::vector<bool> isBestInlier;
	if (matching == false) {
		transformationFound = robustTransformationEstimation(matches,
				programParameters.transformationPairNumber,
				firstFrame->keypoints3d, programParameters.inlierThreshold,
				secondFrame->keypoints3d, constraints,
				programParameters.wantedInlierRatio, res, saveInliers,
				turnOffConstrains, isBestInlier);
	} else {
		transformationFound = robustTransformationEstimation(matches,
				programParameters.transformationPairNumber,
				firstFrame->matchingKeypoints3d,
				programParameters.inlierThreshold,
				secondFrame->matchingKeypoints3d, constraints,
				programParameters.wantedInlierRatio, res, saveInliers,
				turnOffConstrains, isBestInlier);
	}


	if (transformationFound == true && programParameters.g2o_vs_sba == true) {
		std::ofstream zapis;
		char name[40];
		sprintf(name, "g2osba_%04d_%04d.log", int(firstFrame->vertexId),
				int(secondFrame->vertexId));
		zapis.open(name);
		if (matching == true) {

			for (int i = 0, k = 0; i < matches.size(); i++) {
				if (isBestInlier[i] == true) {

					int id = matches[i].first, id2 = matches[i].second;
					//std::cout<<"ids : " << id << " " << id2 << std::endl;
//					std::cout << keypoints[0].size() << " "
//							<< firstFrame->matchingKeypoints3d.size() << " "
//							<< keypoints[1].size() << " "
//							<< secondFrame->matchingKeypoints3d.size() << " "
//							<< std::endl;

					zapis << "-1 " << keypoints[0][id].pt.x << " "
							<< keypoints[0][id].pt.y << " "
							<< firstFrame->matchingKeypoints3d[id](0) << " "
							<< firstFrame->matchingKeypoints3d[id](1) << " "
							<< firstFrame->matchingKeypoints3d[id](2);
					zapis << " -1 " << keypoints[1][id2].pt.x << " "
							<< keypoints[1][id2].pt.y << " "
							<< secondFrame->matchingKeypoints3d[id2](0) << " "
							<< secondFrame->matchingKeypoints3d[id2](1) << " "
							<< secondFrame->matchingKeypoints3d[id2](2);
					zapis << std::endl;

					Eigen::Matrix3f infMat = getInformationMatrix(
							keypoints[0][id].pt.x, keypoints[0][id].pt.y,
							firstFrame->matchingKeypoints3d[id](2));

					zapis << infMat(0, 0) << " " << infMat(0, 1) << " "
							<< infMat(0, 2) << " " << infMat(1, 1) << " "
							<< infMat(1, 2) << " " << infMat(2, 2) << " ";

					infMat = getInformationMatrix(keypoints[1][id2].pt.x,
							keypoints[1][id2].pt.y,
							secondFrame->matchingKeypoints3d[id2](2));

					zapis << infMat(0, 0) << " " << infMat(0, 1) << " "
							<< infMat(0, 2) << " " << infMat(1, 1) << " "
							<< infMat(1, 2) << " " << infMat(2, 2) << " ";
					zapis << std::endl;

				}
			}
		} else {
			for (int i = 0, k = 0; i < matches.size(); i++) {
				if (isBestInlier[i] == true) {
					int id = matches[i].first, id2 = matches[i].second;
//					std::cout<<"ids : " << id << " " << id2 << std::endl;
//					std::cout << "key id : " << firstFrame->keypointsId[id]
//							<< " " << secondFrame->keypointsId[id] << std::endl;
//					std::cout << firstFrame->keypoints.size() << " "
//							<< firstFrame->matchingKeypoints3d.size() << " "
//							<<secondFrame->keypoints.size() << " "
//							<< secondFrame->matchingKeypoints3d.size() << " "
//							<< std::endl;


					zapis << firstFrame->keypointsId[id] << " "
							<< firstFrame->keypoints[id].x << " "
							<< firstFrame->keypoints[id].y << " "
							<< firstFrame->keypoints3d[id](0) << " "
							<< firstFrame->keypoints3d[id](1) << " "
							<< firstFrame->keypoints3d[id](2) << " ";
					zapis << secondFrame->keypointsId[id2] << " "
							<< secondFrame->keypoints[id2].x << " "
							<< secondFrame->keypoints[id2].y << " "
							<< secondFrame->keypoints3d[id2](0) << " "
							<< secondFrame->keypoints3d[id2](1) << " "
							<< secondFrame->keypoints3d[id2](2);
					zapis << std::endl;

					Eigen::Matrix3f infMat = firstFrame->informationMatrix[id];

					zapis << infMat(0, 0) << " " << infMat(0, 1) << " "
							<< infMat(0, 2) << " " << infMat(1, 1) << " "
							<< infMat(1, 2) << " " << infMat(2, 2) << " ";

					infMat = secondFrame->informationMatrix[id2];

					zapis << infMat(0, 0) << " " << infMat(0, 1) << " "
							<< infMat(0, 2) << " " << infMat(1, 1) << " "
							<< infMat(1, 2) << " " << infMat(2, 2) << " ";
					zapis << std::endl;

				}
			}
		}
		zapis << std::endl;
		zapis.close();
	}




if (programParameters.verbose != 0) {
	std::cout << "Transformation found: " << transformationFound << std::endl;
}
//	if (matching == true)
//	{
//		wasInlierFirst.clear();
//		wasInlierSecond.clear();
//	}

return transformationFound;
}

void Track::createVertex(ProgramParameters programParameters,
	CalibrationParameters calibrationParams, double depthInvScale,
	long int frameId, cv::Mat image) {
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
kbVertex.imageRGB = image.clone();
kbVertex.imageD = frameD[2].clone();
int ii = 0;
std::vector<cv::Point2f>::iterator it = points[1].begin();
std::vector<int>::iterator itDepth = pointDepth.begin(), itID =
		pointsID.begin();
for (std::vector<cv::Point2f>::iterator it = points[1].begin();
		it != points[1].end(); ii++) {

//	double Z = double(frameD[2].at<uint16_t>(roundY(it->y), roundX(it->x)))
//			/ depthInvScale;
	double Z = double(frameD[2].at<uint16_t>(roundSize(it->y, frameD[2].rows), roundSize(it->x, frameD[2].cols)))
				/ depthInvScale;

	if (abs(Z) > 0.001) {
		kbVertex.keypointsId.push_back(*itID);
		kbVertex.keypoints.push_back(*it);
		kbVertex.keypoints3d.push_back(
				RGBDclass::point2Dto3D(*it, Z, calibrationParams.cameraMatrix,
						calibrationParams.distCoeffs));
		kbVertex.informationMatrix.push_back(
				getInformationMatrix(it->x, it->y, Z));
		++it, ++itDepth, ++itID;
	} else {
//			++it, ++itDepth, ++itID;
		it = points[1].erase(it);
		itDepth = pointDepth.erase(itDepth);
		itID = pointsID.erase(itID);

	}
}
kbVertex.inGraph.resize(kbVertex.keypoints.size());

vertexHistory.push_back(kbVertex);
if (vertexHistory.size() > programParameters.historySize) {
	vertexHistory.erase(vertexHistory.begin());
}

wasInlierSecond.clear();
wasInlierSecond.resize(kbVertex.keypoints.size(), 0);
}

Eigen::Matrix3f Track::getInformationMatrix(double u, double v, double z) {

double varU = 1.1046, varV = 0.64160;
double distVarCoefs[4] = { -8.9997e-06, 3.069e-003, 3.6512e-006, -0.0017512e-3 };
Eigen::Matrix3f Ruvd, J, cov;

Ruvd << varU, 0, 0, 0, varV, 0, 0, 0, 0;

J << 0.0017 * z, 0, (0.0017 * u - 0.549), 0, 0.0017 * z, (0.0017 * v - 0.443), 0, 0, 1;
Ruvd(2, 2) = (distVarCoefs[0] * pow(z, 3.0) + distVarCoefs[1] * pow(z, 2.0)
		+ distVarCoefs[2] * z + distVarCoefs[3]) / 3.0;
cov = J * Ruvd * J.transpose();

return cov.inverse();
}

int Track::computeRANSACIterationCount(double successRate, double inlierRate,
	int numberOfPairs) {
return (log(1 - successRate) / log(1 - pow(inlierRate, numberOfPairs)));
}

double Track::matrixError(Eigen::Matrix4f x, Eigen::Matrix4f y) {
// Euclidean distance
return pow(x(0, 3) - y(0, 3), 2) + pow(x(1, 3) - y(1, 3), 2)
		+ pow(x(2, 3) - y(2, 3), 2);
//return (x-y).operatorNorm();
}
