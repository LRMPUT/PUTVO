#include "Includes/includes.h"
#include "kinect.h"
#include <opencv2/core/eigen.hpp>


int main(int argc, char *argv[])
{

	if (argc == 13) {
		cv::Mat src(1, 3, CV_32FC1), dst(3, 3, CV_32FC1);
		src.at<float>(0, 0) = atof(argv[1]);
		src.at<float>(0, 1) = atof(argv[2]);
		src.at<float>(0, 2) = atof(argv[3]);
		cv::Rodrigues(src, dst);

		Eigen::Matrix3f matrix;
		cv::cv2eigen(dst, matrix);

		Eigen::Matrix4f mat;
		mat.block<3,3>(0,0) = matrix;
		mat(0,3) = atof(argv[4]);
		mat(1,3) = atof(argv[5]);
		mat(2,3) = atof(argv[6]);
		mat(3,3) = 1.0;

		src.at<float>(0, 0) = atof(argv[7]);
		src.at<float>(0, 1) = atof(argv[8]);
		src.at<float>(0, 2) = atof(argv[9]);
		cv::Rodrigues(src, dst);
		cv::cv2eigen(dst, matrix);

		Eigen::Matrix4f znacznik;
		znacznik.block<3, 3>(0, 0) = matrix;
		znacznik(0, 3) = atof(argv[10]);
		znacznik(1, 3) = atof(argv[11]);
		znacznik(2, 3) = atof(argv[12]);
		znacznik(3, 3) = 1.0;

		Eigen::Matrix4f resultingPosition = mat * znacznik;

		Eigen::Quaternionf quat(resultingPosition.block<3,3>(0,0));
		std::cout <<resultingPosition(0,3)<<" "<<resultingPosition(1,3)<<" "<<resultingPosition(2,3)<<" "<< quat.x() << " " << quat.y() << " " << quat.z() << " "
				<< quat.w() << std::endl;
	}else if (argc == 14) {
		cv::Mat src(1, 3, CV_32FC1), dst(3, 3, CV_32FC1);
		src.at<float>(0, 0) = atof(argv[1]);
		src.at<float>(0, 1) = atof(argv[2]);
		src.at<float>(0, 2) = atof(argv[3]);
		cv::Rodrigues(src, dst);

		Eigen::Matrix3f matrix;
		cv::cv2eigen(dst, matrix);


		Eigen::Matrix4f gtOld;
		gtOld.block<3,3>(0,0) = matrix;
		gtOld(0,3) = atof(argv[4]);
		gtOld(1,3) = atof(argv[5]);
		gtOld(2,3) = atof(argv[6]);
		gtOld(3,3) = 1.0;

		src.at<float>(0, 0) = atof(argv[7]);
		src.at<float>(0, 1) = atof(argv[8]);
		src.at<float>(0, 2) = atof(argv[9]);
		cv::Rodrigues(src, dst);
		cv::cv2eigen(dst, matrix);

		Eigen::Matrix4f camera;
		camera.block<3, 3>(0, 0) = matrix;
		camera(0, 3) = atof(argv[10]);
		camera(1, 3) = atof(argv[11]);
		camera(2, 3) = atof(argv[12]);
		camera(3, 3) = 1.0;

		Eigen::Matrix4f intertial = Eigen::Matrix4f::Zero();
		intertial(1, 0) = intertial(0,1) = intertial(2,2) = -1.0;
		intertial(0, 3) = 0;
		intertial(1, 3) = 0;
		intertial(2, 3) = 0;
		intertial(3, 3) = 1.0;

		Eigen::Matrix4f resultingPosition = gtOld * camera * intertial;

		Eigen::Matrix3f x = resultingPosition.block<3,3>(0,0);
		cv::eigen2cv(x, dst);
		cv::Rodrigues(dst, src);

		std::cout << src.at<float>(0, 0) << " " << src.at<float>(0, 1) << " "
				<< src.at<float>(0, 2) << " " << resultingPosition(0, 3) << " "
				<< resultingPosition(1, 3) << " " << resultingPosition(2, 3)
				<< std::endl;
	}
	else {
		Kinect *Program = new Kinect();
		if (argc == 3)
		{
			Program->programParameters.TrackingLength = atoi(argv[1]);
			Program->depthInvScale = atoi(argv[2]);
		}

		Program->TrackingRun();
		delete Program;
	}


	// Connecting data into 1 cloud
//	if (argc  == 2)
//	{
//		ifstream p;
//		int ile_chmur = atoi(argv[1]);
//
//		// Max 40 clouds to connect together, but it will diverge sooner ...
//		Eigen::Matrix4f Trans[40];
//		// Reading calculated Transformations
//		for(int i=1;i<ile_chmur;i++)
//		{
//			string str = "WYN" + convertInt(i);
//			p.open(str.c_str());
//			int t=0;
//			while(!p.eof())
//			{
//				double a;
//				p>>a;
//				Trans[i](t/4, t%4 ) = a;
//				t++;
//				if( t== 16) break;
//			}
//			p.close();
//			cout<<i << " : " <<endl<<Trans[i]<<endl;
//		}
//
//		// Set initial position to appropriate data
//
//
//		Eigen::Quaternion<float> Q_ST( -0.1170,  0.0666,-0.4608  ,  0.8772 );
//		Program->KinectInitial.block<3,3>(0,0) = Q_ST.toRotationMatrix();
//		Program->KinectInitial(0,3) = -0.0730;
//		Program->KinectInitial(1,3) =  -0.4169;
//		Program->KinectInitial(2,3) =  1.5916;
//
//		/*// desk2
//		Eigen::Quaternion<float> Q_ST(0.5466, -0.3131,-0.2604  , 0.7317  );
//		Program->KinectInitial.block<3,3>(0,0) = Q_ST.toRotationMatrix();
//		Program->KinectInitial(0,3) = 1.2905;
//		Program->KinectInitial(1,3) =  0.0005;
//		Program->KinectInitial(2,3) =  1.5678  ;*/
//
//		/* // desk
//		Eigen::Quaternion<float> Q_ST(0.4393,-0.1770 ,-0.3879  ,0.7907  );
//		Program->KinectInitial.block<3,3>(0,0) = Q_ST.toRotationMatrix();
//		Program->KinectInitial(0,3) = 1.2334;
//		Program->KinectInitial(1,3) = -0.0113 ;
//		Program->KinectInitial(2,3) =  1.6941 ;*/
//
//		// Matrix changing the coordinate system from Kinect's one to
//		// Freiburg's standard camera coordinate
//		Eigen::Matrix4f zamiana_osi = Eigen::Matrix4f::Identity();
//		zamiana_osi(1,1) = zamiana_osi(2,2) = 0;
//		zamiana_osi(2,1) = 1;
//		zamiana_osi(1,2) = -1;
//
//		// These transformations are summed together
//		Eigen::Matrix4f zb[40];
//		for(int i=1;i<ile_chmur+1;i++)
//		{
//			cout<<"iteracja : " <<i<<endl;
//
//			string a = convertInt(i)+".png";
//			string b = convertInt(i) + ".xml";
//
//			cout<<b<<" | "<<a<<endl;
//
//			(Program->RGBD)->LoadRGBD2(b,a,1);
//			Program->chmura_color[1] = (Program->RGBD)->BuildPointCloudFromRGBD(1);
//			Program->FiltrowanieColor(1);
//
//			// Creating global transformation
//			zb[i] = Eigen::Matrix4f::Identity();
//			for (int j=ile_chmur;j>i;j--)
//			{
//				TransformSelf(Program->chmura_color[1], Trans[j-1]);
//				zb[i] = Trans[j-1] * zb[i];
//			}
//			TransformSelf(Program->chmura_color[1], zamiana_osi);
//			TransformSelf(Program->chmura_color[1], Program->KinectInitial);
//			zb[i] = Program->KinectInitial*zamiana_osi*zb[i];
//
//			// Creating sum cloud
//			if ( i == 1 ) copyPointCloud(*Program->chmura_color[1],*Program->chmura_color[0]);
//			else *Program->chmura_color[0] = *Program->chmura_color[0] + *Program->chmura_color[1];
//
//		}
//		ShowCloudXYZRGBA(Program->chmura_color[0]);
//
//		ofstream za;
//		za.open("FreiburgEval");
//		for(int i=1;i<ile_chmur+1	;i++)
//		{
//
//
//			Eigen::Quaternion<float> Q(zb[i].block<3,3>(0,0));
//			za<<zb[i](0,3)<<" "<<zb[i](1,3)<<" "<<zb[i](2,3)<<" "<<Q.coeffs().x()<< " "<<Q.coeffs().y()<<" "<<Q.coeffs().z()<< " "<<Q.coeffs().w()<<endl;
//		}
//		za.close();
//
//
//	}


    return 0;
}
