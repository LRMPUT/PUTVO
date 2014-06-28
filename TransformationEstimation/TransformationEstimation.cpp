#include "TransformationEstimation.h"

template <typename T> int sgn(T val)
{
    return (val > T(0)) - (val < T(0));
}

void Kabsch(Eigen::MatrixXf &P, Eigen::MatrixXf &Q, Eigen::Matrix4f &Opt)
{
	Eigen::Vector3f centerOfMassIn, centerOfMassOut;

	int numberOfPoints = P.rows();

	// Calculating center of mass
	for (int i=0;i<3;i++)
	{
		centerOfMassIn[i] = P.col(i).mean();
		centerOfMassOut[i] = Q.col(i).mean();
	}
		
	// Moving to center of mass
	for(int j=0;j<numberOfPoints;j++)
	{
		P.row(j) -= centerOfMassIn;
		Q.row(j) -= centerOfMassOut;
	} 
			
	// A = P * Q
	Eigen::MatrixXf A = P.transpose() * Q;
			
	// SVD of A
	Eigen::JacobiSVD<Eigen::MatrixXf> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
	Eigen::MatrixXf V = svd.matrixU(), W = svd.matrixV();

	// Ensuring it is right-handed coordinate system
	Eigen::Matrix3f U = Eigen::MatrixXf::Identity(3,3);
	U(2,2) = sgn(A.determinant());
			
	// Optimal rotation matrix
	U = W * U * V.transpose();
			
	// Computing the translation
	Eigen::Vector3f T;
	T = -centerOfMassIn;
	T = U*T;
	T += centerOfMassOut;

	// Optimal transformation
	Opt <<  Eigen::Matrix4f::Identity();
	Opt.block<3,3>(0,0) = U.block<3,3>(0,0);
	Opt.block<3,1>(0,3) = T.head<3>();
}

void Umeyama(Eigen::MatrixXf &P, Eigen::MatrixXf &Q, Eigen::Matrix4f &Opt) {
	Opt = Eigen::umeyama(P.transpose(), Q.transpose(), false);
	if (std::isnan(Opt(0, 0))) {
		Opt = Eigen::Matrix4f::Identity();
	}
}
