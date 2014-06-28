#ifndef _KABSCH
#define _KABSCH

#include <Eigen/Eigen>
#include <Eigen/Geometry>

// Calculating optimal transformation matrix
void Kabsch(Eigen::MatrixXf &P, Eigen::MatrixXf &Q, Eigen::Matrix4f &Opt);

void Umeyama(Eigen::MatrixXf &P, Eigen::MatrixXf &Q, Eigen::Matrix4f &Opt);

#endif
