#ifndef _KABSCH
#define _KABSCH

#include <Eigen/Eigen>

// Calculating optimal transformation matrix
void Kabsch(Eigen::MatrixXf &P, Eigen::MatrixXf &Q, Eigen::Matrix4f &Opt);

#endif
