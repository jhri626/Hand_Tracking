// lie_utils.h
// Header file for Lie algebra utilities (so(3))

#include <pose_utils.h> 
#pragma once
#include <Eigen/Dense>

namespace lie_utils {

Eigen::Quaterniond axis_align(
    const Eigen::Quaterniond& q1,
    const Eigen::Vector3d& p1);


// Converts a 3D vector into its corresponding skew-symmetric matrix (so(3))
Eigen::Matrix3d vecToso3(const Eigen::Vector3d& vec);

// Computes the matrix exponential of an so(3) element: exp([vec]_x * theta)
Eigen::Matrix3d Matexp3(const Eigen::Vector3d& vec, double theta);

Eigen::Matrix4d computeRelativeSE3(
    const Eigen::Quaterniond& q1,
    const Eigen::Vector3d& p1,
    const Eigen::Quaterniond& q2,
    const Eigen::Vector3d& p2);

Eigen::Matrix3d inverseSO3(const Eigen::Matrix3d& SO3);
Eigen::Matrix4d inverseSE3(const Eigen::Matrix4d& SE3);

} // namespace ik
