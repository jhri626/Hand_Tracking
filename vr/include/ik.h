#ifndef IK_H
#define IK_H

#include <cmath>
#include <iostream>
#include <Eigen/Geometry>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <ceres/ceres.h>
#include <pose_utils.h>

namespace ik {

  // Forward declarations
  Eigen::Matrix4d computeRelativeSE3(
      const Eigen::Quaterniond& q1,
      const Eigen::Vector3d& p1,
      const Eigen::Quaterniond& q2,
      const Eigen::Vector3d& p2);
  
  Eigen::Vector3d forwardKinematics(const double* theta, const double L1, const double L2);
  
  Eigen::Vector2d inversekinematics(
      const ros::Publisher& pub,
      const geometry_msgs::Pose& pose_ref,
      const geometry_msgs::Pose& pose_target,
      const geometry_msgs::Pose& pose_meta,
      const geometry_msgs::Pose& pose_proxi,
      double L1, double L2, double theta_init_x, double theta_init_y);
  
  
  // Cost functor for Ceres Solver
  struct IKCostFunctor {
      IKCostFunctor(const Eigen::Vector3d& target_position, double l1, double l2)
          : target_position_(target_position), L1_(l1), L2_(l2) {}
  
      template <typename T>
      bool operator()(const T* const theta, T* residual) const {
          T x = T(L2_) * ceres::cos(theta[0]) * ceres::sin(theta[1]);
          T y = - T(L2_) * ceres::sin(theta[0]);
          T z = - T(L1_) - T(L2_) * ceres::cos(theta[0]) * ceres::cos(theta[1]);
  
          residual[0] = x - T(target_position_.x());
          residual[1] = y - T(target_position_.y());
          residual[2] = z - T(target_position_.z());
          return true;
      }
  
    private:
      const Eigen::Vector3d target_position_;
      const double L1_, L2_;
  };
  
  } // namespace ik
  
  #endif // IK_H