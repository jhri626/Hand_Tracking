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
  
  Eigen::Vector3d forwardKinematics(const double* theta, const double L1, const double L2);
  
  Eigen::Vector2d inversekinematics(
      const ros::Publisher& pub,
      const Eigen::Quaterniond q_ref, 
      const Eigen::Vector3d p_ref,
      const geometry_msgs::Pose& pose_target,
      double L1, double L2, double theta_init_x, double theta_init_y);
  
  
  // Cost functor for Ceres Solver
  struct IKCostFunctor {
    IKCostFunctor(const Eigen::Vector3d& target_position,
                  double l1, double l2,
                  double ratio_weight,
                  double eps)
        : target_position_(target_position),
          L1_(l1), L2_(l2),
          w_(ratio_weight),
          eps_(eps) {}
  
    template <typename T>
    bool operator()(const T* const theta, T* residual) const {
      T x =  T(L2_) * ceres::cos(theta[1]) * ceres::sin(theta[0]);
      T y =  - T(L2_) * ceres::sin(theta[1]);
      T z = - T(L1_) - T(L2_) * ceres::cos(theta[0]) * ceres::cos(theta[1]);
      residual[0] = x - T(target_position_.x());
      residual[1] = y - T(target_position_.y());
      residual[2] = z - T(target_position_.z());
  

      T denom = ceres::sqrt(theta[0] * theta[0] + T(eps_));
      residual[3] = w_ * ( theta[1] / denom );
  
      return true;
    }
  
   private:
    const Eigen::Vector3d target_position_;
    const double L1_, L2_;
    const double w_;  
    const double eps_;  
  };
  
  
  } // namespace ik
  
  #endif // IK_H