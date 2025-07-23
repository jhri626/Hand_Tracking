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

  
  Eigen::Vector2d inversekinematics(
      const ros::Publisher& pub,
      const Eigen::Quaterniond q_ref, 
      const Eigen::Vector3d p_ref,
      const geometry_msgs::Pose& pose_target,
      double L1, double L2, double theta_init_x, double theta_init_y, const std::string& mode);
  Eigen::Vector3d inversekinematicsIndex(
    const ros::Publisher& pub,
    const Eigen::Quaterniond q_ref, 
    const Eigen::Vector3d p_ref,
    const geometry_msgs::Pose& pose_target,
    double L1, double L2, double theta_init_1, double theta_init_2,double theta_init_3, const std::string& mode);
  
  
  // Cost functor for Ceres Solver
  struct thumbIKCostFunctor {
    thumbIKCostFunctor(const Eigen::Vector3d& target_position,
                  double l1, double l2,
                  double ratio_weight,
                  double eps)
        : target_position_(target_position),
          L1_(l1), L2_(l2),
          w_(ratio_weight),
          eps_(eps) {}
  
    template <typename T>
    bool operator()(const T* const theta, T* residual) const {
      T x = ceres::cos(M_PI *9/18) * ( - T(L2_) * ceres::sin(theta[1]) ) - ceres::sin(M_PI *9/18) * ( - T(L2_) * ceres::cos(theta[1]) * ceres::sin(theta[0]));
      T y = ceres::sin(M_PI *9/18) * ( - T(L2_) * ceres::sin(theta[1]) ) + ceres::cos(M_PI *9/18) * ( - T(L2_) * ceres::cos(theta[1]) * ceres::sin(theta[0]));
      T z = - T(L1_) - T(L2_) * ceres::cos(theta[0]) * ceres::cos(theta[1]);
      residual[0] = x - T(target_position_.x());
      residual[1] = y - T(target_position_.y());
      residual[2] = z - T(target_position_.z());
  

      T denom = (theta[0] * theta[0] + T(eps_));
      residual[3] = w_ * ( theta[1] / denom );
  
      return true;
    }
  
   private:
    const Eigen::Vector3d target_position_;
    const double L1_, L2_;
    const double w_;  
    const double eps_;  
  };

  struct indexIKCostFunctor {
    indexIKCostFunctor(const Eigen::Vector3d& target_position,
                  double l1, double l2
                )
        : target_position_(target_position),
          L1_(l1), L2_(l2) {}
  
    template <typename T>
    bool operator()(const T* const theta, T* residual) const {

      // Eigen::Matrix<T,4,4> T1;
      // T1 << ceres::cos(theta[2]), T(0), ceres::sin(theta[2]), T(0),
      //       T(0),                 T(1),                 T(0), -T(0),
      //       -ceres::sin(theta[2]),T(0), ceres::cos(theta[2]), T(0),
      //       T(0),                 T(0),                 T(0), T(1);
      // Eigen::Matrix<T,4,4> T2;
      // T2 << T(1), T(0),                T(0),                  T(0),
      //       T(0), ceres::cos(theta[0]), ceres::sin(theta[0]), T(0),
      //       T(0), -ceres::sin(theta[0]),ceres::cos(theta[0]), -T(0),
      //       T(0), T(0),                 T(0),                 T(1);

      // Eigen::Matrix<T,4,4> T3;
      // T3 << T(1), T(0),                T(0),                T(0),
      //       T(0), ceres::cos(theta[1]), ceres::sin(theta[1]), T(0),
      //       T(0), -ceres::sin(theta[1]), ceres::cos(theta[1]), -T(0),
      //       T(0), T(0),                T(0),                T(1);
      
      // Eigen::Matrix<T,3,1> q = (Eigen::Matrix<T,3,1>() << T(0), T(0), T(-L1_)).finished();
      // Eigen::Matrix<T,3,3> I = Eigen::Matrix<T,3,3>::Identity();
      // T3.template block<3,1>(0,3) = (I - T3.template block<3,3>(0,0)) * q;
            
      // Eigen::Matrix<T,4,1> p_local;
      // p_local << T(0), T(0), T(-(L2_ +L1_)), T(1);

      // Eigen::Matrix<T,4,1> p_world = T1 * (T2 * (T3* p_local));

      T x = ceres::sin(theta[2]) * (L2_*ceres::sin(theta[1]) * ceres::sin(theta[0]) -L2_ * ceres::cos(theta[0]) * ceres::cos(theta[1]) - L1_*ceres::cos(theta[0]));
      T y = - L2_*ceres::sin(theta[1]) * ceres::cos(theta[0]) -L2_*ceres::sin(theta[0]) * ceres::cos(theta[1]) - L1_*ceres::sin(theta[0]);
      T z = ceres::cos(theta[2]) * (L2_*ceres::sin(theta[1]) * ceres::sin(theta[0]) -L2_ * ceres::cos(theta[0]) * ceres::cos(theta[1]) - L1_*ceres::cos(theta[0]));
      
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