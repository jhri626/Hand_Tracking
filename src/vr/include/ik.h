#ifndef IK_H
#define IK_H

#include <cmath>
#include <iostream>
#include <Eigen/Geometry>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <ceres/ceres.h>
#include <pose_utils.h>
#include <hand_number.h>
#include <algorithm>

#include <type_traits> // for debug


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

    Eigen::Vector2d Anyteleopmethod(const Eigen::Quaterniond q_ref,
                                  const Eigen::Vector3d p_ref,
                                  const geometry_msgs::Pose& pose_inter,
                                  const geometry_msgs::Pose& pose_target,
                                  double d_pre,
                                  double AA);

  // Compute MCP flexion angle from prismatic displacement d
    float computeMCPFlexionFromD(float d);
  
  
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
                  double l1, double l2, double ratio_weight, double eps
                )
        : target_position_(target_position),
          L1_(l1), L2_(l2),
          w_(ratio_weight),
          eps_(eps) {}

  
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
      T denom = (theta[0] * theta[0] + theta[1] * theta[1] + T(eps_));
      residual[3] = w_ * ( theta[2] / denom );
      return true;
      
    }
  
   private:
    const Eigen::Vector3d target_position_;
    const double L1_, L2_;
    const double w_;  
    const double eps_; 
  };

// Functor for end-effector position residuals without debug logging
struct EEPositionFromDA_Functor {
    explicit EEPositionFromDA_Functor(const Eigen::Vector3d& inter_pos ,const Eigen::Vector3d& target_pos)
        : inter_position_(inter_pos), target_position_(target_pos) {}

    template <typename T>
    bool operator()(const T* const vars, T* residual) const {
        
      T mcp_f = vars[0];
      T mcp_a = vars[1];
      T pip = -0.1108 * ceres::pow(mcp_f, T(4.0)) + 0.3230 * ceres::pow(mcp_f, T(3.0)) - 0.2964 * ceres::pow(mcp_f, T(2.0)) + 1.2475 * mcp_f + T(0.0034);
      T dip =  0.1005 * ceres::pow(mcp_f, T(5.0)) + 0.3467 * ceres::pow(mcp_f, T(4.0)) - 0.4016 * ceres::pow(mcp_f, T(3.0)) + 0.3414 * ceres::pow(mcp_f, T(2.0)) + 0.6245 * mcp_f - T(0.00087);

      
      T inter_x = - (HAND_la + HAND_lm*cos(mcp_f)) * sin(mcp_a);
      T inter_y = - (HAND_lm*sin(mcp_f));
      T inter_z = - (HAND_la + HAND_lm*cos(mcp_f)) * cos(mcp_a);
      
      T x = inter_x - (HAND_lp*cos(mcp_f + pip) + HAND_ld*cos(mcp_f + pip + dip)) * sin(mcp_a);
      T y = inter_y - (HAND_lp*sin(mcp_f + pip) + HAND_ld*sin(mcp_f + pip + dip));
      T z = inter_z - (HAND_lp*cos(mcp_f + pip) + HAND_ld*cos(mcp_f + pip + dip)) * cos(mcp_a);

      residual[0] = x - T(target_position_.x());
      residual[1] = y - T(target_position_.y());
      residual[2] = z - T(target_position_.z());

      residual[3] = x - T(inter_position_.x());
      residual[4] = y - T(inter_position_.y());
      residual[5] = z - T(inter_position_.z());
      

      return true;
    }

  private:
      const Eigen::Vector3d target_position_;
      const Eigen::Vector3d inter_position_;
  };
  



  struct ThetaRegularizationFunctor {
      ThetaRegularizationFunctor(const double d_prev, const double aa_prev, double weight)
          : d_prev_(d_prev), aa_prev_(aa_prev), weight_(weight) {}

      template <typename T>
      bool operator()(const T* const theta, T* residual) const {
          residual[0] = T(weight_) * (theta[0] - T(d_prev_));  // FE
          residual[1] = T(weight_) * (theta[1] - T(aa_prev_)); // AA
          return true;
      }

    private:
        const double d_prev_;
        const double aa_prev_;
        const double weight_;
  };

    


  
  
  } // namespace ik
  
  #endif // IK_H