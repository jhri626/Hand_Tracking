#include <ik.h>
#include <cmath>
#include <pose_utils.h> 
#include <iostream>
#include <Eigen/Geometry>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <ceres/ceres.h>
#include <lie_utils.h>


namespace ik {
    Eigen::Vector3d forwardKinematics(const double* theta, const double L1, const double L2) {
        double x = L2 * std::cos(theta[0]) * std::sin(theta[1]);
        double y = - L2 * std::sin(theta[1]);
        double z = L1 + L2 * std::cos(theta[0]) * std::cos(theta[1]);
        return Eigen::Vector3d(x, y, z);
      }

      

    Eigen::Vector2d inversekinematics(const ros::Publisher& pub, const Eigen::Quaterniond q_ref, const Eigen::Vector3d p_ref,
                   const geometry_msgs::Pose& pose_target,double L1, double L2, double theta_init_x, double theta_init_y)
    {
        // 1) Convert poses to Eigen

        Eigen::Quaterniond q_tgt(
            pose_target.orientation.w,
            pose_target.orientation.x,
            pose_target.orientation.y,
            pose_target.orientation.z);

        Eigen::Vector3d p_tgt(
            pose_target.position.x,
            pose_target.position.y,
            pose_target.position.z);

        // 2) Compute relative SE(3) and extract translation
        Eigen::Matrix4d T_rel = lie_utils::computeRelativeSE3(q_ref, p_ref, q_tgt, p_tgt);
        Eigen::Vector3d target_pos = T_rel.block<3,1>(0,3);

        std::cout<<"target pos"<<target_pos<<std::endl;

        // 3) Set up Ceres problem
        double theta[2] = {theta_init_x,theta_init_y};  // initial guess

        ceres::Problem problem;
        auto* cost_function =
        new ceres::AutoDiffCostFunction<IKCostFunctor, 4, 2>(
            new IKCostFunctor(target_pos, L1, L2, 0.05, 1e-6));
        problem.AddResidualBlock(cost_function, nullptr, theta);

        problem.SetParameterLowerBound(theta, 0, -M_PI);
        problem.SetParameterUpperBound(theta, 0,  M_PI);
        problem.SetParameterLowerBound(theta, 1, -0.15 - 0.5);
        problem.SetParameterUpperBound(theta, 1,  0.15 + 0.5);

        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        // options.minimizer_progress_to_stdout = true;
        options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
        options.max_num_iterations   = 100;

        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        Eigen::Vector3d proxi = -L1 * (q_ref.toRotationMatrix().col(2)).normalized();

        Eigen::Vector3d joint1(
            - q_ref.toRotationMatrix().col(0)
        );
        Eigen::Vector3d joint2(
            - q_ref.toRotationMatrix().col(1)
        );


        Eigen::Vector3d newproxi = lie_utils::Matexp3(joint2,theta[1]) * lie_utils::Matexp3(joint1,theta[0]) * proxi * (L2)/L1 ;



        pub.publish(vectorToArrowMarker(p_ref,proxi,"world","v1",1,1,0,0));
        pub.publish(vectorToArrowMarker(p_ref+proxi,newproxi,"world","v1",2,0,1,0));

        

        // std::cout<<"work?"<<std::endl;

        // 4) Return optimized angles
        return Eigen::Vector2d(theta[0], theta[1]);
    }
}