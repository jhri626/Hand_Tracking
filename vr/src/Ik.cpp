#include <ik.h>
#include <cmath>
#include <pose_utils.h> 
#include <iostream>
#include <Eigen/Geometry>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <ceres/ceres.h>
#include <ceres/loss_function.h>
#include <lie_utils.h>


namespace ik {
    Eigen::Vector2d inversekinematics(const ros::Publisher& pub, const Eigen::Quaterniond q_ref, const Eigen::Vector3d p_ref,
                   const geometry_msgs::Pose& pose_target,double L1, double L2, double theta_init_x, double theta_init_y, const std::string& mode)
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

        // std::cout<<"target pos"<<target_pos<<std::endl;

        // 3) Set up Ceres problem
          // initial guess

        ceres::Problem problem;
        if (mode == "thumb"){
            // double theta[2] = {
            //     std::clamp(theta_init_x, 0.0, M_PI),
            //     std::clamp(theta_init_y, -M_PI/4, M_PI/4)
            // };
            double theta[2];
        theta[0] = std::max(0.0, std::min(theta_init_x, M_PI));
        theta[1] = std::max(0.0, std::min(theta_init_y, M_PI)); 

            auto* cost_function =
        new ceres::AutoDiffCostFunction<thumbIKCostFunctor, 4, 2>(
            new thumbIKCostFunctor(target_pos, L1, L2, 0.015, 1e-6));
        problem.AddResidualBlock(cost_function, nullptr, theta);

        // problem.SetParameterLowerBound(theta, 0, theta_init_x < M_PI ? (theta_init_x > M_PI/9 ? theta_init_x - M_PI/4 : 0) : 0);
        // problem.SetParameterUpperBound(theta, 0, theta_init_x > 0 ? (theta_init_x < M_PI - M_PI/9 ? theta_init_x + M_PI/4 : M_PI) : M_PI);
        // problem.SetParameterLowerBound(theta, 1, theta_init_y > -M_PI/4 + M_PI/9 ? theta_init_y - M_PI/4 : -M_PI/4);
        // problem.SetParameterUpperBound(theta, 1,  theta_init_y < M_PI/4 - M_PI/9 ? theta_init_y + M_PI/4 : M_PI/4);

        problem.SetParameterLowerBound(theta, 0, 0);
        problem.SetParameterUpperBound(theta, 0,  M_PI);
        problem.SetParameterLowerBound(theta, 1, -M_PI/2);
        problem.SetParameterUpperBound(theta, 1,  M_PI/2);
        

        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        // options.minimizer_progress_to_stdout = true;
        options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
        options.max_num_iterations   = 100;

        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        Eigen::Vector3d proxi = -L1 * (q_ref.toRotationMatrix().col(2)).normalized();

        Eigen::Vector3d joint1(
            q_ref.toRotationMatrix().col(0)
        );
        Eigen::Vector3d joint2(
            q_ref.toRotationMatrix().col(1)
        );


        Eigen::Vector3d newproxi(
            ceres::cos(M_PI *4.5/18) * ( - (L2) * ceres::sin(theta[1]) ) - ceres::sin(M_PI *4.5/18) * ( - (L2) * ceres::cos(theta[1]) * ceres::sin(theta[0])),
            ceres::sin(M_PI *4.5/18) * ( - (L2) * ceres::sin(theta[1]) ) + ceres::cos(M_PI *4.5/18) * ( - (L2) * ceres::cos(theta[1]) * ceres::sin(theta[0])),
            - (L1) - (L2) * ceres::cos(theta[0]) * ceres::cos(theta[1])
        );

        newproxi = q_ref.toRotationMatrix() * newproxi;


        return Eigen::Vector2d(theta[0], theta[1]);
        }
    }

    /* Not used in now

    Eigen::Vector3d inversekinematicsIndex(const ros::Publisher& pub, const Eigen::Quaterniond q_ref, const Eigen::Vector3d p_ref,
        const geometry_msgs::Pose& pose_target,double L1, double L2, double theta_init_1,double theta_init_2, double theta_init_3, const std::string& mode)
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
            // std::cout<< "L1 :"<<L1<<", L2 : "<<L2<<std::endl;
            // std::cout<< "Target :"<<target_pos<<std::endl;

            ceres::Problem problem;
            double theta[3] = {theta_init_1,theta_init_2,theta_init_3};
            theta[0] = std::max(0.0, std::min(theta_init_1, M_PI));
            theta[1] = std::max(0.0, std::min(theta_init_2, M_PI));
            theta[2] = std::max(0.0, std::min(theta_init_3, M_PI));
                auto* cost_function =
            new ceres::AutoDiffCostFunction<indexIKCostFunctor, 2, 3>(
                new indexIKCostFunctor(target_pos, L1, L2));
            problem.AddResidualBlock(cost_function, new ceres::HuberLoss(0.01), theta);
            

            // problem.SetParameterLowerBound(theta, 0, theta_init_1 > 0 ? (theta_init_1 > M_PI/9 ? theta_init_1 - M_PI/9 : 0) : 0);
            // problem.SetParameterUpperBound(theta, 0, theta_init_1 < 2 * M_PI/3 ? (theta_init_1 < 2 * M_PI/3 - M_PI/9 ? theta_init_1 + M_PI/9 : 2 * M_PI/3) : 2 * M_PI/3);
            // problem.SetParameterLowerBound(theta, 1, theta_init_2 > 0 ? (theta_init_2 > M_PI/9 ? theta_init_2 - M_PI/9 : 0) : 0);
            // problem.SetParameterUpperBound(theta, 1, theta_init_2 < 2 * M_PI/3 ? (theta_init_2 < 2 * M_PI/3 - M_PI/9 ? theta_init_2 + M_PI/9 : 2 * M_PI/3) : 2 * M_PI/3);
            // problem.SetParameterLowerBound(theta, 2, theta_init_3 > -M_PI/4 + M_PI/9 ? theta_init_3 - M_PI/4 : -M_PI/4);
            // problem.SetParameterUpperBound(theta, 2,  theta_init_3 < M_PI/4 - M_PI/9 ? theta_init_3 + M_PI/4 : M_PI/4);
        
            problem.SetParameterLowerBound(theta, 0, 0);
            problem.SetParameterUpperBound(theta, 0,  M_PI/2);
            problem.SetParameterLowerBound(theta, 1, 0);
            problem.SetParameterUpperBound(theta, 1,  2 * M_PI/3);
            problem.SetParameterLowerBound(theta, 2, -M_PI/4);
            problem.SetParameterUpperBound(theta, 2,  M_PI/4);
        
            ceres::Solver::Options options;
            options.linear_solver_type = ceres::DENSE_QR;
            // options.minimizer_progress_to_stdout = true;
            options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
            // options.max_num_iterations   = 20000;
    
            ceres::Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);
            
            // std::cout<< "theta 0 :"<<theta[0]<<", theta 1 : "<< theta[1] <<", theta 2 : "<< theta[2]<<std::endl;
            
            
    
            // --- Build homogeneous transform T1: rotation about Z by theta2 ---
            Eigen::Matrix4d T1;
            T1 << std::cos(theta[2]),  0.0, std::sin(theta[2]), 0.0,
                0.0,                1.0, 0.0,               0.0,
                -std::sin(theta[2]),  0.0, std::cos(theta[2]), 0.0,
                0.0,                0.0, 0.0,               1.0;
    
            // --- Build homogeneous transform T2: rotation about X by theta0 ---
            Eigen::Matrix4d T2;
            T2 << 1.0, 0.0,               0.0,                0.0,
                0.0, std::cos(theta[0]),  std::sin(theta[0]),   0.0,
                0.0, -std::sin(theta[0]), std::cos(theta[0]),  -0.0,
                0.0, 0.0,               0.0,                1.0;
    
            // --- Build homogeneous transform T3: rotation about Y by theta1 + translation block update ---
            Eigen::Matrix4d T3;
            T3 << 1.0, 0.0,                0.0,               0.0,
                0.0, std::cos(theta[1]),   std::sin(theta[1]),  0.0,
                0.0, -std::sin(theta[1]),  std::cos(theta[1]), -0.0,
                0.0, 0.0,                0.0,               1.0;
    
            // Compute the translation part of T3 so that the link1 offset L1 is applied:
            // q = (0, 0, -L1)^T
            Eigen::Vector3d q(0.0, 0.0, -L1);
            // I - R part of T3.block<3,3>
            Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
            Eigen::Matrix3d R3 = T3.block<3,3>(0,0);
            // Set the 3×1 translation block (upper right) of T3
            T3.block<3,1>(0,3) = (I - R3) * q;
    
            // --- Local end-effector position in its own frame ---
            Eigen::Vector4d p_local;
            p_local << 0.0, 0.0, -(L1 + L2), 1.0;  
    
            // --- Compute world position by chaining transforms ---
            Eigen::Vector4d p_world = T1 * (T2 * (T3 * p_local));
    
            
            Eigen::Vector3d new_proxi = q_ref.toRotationMatrix() * p_world.block<3,1>(0,0);
            Eigen::Vector3d x(1,0,0);
            Eigen::Vector3d y(0,1,0);
            Eigen::Vector3d z(0,0,1);
            Eigen::Vector3d proxi = -L1 * z;
            proxi = q_ref.toRotationMatrix() * lie_utils::Matexp3(lie_utils::vecToso3(y),theta[2]) * lie_utils::Matexp3(lie_utils::vecToso3(-x),theta[0]) * proxi;
    
            
            pub.publish(vectorToArrowMarker(p_ref,proxi,"world","v1",3,1,0,0));
            pub.publish(vectorToArrowMarker(p_ref+proxi,new_proxi-proxi,"world","v1",4,0,1,0));
            return Eigen::Vector3d(theta[0] , theta[1], theta[2]);
        }
        // std::cout<<"work?"<<std::endl;

        // 4) Return optimized angles

        */
}