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
#include"utils.h"


namespace ik {
    Eigen::Vector2d inversekinematics(const ros::Publisher& pub, const Eigen::Quaterniond q_ref, const Eigen::Vector3d p_ref,
                   const geometry_msgs::Pose& pose_target, double L1, double L2, double theta_init_x, double theta_init_y)
    {
        // 1) Convert poses to Eigen
        Eigen::Quaterniond q_tgt = getQuaternionfromPose(pose_target);
        Eigen::Vector3d p_tgt = getPositionfromPose(pose_target);

        // 2) Compute relative SE(3) and extract translation
        Eigen::Matrix4d T_rel = mr::computeRelativeSE3(q_ref, p_ref, q_tgt, p_tgt);
        Eigen::Vector3d target_pos = T_rel.block<3,1>(0,3);

        

        // 3) Set up Ceres problem
          // initial guess

        ceres::Problem problem;
        double theta[2];

            auto* cost_function =
        new ceres::AutoDiffCostFunction<thumbIKCostFunctor, 4, 2>(
            new thumbIKCostFunctor(target_pos, L1, L2, 0.015, 1e-6));
        problem.AddResidualBlock(cost_function, nullptr, theta);


        double regularization_weight = 0.01;
        ceres::CostFunction* regularization =
            new ceres::AutoDiffCostFunction<ThetaRegularizationFunctor, 2, 2>(
                new ThetaRegularizationFunctor(theta_init_x, theta_init_y, regularization_weight));
        problem.AddResidualBlock(regularization, nullptr, theta);

        problem.SetParameterLowerBound(theta, 0, 0);
        problem.SetParameterUpperBound(theta, 0,  M_PI);
        problem.SetParameterLowerBound(theta, 1, -M_PI/2);
        problem.SetParameterUpperBound(theta, 1,  M_PI/2);
        

        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        // options.minimizer_progress_to_stdout = true;
        options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
        // options.max_num_iterations   = 100;

        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        Eigen::Vector3d proxi(
            0,
            -std::sin(theta[1]) * L1,
            -std::cos(theta[1]) * L1
        );

        Eigen::Vector3d joint1(
            q_ref.toRotationMatrix().col(0)
        );
        Eigen::Vector3d joint2(
            q_ref.toRotationMatrix().col(1)
        );


        Eigen::Vector3d newproxi(
            L2 * std::sin(theta[0]),
            - std::sin(theta[1]) * ( L1 + L2 * std::cos(theta[0])),
            -std::cos(theta[1]) * ( L1 + L2 * std::cos(theta[0]))
        );

        proxi = q_ref.toRotationMatrix() * proxi;
        newproxi = q_ref.toRotationMatrix() * newproxi;



        pub.publish(vectorToArrowMarker(p_ref,proxi,"hmd_frame","v1",3,1,0,0));
        pub.publish(vectorToArrowMarker(p_ref+proxi,newproxi-proxi,"hmd_frame","v1",4,0,1,0));

        return Eigen::Vector2d(theta[0], theta[1]);
        
    }

    /* Not used in now*/

    Eigen::Vector3d inversekinematicsIndex(const ros::Publisher& pub, const Eigen::Quaterniond q_ref, const Eigen::Vector3d p_ref,
        const geometry_msgs::Pose& pose_target,double L1, double L2, double theta_init_1,double theta_init_2, double theta_init_3, const std::string& mode)
        {
                // 1) Convert poses to Eigen
            Eigen::Quaterniond q_tgt = getQuaternionfromPose(pose_target);
            Eigen::Vector3d p_tgt = getPositionfromPose(pose_target);
            
            // 2) Compute relative SE(3) and extract translation
            Eigen::Matrix4d T_rel = mr::computeRelativeSE3(q_ref, p_ref, q_tgt, p_tgt);
            Eigen::Vector3d target_pos = T_rel.block<3,1>(0,3);
            

            ceres::Problem problem;
            double theta[3] = {theta_init_1,theta_init_2,theta_init_3};
            theta[0] = theta_init_1;
            theta[1] = theta_init_2;
            theta[2] = theta_init_3;
                auto* cost_function =
            new ceres::AutoDiffCostFunction<indexIKCostFunctor, 4, 3>(
                new indexIKCostFunctor(target_pos, L1, L2, 0.02, 1e-6));
            problem.AddResidualBlock(cost_function, nullptr, theta);
            

            problem.SetParameterLowerBound(theta, 0, theta_init_1 > 0 ? (theta_init_1 > M_PI/45 ? theta_init_1 - M_PI/45 : 0) : 0);
            problem.SetParameterUpperBound(theta, 0, theta_init_1 < 2 * M_PI/3 ? (theta_init_1 < 2 * M_PI/3 - M_PI/45 ? theta_init_1 + M_PI/45 : 2 * M_PI/3) : 2 * M_PI/3);
            problem.SetParameterLowerBound(theta, 1, theta_init_2 > 0 ? (theta_init_2 > M_PI/45 ? theta_init_2 - M_PI/45 : 0) : 0);
            problem.SetParameterUpperBound(theta, 1, theta_init_2 < 2 * M_PI/3 ? (theta_init_2 < 2 * M_PI/3 - M_PI/9 ? theta_init_2 + M_PI/45 : 2 * M_PI/3) : 2 * M_PI/3);
            problem.SetParameterLowerBound(theta, 2, theta_init_3 > -M_PI/4 + M_PI/45 ? theta_init_3 - M_PI/45 : -M_PI/4);
            problem.SetParameterUpperBound(theta, 2,  theta_init_3 < M_PI/4 - M_PI/45 ? theta_init_3 + M_PI/45 : M_PI/4);
            if (theta_init_1 < 0 || theta_init_1 > 2 * M_PI/3)
            {
                problem.SetParameterLowerBound(theta, 0, 0);
                problem.SetParameterUpperBound(theta, 0,  M_PI/2);
            }
            if (theta_init_2 < 0 || theta_init_2 > 2 * M_PI/3)
            {
                problem.SetParameterLowerBound(theta, 1, 0);
                problem.SetParameterUpperBound(theta, 1,  M_PI/2);
            }
            if (theta_init_3 < -M_PI/45 || theta_init_3 > M_PI/45)
            {
                problem.SetParameterLowerBound(theta, 2, -M_PI/4);
                problem.SetParameterUpperBound(theta, 2,  M_PI/4);
            }
        
            ceres::Solver::Options options;
            options.linear_solver_type = ceres::DENSE_QR;
            // options.minimizer_progress_to_stdout = true;
            options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
            // options.max_num_iterations   = 20000;
    
            ceres::Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);
    
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
            proxi = q_ref.toRotationMatrix() * mr::MatrixExp3(mr::VecToso3(y*theta[2])) * mr::MatrixExp3(mr::VecToso3(-x*theta[0])) * proxi;

            pub.publish(vectorToArrowMarker(p_ref,proxi,"hmd_frame","v1",3,1,0,0));
            pub.publish(vectorToArrowMarker(p_ref+proxi,new_proxi-proxi,"hmd_frame","v1",4,0,1,0));
            // pub.publish(vectorToArrowMarker(p_ref,proxi_2,"hmd_frame","v1",5,0,0,1));
            return Eigen::Vector3d(theta[0] , theta[1], theta[2]);
        }
        

        // 4) Return optimized angles




        Eigen::Vector2d Anyteleopmethod(const Eigen::Quaterniond q_ref, const Eigen::Vector3d p_ref, const geometry_msgs::Pose& pose_inter,
                const geometry_msgs::Pose& pose_target, double d_pre, double AA, int idx)
        {
                // 1) Convert poses to Eigen
            Eigen::Quaterniond q_tgt = getQuaternionfromPose(pose_target);
            Eigen::Vector3d p_tgt = getPositionfromPose(pose_target);
            Eigen::Quaterniond q_inter = getQuaternionfromPose(pose_inter);
            Eigen::Vector3d p_inter = getPositionfromPose(pose_inter);

            // 2) Compute relative SE(3) and extract translation
            Eigen::Matrix4d T_rel = mr::computeRelativeSE3(q_ref, p_ref, q_tgt, p_tgt);
            Eigen::Matrix4d T_inter = mr::computeRelativeSE3(q_ref, p_ref, q_inter, p_inter);
            double alpha = 1.2;
            Eigen::Vector3d target_pos = alpha * T_rel.block<3,1>(0,3) * 1000 ;
            Eigen::Vector3d inter_pos = alpha * T_rel.block<3,1>(0,3) * 1000 ;

            double theta[2] = {d_pre,AA};
            // double theta[2] = {22.4,0.0};
            ceres::Problem problem;
            if (idx != 0)
            {
                
                ceres::CostFunction* cost_function =
                    new ceres::AutoDiffCostFunction<EEPositionFromDA_Functor, 6, 2>(
                        new EEPositionFromDA_Functor(inter_pos, target_pos));
                    
                problem.AddResidualBlock(cost_function, nullptr, theta);
            }
            else
            {
                
                ceres::CostFunction* cost_function =
                    new ceres::AutoDiffCostFunction<EEPositionFromDA_Functor_Thumb, 6, 2>(
                        new EEPositionFromDA_Functor_Thumb(inter_pos, target_pos));
                    
                problem.AddResidualBlock(cost_function, nullptr, theta);
            }
            

            double regularization_weight = 0.01;
            ceres::CostFunction* regularization =
                new ceres::AutoDiffCostFunction<ThetaRegularizationFunctor, 2, 2>(
                    new ThetaRegularizationFunctor(d_pre, AA, regularization_weight));
            problem.AddResidualBlock(regularization, nullptr, theta);

            problem.SetParameterLowerBound(theta, 0, 0);
            problem.SetParameterUpperBound(theta, 0, 1.3);   
            problem.SetParameterLowerBound(theta, 1, -0.5);
            problem.SetParameterUpperBound(theta, 1, 0.5);
            
            ceres::Solver::Options options;
            options.linear_solver_type = ceres::DENSE_QR;

            // if (idx == 0 ) options.minimizer_progress_to_stdout = true;
            options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
            // options.max_num_iterations   = 20000;
    
            ceres::Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);
            
            return Eigen::Vector2d(theta[0] , theta[1]);
        }

        
}