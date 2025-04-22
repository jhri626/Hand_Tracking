#include <ik.h>
#include <cmath>
#include <pose_utils.h> 
#include <iostream>
#include <Eigen/Geometry>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <ceres/ceres.h>


namespace ik {
    Eigen::Matrix4d computeRelativeSE3(
        const Eigen::Quaterniond& q1,
        const Eigen::Vector3d& p1,
        const Eigen::Quaterniond& q2,
        const Eigen::Vector3d& p2)
    {
        // Build 4×4 transform T1 from pose1
        Eigen::Matrix4d T1 = Eigen::Matrix4d::Identity();
        T1.block<3,3>(0,0) = q1.normalized().toRotationMatrix();  // rotation
        T1.block<3,1>(0,3) = p1;                                  // translation

        std::cout<<"p1 : "<<p1<<std::endl;

        // Build 4×4 transform T2 from pose2
        Eigen::Matrix4d T2 = Eigen::Matrix4d::Identity();
        T2.block<3,3>(0,0) = q2.normalized().toRotationMatrix();
        T2.block<3,1>(0,3) = p2;

        std::cout<< "p2 : "<<p2<<std::endl;

        // Compute relative transform: T_rel = T1^{-1} * T2
        Eigen::Matrix4d T_rel = T1.inverse() * T2;
        return T_rel;
    }


    Eigen::Vector3d forwardKinematics(const double* theta, const double L1, const double L2) {
        double x = L2 * std::cos(theta[0]) * std::sin(theta[1]);
        double y = - L2 * std::sin(theta[1]);
        double z = L1 + L2 * std::cos(theta[0]) * std::cos(theta[1]);
        return Eigen::Vector3d(x, y, z);
      }

      

    Eigen::Vector2d inversekinematics(const ros::Publisher& pub, const geometry_msgs::Pose& pose_ref,
                   const geometry_msgs::Pose& pose_target,
                   const geometry_msgs::Pose& pose_meta,
                   const geometry_msgs::Pose& pose_proxi, // for debug
                   double L1, double L2)
    {
        // 1) Convert poses to Eigen
        Eigen::Quaterniond q_ref(
            pose_ref.orientation.w,
            pose_ref.orientation.x,
            pose_ref.orientation.y,
            pose_ref.orientation.z);

        Eigen::Quaterniond q_tgt(
            pose_target.orientation.w,
            pose_target.orientation.x,
            pose_target.orientation.y,
            pose_target.orientation.z);

        Eigen::Vector3d p_ref(
            pose_ref.position.x,
            pose_ref.position.y,
            pose_ref.position.z);
        Eigen::Vector3d p_tgt(
            pose_target.position.x,
            pose_target.position.y,
            pose_target.position.z);

        Eigen::Vector3d p_meta(
            pose_meta.position.x,
            pose_meta.position.y,
            pose_meta.position.z);

        Eigen::Vector3d p_proxi(
            pose_proxi.position.x,
            pose_proxi.position.y,
            pose_proxi.position.z); // for debug

        // 2) Compute relative SE(3) and extract translation
        Eigen::Quaterniond q_new_ref = alignWristToMeta(q_ref,p_ref,p_meta);
        Eigen::Matrix4d T_rel = computeRelativeSE3(q_new_ref, p_ref, q_tgt, p_tgt);
        Eigen::Vector3d target_pos = T_rel.block<3,1>(0,3);

        std::cout<<"target pos"<<target_pos<<std::endl;

        // 3) Set up Ceres problem
        double theta[2] = {0.0, 0.0};  // initial guess

        ceres::Problem problem;
        auto* cost_function =
        new ceres::AutoDiffCostFunction<IKCostFunctor, 3, 2>(
            new IKCostFunctor(target_pos, L1, L2));
        problem.AddResidualBlock(cost_function, nullptr, theta);

        problem.SetParameterLowerBound(theta, 0, -M_PI);
        problem.SetParameterUpperBound(theta, 0,  M_PI);
        problem.SetParameterLowerBound(theta, 1, -M_PI/4);
        problem.SetParameterUpperBound(theta, 1,  M_PI/4);

        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        // options.minimizer_progress_to_stdout = true;
        options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
        options.max_num_iterations   = 100;

        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        Eigen::Vector3d proxi = L2 * (p_meta-p_ref).normalized();
        Eigen::Vector3d joint1(
            - q_new_ref.toRotationMatrix().col(0)
        );
        Eigen::Vector3d joint2(
            - q_new_ref.toRotationMatrix().col(1)
        );

        proxi = Matexp3(joint2,theta[1]) * Matexp3(joint1,theta[0]) * proxi ;



        pub.publish(vectorToArrowMarker(p_ref,p_meta-p_ref,"world","v1",1,1,0,0));
        pub.publish(vectorToArrowMarker(p_meta,proxi,"world","v1",2,0,1,0));

        

        // std::cout<<"work?"<<std::endl;

        // 4) Return optimized angles
        return Eigen::Vector2d(theta[0], theta[1]);
    }

    Eigen::Quaterniond alignWristToMeta(
        const Eigen::Quaterniond& q1,
        const Eigen::Vector3d& p1,
        const Eigen::Vector3d& p2)
        {
            Eigen::Vector3d new_z_axis = (p1 - p2).normalized();
            Eigen::Matrix3d wristSO3= q1.normalized().toRotationMatrix();
            Eigen::Vector3d z_axis = wristSO3.col(2);

            Eigen::Vector3d screw = z_axis.cross(new_z_axis).normalized();
            double theta = pose_utils::computeAngle(z_axis,new_z_axis) * M_PI/180;

            Eigen::Matrix3d SO3 = Matexp3(screw, theta);

            Eigen::Matrix3d newframe =  SO3 * wristSO3;
            
            // std::cout << "theta" << theta <<std::endl;
            // std::cout << "z_axis" << z_axis <<std::endl;
            // std::cout << "screw" << screw <<std::endl;
            // std::cout << "new_z_axis" << new_z_axis.normalized() <<std::endl;
            // std::cout << "SO3" << SO3 <<std::endl;
            // std::cout << "newframe" << newframe <<std::endl;
            
            Eigen::Quaterniond q(newframe);

            return q;
        }
    
    

    Eigen::Matrix3d vecToso3(const Eigen::Vector3d& vec)
    {
        Eigen::Matrix3d so3 = Eigen::Matrix3d::Zero();
        so3(0,1) = -vec(2);
        so3(0,2) = vec(1);
        so3(1,2) = -vec(0);
        so3(1,0) = vec(2);
        so3(2,0) = -vec(1);
        so3(2,1) = vec(0);

        return so3;

    }
    
    Eigen::Matrix3d Matexp3(const Eigen::Vector3d& vec,const double& theta)
    {
        Eigen::Matrix3d SO3 = Eigen::Matrix3d::Zero();
        Eigen::Matrix3d so3 = vecToso3(vec);
        SO3 = Eigen::Matrix3d::Identity() + std::sin(theta) * so3 + (1-std::cos(theta)) * so3 * so3 ; 
        return SO3;
    }
}