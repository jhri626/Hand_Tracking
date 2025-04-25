// lie_utils.cpp
// This file contains Lie algebra utilities for so(3) operations.

#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <pose_utils.h>
#include <lie_utils.h>

// protion of Kinematics using skrew 
// have to be updated more...

namespace lie_utils {

Eigen::Quaterniond axis_align( // function to align one axis to one vector
    const Eigen::Quaterniond& q1,
    const Eigen::Vector3d& p1,
    const Eigen::Vector3d& p2)
    {
        Eigen::Vector3d new_z_axis = (p1 -p2).normalized();
        Eigen::Matrix3d wristSO3= q1.normalized().toRotationMatrix();
        Eigen::Vector3d z_axis = wristSO3.col(2);

        Eigen::Vector3d screw = z_axis.cross(new_z_axis).normalized();
        double theta = pose_utils::computeAngle(z_axis,new_z_axis) * M_PI/180;

        Eigen::Matrix3d SO3 = Matexp3(screw, theta);

        Eigen::Matrix3d newframe =  SO3 * wristSO3;
        
        // std::cout << "theta" << theta <<std::endl;
        // std::cout << "z_axis" << z_axis <<std::endl;
        // std::cout << "screw" << screw <<std::endl;
        // std::cout << "new_z_axis" << new_z_axis <<std::endl;
        // std::cout << "SO3" << SO3 <<std::endl;
        // std::cout << "newframe" << newframe <<std::endl;
        
        Eigen::Quaterniond q(newframe);

        return q;
    }

// Converts a 3D vector into its corresponding skew-symmetric matrix (so(3))
Eigen::Matrix3d vecToso3(const Eigen::Vector3d& vec) {
    Eigen::Matrix3d so3 = Eigen::Matrix3d::Zero();
    so3(0,1) = -vec(2);
    so3(0,2) =  vec(1);
    so3(1,2) = -vec(0);
    so3(1,0) =  vec(2);
    so3(2,0) = -vec(1);
    so3(2,1) =  vec(0); // Replicated from original implementation
    return so3;
}

// Computes the matrix exponential of an so(3) element: exp([vec]_x * theta)
Eigen::Matrix3d Matexp3(const Eigen::Vector3d& vec, double theta) {
    Eigen::Matrix3d so3 = vecToso3(vec);
    return Eigen::Matrix3d::Identity()
         + std::sin(theta) * so3
         + (1.0 - std::cos(theta)) * so3 * so3;
}

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

    // TODO : Make SE(3) & SO(3) inverse function
    Eigen::Matrix4d T_rel = T1.inverse() * T2;
    return T_rel;
}



} 
