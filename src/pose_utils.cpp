#include <ros/ros.h>
#include <pose_utils.h>
#include <HMD.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <iostream>
#include <Eigen/Geometry>
#include <cmath>
namespace pose_utils {
    // Function to convert a geometry_msgs::Pose's quaternion into Euler angles (roll, pitch, yaw)
    geometry_msgs::Vector3 poseToEulerAngles(const geometry_msgs::Pose &pose_ref,const geometry_msgs::Pose &pose_target) {
        // Create an Eigen quaternion from the pose's orientation.
        // Eigen::Quaterniond takes the order (w, x, y, z)
        Eigen::Quaterniond q_ref(
            pose_ref.orientation.w,
            pose_ref.orientation.x,
            pose_ref.orientation.y,
            pose_ref.orientation.z
        );
        Eigen::Quaterniond q_target(
            pose_target.orientation.w,
            pose_target.orientation.x,
            pose_target.orientation.y,
            pose_target.orientation.z
        );
        
        // Normalize the quaternion to ensure numerical stability
        q_ref.normalize();
        q_target.normalize();

        Eigen::Quaterniond q_relative = q_ref.conjugate() * q_target;

        // Compute Euler angles directly from the quaternion.
        // Roll (rotation around X-axis)
        double roll  = std::atan2(2.0 * (q_relative.w() * q_relative.x() + q_relative.y() * q_relative.z()),
        1.0 - 2.0 * (q_relative.x() * q_relative.x() + q_relative.y() * q_relative.y()));
        double pitch = std::asin(2.0 * (q_relative.w() * q_relative.y() - q_relative.z() * q_relative.x()));
        double yaw   = std::atan2(2.0 * (q_relative.w() * q_relative.z() + q_relative.x() * q_relative.y()),
            1.0 - 2.0 * (q_relative.y() * q_relative.y() + q_relative.z() * q_relative.z()));

        // std::cout << "Euler angles (radians):\n";
        // std::cout << "Roll: "  << roll  << ", Pitch: " << pitch << ", Yaw: " << yaw << "\n";
        // std::cout << "Euler angles (degrees):\n";
        // std::cout << "Roll: "  << roll * 180.0 / M_PI  
        //           << ", Pitch: " << pitch * 180.0 / M_PI 
        //           << ", Yaw: " << yaw * 180.0 / M_PI << "\n";
        geometry_msgs::Vector3 euler_angles;
        euler_angles.x = roll;
        euler_angles.y = pitch;
        euler_angles.z = yaw;

        return euler_angles;
    }


    Eigen::Quaterniond computeRelativeQuaternion(const Eigen::Quaterniond &q_ref, 
        const Eigen::Quaterniond &q_target) {
    // For unit quaternions, the inverse is equal to the conjugate.
    // The relative quaternion is computed as: q_relative = q_ref^{-1} * q_target.
    return q_ref.conjugate() * q_target;
    }

    // Eigen::Quaterniond computeNewcord(const geometry_msgs::Pose &pose_wrist,const geometry_msgs::Pose &pose_target,const geometry_msgs::Pose &pose_palm) {
    //     Eigen::Vector3d wrist_position(
    //         pose_wrist.position.x,
    //         pose_wrist.position.y,
    //         pose_wrist.position.z
    //     );  

    //     Eigen::Quaterniond q_wrist(
    //         pose_wrist.orientation.w,
    //         pose_wrist.orientation.x,
    //         pose_wrist.orientation.y,
    //         pose_wrist.orientation.z
    //     );

    //     Eigen::Vector3d target_position(
    //         pose_target.position.x,
    //         pose_target.position.y,
    //         pose_target.position.z
    //     );

    //     Eigen::Vector3d palm_position(
    //         pose_palm.position.x,
    //         pose_palm.position.y,
    //         pose_palm.position.z
    //     );


    //     Eigen::Vector3d z_axis = (wrist_position - target_position).normalized();
    //     // Eigen::Vector3d z_axis = (wrist_position - target_position).normalized();



    // }

}