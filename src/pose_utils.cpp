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

    Eigen::Vector4d computePlane(const Eigen::Vector3d &v1,const Eigen::Vector3d &v2,const Eigen::Vector3d &palm_position) {

        Eigen::Vector3d normal = v1.cross(v2);
        normal.normalize(); // Optional: normalize the normal vector

        double D = -normal.dot(palm_position);

        Eigen::Vector4d plane;
        plane << normal, D;
        return plane;
    }

    double computeAngle(const Eigen::Vector3d &v1,const Eigen::Vector3d &v2 )
    {
        Eigen::Vector3d u1 = v1.normalized();
        Eigen::Vector3d u2 = v2.normalized();
        double dot = u1.dot(u2);
        dot = std::min(1.0, std::max(-1.0, dot));
        double angle_rad = std::acos(dot);

        // Optional: convert to degrees
        double angle_deg = angle_rad * 180.0 / M_PI;
        // TODO: fix this part angle could be negative value
        // if(angle_deg > 90) angle_deg = 180.0-angle_deg;
        
        return angle_deg;
    }

    Eigen::Vector2d jointAngle(const geometry_msgs::Pose &pose_meta,const geometry_msgs::Pose &pose_palm,
        const geometry_msgs::Pose &pose_proxi,const geometry_msgs::Pose &pose_inter)
    {
        Eigen::Vector3d meta_position(
            pose_meta.position.x,
            pose_meta.position.y,
            pose_meta.position.z
        );  

        Eigen::Vector3d palm_position(
            pose_palm.position.x,
            pose_palm.position.y,
            pose_palm.position.z
        );

        Eigen::Vector3d proxi_position(
            pose_proxi.position.x,
            pose_proxi.position.y,
            pose_proxi.position.z
        );

        Eigen::Vector3d inter_position(
            pose_inter.position.x,
            pose_inter.position.y,
            pose_inter.position.z
        );  

        Eigen::Vector3d v1 = proxi_position - palm_position;
        Eigen::Vector3d v2 = meta_position - palm_position;
        Eigen::Vector3d v3 = inter_position - proxi_position;

        Eigen::Quaterniond q_meta(
            pose_meta.orientation.w,
            pose_meta.orientation.x,
            pose_meta.orientation.y,
            pose_meta.orientation.z
        );

        Eigen::Matrix3d mat = q_meta.normalized().toRotationMatrix();
        Eigen::Vector3d x_axis = mat.col(0);


        Eigen::Vector4d plane = computePlane(v1,v2,palm_position);
        Eigen::Vector3d normal = plane.head<3>();
        Eigen::Vector3d projectionV3 = v3 - v3.dot(normal)*normal;

        
        double jointFE = computeAngle(v3,projectionV3);
        double jointAA = computeAngle(projectionV3,v1-v2);

        if(v3.dot(v1-v2)<0)
        {
            std::cout<<"reverse"<<std::endl;
            jointFE = 90 + jointFE;
            jointAA = 180 - jointAA ;
        } // heuistic way

        if (x_axis.dot(v3)<0)
        {
            jointAA = -jointAA;
        }

        Eigen::Vector2d angle(
            jointFE,
            jointAA
        );
        
        return angle;
    }



}