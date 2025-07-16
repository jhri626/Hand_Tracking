#include <ros/ros.h>
#include <pose_utils.h>
#include <HMD.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <iostream>
#include <Eigen/Geometry>
#include <cmath>

visualization_msgs::Marker vectorToArrowMarker(
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& vec,
    const std::string& frame_id,
    const std::string& ns,
    int id,
    float r, float g, float b
) {
    visualization_msgs::Marker arrow;
    arrow.header.frame_id = frame_id;
    arrow.header.stamp = ros::Time::now();
    arrow.ns = ns;
    arrow.id = id;
    arrow.type = visualization_msgs::Marker::ARROW;
    arrow.action = visualization_msgs::Marker::ADD;

    geometry_msgs::Point p_start, p_end;
    p_start.x = start.x(); p_start.y = start.y(); p_start.z = start.z();
    p_end.x = start.x() + vec.x();
    p_end.y = start.y() + vec.y();
    p_end.z = start.z() + vec.z();

    arrow.points.push_back(p_start);
    arrow.points.push_back(p_end);

    arrow.scale.x = 0.01;  // shaft diameter
    arrow.scale.y = 0.02;  // head diameter
    arrow.scale.z = 0.02;  // head length

    arrow.color.r = r;
    arrow.color.g = g;
    arrow.color.b = b;
    arrow.color.a = 1.0;

    return arrow;
}


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

    Eigen::Vector3d computePlane(const geometry_msgs::Pose &pose_meta, const geometry_msgs::Pose &pose_proxi_1,const geometry_msgs::Pose &pose_proxi_2) 
    {
        
        Eigen::Vector3d meta_position(
            pose_meta.position.x,
            pose_meta.position.y,
            pose_meta.position.z
        );  

        Eigen::Vector3d proxi_position_1(
            pose_proxi_1.position.x,
            pose_proxi_1.position.y,
            pose_proxi_1.position.z
        );

        Eigen::Vector3d proxi_position_2(
            pose_proxi_2.position.x,
            pose_proxi_2.position.y,
            pose_proxi_2.position.z
        );

        Eigen::Vector3d v1 = proxi_position_1 - meta_position;
        Eigen::Vector3d v2 = proxi_position_2 - meta_position;

        Eigen::Vector3d normal = v2.cross(v1);
        normal.normalize(); // Optional: normalize the normal vector
        
        return normal;
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

    Eigen::Vector2d jointAngle(ros::Publisher& pub,Eigen::Vector3d &normal,const geometry_msgs::Pose &pose_meta, const geometry_msgs::Pose &pose_proxi,const geometry_msgs::Pose &pose_inter)
    {
        Eigen::Vector3d meta_position(
            pose_meta.position.x,
            pose_meta.position.y,
            pose_meta.position.z
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

        Eigen::Vector3d v1 = proxi_position - meta_position;
        Eigen::Vector3d v3 = inter_position - proxi_position;

        Eigen::Quaterniond q_meta(
            pose_meta.orientation.w,
            pose_meta.orientation.x,
            pose_meta.orientation.y,    
            pose_meta.orientation.z
        );

        Eigen::Matrix3d mat = q_meta.normalized().toRotationMatrix();
        Eigen::Vector3d x_axis = mat.col(0);

        Eigen::Vector3d v4 = normal.cross(v1);
        Eigen::Vector3d projectionV3 = v3 - v3.dot(normal)*normal;
        


        
        double jointFE = computeAngle(v3,projectionV3);
        double jointAA = 90 - computeAngle(v3,v4);

        if(v3.dot(v1)<0)
        {
            std::cout<<"reverse"<<std::endl;
            jointFE = 90 + jointFE;
            jointAA = 180 - jointAA ;
        } // heuistic way

        // if (x_axis.dot(v3)<0)
        // {
        //     jointAA = -jointAA;
        // }

        Eigen::Vector2d angle(
            jointFE,
            jointAA
        );
        
        return angle;
    }
}


