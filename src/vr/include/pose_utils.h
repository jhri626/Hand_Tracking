#ifndef POSE_UTILS_H
#define POSE_UTILS_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <Eigen/Geometry>
#include <string>
#include <cmath>
#include <iostream>

visualization_msgs::msg::Marker vectorToArrowMarker(
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& vec,
    const std::string& frame_id,
    const std::string& ns,
    int id,
    float r, float g, float b
);

namespace pose_utils {

// Function to compute the relative rotation between two poses' orientations.
// The function computes the relative quaternion:
//   q_relative = q_ref.conjugate() * q_target
// and then converts it to Euler angles (roll, pitch, yaw),
// printing the result in both radians and degrees.
geometry_msgs::msg::Vector3 poseToEulerAngles(
    const geometry_msgs::msg::Pose& pose_ref,
    const geometry_msgs::msg::Pose& pose_target
);

// Function to compute the relative quaternion given two unit quaternions.
// Returns q_relative = q_ref.conjugate() * q_target.
Eigen::Quaterniond computeRelativeQuaternion(const Eigen::Quaterniond &q_ref,
                                            const Eigen::Quaterniond &q_target);


double computeAngle(
    const Eigen::Vector3d &v1,
    const Eigen::Vector3d &v2 
);

Eigen::Vector3d computePlane(
    const geometry_msgs::msg::Pose& pose_meta,
    const geometry_msgs::msg::Pose& pose_proxi_1,
    const geometry_msgs::msg::Pose& pose_proxi_2
);

Eigen::Vector2d jointAngle(
const Eigen::Vector3d& normal,
const geometry_msgs::msg::Pose& pose_meta,
const geometry_msgs::msg::Pose& pose_proxi,
const geometry_msgs::msg::Pose& pose_inter
);

} // namespace pose_utils

#endif // POSE_UTILS_H
