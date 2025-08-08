#ifndef POSE_UTILS_H
#define POSE_UTILS_H

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Geometry>
#include <cmath>
#include <iostream>

visualization_msgs::Marker vectorToArrowMarker(
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
geometry_msgs::Vector3 poseToEulerAngles(const geometry_msgs::Pose &pose_ref,const geometry_msgs::Pose &pose_target);

// Function to compute the relative quaternion given two unit quaternions.
// Returns q_relative = q_ref.conjugate() * q_target.
Eigen::Quaterniond computeRelativeQuaternion(const Eigen::Quaterniond &q_ref,
                                               const Eigen::Quaterniond &q_target);

Eigen::Vector3d computePlane(const geometry_msgs::Pose &pose_meta, const geometry_msgs::Pose &pose_proxi_1,const geometry_msgs::Pose &pose_proxi_2); 

double computeAngle(const Eigen::Vector3d &v1,const Eigen::Vector3d &v2 );

Eigen::Vector2d jointAngle(ros::Publisher& pub,const Eigen::Vector3d &normal, const geometry_msgs::Pose &pose_meta, const geometry_msgs::Pose &pose_proxi,const geometry_msgs::Pose &pose_inter);
} // namespace pose_utils

#endif // POSE_UTILS_H
