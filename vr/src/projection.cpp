#include <cmath>
#include <pose_utils.h> 
#include <iostream>
#include <Eigen/Geometry>
#include <lie_utils.h>

namespace proj{
    Eigen::Vector3d projectOntoSphere(const Eigen::Vector3d &v, double radius) {
        const double norm = v.norm();
        if (norm == 0.0) {
            throw std::invalid_argument("Cannot project the origin onto the sphere.");
        }
        return v.normalized() * radius;
    }
    
    // Convert Cartesian coordinates to spherical coordinates
    // @param v: the input point in Cartesian (assumed non-zero)
    // @param out_r: output radius
    // @param out_inclination: output inclination (polar angle) in radians
    // @param out_azimuth: output azimuth (azimuthal angle) in radians
    Eigen::Vector2d cartesianToSpherical(const Vector3d &v,
                                     double &out_r,
                                     double &out_inclination,
                                     double &out_azimuth) {
        out_r = v.norm();
        if (out_r == 0.0) {
            throw std::invalid_argument("Cannot compute spherical coordinates of the origin.");
        }
        // Polar angle measured from the positive Z-axis
        out_inclination = std::acos(v.z() / out_r);
        // Azimuthal angle in the X-Y plane from positive X-axis
        out_azimuth = std::atan2(v.y(), v.x());
    }
}