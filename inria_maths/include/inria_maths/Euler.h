#ifndef INRIA_MATHS_EULER_H
#define INRIA_MATHS_EULER_H

#include <cmath>
#include <Eigen/Dense>

namespace inria {

/**
 * Manually convert the given rotation matrix
 * to [Roll, Pitch, Yaw] ZYX intrinsic euler 
 * angle (Better range than Eigen conversion).
 */
inline Eigen::Vector3d MatrixToEulerIntrinsic(
    const Eigen::Matrix3d& mat)
{
    //Eigen euler angles and with better range)
    Eigen::Vector3d angles;
    //Roll
    angles.x() = std::atan2(mat(2, 1), mat(2, 2));
    //Pitch
    angles.y() = std::atan2(-mat(2, 0), 
        sqrt(mat(0, 0)*mat(0, 0) 
            + mat(1, 0)*mat(1, 0)));
    //Yaw
    angles.z() = std::atan2(mat(1, 0), mat(0, 0));

    return angles;
}

/**
 * Convert given Euler angles in [Roll, Pitch, Yaw]
 * ZYX intrinsic format to rotation matrix
 */
inline Eigen::Matrix3d EulerIntrinsicToMatrix(
    const Eigen::Vector3d& angles)
{
    Eigen::AngleAxisd yawRot(angles.z(), Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd pitchRot(angles.y(), Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd rollRot(angles.x(), Eigen::Vector3d::UnitX());
    Eigen::Quaternion<double> quat = yawRot * pitchRot * rollRot;
    return quat.matrix();
}

}

#endif

