#ifndef INRIA_MATHS_AXISANGLE_H
#define INRIA_MATHS_AXISANGLE_H

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <stdexcept>

namespace inria {

/**
 * Axis angle representation. 
 * The direction is the rotation axis and 
 * the norm is the angle in radian.
 */

/**
 * Compute and return the rotation matrix with given
 * axis. Identity is returned if null vector is given.
 * Rotation angle have to be in -M_PI:M_PI
 */
inline Eigen::Matrix3d AxisToMatrix(const Eigen::Vector3d& axis)
{
    double theta = axis.norm();
    if (fabs(theta) > M_PI) {
        throw std::logic_error("AxisAngle unbounded angle (in -M_PI:M_PI)");
    }
    if (theta <= 0.0) {
        return Eigen::Matrix3d::Identity();
    } else {
        Eigen::Vector3d vect = axis.normalized();
        Eigen::AngleAxisd axisAngle(theta, vect);
        return axisAngle.matrix();
    }
}

/**
 * Convert and return the rotation axis vector from
 * given rotation matrix.
 * Null vector is returned if identity matrix is given.
 * No check is done on input matrix format.
 * See https://en.wikipedia.org/wiki/Axis%E2%80%93angle_representation
 */
inline Eigen::Vector3d MatrixToAxis(const Eigen::Matrix3d& mat)
{
    double val = (mat.trace()-1.0)/2.0;
    if (std::fabs(val) > 1.1) {
        throw std::logic_error("inria::MatrixToAxis: Invalid input matrix");
    }
    //Clamp in case of small numerical errors
    if (val > 1.0) val = 1.0;
    if (val < -1.0) val = -1.0;
    double theta = std::acos(val);
    //Double check for NaN (acos)
    if (std::isnan(theta)) {
        throw std::logic_error("inria::MatrixToAxis: Error NaN");
    }
    Eigen::Vector3d axis = Eigen::Vector3d::Zero();
    if (std::fabs(theta) > 1e-6) {
        axis(0) = mat(2,1)-mat(1,2);
        axis(1) = mat(0,2)-mat(2,0);
        axis(2) = mat(1,0)-mat(0,1);
        axis *= theta/(2.0*std::sin(theta));
    }

    return axis;
}

/**
 * Conversion from rotation axis 
 * (first or second) differential to actual 
 * angular velocity or acceleration in world frame
 * Reference:
 * Representing attitude: Euler angles, unit quaternions, and rotation vectors
 * Diebel 2006 
 * Page 21 (eq. 259)
 * @param axis World to moving frame rotation axis (rotation vector)
 * @param axisDiff Time differential of axis (rotation vector rate)
 * @return rotation velocity or acceleration of moving frame 
 * in world frame (angular velocity/acceleration)
 */
inline Eigen::Vector3d AxisDiffToAngularDiff(
    const Eigen::Vector3d& axis, const Eigen::Vector3d& axisDiff)
{
    double v = axis.norm();
    double v1 = axis(0);
    double v2 = axis(1);
    double v3 = axis(2);
    double cv2 = cos(v/2.0);
    double sv2 = sin(v/2.0);
    double a = cv2*v - 2.0*sv2;
    //Check for identity rotation
    if (v < 1e-6) {
        return axisDiff;
    }

    //Eq. 237
    Eigen::Matrix<double, 3, 4> W;
    W <<
        -v1*sv2, v*cv2,   -v3*sv2, v2*sv2,
        -v2*sv2, v3*sv2,  v*cv2,   -v1*sv2,
        -v3*sv2, -v2*sv2, v1*sv2,  v*cv2;
    W *= 1.0/v;
    //Eq. 214
    Eigen::Matrix<double, 4, 3> G;
    G <<
        -v1*v*v*sv2,           -v2*v*v*sv2,           -v3*v*v*sv2,
        2.0*v*v*sv2 + v1*v1*a, v1*v2*a,               v1*v3*a,
        v1*v2*a,               2.0*v*v*sv2 + v2*v2*a, v2*v3*a,
        v1*v3*a,               v2*v3*a,               2.0*v*v*sv2 + v3*v3*a;
    G *= 1.0/(2.0*v*v*v);

    //Eq. 259
    return 2.0*W*G*axisDiff;
}

}

#endif

