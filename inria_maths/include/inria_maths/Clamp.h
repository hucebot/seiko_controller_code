#ifndef INRIA_MATHS_CLAMP_H
#define INRIA_MATHS_CLAMP_H

#include <algorithm>
#include <stdexcept>
#include <Eigen/Dense>
#include <inria_maths/AxisAngle.h>

namespace inria {

/**
 * Bound and return the given value within [min:max] range
 */
template <typename T>
inline T ClampRange(T value, T min, T max)
{
    if (max < min) {
        throw std::logic_error(
            "inria::ClampRange: Invalid range.");
    }
    return std::min(std::max(value, min), max);
}

/**
 * Bound and return the given value within [-max:max] range
 */
template <typename T>
inline T ClampAbsolute(T value, T max)
{
    if (max < 0.0) {
        throw std::logic_error(
            "inria::ClampAbsolute: Invalid absolute bound.");
    }
    return ClampRange(value, -max, max);
}

/**
 * Clamp a vector norm according 
 * to given maximum norm limit
 */
inline Eigen::VectorXd ClampVectorNorm(
    const Eigen::VectorXd& vect,
    double limitMaxNorm)
{
    double currentNorm = vect.norm();
    if (currentNorm > limitMaxNorm) {
        return limitMaxNorm*vect.normalized();
    } else {
        return vect;
    }
}

/**
 * Clamp a vector absolute components 
 * according to given maximum component limit
 */
inline Eigen::VectorXd ClampVectorMaxComponent(
    const Eigen::VectorXd& vect,
    double limitMaxValue)
{
    double currentMax = vect.cwiseAbs().maxCoeff();
    if (currentMax > limitMaxValue) {
        return limitMaxValue*vect.normalized();
    } else {
        return vect;
    }
}

/**
 * Clamp all absolute vector components 
 * according to given maximum component limit
 */
inline Eigen::VectorXd ClampVectorAllComponent(
    const Eigen::VectorXd& vect,
    double limitMaxValue)
{
    Eigen::VectorXd cpyVect = vect;
    for (size_t i=0;i<(size_t)cpyVect.size();i++) {
        cpyVect(i) = ClampAbsolute(cpyVect(i), limitMaxValue);
    }
    return cpyVect;
}

/**
 * Compute and return the signed distance from given value 
 * to the bounds of the [min:max] range if value is outside 
 * or zero if it is inside.
 */
template <typename T>
inline T ClampDistFromRange(T value, T min, T max)
{
    if (max < min) {
        throw std::logic_error(
            "inria::ClampDistFromRange: Invalid range.");
    }

    if (value > max) {
        return max - value;
    }
    if (value < min) {
        return min - value;
    } else {
        return 0.0;
    }
}

/**
 * Compute and return the signed distance from given value 
 * to the bounds of the [-max:max] range if value is outside 
 * or zero if it is inside.
 */
template <typename T>
inline T ClampDistFromAbsolute(T value, T max)
{
    if (max < 0.0) {
        throw std::logic_error(
            "inria::ClampDistFromAbsolute: Invalid absolute bound.");
    }
    return ClampDistFromRange(value, -max, max);
}

/**
 * Add delta vector to given point position or orientation while
 * clamping the (sphere) distance to a given reference.
 * The clamping only happens along the direction 
 * of the given update vector.
 *
 * @param center Position or orientation reference 
 * at the center of the valid space.
 * @param radius The sphere radius around center 
 * defining the valid space.
 * @param point Position or orientation Initial point 
 * from where to compute the update.
 * @param delta Desired velocity or axis angle vector 
 * to add to point.
 */
inline Eigen::Vector3d ClampPositionUpdateIntoSphere(
    const Eigen::Vector3d& center,
    double radius,
    const Eigen::Vector3d& point,
    const Eigen::Vector3d& delta)
{
    double deltaNorm = delta.norm();
    if (deltaNorm < 1e-6) {
        //No update delta
        return point;
    }

    //Compute solutions for line sphere intersection
    Eigen::Vector3d deltaVect = delta.normalized();
    Eigen::Vector3d tmpVect = point - center;
    double discriminant = 
        std::pow(deltaVect.dot(tmpVect), 2) - (tmpVect.squaredNorm() - radius*radius);
    if (discriminant >= 0.0) {
        //Select the farer point on the sphere in the direction of the 
        //delta vector or the closer if the directions are both negative
        double solution1 = -deltaVect.dot(tmpVect) + std::sqrt(discriminant);
        double solution2 = -deltaVect.dot(tmpVect) - std::sqrt(discriminant);
        double length = std::max(solution1, solution2);
        //Clamp this delta to update length
        length = std::min(deltaNorm, length);
        //Return updated point
        return point + length*deltaVect;
    } else {
        //No solution
        return point;
    }
}
inline Eigen::Matrix3d ClampOrientationUpdateIntoSphere(
    const Eigen::Matrix3d& center,
    double radius,
    const Eigen::Matrix3d& point,
    const Eigen::Vector3d& delta)
{
    double deltaNorm = delta.norm();
    if (deltaNorm < 1e-6) {
        //No update delta
        return point;
    }

    //Compute solutions for line sphere intersection
    Eigen::Vector3d deltaVect = delta.normalized();
    Eigen::Vector3d tmpVect = MatrixToAxis(point*center.transpose());
    double discriminant = 
        std::pow(deltaVect.dot(tmpVect), 2) - (tmpVect.squaredNorm() - radius*radius);
    if (discriminant >= 0.0) {
        //Select the farer point on the sphere in the direction of the 
        //delta vector or the closer if the directions are both negative
        double solution1 = -deltaVect.dot(tmpVect) + std::sqrt(discriminant);
        double solution2 = -deltaVect.dot(tmpVect) - std::sqrt(discriminant);
        double length = std::max(solution1, solution2);
        //Clamp this delta to update length
        length = std::min(deltaNorm, length);
        //Return updated point
        return AxisToMatrix(length*deltaVect)*point;
    } else {
        //No solution
        return point;
    }
}

}

#endif

