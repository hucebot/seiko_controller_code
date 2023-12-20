#ifndef INRIA_CONTROLLER_RHIOEIGEN_HPP
#define INRIA_CONTROLLER_RHIOEIGEN_HPP

#include <Eigen/Dense>

/**
 * Eigen specialized extension for RhIO wrapper
 */
namespace RhIO {

template <>
inline void WrapperVect4d::operator=(const Eigen::Quaterniond& quat)
{
    set(0, quat.x());
    set(1, quat.y());
    set(2, quat.z());
    set(3, quat.w());
}

template <>
inline WrapperVect2d::operator Eigen::Vector2d() const
{
    Eigen::Vector2d vect;
    vect.x() = get(0);
    vect.y() = get(1);
    return vect;
}

template <>
inline WrapperVect3d::operator Eigen::Vector3d() const
{
    Eigen::Vector3d vect;
    vect.x() = get(0);
    vect.y() = get(1);
    vect.z() = get(2);
    return vect;
}

template <>
inline WrapperVect4d::operator Eigen::Quaterniond() const
{
    Eigen::Quaterniond quat;
    quat.x() = get(0);
    quat.y() = get(1);
    quat.z() = get(2);
    quat.w() = get(3);
    return quat;
}

}

#endif

