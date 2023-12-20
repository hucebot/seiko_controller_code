#ifndef INRIA_MATHS_SKEW_H
#define INRIA_MATHS_SKEW_H

#include <Eigen/Dense>

namespace inria {

/**
 * Build and return the skew rotation matrix
 * from given angular velocity vector
 */
inline Eigen::Matrix3d SkewMatrix(const Eigen::Vector3d& vect)
{
    Eigen::Matrix3d mat;
    mat << 
        0.0,       -vect.z(), +vect.y(),
        +vect.z(), 0.0,       -vect.x(),
        -vect.y(), +vect.x(), 0.0;
    return mat;
}

}

#endif

