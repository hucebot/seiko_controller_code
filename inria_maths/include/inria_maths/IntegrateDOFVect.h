#ifndef INRIA_MATHS_INTEGRATEDOFVECT_H
#define INRIA_MATHS_INTEGRATEDOFVECT_H

#include <stdexcept>
#include <Eigen/Dense>

namespace inria {

/**
 * Increment a degree of freedom position vector with 
 * a delta velocity vector while using the special 
 * RBDL storing convention for position.
 * Handle the problem of the floating base orientation
 * represented as a quaternion.
 *
 * @param vectPos Position DOF vector in RBDL 
 * format (sizeVectPos()).
 * @param vectDelta Delta DOF vector in RBDL 
 * velocity format (sizeVectVel()).
 * @return the integrated degree of freedom
 * position vector taking into account the quaternion
 * orientation (vectPos + vectDelta).
 */
inline Eigen::VectorXd IntegrateDOFVect(
    const Eigen::VectorXd& vectPos,
    const Eigen::VectorXd& vectDelta)
{
    //Size checks
    size_t sizeDOF = vectDelta.size();
    if (
        sizeDOF <= 7 ||
        (size_t)vectPos.size() != sizeDOF+1
    ) {
        throw std::logic_error(
            "inria::IntegrateDOFVect: Invalid input vectors.");
    }
    size_t sizeJoint = sizeDOF-6;

    Eigen::VectorXd vectNew = vectPos;
    Eigen::Quaterniond quatOld = Eigen::Quaterniond(
        vectPos(sizeDOF),
        vectPos(3),
        vectPos(4),
        vectPos(5));
    Eigen::Vector3d diff(
        vectDelta(3),
        vectDelta(4),
        vectDelta(5));
    vectNew.segment(0, 3) += vectDelta.segment(0, 3);
    vectNew.segment(6, sizeJoint) += vectDelta.segment(6, sizeJoint);
    Eigen::Quaterniond quatDelta;
    if (diff.norm() > 1e-10) {
        quatDelta = Eigen::Quaterniond(
            Eigen::AngleAxisd(diff.norm(), diff.normalized()));
    } else {
        quatDelta.setIdentity();
    }
    Eigen::Quaterniond quatNew = quatOld*quatDelta;
    quatNew.normalize();
    vectNew(sizeDOF) = quatNew.w();
    vectNew(3) = quatNew.x();
    vectNew(4) = quatNew.y();
    vectNew(5) = quatNew.z();

    return vectNew;
}

}

#endif

