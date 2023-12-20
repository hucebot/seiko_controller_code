#include <inria_maths/PoseCommandProcessing.hpp>
#include <inria_maths/Euler.h>

namespace inria {

PoseCommandProcessing::PoseCommandProcessing() :
    scalingLin(1.0),
    posOffset(Eigen::Vector3d::Zero()),
    matOffset(Eigen::Matrix3d::Identity()),
    posCentered(Eigen::Vector3d::Zero()),
    matCentered(Eigen::Matrix3d::Identity()),
    isIntegrating(false),
    posAnchor(Eigen::Vector3d::Zero()),
    matAnchor(Eigen::Matrix3d::Identity()),
    posIntegrated(Eigen::Vector3d::Zero()),
    matIntegrated(Eigen::Matrix3d::Identity())
{
}

void PoseCommandProcessing::update(
    bool isClutch,
    const Eigen::Vector3d& posRaw,
    const Eigen::Matrix3d& matRaw)
{
    //Reset offset when the clutch is not enabled
    if (!isClutch) {
        posOffset = posRaw;
        matOffset = matRaw;
    }
    //Integrate pose when the clutch is released
    if (isIntegrating && !isClutch) {
        posAnchor = posIntegrated;
        matAnchor = matIntegrated;
    }
    //Compute offset pose from raw pose.
    //The orientation offset is split into the yaw component 
    //and the remaining roll/pitch components.
    Eigen::Vector3d tmpEuler1 = MatrixToEulerIntrinsic(matOffset);
    Eigen::Vector3d tmpEuler2 = MatrixToEulerIntrinsic(matOffset);
    tmpEuler1.x() = 0.0;
    tmpEuler1.y() = 0.0;
    tmpEuler2.z() = 0.0;
    Eigen::Matrix3d tmpMat1 = EulerIntrinsicToMatrix(tmpEuler1);
    Eigen::Matrix3d tmpMat2 = EulerIntrinsicToMatrix(tmpEuler2);
    posCentered = 
        tmpMat1.transpose()*scalingLin*(posRaw - posOffset);
    matCentered = 
        tmpMat1.transpose()*matRaw*tmpMat2.transpose();
    //Compute integrated poses. Reference pose is integrated 
    //when the clutch is released. Centered pose is added to 
    //the reference pose in the frame of the reference pose.
    if (isIntegrating) {
        posIntegrated = posAnchor + matAnchor*posCentered;
        matIntegrated = matAnchor*matCentered;
    } else {
        posIntegrated = posAnchor;
        matIntegrated = matAnchor;
    }
    isIntegrating = isClutch;
}

}

