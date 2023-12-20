#ifndef INRIA_MATHS_POSECOMMANDPROCESSING_HPP
#define INRIA_MATHS_POSECOMMANDPROCESSING_HPP

#include <Eigen/Dense>

namespace inria {

/**
 * PoseCommandProcessing
 *
 * Processing of raw pose outputted by 
 * the Vive tracking. 
 * Up is assumed to be positive Z axis.
 */
struct PoseCommandProcessing
{
    //Scaling factor parameter applied on 
    //integrated translation motion.
    double scalingLin;
    //Calibration offset pose
    Eigen::Vector3d posOffset;
    Eigen::Matrix3d matOffset;
    //Pose of the hand offset at the origin
    //when trigger is not pushed
    Eigen::Vector3d posCentered;
    Eigen::Matrix3d matCentered;
    //Anchor and integrated pose from 
    //motion of centered pose
    bool isIntegrating;
    Eigen::Vector3d posAnchor;
    Eigen::Matrix3d matAnchor;
    Eigen::Vector3d posIntegrated;
    Eigen::Matrix3d matIntegrated;

    /**
     * Empty initialization
     */
    PoseCommandProcessing();

    /**
     * Update internal pose processing 
     * with given raw pose.
     *
     * @param isClutch If true, pose motion integration
     * is enabled. Anchor coordinate frame is reset and offset 
     * when clutch is enabled.
     * @param posRaw Absolute position with A as up.
     * @param matRaw Absolute orientation with Z as up.
     */
    void update(
        bool isClutch,
        const Eigen::Vector3d& posRaw,
        const Eigen::Matrix3d& matRaw);
};

}

#endif

