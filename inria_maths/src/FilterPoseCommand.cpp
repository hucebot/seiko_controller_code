#include <inria_maths/FilterPoseCommand.hpp>
#include <inria_maths/Clamp.h>
#include <inria_maths/AxisAngle.h>

namespace inria {

FilterPoseCommand::FilterPoseCommand() :
    _isInitialized(false),
    _isLocalFrame(false),
    _useRawVRPose(true),
    _clampRadiusLin(),
    _clampRadiusAng(),
    _anchorPos(),
    _anchorMat(),
    _targetPos(),
    _targetMat(),
    _poseProcessing(),
    _filterLowpassPos(),
    _filterLowpassMat(),
    _filterBangbangPos(),
    _filterBangbangMat()
{
    //Default parameters and state
    setParameters(true, false, 1.0, 0.01, 0.1, 1.0, 0.01, 0.02, 0.1, 0.2);
    reset(Eigen::Vector3d::Zero(), Eigen::Matrix3d::Identity());
}

void FilterPoseCommand::setParameters(
    bool useRawVRPose,
    bool isLocalFrame,
    double scalingLin,
    double clampRadiusLin, double clampRadiusAng,
    double cutoffFreq,
    double maxVelLin, double maxAccLin,
    double maxVelAng, double maxAccAng)
{
    _useRawVRPose = useRawVRPose;
    _isLocalFrame = isLocalFrame;
    _poseProcessing.scalingLin = scalingLin;
    _clampRadiusLin = clampRadiusLin;
    _clampRadiusAng = clampRadiusAng;
    _filterLowpassPos.cutoffFrequency() = cutoffFreq;
    _filterLowpassMat.cutoffFrequency() = cutoffFreq;
    _filterBangbangPos.maxVel() = maxVelLin;
    _filterBangbangPos.maxAcc() = maxAccLin;
    _filterBangbangMat.maxVel() = maxVelAng;
    _filterBangbangMat.maxAcc() = maxAccAng;
}

void FilterPoseCommand::update(
    double dt, 
    bool isClutch,
    const Eigen::Vector3d& rawPos, 
    const Eigen::Matrix3d& rawMat, 
    const Eigen::Vector3d& velLin, 
    const Eigen::Vector3d& velAng, 
    const Eigen::Vector3d& centerPos,
    const Eigen::Matrix3d& centerMat)
{
    if (_useRawVRPose) {
        //Initial input offset
        if (!_isInitialized) {
            _poseProcessing.posOffset = rawPos;
            _poseProcessing.matOffset = rawMat;
            _isInitialized = true;
        }
        //Integrate the anchor pose and the raw pose clutch is released
        if (!isClutch && _poseProcessing.isIntegrating) {
            _anchorPos = _targetPos;
            _anchorMat = _targetMat;
            //Re-project the anchor on the clamping sphere 
            //when the target went too far away from the reference center
            Eigen::Vector3d vectLin = 
                _anchorPos-centerPos;
            Eigen::Vector3d vectAng = 
                MatrixToAxis(_anchorMat*centerMat.transpose());
            if (vectLin.norm() > _clampRadiusLin) {
                _anchorPos = 
                    centerPos + 
                    _clampRadiusLin*vectLin.normalized();
            }
            if (vectAng.norm() > _clampRadiusAng) {
                _anchorMat = 
                    AxisToMatrix(_clampRadiusAng*vectAng.normalized())
                    * centerMat;
            }
        }
        //Raw input pose processing
        _poseProcessing.update(
            isClutch, rawPos, rawMat);
        //Integration of velocity command and clamping into 
        //sphere to prevent the integrated pose to move too 
        //far from given center pose.
        Eigen::Vector3d deltaLin;
        Eigen::Vector3d deltaAng;
        if (_isLocalFrame) {
            deltaLin = dt*centerMat*velLin;
            deltaAng = dt*centerMat*velAng;
        } else {
            deltaLin = dt*velLin;
            deltaAng = dt*velAng;
        }
        _anchorPos = ClampPositionUpdateIntoSphere(
            centerPos, _clampRadiusLin, _anchorPos, deltaLin);
        _anchorMat = ClampOrientationUpdateIntoSphere(
            centerMat, _clampRadiusAng, _anchorMat, deltaAng);
        //Apply offset processed pose to anchor pose
        if (_isLocalFrame) {
            _targetPos = 
                centerMat*_poseProcessing.posCentered + _anchorPos;
            _targetMat = 
                _anchorMat*_poseProcessing.matCentered;
        } else {
            _targetPos = 
                _poseProcessing.posCentered + _anchorPos;
            _targetMat = 
                _poseProcessing.matCentered*_anchorMat;
        }
    } else {
        _targetPos = rawPos;
        _targetMat = rawMat;
    }
    //Update lowpass filters
    _filterLowpassPos.update(_targetPos, dt);
    _filterLowpassMat.update(_targetMat, dt);
    //Update bangbang filters
    _filterBangbangPos.update(_filterLowpassPos.value(), dt);
    _filterBangbangMat.update(_filterLowpassMat.valueMatrix(), dt);
}

const Eigen::Vector3d& FilterPoseCommand::anchorPos() const
{
    return _anchorPos;
}
const Eigen::Matrix3d& FilterPoseCommand::anchorMat() const
{
    return _anchorMat;
}

const Eigen::Vector3d& FilterPoseCommand::targetPos() const
{
    return _targetPos;
}
const Eigen::Matrix3d& FilterPoseCommand::targetMat() const
{
    return _targetMat;
}

const Eigen::Vector3d& FilterPoseCommand::valuePos() const
{
    return _filterBangbangPos.value();
}
const Eigen::Matrix3d& FilterPoseCommand::valueMat() const
{
    return _filterBangbangMat.value();
}

void FilterPoseCommand::reset(
    const Eigen::Vector3d& pos,
    const Eigen::Matrix3d& mat)
{
    _isInitialized = false;
    _poseProcessing.isIntegrating = false;
    _anchorPos = pos;
    _anchorMat = mat;
    _targetPos = pos;
    _targetMat = mat;
    _filterLowpassPos.reset(pos);
    _filterLowpassMat.reset(mat);
    _filterBangbangPos.reset(pos);
    _filterBangbangMat.reset(mat);
}

}

