#ifndef INRIA_MATHS_FILTERPOSECOMMAND_HPP
#define INRIA_MATHS_FILTERPOSECOMMAND_HPP

#include <Eigen/Dense>
#include <inria_maths/FilterExponential.hpp>
#include <inria_maths/FilterExponentialRotation.hpp>
#include <inria_maths/TrajectoryBangBangAcc.hpp>
#include <inria_maths/PoseCommandProcessing.hpp>

namespace inria {

/**
 * FilterPoseCommand
 *
 * Filter for Cartesian pose command using position 
 * and velocity command. Apply lowpass and bangbangacc filters
 * in addition to special clamping scheme in velocity command
 */
class FilterPoseCommand
{
    public:

        /**
         * Default initialization
         */
        FilterPoseCommand();

        /**
         * Assign filter parameters
         *
         * @param useRawVRPose If true, input pose is a raw
         * pose from VR world with Z up and is processed 
         * with isClutch. If false, input pose is directly 
         * that target pose in world frame and velocity command
         * is not used.
         * @param isLocalFrame If true, commands are assumed
         * given in local effector frame, else world frame.
         * @param scalingLin Scaling apply on linear motion of 
         * offset pôse commands.
         * @param clampRadiusLin Radius for sphere clamping 
         * of linear velocity commands.
         * @param clampRadiusAng Radius for sphere clamping
         * of angular velocity commands.
         * @param cutoffFreq Cutfoff frequency of lowpass filter.
         * @param maxVelLin Maximum linear velocity.
         * @param maxAccLin Maximum linear acceleration
         * @param maxVelAng Maximum angular velocity.
         * @param maxAccAng Maximum angular acceleration.
         */
        void setParameters(
            bool useRawVRPose,
            bool isLocalFrame,
            double scalingLin,
            double clampRadiusLin, double clampRadiusAng,
            double cutoffFreq,
            double maxVelLin, double maxAccLin,
            double maxVelAng, double maxAccAng);

        /**
         * Update internal state with given pose offset 
         * and velocity command.
         *
         * @param dt Time step.
         * @param isClutch If true, the motion of raw pose
         * should be interated.
         * @param rawPos Raw position input with Z as up.
         * àparam rawMat Raw orientation input with Z as up.
         * @param velLin Linear velocity command to be integrated.
         * @param velAng Angular velocity command to be integrated
         * @param centerPos Center position used in clamping 
         * not to move too far away (radiusLin at max).
         * @param centerMat Center orientation used in clamping 
         * not to move too far away (radiusAng at max).
         */
        void update(
            double dt, 
            bool isClutch,
            const Eigen::Vector3d& rawPos, 
            const Eigen::Matrix3d& rawMat, 
            const Eigen::Vector3d& velLin, 
            const Eigen::Vector3d& velAng, 
            const Eigen::Vector3d& centerPos,
            const Eigen::Matrix3d& centerMat);

        /**
         * @return internal integrated anchor pose
         */
        const Eigen::Vector3d& anchorPos() const;
        const Eigen::Matrix3d& anchorMat() const;
        
        /**
         * @return internal pre-filtering target pose
         */
        const Eigen::Vector3d& targetPos() const;
        const Eigen::Matrix3d& targetMat() const;

        /**
         * @return filtered position or orientation
         */
        const Eigen::Vector3d& valuePos() const;
        const Eigen::Matrix3d& valueMat() const;

        /**
         * Reset internal filter state to 
         * given position and orientation.
         */
        void reset(
            const Eigen::Vector3d& pos,
            const Eigen::Matrix3d& mat);

    private:

        /**
         * If false, raw input pose offset will
         * be initialized at next update() call
         */
        bool _isInitialized;
        
        /**
         * If true, the input raw pose is assumed to be
         * a non processed pose from VR world (with Z up).
         * If false, the input pose is the pose target 
         * in world frame (isClutch and velocity command 
         * are not used).
         */
        bool _useRawVRPose;

        /**
         * If true, commands are assmed to be in local 
         * effector frame, else in world frame
         */
        bool _isLocalFrame;

        /**
         * Radius for velocity command sphere clamping
         */
        double _clampRadiusLin;
        double _clampRadiusAng;

        /**
         * Anchor position and orientation
         * integrated reference for velocity and position 
         * command pre-filtering
         */
        Eigen::Vector3d _anchorPos;
        Eigen::Matrix3d _anchorMat;

        /**
         * Target (anchor + offset) position and orientation 
         * pre-filtering
         */
        Eigen::Vector3d _targetPos;
        Eigen::Matrix3d _targetMat;

        /**
         * Input pose processing from raw not offset pose
         */
        PoseCommandProcessing _poseProcessing;

        /**
         * Lowpass filters for position and orientation
         */
        FilterExponential<Eigen::Vector3d> _filterLowpassPos;
        FilterExponentialRotation _filterLowpassMat;

        /**
         * Max velocity and acceleration filters for
         * position and orientation
         */
        FilterBangBangAcc<Eigen::Vector3d> _filterBangbangPos;
        FilterBangBangAccRotation _filterBangbangMat;
};

}

#endif

