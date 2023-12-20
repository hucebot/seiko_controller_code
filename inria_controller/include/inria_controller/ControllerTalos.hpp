#ifndef INRIA_CONTROLLER_CONTROLLERTALOS_HPP
#define INRIA_CONTROLLER_CONTROLLERTALOS_HPP

#include <RhIO.hpp>
#include <inria_controller/ControllerBase.hpp>
#include <inria_controller/TaskBaseTalos.hpp>
#include <inria_utils/CircularBuffer.hpp>
#include <inria_utils/TransportValueUDP.hpp>

namespace inria {

/**
 * ControllerTalos
 *
 * ROS control real time controller
 * for Talos robot.
 */
class ControllerTalos : public ControllerBase<
    ControllerTalosState_t, ControllerTalosCommand_t>
{
    public:
        
        /**
         * Specific Talos initialization
         */
        ControllerTalos();

        /**
         * Initialization from a non-realtime thread.
         * Manual initialization in order to claim and
         * access to all robot sensors
         * 
         * @param hw The specific hardware interface 
         * to the robot used by this controller.
         * @param root_nh A NodeHandle in the root of the controller 
         * manager namespace.
         * This is where the ROS interfaces are setup 
         * (publishers, subscribers, services).
         * @param controller_nh A NodeHandle in the namespace of 
         * the controller.
         * This is where the controller-specific configuration resides.
         * @param[out] claimed_resources The resources 
         * claimed by this controller.
         * @returns True if initialization was successful 
         * and the controller is ready to be started.
         */
        virtual bool initRequest(
            hardware_interface::RobotHW* robot_hw,
            ros::NodeHandle& root_nh, 
            ros::NodeHandle& controller_nh,
            controller_interface::ControllerBase::ClaimedResources& 
                claimed_resources) override;

    private:
        
        /**
         * Receiver for UDP value assignment forwarded to RhIO
         */
        TransportValueUDPServer _serverUDP;

        /**
         * Model frame indexes for state estimation 
         */
        unsigned int _indexFrameInModelFootSoleLeft;
        unsigned int _indexFrameInModelFootSoleRight;
        unsigned int _indexFrameInModelFootFTLeft;
        unsigned int _indexFrameInModelFootFTRight;
        unsigned int _indexFrameInModelHandFrameLeft;
        unsigned int _indexFrameInModelHandFrameRight;
        unsigned int _indexFrameInModelHandFTLeft;
        unsigned int _indexFrameInModelHandFTRight;
        unsigned int _indexFrameInModelIMULink;
        unsigned int _indexFrameInModelTorsoLink;
        size_t _indexInContainerFTFootLeft;
        size_t _indexInContainerFTFootRight;
        size_t _indexInContainerFTHandLeft;
        size_t _indexInContainerFTHandRight;
        size_t _indexInContainerIMUBaseLink;

        /**
         * Offset biases and circular buffers for feet 
         * force-torque and joint torque sensors calibration.
         * 4000 samples at 2kHz = 2 seconds.
         */
        Eigen::Vector6d _biasWrenchFootLeft;
        Eigen::Vector6d _biasWrenchFootRight;
        Eigen::Vector6d _biasWrenchHandLeft;
        Eigen::Vector6d _biasWrenchHandRight;
        Eigen::VectorXd _biasTorqueJoint;
        CircularBuffer<Eigen::Vector6d, 4000> _bufferWrenchFootLeft;
        CircularBuffer<Eigen::Vector6d, 4000> _bufferWrenchFootRight;
        CircularBuffer<Eigen::Vector6d, 4000> _bufferWrenchHandLeft;
        CircularBuffer<Eigen::Vector6d, 4000> _bufferWrenchHandRight;
        CircularBuffer<Eigen::VectorXd, 4000> _bufferTorqueJoint;

        /**
         * RhIO wrapper for tare configuration and 
         * wrench state monitoring
         */
        RhIO::WrapperBool _rhioIsTransformFT;
        RhIO::WrapperBool _rhioAskTare;
        RhIO::WrapperBool _rhioAskTareFeet;
        RhIO::WrapperBool _rhioAskTareHands;
        RhIO::WrapperVect4d _rhioStateBaseQuat;
        RhIO::WrapperVect3d _rhioStateFootLeftTorque;
        RhIO::WrapperVect3d _rhioStateFootLeftForce;
        RhIO::WrapperVect2d _rhioStateFootLeftCoP;
        RhIO::WrapperVect2d _rhioStateFootLeftFriction;
        RhIO::WrapperVect3d _rhioStateFootRightForce;
        RhIO::WrapperVect3d _rhioStateFootRightTorque;
        RhIO::WrapperVect2d _rhioStateFootRightCoP;
        RhIO::WrapperVect2d _rhioStateFootRightFriction;
        RhIO::WrapperVect3d _rhioStateHandLeftTorque;
        RhIO::WrapperVect3d _rhioStateHandLeftForce;
        RhIO::WrapperVect3d _rhioStateHandRightTorque;
        RhIO::WrapperVect3d _rhioStateHandRightForce;
        
        /**
         * Indexes for gripper hardware resource
         */
        size_t _indexInContainerJointPosGripperLeft;
        size_t _indexInContainerJointPosGripperRight;
        
        /**
         * RhIO wrapper and filter for target 
         * gripper closing position ratio
         */
        RhIO::WrapperFloat _rhioGripperLeftPosRatio;
        RhIO::WrapperFloat _rhioGripperRightPosRatio;
        RhIO::WrapperFloat _rhioGripperLeftCurrentRatio;
        RhIO::WrapperFloat _rhioGripperRightCurrentRatio;
        FilterBangBangAcc<double> _filterPosGripperLeft;
        FilterBangBangAcc<double> _filterPosGripperRight;

        /**
         * Overload floating base state estimation and
         * state estimation for feet wrench estimation
         */
        virtual void updateAssignTaskState(double time, double dt) override;
        
        /**
         * Overload base command processing
         */
        virtual void updateCommandInterpolation(double time, double dt) override;
};

}

#endif

