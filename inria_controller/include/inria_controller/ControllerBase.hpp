#ifndef INRIA_CONTROLLER_CONTROLLERBASE_HPP
#define INRIA_CONTROLLER_CONTROLLERBASE_HPP

#include <vector>
#include <map>
#include <string>
#include <atomic>
#include <memory>
#include <Eigen/Dense>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wreturn-stack-address"
#pragma GCC diagnostic ignored "-Wreinterpret-base-class"
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/force_torque_sensor_interface.h>
#include <hardware_interface/imu_sensor_interface.h>
#pragma GCC diagnostic pop
#include <pal_hardware_interfaces/current_limit_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <RhIO.hpp>
#include <inria_maths/FilterExponential.hpp>
#include <inria_maths/FilterExponentialRotation.hpp>
#include <inria_controller/TaskBase.hpp>
#include <inria_controller/ThreadManager.hpp>

namespace inria {

/**
 * ControllerBase
 *
 * Base class for ROS control real time controller
 * communicating with PAL hardware.
 * @param T_State Type for controller state data.
 * @param T_Command Type for controller command data;
 */
template <typename T_State, typename T_Command>
class ControllerBase : public controller_interface::ControllerBase
{
    public:
        
        /**
         * Creation.
         */
        ControllerBase();

        /**
         * Destruction.
         */
        virtual ~ControllerBase();
        
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

        /** 
         * Called from the real-time thread 
         * just before the first call to update().
         *
         * @param time The current time
         */
        virtual void starting(const ros::Time& time) override;

        /** 
         * Called periodically by the realtime thread when 
         * the controller is running.
         * 
         * @param time The current time
         * @param period The time passed since the 
         * last call to update()
         */
        virtual void update(
            const ros::Time& time, 
            const ros::Duration& period) override;

        /** 
         * Called from the realtime thread just after the last
         * update call before the controller is stopped.
         *
         * @param time The current time
         */
        virtual void stopping(const ros::Time& time) override;

    protected:

        /**
         * Data structure to hold lowlevel ros control
         * handles and processing and logging data
         */
        //Position command joint
        struct DataJointPos_t {
            //Resource name
            std::string name;
            //Resource handle on joint state and command
            hardware_interface::JointHandle handleJoint;
            //Resource handle on joint current limit
            pal_ros_control::CurrentLimitHandle handleCurrent;
            //True is the joint has absolute position sensor
            bool hasAbsPos;
            //True if the joint has available current limit
            bool hasCurrent;
            //True if the joint has torque sensor
            bool hasTorque;
            //Desired current ratio when running
            double currentRatioCommanded;
            //Next and previous position command 
            //to be sent to lowlevel
            double positionCommanded;
            double prevPositionCommanded;
            //RhIO wrappers for lowlevel monitoring
            RhIO::WrapperFloat rhioLowlevelPos;
            RhIO::WrapperFloat rhioLowlevelPosAbs;
            RhIO::WrapperFloat rhioLowlevelVel;
            RhIO::WrapperFloat rhioLowlevelTau;
            RhIO::WrapperFloat rhioLowlevelEffort;
            RhIO::WrapperFloat rhioLowlevelCmdRaw;
            RhIO::WrapperFloat rhioLowlevelCmdFiltered;
            //Lowpass filters
            FilterExponential<double> filterPos;
            FilterExponential<double> filterVel;
            FilterExponential<double> filterTau;
            FilterExponential<double> filterCmd;
            //Joint limits from model
            double limitLowerPos;
            double limitUpperPos;
            double limitAbsVel;
            double limitAbsTau;
            //Joint index in loaded URDF model (or -1 if missing)
            size_t indexJointInModel;
            //Final position for posture initialization
            double posInitEnd;
            //RhIO wrappers for estimated (filtered) state monitoring
            RhIO::WrapperFloat rhioStatePos;
            RhIO::WrapperFloat rhioStateVel;
            RhIO::WrapperFloat rhioStateTau;
            //Future command for interpolation
            double nextCommandPosition;
            double nextCommandTime;
        };
        //Velocity command joint
        struct DataJointVel_t {
            //Resource name
            std::string name;
            //Resource handle on joint state and command
            hardware_interface::JointHandle handleJoint;
            //Resource handle on joint current limit
            pal_ros_control::CurrentLimitHandle handleCurrent;
            //True is the joint has absolute position sensor
            bool hasAbsPos;
            //True if the joint has available current limit
            bool hasCurrent;
            //Desired current ratio when running
            double currentRatioCommanded;
            //Next and previous velocity command 
            //to be sent to lowlevel
            double velocityCommanded;
            double prevVelocityCommanded;
            //RhIO wrappers for lowlevel monitoring
            RhIO::WrapperFloat rhioLowlevelPos;
            RhIO::WrapperFloat rhioLowlevelPosAbs;
            RhIO::WrapperFloat rhioLowlevelVel;
            RhIO::WrapperFloat rhioLowlevelEffort;
            RhIO::WrapperFloat rhioLowlevelCmdRaw;
            RhIO::WrapperFloat rhioLowlevelCmdFiltered;
            //Lowpass filters
            FilterExponential<double> filterVel;
            FilterExponential<double> filterCmd;
            //Joint limits from model
            double limitAbsVel;
        };
        //Force Torque sensor
        struct DataForceTorque_t {
            //Resource name
            std::string name;
            //Resource handle
            hardware_interface::ForceTorqueSensorHandle handle;
            //RhIO wrappers for monitoring
            RhIO::WrapperVect3d rhioForce;
            RhIO::WrapperVect3d rhioTorque;
            //Lowpass filters
            FilterExponential<Eigen::Vector3d> filterForce;
            FilterExponential<Eigen::Vector3d> filterTorque;
        };
        //IMU sensor
        struct DataImu_t {
            //Resource name
            std::string name;
            //Resource handle
            hardware_interface::ImuSensorHandle handle;
            //RhIO wrapper for monitoring
            RhIO::WrapperVect4d rhioOrientation;
            RhIO::WrapperVect3d rhioVelAng;
            RhIO::WrapperVect3d rhioAccLin;
            //Lowpass filters
            FilterExponentialRotation filterOrientation;
            FilterExponential<Eigen::Vector3d> filterVelAng;
            FilterExponential<Eigen::Vector3d> filterAccLin;
        };

        /**
         * Controller state for safety
         */
        enum State_t : int {
            STOPPED_NOPOWER = 0,
            INIT_POWER = 1,
            STOPPED_HOLD = 2,
            INIT_POSTURE = 3,
            RUNNING = 4,
        };

        /**
         * Set of joint names to exclude from low level and
         * for which to assume absolute position and torque sensor is available
         */
        std::set<std::string> _setJointsExclude;
        std::set<std::string> _setJointsWithAbsPosSensor;
        std::set<std::string> _setJointsWithTorqueSensor;

        /**
         * Lowlevel data joints and force torque sensors 
         * and IMUs containers
         */
        std::vector<DataJointPos_t> _containerJointPos;
        std::vector<DataJointVel_t> _containerJointVel;
        std::vector<DataForceTorque_t> _containerForceTorque;
        std::vector<DataImu_t> _containerImu;
        
        /**
         * Model instance for state estimation 
         */
        Model _model;

        /**
         * State and transition variables
         */
        State_t _state;
        RhIO::WrapperBool _rhioIsNoSetCommand;
        double _timeTransition;
        RhIO::WrapperInt _rhioState;
        RhIO::WrapperBool _rhioAskNoPower;
        RhIO::WrapperBool _rhioAskHold;
        RhIO::WrapperBool _rhioAskRunning;
        RhIO::WrapperBool _rhioOnStartStateHold;
        RhIO::WrapperBool _rhioOnErrorStateHold;
        RhIO::WrapperFloat _rhioDurationInitPower;
        RhIO::WrapperFloat _rhioDurationInitPosture;

        /**
         * Parameters for safety checks
         */
        RhIO::WrapperFloat _rhioLimitPositionMargin;
        RhIO::WrapperFloat _rhioLimitRangeTime;

        /**
         * RhIO timing state in milliseconds
         */
        std::chrono::time_point<std::chrono::steady_clock> _timeUpdate;
        RhIO::WrapperFloat _rhioTimingDuration;
        RhIO::WrapperFloat _rhioTimingPeriodROS;
        RhIO::WrapperFloat _rhioTimingPeriodReal;

        /**
         * State and command filtering parameters
         */
        RhIO::WrapperFloat _rhioCutoffFreqJointPos;
        RhIO::WrapperFloat _rhioCutoffFreqJointVel;
        RhIO::WrapperFloat _rhioCutoffFreqJointTau;
        RhIO::WrapperFloat _rhioCutoffFreqJointCmd;
        RhIO::WrapperFloat _rhioCutoffFreqFT;
        RhIO::WrapperFloat _rhioCutoffFreqIMUOrientation;
        RhIO::WrapperFloat _rhioCutoffFreqIMUVelAng;
        RhIO::WrapperFloat _rhioCutoffFreqIMUAccLin;
        
        /**
         * Manager for task in non RT thread
         */
        ThreadManager<T_State, T_Command> _threadManager;

        /**
         * Initialize and append data structure for joint,  
         * force torque sensor and IMU hardware resource
         */
        void initDataJointPos(
            const std::string& name,
            hardware_interface::JointHandle handleJoint);
        void initDataJointVel(
            const std::string& name,
            hardware_interface::JointHandle handleJoint);
        void initDataCurrent(
            const std::string& name,
            pal_ros_control::CurrentLimitHandle handleCurrent);
        void initDataForceTorque(
            const std::string& name,
            hardware_interface::ForceTorqueSensorHandle handle);
        void initDataImu(
            const std::string& name,
            hardware_interface::ImuSensorHandle handle);

        /**
         * Read lowlevel sensors, apply filter, assign joint 
         * state to model and publish to RhIO lowlevel
         */
        virtual void updateReadLowlevelAndFiltering(double time, double dt);

        /**
         * Compute and assign state estimation
         */
        virtual void updateAssignTaskState(double time, double dt);

        /**
         * Send state to non RT task 
         * and publish it to RhIO state
         */
        virtual void updatePublishTaskState(double time, double dt);

        /**
         * Retrieve command from non RT task
         */
        virtual void updateProcessTaskCommand(double time, double dt);

        /**
         * Compute command interpolation
         */
        virtual void updateCommandInterpolation(double time, double dt);

        /**
         * Check if joint state or command violate safety limits
         */
        virtual void updateCheckSafetyLimits(double dt);

        /**
         * Manage state machine transition
         */
        virtual void updateStateMachine(double time, double dt);
};

}

#endif

