#include <inria_controller/TaskBaseTalos.hpp>
#include <inria_controller/ControllerBase.hpp>
#include <inria_model/Model.hpp>
#include <inria_maths/Angle.h>
#include <inria_maths/Clamp.h>
#include <inria_utils/Filesystem.h>
#include <inria_utils/Scheduling.h>

namespace inria {

template <typename T_State, typename T_Command>
ControllerBase<T_State, T_Command>::ControllerBase() :
    _setJointsExclude(),
    _setJointsWithAbsPosSensor(),
    _setJointsWithTorqueSensor(),
    _containerJointPos(),
    _containerJointVel(),
    _containerForceTorque(),
    _containerImu(),
    _model(),
    _state(State_t::RUNNING),
    _rhioIsNoSetCommand(),
    _timeTransition(0.0),
    _rhioState(),
    _rhioAskNoPower(),
    _rhioAskHold(),
    _rhioAskRunning(),
    _rhioOnStartStateHold(),
    _rhioOnErrorStateHold(),
    _rhioDurationInitPower(),
    _rhioDurationInitPosture(),
    _rhioLimitPositionMargin(),
    _rhioLimitRangeTime(),
    _timeUpdate(),
    _rhioTimingDuration(),
    _rhioTimingPeriodROS(),
    _rhioTimingPeriodReal(),
    _rhioCutoffFreqJointPos(),
    _rhioCutoffFreqJointVel(),
    _rhioCutoffFreqJointTau(),
    _rhioCutoffFreqJointCmd(),
    _rhioCutoffFreqFT(),
    _rhioCutoffFreqIMUOrientation(),
    _rhioCutoffFreqIMUVelAng(),
    _rhioCutoffFreqIMUAccLin(),
    _threadManager()
{
    //Define thread name
    SystemSetThreadName("inria_ctl_manager");

    //RhIO initialization
    RhIO::reset();
    //Configure RhIO server information
    RhIO::Root.newChild("/server");
    RhIO::Root.newStr("/server/hostname")
        ->persisted(false)
        ->defaultValue("Base");

    //RhIO configuration
    //Tree creation
    RhIO::Root.newChild("/lowlevel");
    RhIO::Root.newChild("/lowlevel/pos");
    RhIO::Root.newChild("/lowlevel/pos_abs");
    RhIO::Root.newChild("/lowlevel/vel");
    RhIO::Root.newChild("/lowlevel/tau");
    RhIO::Root.newChild("/lowlevel/effort");
    RhIO::Root.newChild("/lowlevel/cmd_raw");
    RhIO::Root.newChild("/lowlevel/cmd_filtered");
    RhIO::Root.newChild("/controller");
    RhIO::Root.newChild("/state");
    RhIO::Root.newChild("/state/pos");
    RhIO::Root.newChild("/state/vel");
    RhIO::Root.newChild("/state/tau");
    //Controller parameters
    RhIO::IONode& nodeController = RhIO::Root.child("/controller");
    _rhioIsNoSetCommand.create(nodeController, "is_no_set_command")
        ->comment("If true, commands are not send to lowlevel")
        ->defaultValue(false);
    _rhioState.create(nodeController, "state")
        ->comment("State machine")
        ->defaultValue(_state);
    _rhioAskNoPower.create(nodeController, "ask_nopower")
        ->comment("Stop controller and disable all motors current (emergency)")
        ->defaultValue(false);
    _rhioAskHold.create(nodeController, "ask_hold")
        ->comment("Stop controller and hold posture")
        ->defaultValue(false);
    _rhioAskRunning.create(nodeController, "ask_running")
        ->comment("Start controller (init)")
        ->defaultValue(false);
    _rhioOnStartStateHold.create(nodeController, "on_start_state_hold")
        ->comment("Start controller in hold state instead of no power")
        ->persisted(true)
        ->defaultValue(true);
    _rhioOnErrorStateHold.create(nodeController, "on_error_state_hold")
        ->comment("Switch to hold state instead of no power on safety error")
        ->persisted(true)
        ->defaultValue(false);
    _rhioDurationInitPower.create(nodeController, "duration_init_power")
        ->comment("Duration for power transition")
        ->persisted(true)
        ->minimum(0.0)
        ->defaultValue(2.0);
    _rhioDurationInitPosture.create(nodeController, "duration_init_posture")
        ->comment("Duration for posture transition")
        ->persisted(true)
        ->minimum(0.0)
        ->defaultValue(10.0);
    _rhioLimitPositionMargin.create(nodeController, "limit_position_margin")
        ->comment("Margin in radian for position limits check")
        ->persisted(true)
        ->defaultValue(DegToRad(0.0));
    _rhioLimitRangeTime.create(nodeController, "limit_range_time")
        ->comment("Minimum time to travel angular range fixing command velocity bound")
        ->persisted(true)
        ->defaultValue(0.75);
    RhIO::Root.newStr("/controller/model_path")
        ->persisted(true)->defaultValue("");
    _rhioTimingDuration.create(nodeController, "timing_duration")
        ->comment("Controller update computation time");
    _rhioTimingPeriodROS.create(nodeController, "timing_period_ros")
        ->comment("Controller update period from ROS");
    _rhioTimingPeriodReal.create(nodeController, "timing_period_real")
        ->comment("Controller update period computed");
    _rhioCutoffFreqJointPos.create(nodeController, "filter_cutoff_freq_joint_pos")
        ->comment("Cutoff frequency for measured joint position")
        ->persisted(true)
        ->minimum(0.0)
        ->defaultValue(5.0);
    _rhioCutoffFreqJointVel.create(nodeController, "filter_cutoff_freq_joint_vel")
        ->comment("Cutoff frequency for measured joint velocity")
        ->persisted(true)
        ->minimum(0.0)
        ->defaultValue(5.0);
    _rhioCutoffFreqJointTau.create(nodeController, "filter_cutoff_freq_joint_tau")
        ->comment("Cutoff frequency for measured joint torque")
        ->persisted(true)
        ->minimum(0.0)
        ->defaultValue(5.0);
    _rhioCutoffFreqJointCmd.create(nodeController, "filter_cutoff_freq_joint_cmd")
        ->comment("Cutoff frequency for commanded joint pos")
        ->persisted(true)
        ->minimum(0.0)
        ->defaultValue(10.0);
    _rhioCutoffFreqFT.create(nodeController, "filter_cutoff_freq_ft")
        ->comment("Cutoff frequency for measured FT wrench")
        ->persisted(true)
        ->minimum(0.0)
        ->defaultValue(5.0);
    _rhioCutoffFreqIMUOrientation.create(nodeController, "filter_cutoff_freq_imu_orientation")
        ->comment("Cutoff frequency for measured IMU orientation")
        ->persisted(true)
        ->minimum(0.0)
        ->defaultValue(5.0);
    _rhioCutoffFreqIMUVelAng.create(nodeController, "filter_cutoff_freq_imu_vel_ang")
        ->comment("Cutoff frequency for measured IMU angular velocity")
        ->persisted(true)
        ->minimum(0.0)
        ->defaultValue(5.0);
    _rhioCutoffFreqIMUAccLin.create(nodeController, "filter_cutoff_freq_imu_acc_lin")
        ->comment("Cutoff frequency for measured IMU linear acceleration")
        ->persisted(true)
        ->minimum(0.0)
        ->defaultValue(5.0);
    //RhIO commands
    RhIO::Root.newCommand(
        "em", "Emergency no power mode",
        [this]
        (const std::vector<std::string>& args) -> std::string
        {
            (void)args;
            this->_rhioAskNoPower.set(true);
            return "Emergency enabled";
        });
    RhIO::Root.newCommand(
        "hold", "Hold current posture mode",
        [this]
        (const std::vector<std::string>& args) -> std::string
        {
            (void)args;
            this->_rhioAskHold.set(true);
            return "Hold enabled";
        });
    RhIO::Root.newCommand(
        "init", "Start initialisation to running mode",
        [this]
        (const std::vector<std::string>& args) -> std::string
        {
            (void)args;
            this->_rhioAskRunning.set(true);
            return "Initialization enabled";
        });
    RhIO::Root.newCommand(
        "write_rhio_logs",
        "Write RhIO logs into binary file. Usage: [filepath]",
        [this](const std::vector<std::string>& args) -> std::string
        {
            if (args.size() != 1) {
                return "Invalid usage";
            } else {
                RhIO::writeLogs(args[0]);
                return "RhIO logs written to: " + args[0];
            }
        });
    
    //Verbose
    ROS_INFO(
        "inria::ControllerBase::constructor: "
        "Controller loaded.");
}

template <typename T_State, typename T_Command>
ControllerBase<T_State, T_Command>::~ControllerBase()
{
    //Stop RhIO server threads
    RhIO::stop();
    
    //Verbose
    ROS_INFO(
        "inria::ControllerBase::destructor: "
        "Controller unloaded.");
}

template <typename T_State, typename T_Command>
bool ControllerBase<T_State, T_Command>::initRequest(
    hardware_interface::RobotHW* robot_hw,
    ros::NodeHandle& root_nh, 
    ros::NodeHandle& controller_nh,
    controller_interface::ControllerBase::ClaimedResources& claimed_resources)
{
    //Check if construction finished cleanly
    if (this->state_ != CONSTRUCTED){
        ROS_ERROR(
            "inria::ControllerBase::initRequest: "
            "Cannot initialize the controller "
            "because it failed to be constructed");
        return false;
    }

    //Get a pointer to the joint position command interface
    hardware_interface::PositionJointInterface* jointPos_hw = 
        robot_hw->get<hardware_interface::PositionJointInterface>();
    if (jointPos_hw == nullptr) {
        ROS_ERROR(
            "inria::ControllerBase::initRequest: "
            "Hardware interface of type "
            "hardware_interface::PositionJointInterface is required");
        return false;
    }
    //Get a pointer to the joint velocity command interface
    hardware_interface::VelocityJointInterface* jointVel_hw = 
        robot_hw->get<hardware_interface::VelocityJointInterface>();
    if (jointVel_hw == nullptr) {
        ROS_WARN(
            "inria::ControllerBase::initRequest: "
            "Hardware interface of type "
            "hardware_interface::VelocityJointInterface not found");
    }
    //Get a point to the joint current interface
    pal_ros_control::CurrentLimitInterface* current_hw = 
        robot_hw->get<pal_ros_control::CurrentLimitInterface>();
    if (current_hw == nullptr) {
        ROS_WARN(
            "inria::ControllerBase::initRequest: "
            "Hardware interface of type "
            "pal_ros_control::CurrentLimitInterface not found");
    }
    //Get a pointer to the ForceTorque interface
    hardware_interface::ForceTorqueSensorInterface* forceTorque_hw = 
        robot_hw->get<hardware_interface::ForceTorqueSensorInterface>();
    if (forceTorque_hw == nullptr) {
        ROS_WARN(
            "inria::ControllerBase::initRequest: "
            "Hardware interface of type "
            "hardware_interface::ForceTorqueSensorInterface not found");
    }
    //Get a pointer to the IMU interface
    hardware_interface::ImuSensorInterface* imu_hw = 
        robot_hw->get<hardware_interface::ImuSensorInterface>();
    if (imu_hw == nullptr) {
        ROS_WARN(
            "inria::ControllerBase::initRequest: "
            "Hardware interface of type "
            "hardware_interface::ImuSensorInterface not found");
    }
    
    //Claim and initialize all joint position state data
    if (jointPos_hw != nullptr) {
        jointPos_hw->clearClaims();
        const std::vector<std::string>& namesJoint = jointPos_hw->getNames();
        for(size_t i=0;i<namesJoint.size();i++) {
            try {
                if (_setJointsExclude.count(namesJoint[i]) != 0) {
                    ROS_INFO_STREAM(
                        "inria::ControllerBase::initRequest: "
                        "Skipping position Joint: " << namesJoint[i]);
                } else {
                    initDataJointPos(
                        namesJoint[i],
                        jointPos_hw->getHandle(namesJoint[i]));
                    ROS_INFO_STREAM(
                        "inria::ControllerBase::initRequest: "
                        "Claim position Joint: " << namesJoint[i]);
                }
            } catch (const hardware_interface::HardwareInterfaceException& e) {
                ROS_ERROR_STREAM(
                    "inria::ControllerBase::initRequest: "
                    "Can not retrieve handle for " 
                    << namesJoint[i] << ": " << e.what());
                return false;
            }
        }
        //Register claimed position joint interfaces
        claimed_resources.push_back(hardware_interface::InterfaceResources(
            hardware_interface::internal::demangledTypeName<hardware_interface::PositionJointInterface>(),
            jointPos_hw->getClaims()));
        jointPos_hw->clearClaims();
    }
    //Claim and initialize all joint velocity state data
    if (jointVel_hw != nullptr) {
        jointVel_hw->clearClaims();
        const std::vector<std::string>& namesJoint = jointVel_hw->getNames();
        for(size_t i=0;i<namesJoint.size();i++) {
            try {
                if (_setJointsExclude.count(namesJoint[i]) != 0) {
                    ROS_INFO_STREAM(
                        "inria::ControllerBase::initRequest: "
                        "Skipping velocity Joint: " << namesJoint[i]);
                } else {
                    initDataJointVel(
                        namesJoint[i],
                        jointVel_hw->getHandle(namesJoint[i]));
                    ROS_INFO_STREAM(
                        "inria::ControllerBase::initRequest: "
                        "Claim velocity Joint: " << namesJoint[i]);
                }
            } catch (const hardware_interface::HardwareInterfaceException& e) {
                ROS_ERROR_STREAM(
                    "inria::ControllerBase::initRequest: "
                    "Can not retrieve handle for " 
                    << namesJoint[i] << ": " << e.what());
                return false;
            }
        }
        //Register claimed velocity joint interfaces
        claimed_resources.push_back(hardware_interface::InterfaceResources(
            hardware_interface::internal::demangledTypeName<hardware_interface::VelocityJointInterface>(),
            jointVel_hw->getClaims()));
        jointVel_hw->clearClaims();
    }
    //Claim current limit interface and associate each motor to its joint
    if (current_hw != nullptr) {
        current_hw->clearClaims();
        const std::vector<std::string>& namesCurrent = current_hw->getNames();
        for(size_t i=0;i<namesCurrent.size();i++) {
            try {
                initDataCurrent(
                    namesCurrent[i],
                    current_hw->getHandle(namesCurrent[i]));
            } catch (const hardware_interface::HardwareInterfaceException& e) {
                ROS_ERROR_STREAM(
                    "inria::ControllerBase::initRequest: "
                    "Can not retrieve current limit handle for" 
                    << namesCurrent[i] << ": " << e.what());
                return false;
            }
        }
        //Register claimed current limit interfaces
        claimed_resources.push_back(hardware_interface::InterfaceResources(
            hardware_interface::internal::demangledTypeName<pal_ros_control::CurrentLimitInterface>(),
            current_hw->getClaims()));
        current_hw->clearClaims();
    }
    //Claim and initialize all force torque sensors data
    if (forceTorque_hw != nullptr) {
        forceTorque_hw->clearClaims();
        const std::vector<std::string>& namesForceTorque = forceTorque_hw->getNames();
        for(size_t i=0;i<namesForceTorque.size();i++) {
            try {
                initDataForceTorque(
                    namesForceTorque[i],
                    forceTorque_hw->getHandle(namesForceTorque[i]));
                ROS_INFO_STREAM(
                    "inria::ControllerBase::initRequest: "
                    "Claim ForceTorque: " << namesForceTorque[i]);
            } catch (const hardware_interface::HardwareInterfaceException& e) {
                ROS_ERROR_STREAM(
                    "inria::ControllerBase::initRequest: "
                    "Can not retrieve handle for " 
                    << namesForceTorque[i] << ": " << e.what());
                return false;
            }
        }
    }
    //Claim and initialize all IMU sensors data
    if (imu_hw != nullptr) {
        imu_hw->clearClaims();
        const std::vector<std::string>& namesImu = imu_hw->getNames();
        for(size_t i=0;i<namesImu.size();i++) {
            try {
                initDataImu(
                    namesImu[i],
                    imu_hw->getHandle(namesImu[i]));
                ROS_INFO_STREAM(
                    "inria::ControllerBase::initRequest: "
                    "Claim IMU: " << namesImu[i]);
            } catch (const hardware_interface::HardwareInterfaceException& e) {
                ROS_ERROR_STREAM(
                    "inria::ControllerBase::initRequest: "
                    "Can not retrieve handle for " 
                    << namesImu[i] << ": " << e.what());
                return false;
            }
        }
    }

    //Load URDF model
    std::string filepath = inria::SystemResolvePath(
        RhIO::Root.getStr("/controller/model_path"));
    ROS_INFO_STREAM(
        "inria::ControllerBase::initRequest: "
        "Use URDF model: " << filepath);
    _model = inria::Model(filepath);
    //Load joint limits 
    for (size_t i=0;i<_containerJointPos.size();i++) {
        const std::string& name = _containerJointPos[i].name;
        if (_model.getMappingDOFs().count(name) != 0) {
            size_t index = _model.getIndexJoint(name);
            _containerJointPos[i].limitLowerPos = _model.jointLimitsLower()[index];
            _containerJointPos[i].limitUpperPos = _model.jointLimitsUpper()[index];
            _containerJointPos[i].limitAbsVel = _model.jointLimitsVelocity()[index];
            _containerJointPos[i].limitAbsTau = _model.jointLimitsTorque()[index];
            _containerJointPos[i].indexJointInModel = index;
            ROS_INFO_STREAM(
                "inria::ControllerBase::initRequest: "
                "Limit joint: " << name << ": index=" << index 
                << " lower=" << _containerJointPos[i].limitLowerPos
                << " upper=" << _containerJointPos[i].limitUpperPos
                << " velocity=" << _containerJointPos[i].limitAbsVel
                << " torque=" << _containerJointPos[i].limitAbsTau);
        } else {
            ROS_INFO_STREAM(
                "inria::ControllerBase::initRequest: "
                "Joint: " << name << ": not in model");
        }
    }
    //Configure RhIO for state estimation publishing
    //Joint state
    for (size_t i=0;i<_containerJointPos.size();i++) {
        const std::string& name = _containerJointPos[i].name;
        if (_model.getMappingDOFs().count(name) != 0) {
            _containerJointPos[i].rhioStatePos.create(
                RhIO::Root.child("/state/pos"), name);
            _containerJointPos[i].rhioStateVel.create(
                RhIO::Root.child("/state/vel"), name);
            _containerJointPos[i].rhioStateTau.create(
                RhIO::Root.child("/state/tau"), name);
        }
    }

    //Verbose successful handles loaded
    this->state_ = INITIALIZED;
    ROS_INFO_STREAM(
        "inria::ControllerBase::initRequest: Initialization successful");
    
    return true;
}

template <typename T_State, typename T_Command>
void ControllerBase<T_State, T_Command>::starting(const ros::Time& time)
{
    (void)time;
    ROS_INFO(
        "inria::ControllerBase::starting: "
        "Starting base RT controller.");
    //Set starting state
    if (_rhioOnStartStateHold) {
        _state = State_t::STOPPED_HOLD;
    } else {
        _state = State_t::STOPPED_NOPOWER;
    }
    //Set commanded position to measured ones
    //and enable full current
    for (size_t i=0;i<_containerJointPos.size();i++) {
        _containerJointPos[i].currentRatioCommanded = 1.0;
        _containerJointPos[i].prevPositionCommanded = 
            _containerJointPos[i].handleJoint.getPosition();
        _containerJointPos[i].positionCommanded = 
            _containerJointPos[i].handleJoint.getPosition();
    }
    //Set commanded velocity to zero
    for (size_t i=0;i<_containerJointVel.size();i++) {
        _containerJointVel[i].currentRatioCommanded = 1.0;
        _containerJointVel[i].prevVelocityCommanded = 0.0;
        _containerJointVel[i].velocityCommanded = 0.0;
    }
    //Set thread name and config
    SystemSetThreadName("inria_controller");
    SystemSetThreadRealTime();
    //Start non RT thread
    _threadManager.startThread();
    _timeUpdate = std::chrono::steady_clock::now();
}

template <typename T_State, typename T_Command>
void ControllerBase<T_State, T_Command>::update(
    const ros::Time& time, 
    const ros::Duration& period)
{
    //Measure the complete execution time
    std::chrono::time_point<std::chrono::steady_clock> timeStart = 
        std::chrono::steady_clock::now();

    //Check controller ROS period
    double dt = period.toSec();
    if (dt <= 0.0) {
        ROS_WARN_STREAM(
            "inria::ControllerBase::update: "
            "Negative or zero period: " << dt);
        dt = 0.0;
    }
        
    //Read lowlevel, filter,  assign model and publish lowlevel
    updateReadLowlevelAndFiltering(time.toSec(), dt);
    
    //Compute state and assign task state structure
    updateAssignTaskState(time.toSec(), dt);
    
    //Send task state structure to tasks and publish it to RhIO
    updatePublishTaskState(time.toSec(), dt);

    //Retrieve command from non RT task
    updateProcessTaskCommand(time.toSec(), dt);
    
    //Compute command interpolation
    updateCommandInterpolation(time.toSec(), dt);

    //Check joint measured signals and command safety limits
    updateCheckSafetyLimits(dt);

    //Update controller state machine
    updateStateMachine(time.toSec(), dt);

    //Final NaN and validity check
    if (!_rhioIsNoSetCommand) {
        double phase = 1.0 - _timeTransition/_rhioDurationInitPower;
        phase = ClampRange(phase, 0.0, 1.0);
        if (
            std::isnan(phase) ||
            phase < 0.0 ||
            phase > 1.0
        ) {
            _rhioIsNoSetCommand = true;
            ROS_ERROR_STREAM(
                "inria::ControllerBase::update: "
                "Invalid current command for lowlevel: phase");
        }
        for (size_t i=0;i<_containerJointPos.size();i++) {
            if (
                _containerJointPos[i].hasCurrent &&
                (std::isnan(_containerJointPos[i].currentRatioCommanded) ||
                _containerJointPos[i].currentRatioCommanded < 0.0 ||
                _containerJointPos[i].currentRatioCommanded > 1.0)
            ) {
                _rhioIsNoSetCommand = true;
                ROS_ERROR_STREAM(
                    "inria::ControllerBase::update: "
                    "Invalid current command for lowlevel: " << _containerJointPos[i].name);
            }
        }
        for (size_t i=0;i<_containerJointVel.size();i++) {
            if (
                _containerJointVel[i].hasCurrent &&
                (std::isnan(_containerJointVel[i].currentRatioCommanded) ||
                _containerJointVel[i].currentRatioCommanded < 0.0 ||
                _containerJointVel[i].currentRatioCommanded > 1.0)
            ) {
                _rhioIsNoSetCommand = true;
                ROS_ERROR_STREAM(
                    "inria::ControllerBase::update: "
                    "Invalid current command for lowlevel: " << _containerJointVel[i].name);
            }
        }
    }
    //Assign actuator current limit
    if (!_rhioIsNoSetCommand) {
        if (_state == State_t::STOPPED_NOPOWER) {
            //No power emergency state
            for (size_t i=0;i<_containerJointPos.size();i++) {
                if (_containerJointPos[i].hasCurrent) {
                    _containerJointPos[i].handleCurrent.setCurrentLimit(0.0);
                }
            }
            for (size_t i=0;i<_containerJointVel.size();i++) {
                if (_containerJointVel[i].hasCurrent) {
                    _containerJointVel[i].handleCurrent.setCurrentLimit(0.0);
                }
            }
        } else if (_state == State_t::INIT_POWER) {
            //Current ramping up initialization state
            double phase = 1.0 - _timeTransition/_rhioDurationInitPower;
            phase = ClampRange(phase, 0.0, 1.0);
            for (size_t i=0;i<_containerJointPos.size();i++) {
                if (_containerJointPos[i].hasCurrent) {
                    _containerJointPos[i].handleCurrent.setCurrentLimit(
                        phase*_containerJointPos[i].currentRatioCommanded);
                }
            }
            for (size_t i=0;i<_containerJointVel.size();i++) {
                if (_containerJointVel[i].hasCurrent) {
                    _containerJointVel[i].handleCurrent.setCurrentLimit(
                        phase*_containerJointVel[i].currentRatioCommanded);
                }
            }
        } else {
            //Actuator fully enabled state
            for (size_t i=0;i<_containerJointPos.size();i++) {
                if (_containerJointPos[i].hasCurrent) {
                    _containerJointPos[i].handleCurrent.setCurrentLimit(
                        _containerJointPos[i].currentRatioCommanded);
                }
            }
            for (size_t i=0;i<_containerJointVel.size();i++) {
                if (_containerJointVel[i].hasCurrent) {
                    _containerJointVel[i].handleCurrent.setCurrentLimit(
                        _containerJointPos[i].currentRatioCommanded);
                }
            }
        }
    }

    //Process and send command to lowlevel
    for (size_t i=0;i<_containerJointPos.size();i++) {
        //Save previous command
        _containerJointPos[i].prevPositionCommanded = 
            _containerJointPos[i].positionCommanded;
        //Apply command filtering
        _containerJointPos[i].filterCmd.cutoffFrequency() = _rhioCutoffFreqJointCmd;
        _containerJointPos[i].filterCmd.update(
            _containerJointPos[i].positionCommanded, dt);
        //Final NaN and validity check
        if (
            !_rhioIsNoSetCommand &&
            (std::isnan(_containerJointPos[i].filterCmd.value()) ||
            _containerJointPos[i].filterCmd.value() < 
                _containerJointPos[i].limitLowerPos + _rhioLimitPositionMargin ||
            _containerJointPos[i].filterCmd.value() > 
                _containerJointPos[i].limitUpperPos - _rhioLimitPositionMargin)
        ) {
            _rhioIsNoSetCommand = true;
            ROS_ERROR_STREAM(
                "inria::ControllerBase::update: "
                "Invalid position command for lowlevel: " << _containerJointPos[i].name);
        }
        //Send commanded position to lowlevel
        if (!_rhioIsNoSetCommand) {
            _containerJointPos[i].handleJoint.setCommand(
                _containerJointPos[i].filterCmd.value());
        }
        //RhIO monitoring
        _containerJointPos[i].rhioLowlevelCmdRaw = 
            _containerJointPos[i].positionCommanded;
        _containerJointPos[i].rhioLowlevelCmdFiltered = 
            _containerJointPos[i].filterCmd.value();
    }
    for (size_t i=0;i<_containerJointVel.size();i++) {
        //Save previous command
        _containerJointVel[i].prevVelocityCommanded = 
            _containerJointVel[i].velocityCommanded;
        //Apply command filtering
        _containerJointVel[i].filterCmd.cutoffFrequency() = _rhioCutoffFreqJointCmd;
        _containerJointVel[i].filterCmd.update(
            _containerJointVel[i].velocityCommanded, dt);
        //Final NaN and validity check
        if (
            !_rhioIsNoSetCommand &&
            (std::isnan(_containerJointVel[i].filterCmd.value()) ||
            std::fabs(_containerJointVel[i].filterCmd.value()) > _containerJointVel[i].limitAbsVel)
        ) {
            _rhioIsNoSetCommand = true;
            ROS_ERROR_STREAM(
                "inria::ControllerBase::update: "
                "Invalid velocity command for lowlevel: " << _containerJointVel[i].name);
        }
        //Send commanded velocity to lowlevel
        if (!_rhioIsNoSetCommand) {
            _containerJointVel[i].handleJoint.setCommand(
                _containerJointVel[i].filterCmd.value());
        }
        //RhIO monitoring
        _containerJointVel[i].rhioLowlevelCmdRaw = 
            _containerJointVel[i].velocityCommanded;
        _containerJointVel[i].rhioLowlevelCmdFiltered = 
            _containerJointVel[i].filterCmd.value();
    }
    
    //Measure complete execution time
    std::chrono::time_point<std::chrono::steady_clock> timeEnd = 
        std::chrono::steady_clock::now();
    std::chrono::duration<double> duration1 = 
        timeEnd - timeStart;
    std::chrono::duration<double> duration2 = 
        timeStart - _timeUpdate;
    _timeUpdate = timeStart;
    _rhioTimingDuration = duration1.count();
    _rhioTimingPeriodROS = dt;
    _rhioTimingPeriodReal = duration2.count();
}

template <typename T_State, typename T_Command>
void ControllerBase<T_State, T_Command>::stopping(const ros::Time& time)
{
    (void)time;
    ROS_INFO(
        "inria::ControllerBase::stopping: "
        "Stopping base RT controller.");
    //Wait for non RT thread stop
    _threadManager.stopThread();
}

template <typename T_State, typename T_Command>
void ControllerBase<T_State, T_Command>::initDataJointPos(
    const std::string& name,
    hardware_interface::JointHandle handleJoint)
{
    _containerJointPos.push_back({
        name, 
        handleJoint,
        pal_ros_control::CurrentLimitHandle(),
        _setJointsWithAbsPosSensor.count(name) != 0,
        false,
        _setJointsWithTorqueSensor.count(name) != 0,
        1.0, 0.0, 0.0,
        RhIO::WrapperFloat(),
        RhIO::WrapperFloat(),
        RhIO::WrapperFloat(),
        RhIO::WrapperFloat(),
        RhIO::WrapperFloat(),
        RhIO::WrapperFloat(),
        RhIO::WrapperFloat(),
        FilterExponential<double>(),
        FilterExponential<double>(),
        FilterExponential<double>(),
        FilterExponential<double>(),
        -M_PI,
        M_PI,
        1e9,
        1e9,
        (size_t)-1,
        0.0,
        RhIO::WrapperFloat(),
        RhIO::WrapperFloat(),
        RhIO::WrapperFloat(),
        0.0, 0.0,
    });
    _containerJointPos.back().rhioLowlevelPos.create(
        RhIO::Root.child("/lowlevel/pos"),
        name);
    if (_containerJointPos.back().hasAbsPos) {
        _containerJointPos.back().rhioLowlevelPosAbs.create(
            RhIO::Root.child("/lowlevel/pos_abs"),
            name);
    }
    _containerJointPos.back().rhioLowlevelVel.create(
        RhIO::Root.child("/lowlevel/vel"),
        name);
    if (_containerJointPos.back().hasTorque) {
        _containerJointPos.back().rhioLowlevelTau.create(
            RhIO::Root.child("/lowlevel/tau"),
            name);
    }
    _containerJointPos.back().rhioLowlevelEffort.create(
        RhIO::Root.child("/lowlevel/effort"),
        name);
    _containerJointPos.back().rhioLowlevelCmdRaw.create(
        RhIO::Root.child("/lowlevel/cmd_raw"),
        name);
    _containerJointPos.back().rhioLowlevelCmdFiltered.create(
        RhIO::Root.child("/lowlevel/cmd_filtered"),
        name);
}
template <typename T_State, typename T_Command>
void ControllerBase<T_State, T_Command>::initDataJointVel(
    const std::string& name,
    hardware_interface::JointHandle handleJoint)
{
    _containerJointVel.push_back({
        name,
        handleJoint,
        pal_ros_control::CurrentLimitHandle(),
        _setJointsWithAbsPosSensor.count(name) != 0,
        false,
        1.0, 0.0, 0.0,
        RhIO::WrapperFloat(),
        RhIO::WrapperFloat(),
        RhIO::WrapperFloat(),
        RhIO::WrapperFloat(),
        RhIO::WrapperFloat(),
        RhIO::WrapperFloat(),
        FilterExponential<double>(),
        FilterExponential<double>(),
        10.0,
    });
    _containerJointVel.back().rhioLowlevelPos.create(
        RhIO::Root.child("/lowlevel/pos"),
        name);
    if (_containerJointVel.back().hasAbsPos) {
        _containerJointVel.back().rhioLowlevelPosAbs.create(
            RhIO::Root.child("/lowlevel/pos_abs"),
            name);
    }
    _containerJointVel.back().rhioLowlevelVel.create(
        RhIO::Root.child("/lowlevel/vel"),
        name);
    _containerJointVel.back().rhioLowlevelEffort.create(
        RhIO::Root.child("/lowlevel/effort"),
        name);
    _containerJointVel.back().rhioLowlevelCmdRaw.create(
        RhIO::Root.child("/lowlevel/cmd_raw"),
        name);
    _containerJointVel.back().rhioLowlevelCmdFiltered.create(
        RhIO::Root.child("/lowlevel/cmd_filtered"),
        name);
}
template <typename T_State, typename T_Command>
void ControllerBase<T_State, T_Command>::initDataCurrent(
    const std::string& name,
    pal_ros_control::CurrentLimitHandle handleCurrent)
{
    //Find the joint associated to the motor
    size_t len = name.size();
    if (len > 6 && name.substr(len-6, 6) != "_motor") {
        throw std::logic_error(
            "ControllerBase::initDataCurrent: "
            "Unsupported current device name: " + name);
    }
    std::string prefix = name.substr(0, len-6);
    for (size_t i=0;i<_containerJointPos.size();i++) {
        if (_containerJointPos[i].name.find(prefix) != std::string::npos) {
            _containerJointPos[i].handleCurrent = handleCurrent;
            _containerJointPos[i].hasCurrent = true;
            ROS_INFO_STREAM(
                "inria::ControllerBase::initRequest: "
                "Claim Current limit: " << name 
                << " ---> " << _containerJointPos[i].name);
            return;
        }
    }
    for (size_t i=0;i<_containerJointVel.size();i++) {
        if (_containerJointVel[i].name.find(prefix) != std::string::npos) {
            _containerJointVel[i].handleCurrent = handleCurrent;
            _containerJointVel[i].hasCurrent = true;
            ROS_INFO_STREAM(
                "inria::ControllerBase::initRequest: "
                "Claim Current limit: " << name 
                << " ---> " << _containerJointVel[i].name);
            return;
        }
    }
    if (_setJointsExclude.count(prefix + "_joint") != 0) {
        return;
    }
    throw std::logic_error(
        "ControllerBase::initDataCurrent: "
        "Associated joint to current device not found: " + name);
}
template <typename T_State, typename T_Command>
void ControllerBase<T_State, T_Command>::initDataForceTorque(
    const std::string& name,
    hardware_interface::ForceTorqueSensorHandle handle)
{
    _containerForceTorque.push_back({
        name, handle,
        RhIO::WrapperVect3d(),
        RhIO::WrapperVect3d(),
        FilterExponential<Eigen::Vector3d>(),
        FilterExponential<Eigen::Vector3d>(),
    });
    RhIO::Root.newChild("/lowlevel/" + name);
    _containerForceTorque.back().rhioForce.create(
        RhIO::Root.child("/lowlevel/" + name),
        "force");
    _containerForceTorque.back().rhioTorque.create(
        RhIO::Root.child("/lowlevel/" + name),
        "torque");
}
template <typename T_State, typename T_Command>
void ControllerBase<T_State, T_Command>::initDataImu(
    const std::string& name,
    hardware_interface::ImuSensorHandle handle)
{
    _containerImu.push_back({
        name, handle,
        RhIO::WrapperVect4d(),
        RhIO::WrapperVect3d(),
        RhIO::WrapperVect3d(),
        FilterExponentialRotation(),
        FilterExponential<Eigen::Vector3d>(),
        FilterExponential<Eigen::Vector3d>(),
    });
    RhIO::Root.newChild("/lowlevel/" + name);
    _containerImu.back().rhioOrientation.create(
        RhIO::Root.child("/lowlevel/" + name),
        "orientation");
    _containerImu.back().rhioVelAng.create(
        RhIO::Root.child("/lowlevel/" + name),
        "vel_ang");
    _containerImu.back().rhioAccLin.create(
        RhIO::Root.child("/lowlevel/" + name),
        "acc_lin");
}

template <typename T_State, typename T_Command>
void ControllerBase<T_State, T_Command>::updateReadLowlevelAndFiltering(double time, double dt)
{
    //Read low level data and compute low pass filtering
    //and publish lowlevel data to RhIO
    for (size_t i=0;i<_containerJointPos.size();i++) {
        //Read and filtering
        _containerJointPos[i].filterPos.cutoffFrequency() = _rhioCutoffFreqJointPos;
        _containerJointPos[i].filterVel.cutoffFrequency() = _rhioCutoffFreqJointVel;
        _containerJointPos[i].filterTau.cutoffFrequency() = _rhioCutoffFreqJointTau;
        _containerJointPos[i].filterPos.update(
            _containerJointPos[i].handleJoint.getPosition(), dt);
        _containerJointPos[i].filterVel.update(
            _containerJointPos[i].handleJoint.getVelocity(), dt);
        if (_containerJointPos[i].hasTorque) {
            _containerJointPos[i].filterTau.update(
                _containerJointPos[i].handleJoint.getTorqueSensor(), dt);
        }
        //Publish to RhIO
        _containerJointPos[i].rhioLowlevelPos = _containerJointPos[i].handleJoint.getPosition();
        if (_containerJointPos[i].hasAbsPos) {
            _containerJointPos[i].rhioLowlevelPosAbs = _containerJointPos[i].handleJoint.getAbsolutePosition();
        }
        _containerJointPos[i].rhioLowlevelVel = _containerJointPos[i].handleJoint.getVelocity();
        if (_containerJointPos[i].hasTorque) {
            _containerJointPos[i].rhioLowlevelTau = _containerJointPos[i].handleJoint.getTorqueSensor();
        }
        _containerJointPos[i].rhioLowlevelEffort = _containerJointPos[i].handleJoint.getEffort();
    }
    for (size_t i=0;i<_containerJointVel.size();i++) {
        //Read and filtering
        _containerJointVel[i].filterVel.cutoffFrequency() = _rhioCutoffFreqJointVel;
        _containerJointVel[i].filterVel.update(
            _containerJointVel[i].handleJoint.getVelocity(), dt);
        //Publish to RhIO
        _containerJointVel[i].rhioLowlevelPos = _containerJointVel[i].handleJoint.getPosition();
        if (_containerJointVel[i].hasAbsPos) {
            _containerJointVel[i].rhioLowlevelPosAbs = _containerJointVel[i].handleJoint.getAbsolutePosition();
        }
        _containerJointVel[i].rhioLowlevelVel = _containerJointVel[i].handleJoint.getVelocity();
        _containerJointVel[i].rhioLowlevelEffort = _containerJointVel[i].handleJoint.getEffort();
    }
    for (size_t i=0;i<_containerForceTorque.size();i++) {
        //Read and filtering
        Eigen::Vector3d force;
        Eigen::Vector3d torque;
        for (size_t j=0;j<3;j++) {
            force(j) = _containerForceTorque[i].handle.getForce()[j];
            torque(j) = _containerForceTorque[i].handle.getTorque()[j];
        }
        _containerForceTorque[i].filterForce.cutoffFrequency() = _rhioCutoffFreqFT;
        _containerForceTorque[i].filterTorque.cutoffFrequency() = _rhioCutoffFreqFT;
        _containerForceTorque[i].filterForce.update(force, dt);
        _containerForceTorque[i].filterTorque.update(torque, dt);
        //Publish to RhIO
        _containerForceTorque[i].rhioForce[0] = _containerForceTorque[i].handle.getForce()[0];
        _containerForceTorque[i].rhioForce[1] = _containerForceTorque[i].handle.getForce()[1];
        _containerForceTorque[i].rhioForce[2] = _containerForceTorque[i].handle.getForce()[2];
        _containerForceTorque[i].rhioTorque[0] = _containerForceTorque[i].handle.getTorque()[0];
        _containerForceTorque[i].rhioTorque[1] = _containerForceTorque[i].handle.getTorque()[1];
        _containerForceTorque[i].rhioTorque[2] = _containerForceTorque[i].handle.getTorque()[2];
    }
    for (size_t i=0;i<_containerImu.size();i++) {
        //Read and filtering
        Eigen::Quaterniond quat;
        quat.x() = _containerImu[i].handle.getOrientation()[0];
        quat.y() = _containerImu[i].handle.getOrientation()[1];
        quat.z() = _containerImu[i].handle.getOrientation()[2];
        quat.w() = _containerImu[i].handle.getOrientation()[3];
        Eigen::Vector3d velAng;
        velAng.x() = _containerImu[i].handle.getAngularVelocity()[0];
        velAng.y() = _containerImu[i].handle.getAngularVelocity()[1];
        velAng.z() = _containerImu[i].handle.getAngularVelocity()[2];
        Eigen::Vector3d accLin;
        accLin.x() = _containerImu[i].handle.getLinearAcceleration()[0];
        accLin.y() = _containerImu[i].handle.getLinearAcceleration()[1];
        accLin.z() = _containerImu[i].handle.getLinearAcceleration()[2];
        _containerImu[i].filterOrientation.cutoffFrequency() = _rhioCutoffFreqIMUOrientation;
        _containerImu[i].filterVelAng.cutoffFrequency() = _rhioCutoffFreqIMUVelAng;
        _containerImu[i].filterAccLin.cutoffFrequency() = _rhioCutoffFreqIMUAccLin;
        _containerImu[i].filterOrientation.update(quat, dt);
        _containerImu[i].filterVelAng.update(velAng, dt);
        _containerImu[i].filterAccLin.update(accLin, dt);
        //Publish to RhIO
        _containerImu[i].rhioOrientation[0] = _containerImu[i].handle.getOrientation()[0];
        _containerImu[i].rhioOrientation[1] = _containerImu[i].handle.getOrientation()[1];
        _containerImu[i].rhioOrientation[2] = _containerImu[i].handle.getOrientation()[2];
        _containerImu[i].rhioOrientation[3] = _containerImu[i].handle.getOrientation()[3];
        _containerImu[i].rhioVelAng[0] = _containerImu[i].handle.getAngularVelocity()[0];
        _containerImu[i].rhioVelAng[1] = _containerImu[i].handle.getAngularVelocity()[1];
        _containerImu[i].rhioVelAng[2] = _containerImu[i].handle.getAngularVelocity()[2];
        _containerImu[i].rhioAccLin[0] = _containerImu[i].handle.getLinearAcceleration()[0];
        _containerImu[i].rhioAccLin[1] = _containerImu[i].handle.getLinearAcceleration()[1];
        _containerImu[i].rhioAccLin[2] = _containerImu[i].handle.getLinearAcceleration()[2];
    }
}

template <typename T_State, typename T_Command>
void ControllerBase<T_State, T_Command>::updateAssignTaskState(double time, double dt)
{
    //Assign joint filtered position and velocity to model
    for (size_t i=0;i<_containerJointPos.size();i++) {
        size_t index = _containerJointPos[i].indexJointInModel;
        if (index != (size_t)-1) {
            _model.setDOFPos(index+6, _containerJointPos[i].filterPos.value());
            _model.setDOFVel(index+6, _containerJointPos[i].filterVel.value());
        }
    }
    _model.updateState();

    //Reset state structure with current timestamp
    _threadManager.refState().isValid = true;
    _threadManager.refState().time = time;
    _threadManager.refState().isRunning = (_state == State_t::RUNNING);
    _threadManager.refState().jointPos.setZero();
    _threadManager.refState().jointVel.setZero();
    _threadManager.refState().jointTau.setZero();
    _threadManager.refState().jointCmd.setZero();
    for (size_t i=0;i<_containerJointPos.size();i++) {
        size_t index = _containerJointPos[i].indexJointInModel;
        if (index != (size_t)-1) {
            //Assign state structure
            _threadManager.refState().jointPos(index) = _containerJointPos[i].filterPos.value();
            _threadManager.refState().jointVel(index) = _containerJointPos[i].filterVel.value();
            if (_containerJointPos[i].hasTorque) {
                _threadManager.refState().jointTau(index) = _containerJointPos[i].filterTau.value();
            }
            _threadManager.refState().jointCmd(index) = _containerJointPos[i].positionCommanded;
        } 
    }
}

template <typename T_State, typename T_Command>
void ControllerBase<T_State, T_Command>::updatePublishTaskState(double time, double dt)
{
    //Publish to RhIO
    for (size_t i=0;i<_containerJointPos.size();i++) {
        size_t index = _containerJointPos[i].indexJointInModel;
        if (index != (size_t)-1) {
            _containerJointPos[i].rhioStatePos = _threadManager.refState().jointPos(index);
            _containerJointPos[i].rhioStateVel = _threadManager.refState().jointVel(index);
            _containerJointPos[i].rhioStateTau = _threadManager.refState().jointTau(index);
        } 
    }

    //Send state to non RT task
    if (_threadManager.isTask()) {
        //Swap lock free buffer
        _threadManager.pushState();
    }
}
        
template <typename T_State, typename T_Command>
void ControllerBase<T_State, T_Command>::updateProcessTaskCommand(double time, double dt)
{
    //Retrieve and process commands from non RT task 
    //(only in RUNNING state)
    if (_state == State_t::RUNNING && _threadManager.isTask()) {
        //Lock free retrieving of non RT command
        _threadManager.pullCommand();
        for (size_t i=0;i<_containerJointPos.size();i++) {
            size_t index = _containerJointPos[i].indexJointInModel;
            if (index != (size_t)-1) {
                if (_threadManager.refCommand().jointIsUsed(index)) {
                    //Detect timing issues
                    if (_threadManager.refCommand().timeState > time) {
                        ROS_WARN(
                            "inria::ControllerBase::updateCommandInterpolation: "
                            "State time in future.");
                    }
                    if (_threadManager.refCommand().timeCmd < _containerJointPos[i].nextCommandTime) {
                        ROS_WARN(
                            "inria::ControllerBase::updateCommandInterpolation: "
                            "Command time not progressing.");
                    }
                    //Retrieve ahead command for smooth interpolation
                    if (_threadManager.refCommand().timeCmd > time) {
                        _containerJointPos[i].nextCommandPosition = 
                            _threadManager.refCommand().jointCmd(index);
                        _containerJointPos[i].nextCommandTime = 
                            _threadManager.refCommand().timeCmd;
                    //No interpolation case
                    } else {
                        _containerJointPos[i].positionCommanded = 
                            _threadManager.refCommand().jointCmd(index);
                        ROS_WARN(
                            "inria::ControllerBase::updateCommandInterpolation: "
                            "Command time in the past.");
                    }
                }
            }
        }
    }
}

template <typename T_State, typename T_Command>
void ControllerBase<T_State, T_Command>::updateCommandInterpolation(double time, double dt)
{
    //Compute command interpolation if given command is in the future
    if (_state == State_t::INIT_POSTURE || _state == State_t::RUNNING) {
        for (size_t i=0;i<_containerJointPos.size();i++) {
            if (_containerJointPos[i].nextCommandTime > time) {
                double deltaTime = _containerJointPos[i].nextCommandTime - time;
                double ratio = dt/(deltaTime+dt);
                double deltaCommand = 
                    _containerJointPos[i].nextCommandPosition - 
                    _containerJointPos[i].positionCommanded;
                if (ratio < 0.0 || ratio > 1.0) {
                    ROS_WARN(
                        "inria::ControllerBase::updateCommandInterpolation: "
                        "Interpolation error.");
                }
                _containerJointPos[i].positionCommanded += ratio*deltaCommand;
            }
        }
    }

    //If actuators are disabled, keep resetting 
    //commanded position to measured position
    if (_state == State_t::STOPPED_NOPOWER) {
        for (size_t i=0;i<_containerJointPos.size();i++) {
            _containerJointPos[i].positionCommanded =
                _containerJointPos[i].handleJoint.getPosition();
        }
    }
    //Set commanded velocity to zero when not running
    if (_state != State_t::RUNNING) {
        for (size_t i=0;i<_containerJointVel.size();i++) {
            _containerJointVel[i].velocityCommanded = 0.0;
        }
    }
}
        
template <typename T_State, typename T_Command>
void ControllerBase<T_State, T_Command>::updateCheckSafetyLimits(double dt)
{
    //No check if the controller is stopped to avoid repeated errors
    if (_state == State_t::STOPPED_NOPOWER || _state == State_t::STOPPED_HOLD) {
        return;
    }

    bool isError = false;
    for (size_t i=0;i<_containerJointPos.size();i++) {
        //Check NaN in lowlevel
        if (
            std::isnan(_containerJointPos[i].handleJoint.getPosition()) || 
            std::isinf(_containerJointPos[i].handleJoint.getPosition())
        ) {
            isError = true;
            ROS_WARN_STREAM(
                "inria::ControllerBase::checkSafetyLimits: "
                "NaN in measured position: " << _containerJointPos[i].name);
        }
        if (
            _containerJointPos[i].hasAbsPos && (
            std::isnan(_containerJointPos[i].handleJoint.getAbsolutePosition()) || 
            std::isinf(_containerJointPos[i].handleJoint.getAbsolutePosition()))
        ) {
            isError = true;
            ROS_WARN_STREAM(
                "inria::ControllerBase::checkSafetyLimits: "
                "NaN in measured absolute position: " << _containerJointPos[i].name);
        }
        if (
            std::isnan(_containerJointPos[i].handleJoint.getVelocity()) || 
            std::isinf(_containerJointPos[i].handleJoint.getVelocity())
        ) {
            isError = true;
            ROS_WARN_STREAM(
                "inria::ControllerBase::checkSafetyLimits: "
                "NaN in measured velocity: " << _containerJointPos[i].name);
        }
        if (
            _containerJointPos[i].hasTorque && (
            std::isnan(_containerJointPos[i].handleJoint.getTorqueSensor()) || 
            std::isinf(_containerJointPos[i].handleJoint.getTorqueSensor()))
        ) {
            isError = true;
            ROS_WARN_STREAM(
                "inria::ControllerBase::checkSafetyLimits: "
                "NaN in measured torque: " << _containerJointPos[i].name);
        }
        if (
            std::isnan(_containerJointPos[i].handleJoint.getEffort()) || 
            std::isinf(_containerJointPos[i].handleJoint.getEffort())
        ) {
            isError = true;
            ROS_WARN_STREAM(
                "inria::ControllerBase::checkSafetyLimits: "
                "NaN in measured effort: " << _containerJointPos[i].name);
        }
        //Check NaN in command
        if (
            std::isnan(_containerJointPos[i].positionCommanded) || 
            std::isinf(_containerJointPos[i].positionCommanded)
        ) {
            isError = true;
            ROS_WARN_STREAM(
                "inria::ControllerBase::checkSafetyLimits: "
                "NaN in command: " << _containerJointPos[i].name);
        }
        //Check measured position limits
        if (
            _containerJointPos[i].handleJoint.getPosition() < 
            _containerJointPos[i].limitLowerPos + _rhioLimitPositionMargin
        ) {
            isError = true;
            ROS_WARN_STREAM(
                "inria::ControllerBase::checkSafetyLimits: "
                "Measured lower position limit violated: " << _containerJointPos[i].name
                << " pos=" << _containerJointPos[i].handleJoint.getPosition()
                << " lower=" << _containerJointPos[i].limitLowerPos + _rhioLimitPositionMargin);
        }
        if (
            _containerJointPos[i].handleJoint.getPosition() >
            _containerJointPos[i].limitUpperPos - _rhioLimitPositionMargin
        ) {
            isError = true;
            ROS_WARN_STREAM(
                "inria::ControllerBase::checkSafetyLimits: "
                "Measured upper position limit violated: " << _containerJointPos[i].name
                << " pos=" << _containerJointPos[i].handleJoint.getPosition()
                << " upper=" << _containerJointPos[i].limitUpperPos - _rhioLimitPositionMargin);
        }
        //Check measured velocity
        if (
            std::fabs(_containerJointPos[i].handleJoint.getVelocity()) >
            _containerJointPos[i].limitAbsVel
        ) {
            isError = true;
            ROS_WARN_STREAM(
                "inria::ControllerBase::checkSafetyLimits: "
                "Measured velocity limit violated: " << _containerJointPos[i].name
                << " vel=" << _containerJointPos[i].handleJoint.getVelocity()
                << " limit=" << _containerJointPos[i].limitAbsVel);
        }
        //Check measured torque
        if (
            _containerJointPos[i].hasTorque &&
            (std::fabs(_containerJointPos[i].handleJoint.getTorqueSensor()) >
            _containerJointPos[i].limitAbsTau)
        ) {
            isError = true;
            ROS_WARN_STREAM(
                "inria::ControllerBase::checkSafetyLimits: "
                "Measured torque limit violated: " << _containerJointPos[i].name);
        }
        //Check commanded position limits
        if (
            _containerJointPos[i].positionCommanded < 
            _containerJointPos[i].limitLowerPos + _rhioLimitPositionMargin
        ) {
            isError = true;
            ROS_WARN_STREAM(
                "inria::ControllerBase::checkSafetyLimits: "
                "Commanded lower position limit violated: " << _containerJointPos[i].name
                << " pos=" << _containerJointPos[i].positionCommanded
                << " lower=" << _containerJointPos[i].limitLowerPos + _rhioLimitPositionMargin);
        }
        if (
            _containerJointPos[i].positionCommanded >
            _containerJointPos[i].limitUpperPos - _rhioLimitPositionMargin
        ) {
            isError = true;
            ROS_WARN_STREAM(
                "inria::ControllerBase::checkSafetyLimits: "
                "Commanded upper position limit violated: " << _containerJointPos[i].name
                << " pos=" << _containerJointPos[i].positionCommanded
                << " upper=" << _containerJointPos[i].limitUpperPos - _rhioLimitPositionMargin);
        }
        //Check command change
        double deltaCommand = std::fabs(
            _containerJointPos[i].positionCommanded - _containerJointPos[i].prevPositionCommanded);
        double rangePosition = 
            _containerJointPos[i].limitUpperPos - _containerJointPos[i].limitLowerPos;
        double deltaLimit = dt*rangePosition/_rhioLimitRangeTime;
        if (deltaCommand > deltaLimit) {
            isError = true;
            ROS_WARN_STREAM(
                "inria::ControllerBase::checkSafetyLimits: "
                "Command change violation: " << _containerJointPos[i].name
                << " prev=" << _containerJointPos[i].prevPositionCommanded
                << " next=" << _containerJointPos[i].positionCommanded
                << " change=" << deltaCommand
                << " bound=" << deltaLimit);
        }
    }
    for (size_t i=0;i<_containerJointVel.size();i++) {
        //Check NaN in lowlevel
        if (
            std::isnan(_containerJointVel[i].handleJoint.getPosition()) || 
            std::isinf(_containerJointVel[i].handleJoint.getPosition())
        ) {
            isError = true;
            ROS_WARN_STREAM(
                "inria::ControllerBase::checkSafetyLimits: "
                "NaN in measured position: " << _containerJointVel[i].name);
        }
        if (
            _containerJointVel[i].hasAbsPos && (
            std::isnan(_containerJointVel[i].handleJoint.getAbsolutePosition()) || 
            std::isinf(_containerJointVel[i].handleJoint.getAbsolutePosition()))
        ) {
            isError = true;
            ROS_WARN_STREAM(
                "inria::ControllerBase::checkSafetyLimits: "
                "NaN in measured absolute position: " << _containerJointVel[i].name);
        }
        if (
            std::isnan(_containerJointVel[i].handleJoint.getVelocity()) || 
            std::isinf(_containerJointVel[i].handleJoint.getVelocity())
        ) {
            isError = true;
            ROS_WARN_STREAM(
                "inria::ControllerBase::checkSafetyLimits: "
                "NaN in measured velocity: " << _containerJointVel[i].name);
        }
        if (
            std::isnan(_containerJointVel[i].handleJoint.getEffort()) || 
            std::isinf(_containerJointVel[i].handleJoint.getEffort())
        ) {
            isError = true;
            ROS_WARN_STREAM(
                "inria::ControllerBase::checkSafetyLimits: "
                "NaN in measured effort: " << _containerJointVel[i].name);
        }
        //Check NaN in command
        if (
            std::isnan(_containerJointVel[i].velocityCommanded) || 
            std::isinf(_containerJointVel[i].velocityCommanded)
        ) {
            isError = true;
            ROS_WARN_STREAM(
                "inria::ControllerBase::checkSafetyLimits: "
                "NaN in command: " << _containerJointVel[i].name);
        }
        //Check measured velocity
        if (
            std::fabs(_containerJointVel[i].handleJoint.getVelocity()) >
            _containerJointVel[i].limitAbsVel
        ) {
            isError = true;
            ROS_WARN_STREAM(
                "inria::ControllerBase::checkSafetyLimits: "
                "Measured velocity limit violated: " << _containerJointVel[i].name
                << " vel=" << _containerJointVel[i].handleJoint.getVelocity()
                << " limit=" << _containerJointVel[i].limitAbsVel);
        }
        //Check commanded velocity limits
        if (
            std::fabs(_containerJointVel[i].velocityCommanded) > 
            _containerJointVel[i].limitAbsVel
        ) {
            isError = true;
            ROS_WARN_STREAM(
                "inria::ControllerBase::checkSafetyLimits: "
                "Commanded velocity limit violated: " << _containerJointVel[i].name
                << " vel=" << _containerJointVel[i].velocityCommanded
                << " limit=" << _containerJointVel[i].limitAbsVel);
        }
        //Check command change
        double deltaCommand = std::fabs(
            _containerJointVel[i].velocityCommanded - _containerJointVel[i].prevVelocityCommanded);
        double rangeVelocity = _containerJointVel[i].limitAbsVel;
        double deltaLimit = dt*rangeVelocity/_rhioLimitRangeTime;
        if (deltaCommand > deltaLimit) {
            isError = true;
            ROS_WARN_STREAM(
                "inria::ControllerBase::checkSafetyLimits: "
                "Command change violation: " << _containerJointVel[i].name
                << " prev=" << _containerJointVel[i].prevVelocityCommanded
                << " next=" << _containerJointVel[i].velocityCommanded
                << " change=" << deltaCommand
                << " bound=" << deltaLimit);
        }
    }
    if (isError) {
        if (_rhioOnErrorStateHold) {
            _state = State_t::STOPPED_HOLD;
            ROS_WARN_STREAM(
                "inria::ControllerBase::checkSafetyLimits: "
                "Safety error: switch to hold state.");
        } else {
            _state = State_t::STOPPED_NOPOWER;
            ROS_WARN_STREAM(
                "inria::ControllerBase::checkSafetyLimits: "
                "Safety error: switch to no power state.");
        }
    }
}

template <typename T_State, typename T_Command>
void ControllerBase<T_State, T_Command>::updateStateMachine(double time, double dt)
{
    //Check non RT task for error
    if (_state == State_t::INIT_POSTURE || _state == State_t::RUNNING) {
        if (_threadManager.isError()) {
            if (_rhioOnErrorStateHold) {
                _state = State_t::STOPPED_HOLD;
                ROS_WARN_STREAM(
                    "inria::ControllerBase::updateStateMachine: "
                    "Task error: switch to hold state.");
            } else {
                _state = State_t::STOPPED_NOPOWER;
                ROS_WARN_STREAM(
                    "inria::ControllerBase::updateStateMachine: "
                    "Task error: switch to no power state.");
            }
        }
    }
    //Process emergency and initialization signals
    if (_rhioAskNoPower) {
        _rhioAskNoPower = false;
        _rhioAskHold = false;
        _rhioAskRunning = false;
        _state = State_t::STOPPED_NOPOWER;
        ROS_INFO_STREAM(
            "inria::ControllerBase::updateStateMachine: "
            "Ask State: no power (emergency).");
    }
    if (_rhioAskHold) {
        _rhioAskHold = false;
        _rhioAskRunning = false;
        if (_state == State_t::INIT_POSTURE || _state == State_t::RUNNING) {
            _state = State_t::STOPPED_HOLD;
            ROS_INFO_STREAM(
                "inria::ControllerBase::updateStateMachine: "
                "Ask State: hold (keep position posture).");
        }
    }
    if (_rhioAskRunning) {
        _rhioAskRunning = false;
        //Setup initialization from no power state or hold state
        if (_state == State_t::STOPPED_NOPOWER || _state == State_t::STOPPED_HOLD) {
            if (_state == State_t::STOPPED_NOPOWER) {
                _state = State_t::INIT_POWER;
                _timeTransition = _rhioDurationInitPower;
                for (size_t i=0;i<_containerJointPos.size();i++) {
                    _containerJointPos[i].positionCommanded = 
                        _containerJointPos[i].handleJoint.getPosition();
                }
            }
            if (_state == State_t::STOPPED_HOLD) {
                _state = State_t::INIT_POSTURE;
                _timeTransition = _rhioDurationInitPosture;
                //Initialize command interpolation for posture trajectory
                for (size_t i=0;i<_containerJointPos.size();i++) {
                    _containerJointPos[i].nextCommandPosition = 
                        _containerJointPos[i].posInitEnd;
                    _containerJointPos[i].nextCommandTime = 
                        time + _rhioDurationInitPosture;
                }
            }
            ROS_INFO_STREAM(
                "inria::ControllerBase::updateStateMachine: "
                "Ask State: running (initialization): " <<
                _threadManager.nameTask());
        }
    }
    //State transitions
    if (_state == State_t::INIT_POWER || _state == State_t::INIT_POSTURE) {
        _timeTransition -= dt;
    }
    if (_state == State_t::INIT_POWER && _timeTransition <= 0.0) {
        _state = State_t::INIT_POSTURE;
        _timeTransition = _rhioDurationInitPosture;
        //Initialize command interpolation for posture trajectory
        for (size_t i=0;i<_containerJointPos.size();i++) {
            _containerJointPos[i].nextCommandPosition = 
                _containerJointPos[i].posInitEnd;
            _containerJointPos[i].nextCommandTime = 
                time + _rhioDurationInitPosture;
        }
    }
    if (_state == State_t::INIT_POSTURE && _timeTransition <= 0.0) {
        _state = State_t::RUNNING;
    }
    //Publish to RhIO
    _rhioState = (int)_state;
}
        
//Explicit template instantiation for robot specialisations
template class ControllerBase<ControllerTalosState_t, ControllerTalosCommand_t>;

}

