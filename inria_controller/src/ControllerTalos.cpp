#include <inria_controller/TaskSEIKO.hpp>

#include <inria_controller/ControllerTalos.hpp>
#include <inria_model/TalosDOFs.h>
#include <inria_maths/Euler.h>

namespace inria {
        
ControllerTalos::ControllerTalos() :
    ControllerBase(),
    _serverUDP(),
    _indexFrameInModelFootSoleLeft(-1),
    _indexFrameInModelFootSoleRight(-1),
    _indexFrameInModelFootFTLeft(-1),
    _indexFrameInModelFootFTRight(-1),
    _indexFrameInModelHandFrameLeft(-1),
    _indexFrameInModelHandFrameRight(-1),
    _indexFrameInModelHandFTLeft(-1),
    _indexFrameInModelHandFTRight(-1),
    _indexFrameInModelIMULink(-1),
    _indexFrameInModelTorsoLink(-1),
    _indexInContainerFTFootLeft(-1),
    _indexInContainerFTFootRight(-1),
    _indexInContainerFTHandLeft(-1),
    _indexInContainerFTHandRight(-1),
    _indexInContainerIMUBaseLink(-1),
    _biasWrenchFootLeft(Eigen::Vector6d::Zero()),
    _biasWrenchFootRight(Eigen::Vector6d::Zero()),
    _biasWrenchHandLeft(Eigen::Vector6d::Zero()),
    _biasWrenchHandRight(Eigen::Vector6d::Zero()),
    _biasTorqueJoint(),
    _bufferWrenchFootLeft(),
    _bufferWrenchFootRight(),
    _bufferWrenchHandLeft(),
    _bufferWrenchHandRight(),
    _bufferTorqueJoint(),
    _rhioIsTransformFT(),
    _rhioAskTare(),
    _rhioAskTareFeet(),
    _rhioAskTareHands(),
    _rhioStateBaseQuat(),
    _rhioStateFootLeftTorque(),
    _rhioStateFootLeftForce(),
    _rhioStateFootLeftCoP(),
    _rhioStateFootLeftFriction(),
    _rhioStateFootRightForce(),
    _rhioStateFootRightTorque(),
    _rhioStateFootRightCoP(),
    _rhioStateFootRightFriction(),
    _rhioStateHandLeftTorque(),
    _rhioStateHandLeftForce(),
    _rhioStateHandRightTorque(),
    _rhioStateHandRightForce(),
    _indexInContainerJointPosGripperLeft(-1),
    _indexInContainerJointPosGripperRight(-1),
    _rhioGripperLeftPosRatio(),
    _rhioGripperRightPosRatio(),
    _rhioGripperLeftCurrentRatio(),
    _rhioGripperRightCurrentRatio(),
    _filterPosGripperLeft(),
    _filterPosGripperRight()
{
    //Specific Talos joint to exclude from lowlevel
    _setJointsExclude = {
        "arm_right_5_joint",
        "arm_right_6_joint",
        "arm_right_7_joint",
        "gripper_left_joint",
        "gripper_right_joint",
        "head_1_joint",
        "head_2_joint",
    };
    
    //Specific Tiago joints which have 
    //an absolute position sensor
    _setJointsWithAbsPosSensor = {
        "leg_left_1_joint",
        "leg_left_2_joint",
        "leg_left_3_joint",
        "leg_left_4_joint",
        "leg_left_5_joint",
        "leg_left_6_joint",
        "leg_right_1_joint",
        "leg_right_2_joint",
        "leg_right_3_joint",
        "leg_right_4_joint",
        "leg_right_5_joint",
        "leg_right_6_joint",
        "arm_left_1_joint",
        "arm_left_2_joint",
        "arm_left_3_joint",
        "arm_left_4_joint",
        "arm_right_1_joint",
        "arm_right_2_joint",
        "arm_right_3_joint",
        "arm_right_4_joint",
        "torso_1_joint",
        "torso_2_joint",
        "gripper_left_joint",
        "gripper_right_joint",
    };

    //Specific Tiago joints which have a torque sensor
    _setJointsWithTorqueSensor = {
        "leg_left_1_joint",
        "leg_left_2_joint",
        "leg_left_3_joint",
        "leg_left_4_joint",
        "leg_left_5_joint",
        "leg_left_6_joint",
        "leg_right_1_joint",
        "leg_right_2_joint",
        "leg_right_3_joint",
        "leg_right_4_joint",
        "leg_right_5_joint",
        "leg_right_6_joint",
        "arm_left_1_joint",
        "arm_left_2_joint",
        "arm_left_3_joint",
        "arm_left_4_joint",
        "arm_right_1_joint",
        "arm_right_2_joint",
        "arm_right_3_joint",
        "arm_right_4_joint",
        "torso_1_joint",
        "torso_2_joint",
    };
    
    //RhIO configuration
    _rhioIsTransformFT.create(RhIO::Root.child("/controller"), "is_transform_ft")
        ->comment("Apply ankle to sole transform to FT sensors")
        ->persisted(true)
        ->defaultValue(true);
    _rhioAskTare.create(RhIO::Root.child("/controller"), "ask_tare")
        ->comment("Tare all force-torque and joint torque sensors")
        ->defaultValue(false);
    _rhioAskTareFeet.create(RhIO::Root.child("/controller"), "ask_tare_feet")
        ->comment("Tare feet force-torque")
        ->defaultValue(false);
    _rhioAskTareHands.create(RhIO::Root.child("/controller"), "ask_tare_hands")
        ->comment("Tare hands force-torque")
        ->defaultValue(false);

    //Specific controller parameters
    RhIO::Root.setStr("/server/hostname", "Talos");
    _rhioOnStartStateHold = true;
    _rhioOnErrorStateHold = false;
    _rhioLimitPositionMargin = DegToRad(0.0);
    RhIO::Root.setStr("/controller/model_path",
        "package://inria_talos_description/urdf/talos_stump.urdf");
    _rhioCutoffFreqFT = 5.0;
    _rhioCutoffFreqJointTau = 2.0;
    
    //Configure RhIO for state estimation publishing
    RhIO::Root.newChild("/state/base");
    RhIO::Root.newChild("/state/foot_left");
    RhIO::Root.newChild("/state/foot_right");
    RhIO::Root.newChild("/state/hand_left");
    RhIO::Root.newChild("/state/hand_right");
    _rhioStateBaseQuat.create(
        RhIO::Root.child("/state/base"), "quat");
    _rhioStateFootLeftTorque.create(
        RhIO::Root.child("/state/foot_left"), "torque");
    _rhioStateFootLeftForce.create(
        RhIO::Root.child("/state/foot_left"), "force");
    _rhioStateFootLeftCoP.create(
        RhIO::Root.child("/state/foot_left"), "cop");
    _rhioStateFootLeftFriction.create(
        RhIO::Root.child("/state/foot_left"), "friction");
    _rhioStateFootRightTorque.create(
        RhIO::Root.child("/state/foot_right"), "torque");
    _rhioStateFootRightForce.create(
        RhIO::Root.child("/state/foot_right"), "force");
    _rhioStateFootRightCoP.create(
        RhIO::Root.child("/state/foot_right"), "cop");
    _rhioStateFootRightFriction.create(
        RhIO::Root.child("/state/foot_right"), "friction");
    _rhioStateHandLeftTorque.create(
        RhIO::Root.child("/state/hand_left"), "torque");
    _rhioStateHandLeftForce.create(
        RhIO::Root.child("/state/hand_left"), "force");
    _rhioStateHandRightTorque.create(
        RhIO::Root.child("/state/hand_right"), "torque");
    _rhioStateHandRightForce.create(
        RhIO::Root.child("/state/hand_right"), "force");
    
    //Configuration of gripper command and filter
    RhIO::Root.newChild("/hands");
    RhIO::IONode& nodeHands = RhIO::Root.child("/hands");
    _rhioGripperLeftPosRatio.create(nodeHands, "left_pos_ratio")
        ->minimum(0.0)
        ->maximum(1.0)
        ->defaultValue(0.0);
    _rhioGripperRightPosRatio.create(nodeHands, "right_pos_ratio")
        ->minimum(0.0)
        ->maximum(1.0)
        ->defaultValue(0.0);
    _rhioGripperLeftCurrentRatio.create(nodeHands, "left_current_ratio")
        ->persisted(true)
        ->minimum(0.0)
        ->maximum(1.0)
        ->defaultValue(0.3);
    _rhioGripperRightCurrentRatio.create(nodeHands, "right_current_ratio")
        ->persisted(true)
        ->minimum(0.0)
        ->maximum(1.0)
        ->defaultValue(0.3);
    _filterPosGripperLeft.maxVel() = 0.5;
    _filterPosGripperLeft.maxAcc() = 0.5;
    _filterPosGripperRight.maxVel() = 0.5;
    _filterPosGripperRight.maxAcc() = 0.5;

    //Specific RhIO commands
    RhIO::Root.newCommand(
        "tare_all", "Calibrate feet/hands force-torque and joint torque sensors",
        [this]
        (const std::vector<std::string>& args) -> std::string
        {
            (void)args;
            this->_rhioAskTare.set(true);
            return "Tare asked";
        });
    RhIO::Root.newCommand(
        "tare_hands", "Calibrate feet/hands force-torque and joint torque sensors",
        [this]
        (const std::vector<std::string>& args) -> std::string
        {
            (void)args;
            this->_rhioAskTareHands.set(true);
            return "Tare asked";
        });
}

bool ControllerTalos::initRequest(
    hardware_interface::RobotHW* robot_hw,
    ros::NodeHandle& root_nh, 
    ros::NodeHandle& controller_nh,
    controller_interface::ControllerBase::ClaimedResources& 
        claimed_resources)
{
    //Call to base initialization
    bool isSuccess = ControllerBase::initRequest(
        robot_hw, root_nh, controller_nh, claimed_resources);
    if (!isSuccess) {
        return false;
    }

    //Configure default joint posture
    for (size_t i=0;i<_containerJointPos.size();i++) {
        _containerJointPos[i].posInitEnd = PostureDefault.at(
            _containerJointPos[i].name);
    }

    //Reset calibration bias offsets
    _biasWrenchFootLeft = Eigen::Vector6d::Zero();
    _biasWrenchFootRight = Eigen::Vector6d::Zero();
    _biasWrenchHandLeft = Eigen::Vector6d::Zero();
    _biasWrenchHandRight = Eigen::Vector6d::Zero();
    _biasTorqueJoint = Eigen::VectorXd::Zero(_model.sizeJoint());

    //Retrieve frame index for RT state estimation
    _indexFrameInModelFootSoleLeft = _model.getIndexFrame("left_sole_link");
    _indexFrameInModelFootSoleRight = _model.getIndexFrame("right_sole_link");
    _indexFrameInModelFootFTLeft = _model.getIndexFrame("leg_left_6_link");
    _indexFrameInModelFootFTRight = _model.getIndexFrame("leg_right_6_link");
    _indexFrameInModelHandFrameLeft = _model.getIndexFrame("left_hand_frame");
    _indexFrameInModelHandFrameRight = _model.getIndexFrame("right_hand_frame");
    _indexFrameInModelHandFTLeft = _model.getIndexFrame("wrist_left_ft_link");
    _indexFrameInModelHandFTRight = _model.getIndexFrame("wrist_right_ft_link");
    _indexFrameInModelIMULink = _model.getIndexFrame("imu_link");
    _indexFrameInModelTorsoLink = _model.getIndexFrame("torso_2_link");
    //Retrieve index of foot force-torque sensors
    for (size_t i=0;i<_containerForceTorque.size();i++) {
        if (_containerForceTorque[i].name == "left_ankle_ft") {
            _indexInContainerFTFootLeft = i;
        }
        if (_containerForceTorque[i].name == "right_ankle_ft") {
            _indexInContainerFTFootRight = i;
        }
        if (_containerForceTorque[i].name == "left_wrist_ft") {
            _indexInContainerFTHandLeft = i;
        }
        if (_containerForceTorque[i].name == "right_wrist_ft") {
            _indexInContainerFTHandRight = i;
        }
    }
    if (
        _indexInContainerFTFootLeft == (size_t)-1 || 
        _indexInContainerFTFootRight == (size_t)-1
    ) {
        ROS_ERROR_STREAM(
            "inria::ControllerTalos::initRequest: "
            "Foot FT sensor not found.");
        return false;
    }
    if (
        _indexInContainerFTHandLeft == (size_t)-1 || 
        _indexInContainerFTHandRight == (size_t)-1
    ) {
        ROS_ERROR_STREAM(
            "inria::ControllerTalos::initRequest: "
            "Hand FT sensor not found.");
        return false;
    }
    //Retrieve index of base IMU
    if (_containerImu.size() == 0 || _containerImu[0].name != "base_imu") {
        ROS_ERROR_STREAM(
            "inria::ControllerTalos::initRequest: "
            "Base IMU not found.");
        return false;
    }
    _indexInContainerIMUBaseLink = 0;
    
    //Retrieve index for hand gripper joints
    for (size_t i=0;i<_containerJointPos.size();i++) {
        if (_containerJointPos[i].name == "gripper_left_joint") {
            _indexInContainerJointPosGripperLeft = i;
            ROS_INFO_STREAM(
                "inria::ControllerTalos::initRequest: "
                "Found hand left gripper position joint");
        }
        if (_containerJointPos[i].name == "gripper_right_joint") {
            _indexInContainerJointPosGripperRight = i;
            ROS_INFO_STREAM(
                "inria::ControllerTalos::initRequest: "
                "Found hand right gripper position joint");
        }
    }
    
    //Allocate and initialize tasks in non RT thread
    _threadManager.initThread(_model, {
        new TaskSEIKO,
    });
    
    //Start RhIO threads
    RhIO::start(
        RhIO::PortServerRep, 
        RhIO::PortServerPub, 
        50,
        200000000UL,
        5000000UL);
    
    //Start UDP receiver
    auto callback = [](
        const std::vector<std::string>& namesBool,
        const std::vector<std::string>& namesInt,
        const std::vector<std::string>& namesFloat,
        const std::vector<bool>& valuesBool,
        const std::vector<int64_t>& valuesInt,
        const std::vector<double>& valuesFloat)
    {
        try {
            for (size_t i=0;i<namesBool.size();i++) {
                RhIO::Root.setBool(namesBool[i], valuesBool[i]);
            }
            for (size_t i=0;i<namesInt.size();i++) {
                RhIO::Root.setInt(namesInt[i], valuesInt[i]);
            }
            for (size_t i=0;i<namesFloat.size();i++) {
                RhIO::Root.setFloat(namesFloat[i], valuesFloat[i]);
            }
        } catch (const std::logic_error& e) {
            ROS_WARN_STREAM(
                "inria::ControllerTalos: Error UDP transport: " << e.what());
        }
    };
    _serverUDP.listen(9997, callback);
    
    //Verbose
    ROS_INFO_STREAM(
        "inria::ControllerTalos::initRequest: Initialization successful");

    return true;
}
        
void ControllerTalos::updateAssignTaskState(double time, double dt)
{
    //Call to base method
    this->ControllerBase::updateAssignTaskState(time, dt);

    //Assign base orientation from filtered IMU pose
    Eigen::Quaterniond tmpIMUQuat = 
        _containerImu[_indexInContainerIMUBaseLink].filterOrientation.valueQuaternion();
    _model.setBaseToMatchFrameOrientation(
        _indexFrameInModelIMULink, tmpIMUQuat);
    //Remove yaw component from base orientation to set it to zero
    //since IMU yaw orientation integration is not reliable
    Eigen::Quaterniond tmpBaseQuat = _model.getBaseQuat();
    Eigen::Vector3d tmpBaseEuler = MatrixToEulerIntrinsic(tmpBaseQuat.toRotationMatrix());
    tmpBaseEuler.z() = 0.0;
    _model.setBaseQuat(Eigen::Quaterniond(EulerIntrinsicToMatrix(tmpBaseEuler)));
    _model.updateState();

    //Retrieve filtered foot wrenches
    Eigen::Vector6d wrenchReadFootLeft = Eigen::Vector6d::Zero();
    wrenchReadFootLeft.segment(0,3) = 
        _containerForceTorque[_indexInContainerFTFootLeft].filterTorque.value();
    wrenchReadFootLeft.segment(3,3) = 
        _containerForceTorque[_indexInContainerFTFootLeft].filterForce.value();
    Eigen::Vector6d wrenchReadFootRight = Eigen::Vector6d::Zero();
    wrenchReadFootRight.segment(0,3) = 
        _containerForceTorque[_indexInContainerFTFootRight].filterTorque.value();
    wrenchReadFootRight.segment(3,3) = 
        _containerForceTorque[_indexInContainerFTFootRight].filterForce.value();
    
    //Retrieve filtered hand wrenches
    Eigen::Vector6d wrenchReadHandLeft = Eigen::Vector6d::Zero();
    wrenchReadHandLeft.segment(0,3) = 
        _containerForceTorque[_indexInContainerFTHandLeft].filterTorque.value();
    wrenchReadHandLeft.segment(3,3) = 
        _containerForceTorque[_indexInContainerFTHandLeft].filterForce.value();
    Eigen::Vector6d wrenchReadHandRight = Eigen::Vector6d::Zero();
    wrenchReadHandRight.segment(0,3) = 
        _containerForceTorque[_indexInContainerFTHandRight].filterTorque.value();
    wrenchReadHandRight.segment(3,3) = 
        _containerForceTorque[_indexInContainerFTHandRight].filterForce.value();

    //Optionally apply ankle frame to foot center frame wrench transformation
    Eigen::Vector6d wrenchStateFootLeft;
    Eigen::Vector6d wrenchStateFootRight;
    if (_rhioIsTransformFT) {
        //Transform wrenches from FT frames to center of foot frames
        wrenchStateFootLeft = _model.wrenchTransform(
            _indexFrameInModelFootFTLeft, _indexFrameInModelFootSoleLeft, wrenchReadFootLeft);
        wrenchStateFootRight = _model.wrenchTransform(
            _indexFrameInModelFootFTRight, _indexFrameInModelFootSoleRight, wrenchReadFootRight);
        
        //Compute foot wrenches generated by gravity applied on foot CoM.
        //Retrieve mass and center of mass of the feet
        double massFootLeft = _model.getBodyMass(_indexFrameInModelFootFTLeft);
        double massFootRight = _model.getBodyMass(_indexFrameInModelFootFTRight);
        Eigen::Vector3d comFootLeft = _model.getBodyCoM(_indexFrameInModelFootFTLeft);
        Eigen::Vector3d comFootRight = _model.getBodyCoM(_indexFrameInModelFootFTRight);
        //Compute gravity force applied on foot CoM rotated 
        //in the foot torque-force sensor frame
        Eigen::Vector3d gravityFootLeft = Eigen::Vector3d::Zero();
        Eigen::Vector3d gravityFootRight = Eigen::Vector3d::Zero();
        gravityFootLeft.z() = massFootLeft*9.81;
        gravityFootRight.z() = massFootRight*9.81;
        gravityFootLeft = 
            _model.orientation(0, _indexFrameInModelFootFTLeft)*gravityFootLeft;
        gravityFootRight = 
            _model.orientation(0, _indexFrameInModelFootFTRight)*gravityFootRight;
        //Compute the wrench created by gravity when applied 
        //on foot torque-force sensor frame
        Eigen::Vector6d wrenchGravFootLeft = Eigen::Vector6d::Zero();
        Eigen::Vector6d wrenchGravFootRight = Eigen::Vector6d::Zero();
        wrenchGravFootLeft.segment(0, 3) = comFootLeft.cross(gravityFootLeft);
        wrenchGravFootLeft.segment(3, 3) = gravityFootLeft;
        wrenchGravFootRight.segment(0, 3) = comFootRight.cross(gravityFootRight);
        wrenchGravFootRight.segment(3, 3) = gravityFootRight;

        //Add wrench generated by foot gravity
        wrenchStateFootLeft += wrenchGravFootLeft;
        wrenchStateFootRight += wrenchGravFootRight;
    } else {
        //No transform and no foot weight compensation
        wrenchStateFootLeft = wrenchReadFootLeft;
        wrenchStateFootRight = wrenchReadFootRight;
    }

    //Transform wrenches from wrist frames to hand frames
    Eigen::Vector6d wrenchStateHandLeft;
    Eigen::Vector6d wrenchStateHandRight;
    if (_rhioIsTransformFT) {
        //Transform wrenches from FT frames to hand frames
        wrenchStateHandLeft = _model.wrenchTransform(
            _indexFrameInModelHandFTLeft, _indexFrameInModelHandFrameLeft, wrenchReadHandLeft);
        wrenchStateHandRight = _model.wrenchTransform(
            _indexFrameInModelHandFTRight, _indexFrameInModelHandFrameRight, wrenchReadHandRight);
    } else {
        //No transform
        wrenchStateHandLeft = wrenchReadHandLeft;
        wrenchStateHandRight = wrenchReadHandRight;
    }

    //Store in circular buffer non calibrated wrench measurements
    _bufferWrenchFootLeft.append(wrenchStateFootLeft);
    _bufferWrenchFootRight.append(wrenchStateFootRight);
    _bufferWrenchHandLeft.append(wrenchStateHandLeft);
    _bufferWrenchHandRight.append(wrenchStateHandRight);
    //Compute and store in circular buffers the expected joint torque 
    //correction from raw measurements
    _bufferTorqueJoint.append(
        _threadManager.refState().jointTau - 
        _model.computeJointTorqueSingleContact(_indexFrameInModelIMULink));
    
    //Apply tare calibration bias offsets
    wrenchStateFootLeft -= _biasWrenchFootLeft;
    wrenchStateFootRight -= _biasWrenchFootRight;
    wrenchStateHandLeft -= _biasWrenchHandLeft;
    wrenchStateHandRight -= _biasWrenchHandRight;
    _threadManager.refState().jointTau -= _biasTorqueJoint;
    
    //Compute measured center of pressure and friction coefficient
    Eigen::Vector2d copFootLeft = Eigen::Vector2d::Zero();
    Eigen::Vector2d copFootRight = Eigen::Vector2d::Zero();
    Eigen::Vector2d frictionFootLeft = Eigen::Vector2d::Zero();
    Eigen::Vector2d frictionFootRight = Eigen::Vector2d::Zero();
    if (std::fabs(wrenchStateFootLeft(5)) > 1.0) {
        copFootLeft.x() = -wrenchStateFootLeft(1)/std::fabs(wrenchStateFootLeft(5));
        copFootLeft.y() = wrenchStateFootLeft(0)/std::fabs(wrenchStateFootLeft(5));
        frictionFootLeft.x() = wrenchStateFootLeft(3)/std::fabs(wrenchStateFootLeft(5));
        frictionFootLeft.y() = wrenchStateFootLeft(4)/std::fabs(wrenchStateFootLeft(5));
    }
    if (std::fabs(wrenchStateFootRight(5)) > 1.0) {
        copFootRight.x() = -wrenchStateFootRight(1)/std::fabs(wrenchStateFootRight(5));
        copFootRight.y() = wrenchStateFootRight(0)/std::fabs(wrenchStateFootRight(5));
        frictionFootRight.x() = wrenchStateFootRight(3)/std::fabs(wrenchStateFootRight(5));
        frictionFootRight.y() = wrenchStateFootRight(4)/std::fabs(wrenchStateFootRight(5));
    }

    //Publish wrenches state to RhIO
    _rhioStateBaseQuat = _model.getBaseQuat();
    _rhioStateFootLeftTorque = wrenchStateFootLeft.segment(0,3);
    _rhioStateFootLeftForce = wrenchStateFootLeft.segment(3,3);
    _rhioStateFootRightTorque = wrenchStateFootRight.segment(0,3);
    _rhioStateFootRightForce = wrenchStateFootRight.segment(3,3);
    _rhioStateFootLeftCoP = copFootLeft;
    _rhioStateFootLeftFriction = frictionFootLeft;
    _rhioStateFootRightCoP = copFootRight;
    _rhioStateFootRightFriction = frictionFootRight;
    _rhioStateHandLeftTorque = wrenchStateHandLeft.segment(0,3);
    _rhioStateHandLeftForce = wrenchStateHandLeft.segment(3,3);
    _rhioStateHandRightTorque = wrenchStateHandRight.segment(0,3);
    _rhioStateHandRightForce = wrenchStateHandRight.segment(3,3);
    
    //Compute and store calibration bias from averaged buffer
    if (_rhioAskTare) {
        if (
            _bufferTorqueJoint.size() > 0
        ) {
            //Compute average raw measurements
            Eigen::VectorXd sumTorqueJoint = Eigen::VectorXd::Zero(_model.sizeJoint());
            for (size_t i=0;i<_bufferTorqueJoint.size();i++) {
                sumTorqueJoint += _bufferTorqueJoint.get(i);
            }
            //Assign bias
            _biasTorqueJoint = sumTorqueJoint/(double)_bufferTorqueJoint.size();
            //Verbose
            ROS_INFO_STREAM(
                "inria::ControllerTalos::update: Tare joint torque: "
                << _biasTorqueJoint.transpose());
        }
    }
    if (_rhioAskTare || _rhioAskTareFeet) {
        if (
            _bufferWrenchFootLeft.size() > 0 &&
            _bufferWrenchFootRight.size() > 0
        ) {
            //Compute average raw measurements
            Eigen::Vector6d sumWrenchFootLeft = Eigen::Vector6d::Zero();
            Eigen::Vector6d sumWrenchFootRight = Eigen::Vector6d::Zero();
            for (size_t i=0;i<_bufferWrenchFootLeft.size();i++) {
                sumWrenchFootLeft += _bufferWrenchFootLeft.get(i);
            }
            for (size_t i=0;i<_bufferWrenchFootRight.size();i++) {
                sumWrenchFootRight += _bufferWrenchFootRight.get(i);
            }
            //Assign bias
            _biasWrenchFootLeft = sumWrenchFootLeft/(double)_bufferWrenchFootLeft.size();
            _biasWrenchFootRight = sumWrenchFootRight/(double)_bufferWrenchFootRight.size();
            //Verbose
            ROS_INFO_STREAM(
                "inria::ControllerTalos::update: Tare left foot wrench: "
                << _biasWrenchFootLeft.transpose());
            ROS_INFO_STREAM(
                "inria::ControllerTalos::update: Tare right foot wrench: "
                << _biasWrenchFootRight.transpose());
        }
    }
    if (_rhioAskTare || _rhioAskTareHands) {
        if (
            _bufferWrenchHandLeft.size() > 0 &&
            _bufferWrenchHandRight.size() > 0
        ) {
            //Compute average raw measurements
            Eigen::Vector6d sumWrenchHandLeft = Eigen::Vector6d::Zero();
            Eigen::Vector6d sumWrenchHandRight = Eigen::Vector6d::Zero();
            for (size_t i=0;i<_bufferWrenchHandLeft.size();i++) {
                sumWrenchHandLeft += _bufferWrenchHandLeft.get(i);
            }
            for (size_t i=0;i<_bufferWrenchHandRight.size();i++) {
                sumWrenchHandRight += _bufferWrenchHandRight.get(i);
            }
            //Assign bias
            _biasWrenchHandLeft = sumWrenchHandLeft/(double)_bufferWrenchHandLeft.size();
            _biasWrenchHandRight = sumWrenchHandRight/(double)_bufferWrenchHandRight.size();
            //Verbose
            ROS_INFO_STREAM(
                "inria::ControllerTalos::update: Tare left hand wrench: "
                << _biasWrenchHandLeft.transpose());
            ROS_INFO_STREAM(
                "inria::ControllerTalos::update: Tare right hand wrench: "
                << _biasWrenchHandRight.transpose());
        }
    }
    if (_rhioAskTare || _rhioAskTareFeet || _rhioAskTareHands) {
        _rhioAskTare = false;
        _rhioAskTareFeet = false;
        _rhioAskTareHands = false;
    }

    //Assign task wrenches and forces state
    _threadManager.refState().wrenchFootLeft = wrenchStateFootLeft;
    _threadManager.refState().wrenchFootRight = wrenchStateFootRight;
    _threadManager.refState().wrenchHandLeft = wrenchStateHandLeft;
    _threadManager.refState().wrenchHandRight = wrenchStateHandRight;
    //Assign base orientation
    _threadManager.refState().quatBase = _model.getBaseQuat();
    //Assign torso IMU orientation
    _threadManager.refState().quatTorsoIMU = 
        _containerImu[_indexInContainerIMUBaseLink].filterOrientation.valueQuaternion();
}

void ControllerTalos::updateCommandInterpolation(double time, double dt)
{
    //Gripper target filtering and controller
    if (_state == State_t::RUNNING) {
        if (_indexInContainerJointPosGripperLeft != (size_t)-1) {
            //Retrieve target position
            double ratio = _rhioGripperLeftPosRatio;
            double targetPos = -0.85*ratio - 0.05;
            //Apply filtering
            _filterPosGripperLeft.update(targetPos, dt);
            //Assign command to lowlevel
            _containerJointPos.at(_indexInContainerJointPosGripperLeft).currentRatioCommanded = 
                _rhioGripperLeftCurrentRatio;
            _containerJointPos.at(_indexInContainerJointPosGripperLeft).positionCommanded = 
                _filterPosGripperLeft.value();
        }
        if (_indexInContainerJointPosGripperRight != (size_t)-1) {
            //Retrieve target position
            double ratio = _rhioGripperRightPosRatio;
            double targetPos = -0.85*ratio - 0.05;
            //Apply filtering
            _filterPosGripperRight.update(targetPos, dt);
            //Assign command to lowlevel
            _containerJointPos.at(_indexInContainerJointPosGripperRight).currentRatioCommanded = 
                _rhioGripperRightCurrentRatio;
            _containerJointPos.at(_indexInContainerJointPosGripperRight).positionCommanded = 
                _filterPosGripperRight.value();
        }
    } else {
        _rhioGripperLeftPosRatio = 0.0;
        _rhioGripperRightPosRatio = 0.0;
        _filterPosGripperLeft.reset();
        _filterPosGripperRight.reset();
    }

    //Call to base method
    this->ControllerBase::updateCommandInterpolation(time, dt);
}
        
}

PLUGINLIB_EXPORT_CLASS(inria::ControllerTalos, controller_interface::ControllerBase)

