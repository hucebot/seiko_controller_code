#include <chrono>
#include <inria_controller/TaskSEIKO.hpp>
#include <inria_model/TalosDOFs.h>
#include <inria_maths/Clamp.h>
#include <inria_maths/Deadband.h>

namespace inria {

const char* TaskSEIKO::name() const
{
    return "seiko";
}
        
double TaskSEIKO::schedulingFrequency() const
{
    return 500.0;
}
        
double TaskSEIKO::aheadTimeStep() const
{
    return 0.006;
}

void TaskSEIKO::init(
    RhIO::IONode& rhioNode,
    const std::string& pathModel)
{
    //Model initialization
    _modelGoal = Model(pathModel, "base_link");
    _modelRead = Model(pathModel, "base_link");
    
    //SEIKO Retargeting initialization
    _retargeting = SEIKOTalos(_modelGoal);
    _retargeting.setup();
    _retargeting.reset();

    //Wrenches estimation
    _filteredComplimentaryIsInit = false;
    _lastDeltaWrenchLeft.setZero();
    _lastDeltaWrenchRight.setZero();
    _lastDeltaForceLeft.setZero();
    _lastDeltaForceRight.setZero();
    _filteredWrenchLeft.setZero();
    _filteredWrenchRight.setZero();
    _filteredForceLeft.setZero();
    _filteredForceRight.setZero();
    
    //Initialize effector RhIO Wrappers
    for (const auto& it : _retargeting.getMappingContacts()) {
        const std::string& name = it.first;
        rhioNode.newChild(name);
        RhIO::IONode& node = rhioNode.child(name);
        _effectors.insert(std::make_pair(name, Effector_t()));
        _effectors.at(name).isPoint = it.first.find("hand") != std::string::npos;
        _effectors.at(name).askSwitchEnable.create(node, "cmd_ask_switch_enable");
        _effectors.at(name).askSwitchDisable.create(node, "cmd_ask_switch_disable");
        _effectors.at(name).cmdIsPushMode.create(node, "cmd_is_push_mode");
        _effectors.at(name).cmdVelForceNormal.create(node, "cmd_vel_force_normal");
        _effectors.at(name).cmdIsClutch.create(node, "cmd_is_clutch");
        _effectors.at(name).cmdRawPos.create(node, "cmd_raw_pos");
        _effectors.at(name).cmdRawQuat.create(node, "cmd_raw_quat");
        _effectors.at(name).cmdVelLin.create(node, "cmd_vel_lin");
        _effectors.at(name).cmdVelAng.create(node, "cmd_vel_ang");
        _effectors.at(name).admVelLin.create(node, "adm_vel_lin");
        _effectors.at(name).admVelAng.create(node, "adm_vel_ang");
        _effectors.at(name).isEnabled.create(node, "is_enabled");
        _effectors.at(name).targetPos.create(node, "target_pos");
        _effectors.at(name).goalPos.create(node, "goal_pos");
        _effectors.at(name).contactQuat.create(node, "contact_quat");
        _effectors.at(name).deltaForce.create(node, "delta_force");
        _effectors.at(name).deltaTorque.create(node, "delta_torque");
        _effectors.at(name).goalForce.create(node, "goal_force");
        _effectors.at(name).goalTorque.create(node, "goal_torque");
        _effectors.at(name).readForce.create(node, "read_force");
        _effectors.at(name).readTorque.create(node, "read_torque");
        _effectors.at(name).filteredForce.create(node, "filtered_force");
        _effectors.at(name).filteredTorque.create(node, "filtered_torque");
        _effectors.at(name).velForce.create(node, "vel_force");
        _effectors.at(name).velTorque.create(node, "vel_torque");
        _effectors.at(name).errorForce.create(node, "error_force");
        _effectors.at(name).errorTorque.create(node, "error_torque");
        _effectors.at(name).effortForce.create(node, "effort_force");
        _effectors.at(name).effortTorque.create(node, "effort_torque");
        _effectors.at(name).solForce.create(node, "sol_force");
        _effectors.at(name).solTorque.create(node, "sol_torque");
        _effectors.at(name).stateForce.create(node, "state_force");
        _effectors.at(name).stateTorque.create(node, "state_torque");
    }

    //SEIKO Controller initialization
    _controller = SEIKOController(_modelGoal);
    _controller.addContactPlane(_retargeting.nameFrameFootLeft());
    _controller.addContactPlane(_retargeting.nameFrameFootRight());
    _controller.addContactPoint(_retargeting.nameFrameHandLeft());
    _controller.addContactPoint(_retargeting.nameFrameHandRight());
    //Kinematics limits from SEIKO
    _controller.getConfiguration().limitPosLowerJoint = _retargeting.getJointLimitPosLower();
    _controller.getConfiguration().limitPosUpperJoint = _retargeting.getJointLimitPosUpper();
    //Controller parameters
    _controller.getConfiguration().isIntegrating = true;
    _controller.getConfiguration().weightRegBase = 1e-6;
    _controller.getConfiguration().weightElasticEnergy = 1.0;
    _controller.getConfiguration().weightRegCmdChange = 1e3*Eigen::VectorXd::Ones(_modelGoal.sizeJoint());
    _controller.getConfiguration().weightRegCmdOffset = 1e2*Eigen::VectorXd::Ones(_modelGoal.sizeJoint());
    _controller.getConfiguration().gainPose = 1.0;
    _controller.getPlane(_retargeting.nameFrameFootLeft()).weightWrench = 1e-1*Eigen::Vector6d::Ones();
    _controller.getPlane(_retargeting.nameFrameFootRight()).weightWrench = 1e-1*Eigen::Vector6d::Ones();
    _controller.getPoint(_retargeting.nameFrameHandLeft()).weightForce = 1e1*Eigen::Vector3d::Ones();
    _controller.getPoint(_retargeting.nameFrameHandRight()).weightForce = 1e1*Eigen::Vector3d::Ones();
    _controller.getPlane(_retargeting.nameFrameFootLeft()).weightPos = 2e2;
    _controller.getPlane(_retargeting.nameFrameFootLeft()).weightMat = 2e1;
    _controller.getPlane(_retargeting.nameFrameFootRight()).weightPos = 2e2;
    _controller.getPlane(_retargeting.nameFrameFootRight()).weightMat = 2e1;
    _controller.getPoint(_retargeting.nameFrameHandLeft()).weightPos = 2e1;
    _controller.getPoint(_retargeting.nameFrameHandRight()).weightPos = 2e1;
    _controller.getPoint(_retargeting.nameFrameHandLeft()).weightMat = 100.0;
    _controller.getPoint(_retargeting.nameFrameHandRight()).weightMat = 100.0;
    for (size_t i=0;i<_modelGoal.sizeJoint();i++) {
        _controller.getConfiguration().stiffnessJoint(i) = 
            StiffnessJoint.at(_modelGoal.getNameJoint(i));
    }
    
    //RhIO initialization for wrench filtering
    _rhioParamCutoffFreqComplimentary.create(rhioNode, "cutoff_freq_complimentary")
        ->persisted(true)->defaultValue(0.2);

    //RhIO initialization for controller parameters
    _rhioParamControllerEnabled.create(rhioNode, "controller_enabled")
        ->persisted(true)->defaultValue(true);
    _rhioParamGainP.create(rhioNode, "gain_p")
        ->persisted(true)->minimum(0.0)->defaultValue(0.2);
    _rhioParamGainD.create(rhioNode, "gain_d")
        ->persisted(true)->minimum(0.0)->defaultValue(0.005);
    _rhioParamMaxTauRatio.create(rhioNode, "max_tau_ratio")
        ->persisted(true)->minimum(0.0)->defaultValue(0.75);
    _rhioParamMaxPosCmdOffset.create(rhioNode, "max_pos_cmd_offset")
        ->persisted(true)->minimum(0.1)->defaultValue(0.5);
    
    //Admittance parameters
    _rhioParamAdmEnabled.create(
        rhioNode, "adm_enabled")
        ->persisted(true)->defaultValue(true);
    _rhioParamAdmPointLinDeadbband.create(
        rhioNode, "adm_point_lin_deadband")
        ->persisted(true)->defaultValue(10.0);
    _rhioParamAdmPointLinGain.create(
        rhioNode, "adm_point_lin_gain")
        ->persisted(true)->defaultValue(0.003);
    _rhioParamAdmPointLinMax.create(
        rhioNode, "adm_point_lin_max")
        ->persisted(true)->defaultValue(0.1);
    _rhioParamAdmPlaneLinDeadbband.create(
        rhioNode, "adm_plane_lin_deadband")
        ->persisted(true)->defaultValue(50.0);
    _rhioParamAdmPlaneLinGain.create(
        rhioNode, "adm_plane_lin_gain")
        ->persisted(true)->defaultValue(0.0005);
    _rhioParamAdmPlaneLinMax.create(
        rhioNode, "adm_plane_lin_max")
        ->persisted(true)->defaultValue(0.01);
    _rhioParamAdmPlaneAngDeadbband.create(
        rhioNode, "adm_plane_ang_deadband")
        ->persisted(true)->defaultValue(3.0);
    _rhioParamAdmPlaneAngGain.create(
        rhioNode, "adm_plane_ang_gain")
        ->persisted(true)->defaultValue(0.05);
    _rhioParamAdmPlaneAngMax.create(
        rhioNode, "adm_plane_ang_max")
        ->persisted(true)->defaultValue(0.1);

    //Additional monitoring Wrapper
    _rhioIsExperiment.create(
        rhioNode, "is_experiment")
        ->comment("Use for log post processing")
        ->defaultValue(false);
    _rhioDurationRetargeting.create(rhioNode, "duration_retargeting");
    _rhioDurationController.create(rhioNode, "duration_controller");

    //Create Wrappers for joints
    rhioNode.newChild("goal_model");
    rhioNode.newChild("read_model");
    rhioNode.newChild("flex_model");
    rhioNode.newChild("cmd_offset");
    rhioNode.newChild("stiffness_ratio");
    rhioNode.newChild("tau_goal_ratio");
    rhioNode.newChild("tau_read_ratio");
    rhioNode.newChild("tau_limit_ratio");
    RhIO::IONode& nodeGoalModel = rhioNode.child("goal_model");
    RhIO::IONode& nodeReadModel = rhioNode.child("read_model");
    RhIO::IONode& nodeFlexModel = rhioNode.child("flex_model");
    RhIO::IONode& nodeCmdOffset = rhioNode.child("cmd_offset");
    RhIO::IONode& nodeStiffnessRatio = rhioNode.child("stiffness_ratio");
    RhIO::IONode& nodeTauGoalRatio = rhioNode.child("tau_goal_ratio");
    RhIO::IONode& nodeTauReadRatio = rhioNode.child("tau_read_ratio");
    RhIO::IONode& nodeTauLimitRatio = rhioNode.child("tau_limit_ratio");
    for (size_t i=0;i<_modelGoal.sizeVectPos();i++) {
        _rhioGoalModel.append(nodeGoalModel, _modelGoal.getNameDOF(i));
        _rhioReadModel.append(nodeReadModel, _modelGoal.getNameDOF(i));
        _rhioFlexModel.append(nodeFlexModel, _modelGoal.getNameDOF(i));
    }
    for (size_t i=0;i<_modelGoal.sizeJoint();i++) {
        _rhioCmdOffset.append(nodeCmdOffset, _modelGoal.getNameJoint(i));
        _rhioStiffnessRatio.append(nodeStiffnessRatio, _modelGoal.getNameJoint(i));
        _rhioJointTauGoalRatio.append(nodeTauGoalRatio, _modelGoal.getNameJoint(i));
        _rhioJointTauReadRatio.append(nodeTauReadRatio, _modelGoal.getNameJoint(i));
        _rhioJointTauLimitRatio.append(nodeTauLimitRatio, _modelGoal.getNameJoint(i));
    }
    nodeGoalModel.newStr("model_path")->defaultValue(pathModel);
    nodeReadModel.newStr("model_path")->defaultValue(pathModel);
    nodeFlexModel.newStr("model_path")->defaultValue(pathModel);
}

void TaskSEIKO::update(
    double dt_task,
    double dt_ahead,
    const ControllerTalosState_t& state,
    ControllerTalosCommand_t& command)
{
    //Reset if not running
    if (!state.isRunning) {
        //Model reset
        _modelGoal.setJointPosVect(state.jointCmd);
        _modelGoal.updateState();
        _modelGoal.setBaseToMatchFramePose(
            _retargeting.nameFrameFootLeft(),
            Eigen::Vector3d(0.0, 0.0, 0.0),
            Eigen::Vector3d(0.0, 0.0, 0.0),
            Eigen::Quaterniond::Identity());
        _modelGoal.updateState();
        //SEIKO reset
        _retargeting.reset();
        //Filtering reset
        _filterVelFootLeft.reset(Eigen::Vector6d::Zero());
        _filterVelFootRight.reset(Eigen::Vector6d::Zero());
        _filterVelHandLeft.reset(Eigen::Vector3d::Zero());
        _filterVelHandRight.reset(Eigen::Vector3d::Zero());
        _filteredComplimentaryIsInit = false;
        //Controller reset
        _controller.reset();
        _controller.getConfiguration().statePosture = _modelGoal.getDOFPosVect();
        _controller.getConfiguration().cmdOffsetJoint.setZero();
        _controller.getPlane(_retargeting.nameFrameFootLeft()).stateWrench = 
            _retargeting.stateContactWrench(_retargeting.nameFrameFootLeft());
        _controller.getPlane(_retargeting.nameFrameFootRight()).stateWrench = 
            _retargeting.stateContactWrench(_retargeting.nameFrameFootRight());
        _controller.askSwitchingEnable(_retargeting.nameFrameFootLeft());
        _controller.askSwitchingEnable(_retargeting.nameFrameFootRight());
        for (int i=0;i<3;i++) {
            _controller.runInit();
        }
        //RhIO command reset
        for (auto& it : _effectors) {
            it.second.askSwitchEnable = false;
            it.second.askSwitchDisable = false;
            it.second.cmdIsPushMode = false;
            it.second.cmdVelForceNormal = 0.0;
            it.second.cmdIsClutch = false;
            for (size_t i=0;i<3;i++) {
                it.second.cmdVelLin[i] = 0.0;
                it.second.cmdVelAng[i] = 0.0;
            }
        }
        //Interpolation reset
        _filterInterpolation.reset(state.jointCmd, 2.0);
    }
    
    //Assign measured model state
    _modelRead.setJointPosVect(state.jointPos);
    _modelRead.updateState();
    if (_retargeting.stateContactBase(_retargeting.nameFrameFootLeft()).isEnabled) {
        _modelRead.setBaseToMatchFramePose(
            _retargeting.nameFrameFootLeft(),
            Eigen::Vector3d(0.0, 0.0, 0.0),
            _modelGoal.position(_retargeting.nameFrameFootLeft(), "ROOT"),
            Eigen::Quaterniond(
                _modelGoal.orientation(_retargeting.nameFrameFootLeft(), "ROOT")));
    } else {
        _modelRead.setBaseToMatchFramePose(
            _retargeting.nameFrameFootRight(),
            Eigen::Vector3d(0.0, 0.0, 0.0),
            _modelGoal.position(_retargeting.nameFrameFootRight(), "ROOT"),
            Eigen::Quaterniond(
                _modelGoal.orientation(_retargeting.nameFrameFootRight(), "ROOT")));
    }
    _modelRead.updateState();
    
    //Early exit if task is not running
    if (!state.isRunning) {
        return;
    }
    
    //Override task time step to desired one to reduce potential noise
    dt_task = 1.0/schedulingFrequency();
    
    //Manually set point contact orientation
    Eigen::Matrix3d matHandLeftSurface =
        Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitY()).toRotationMatrix();
    Eigen::Matrix3d matHandRightSurface =
        Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitX()).toRotationMatrix();
    _retargeting.setContactMat(
        _retargeting.nameFrameHandLeft(), matHandLeftSurface);
    _retargeting.setContactMat(
        _retargeting.nameFrameHandRight(), matHandRightSurface);
    _controller.getPoint(_retargeting.nameFrameHandLeft()).contactMat = 
        matHandLeftSurface;
    _controller.getPoint(_retargeting.nameFrameHandRight()).contactMat = 
        matHandRightSurface;
    Eigen::Matrix3d handLocalLeftToWorld = 
        _modelRead.orientation(_retargeting.nameFrameHandLeft(), "ROOT");
    Eigen::Matrix3d handLocalRightToWorld = 
        _modelRead.orientation(_retargeting.nameFrameHandRight(), "ROOT");
    Eigen::Matrix3d handLocalLeftToContact = 
        matHandLeftSurface.transpose() *
        handLocalLeftToWorld;
    Eigen::Matrix3d handLocalRightToContact = 
        matHandRightSurface.transpose() *
        handLocalRightToWorld;

    //Estimate right hand force from arm joint torque
    size_t tmpIndexLow = _modelRead.getIndexJoint("arm_right_1_joint");
    Eigen::VectorXd tmpGravityVector = _modelRead.computeGravityVector();
    Eigen::MatrixXd tmpJacHandRight = _modelRead.pointJacobian("right_hand_frame", "right_hand_frame");
    Eigen::MatrixXd tmpJac = tmpJacHandRight.block(3, 6+tmpIndexLow, 3, 4);
    Eigen::VectorXd tmpTau = state.jointTau.segment(tmpIndexLow, 4);
    Eigen::VectorXd tmpGrav = tmpGravityVector.segment(6+tmpIndexLow, 4);
    Eigen::Vector3d forceHandRight = (tmpJac.transpose()).fullPivHouseholderQr().solve(tmpGrav-tmpTau);
    //Update velocity filtering
    _filterVelFootLeft.timeDelta() = 0.05;
    _filterVelFootRight.timeDelta() = 0.05;
    _filterVelHandLeft.timeDelta() = 0.05;
    _filterVelHandRight.timeDelta() = 0.05;
    _filterVelFootLeft.cutoffFrequency() = 10.0;
    _filterVelFootRight.cutoffFrequency() = 10.0;
    _filterVelHandLeft.cutoffFrequency() = 10.0;
    _filterVelHandRight.cutoffFrequency() = 10.0;
    _filterVelFootLeft.update(state.wrenchFootLeft, dt_task);
    _filterVelFootRight.update(state.wrenchFootRight, dt_task);
    _filterVelHandLeft.update(handLocalLeftToContact*state.wrenchHandLeft.segment(3,3), dt_task);
    _filterVelHandRight.update(handLocalRightToContact*forceHandRight, dt_task);
    
    //Send effectors command to SEIKO Retargeting
    for (auto& it : _effectors) {
        Eigen::Vector3d cmdRawPos;
        Eigen::Quaterniond cmdRawQuat;
        Eigen::Vector3d cmdVelLin;
        Eigen::Vector3d cmdVelAng;
        for (size_t i=0;i<3;i++) {
            cmdRawPos(i) = it.second.cmdRawPos[i];
            cmdVelLin(i) = it.second.cmdVelLin[i];
            cmdVelAng(i) = it.second.cmdVelAng[i];
        }
        cmdRawQuat.x() = it.second.cmdRawQuat[0];
        cmdRawQuat.y() = it.second.cmdRawQuat[1];
        cmdRawQuat.z() = it.second.cmdRawQuat[2];
        cmdRawQuat.w() = it.second.cmdRawQuat[3];
        //Velocity admittance on effectors.
        //Hands admittance
        if (
            it.second.isPoint && 
            !_retargeting.stateContactBase(it.first).isEnabled
        ) {
            Eigen::Vector3d forceInLocal = Eigen::Vector3d::Zero();
            if (it.first == "left_hand_frame") {
                forceInLocal = state.wrenchHandLeft.segment(3,3);
            }
            if (it.first == "right_hand_frame") {
                forceInLocal = forceHandRight;
            }
            Eigen::Vector3d forceInWorld = DeadbandVectorNorm<Eigen::Vector3d>(
                _modelRead.orientation(it.first, "ROOT")*forceInLocal, 
                _rhioParamAdmPointLinDeadbband, true);
            Eigen::Vector3d admittanceVelLin = ClampVectorNorm(
                _rhioParamAdmPointLinGain*forceInWorld,
                _rhioParamAdmPointLinMax);
            if (_rhioParamAdmEnabled) {
                cmdVelLin += admittanceVelLin;
            }
            it.second.admVelLin = admittanceVelLin;
        } else if (it.second.isPoint) {
            it.second.admVelLin = Eigen::Vector3d::Zero();
        }
        //Feet admittance
        if (
            !it.second.isPoint && 
            !_retargeting.stateContactBase(it.first).isEnabled
        ) {
            Eigen::Vector6d wrenchInLocal = Eigen::Vector6d::Zero();
            if (it.first == "left_sole_link") {
                wrenchInLocal = state.wrenchFootLeft;
            }
            if (it.first == "right_sole_link") {
                wrenchInLocal = state.wrenchFootRight;
            }
            Eigen::Vector6d wrenchInWorld;
            wrenchInWorld.segment(0,3) = DeadbandVectorNorm<Eigen::Vector3d>(
                _modelRead.orientation(it.first, "ROOT")*wrenchInLocal.segment(0,3),
                _rhioParamAdmPlaneAngDeadbband, true);
            wrenchInWorld.segment(3,3) = DeadbandVectorNorm<Eigen::Vector3d>(
                _modelRead.orientation(it.first, "ROOT")*wrenchInLocal.segment(3,3),
                _rhioParamAdmPlaneLinDeadbband, true);
            Eigen::Vector3d admittanceVelLin = ClampVectorNorm(
                _rhioParamAdmPlaneLinGain*wrenchInWorld.segment(3,3),
                _rhioParamAdmPlaneLinMax);
            Eigen::Vector3d admittanceVelAng = ClampVectorNorm(
                _rhioParamAdmPlaneAngGain*wrenchInWorld.segment(0,3),
                _rhioParamAdmPlaneAngMax);
            if (_rhioParamAdmEnabled) {
                cmdVelLin += admittanceVelLin;
                cmdVelAng += admittanceVelAng;
            }
            it.second.admVelLin = admittanceVelLin;
            it.second.admVelAng = admittanceVelAng;
        } else if (!it.second.isPoint) {
            it.second.admVelLin = Eigen::Vector3d::Zero();
            it.second.admVelAng = Eigen::Vector3d::Zero();
        }
        //Push commands
        if (_retargeting.stateContactBase(it.first).isEnabled) {
            _retargeting.setPushMode(
                it.first, it.second.cmdIsPushMode);
            _retargeting.setForceNormalVel(
                it.first, it.second.cmdVelForceNormal);
        } else {
            it.second.cmdIsPushMode = false;
            it.second.cmdVelForceNormal = 0.0;
        }
        //Apply commands to SEIKO
        _retargeting.setTargetPose(it.first, 
            it.second.cmdIsClutch,
            cmdRawPos,
            cmdRawQuat.toRotationMatrix());
        _retargeting.setTargetVel(it.first, 
            cmdVelLin, 
            cmdVelAng); 
        if (it.second.askSwitchEnable) {
            it.second.askSwitchEnable = false;
            _retargeting.askSwitching(it.first, true);
        }
        if (it.second.askSwitchDisable) {
            it.second.askSwitchDisable = false;
            _retargeting.askSwitching(it.first, false);
        }
    }
    
    std::chrono::time_point<std::chrono::steady_clock> timeStartRetargeting = 
        std::chrono::steady_clock::now();
    //Compute SEIKO Retargeting update
    bool isSuccess = _retargeting.update(dt_task, dt_ahead);
    if (!isSuccess) {
        std::map<std::string, double> containerRatios = 
            _retargeting.computeConstraintRatios();
        std::stringstream ss;
        for (const auto& it : containerRatios) {
            ss << it.first << " " << it.second << std::endl;
        }
        throw std::runtime_error(
            "inria::TaskSEIKO: SEIKO QP failed: \n" + ss.str());
    }
    std::chrono::time_point<std::chrono::steady_clock> timeStopRetargeting = 
        std::chrono::steady_clock::now();

    std::chrono::time_point<std::chrono::steady_clock> timeStartController = 
        std::chrono::steady_clock::now();
    //Retrieve state
    Eigen::Vector6d goalWrenchLeft = _retargeting.stateContactWrench(_retargeting.nameFrameFootLeft());
    Eigen::Vector6d goalWrenchRight = _retargeting.stateContactWrench(_retargeting.nameFrameFootRight());
    Eigen::Vector3d goalForceLeft = _retargeting.stateContactForce(_retargeting.nameFrameHandLeft());
    Eigen::Vector3d goalForceRight = _retargeting.stateContactForce(_retargeting.nameFrameHandRight());
    Eigen::Vector6d readWrenchLeft = state.wrenchFootLeft;
    Eigen::Vector6d readWrenchRight = state.wrenchFootRight;
    Eigen::Vector3d readForceLeft = 
        handLocalLeftToContact*state.wrenchHandLeft.segment(3,3);
    Eigen::Vector3d readForceRight = 
        handLocalRightToContact*forceHandRight;
    //Update wrench complimentary filtering
    if (!_filteredComplimentaryIsInit) {
        _filteredComplimentaryIsInit = true;
        _lastDeltaWrenchLeft.setZero();
        _lastDeltaWrenchRight.setZero();
        _lastDeltaForceLeft.setZero();
        _lastDeltaForceRight.setZero();
        _filteredWrenchLeft = readWrenchLeft;
        _filteredWrenchRight = readWrenchRight;
        _filteredForceLeft = readForceLeft;
        _filteredForceRight = readForceRight;
    }
    double alphaComplimentary = FilterExponential<double>::getAlphaFromFreq(
        _rhioParamCutoffFreqComplimentary, dt_task);
    if (
        _retargeting.stateContactBase(_retargeting.nameFrameFootLeft()).isEnabled && 
        _controller.getConfiguration().isIntegrating
    ) {
        _filteredWrenchLeft += _lastDeltaWrenchLeft;
        _filteredWrenchLeft = alphaComplimentary*_filteredWrenchLeft + (1.0-alphaComplimentary)*readWrenchLeft;
    } else {
        _filteredWrenchLeft = readWrenchLeft;
    }
    if (
        _retargeting.stateContactBase(_retargeting.nameFrameFootRight()).isEnabled && 
        _controller.getConfiguration().isIntegrating
    ) {
        _filteredWrenchRight += _lastDeltaWrenchRight;
        _filteredWrenchRight = alphaComplimentary*_filteredWrenchRight + (1.0-alphaComplimentary)*readWrenchRight;
    } else {
        _filteredWrenchRight = readWrenchRight;
    }
    if (
        _retargeting.stateContactBase(_retargeting.nameFrameHandLeft()).isEnabled && 
        _controller.getConfiguration().isIntegrating
    ) {
        _filteredForceLeft += _lastDeltaForceLeft;
        _filteredForceLeft = alphaComplimentary*_filteredForceLeft + (1.0-alphaComplimentary)*readForceLeft;
    } else {
        _filteredForceLeft = readForceLeft;
    }
    if (
        _retargeting.stateContactBase(_retargeting.nameFrameHandRight()).isEnabled && 
        _controller.getConfiguration().isIntegrating
    ) {
        _filteredForceRight += _lastDeltaForceRight;
        _filteredForceRight = alphaComplimentary*_filteredForceRight + (1.0-alphaComplimentary)*readForceRight;
    } else {
        _filteredForceRight = readForceRight;
    }
    //Compute wrench errors
    Eigen::Vector6d errorWrenchLeft = goalWrenchLeft - _filteredWrenchLeft;
    Eigen::Vector6d errorWrenchRight = goalWrenchRight - _filteredWrenchRight;
    Eigen::Vector3d errorForceLeft = goalForceLeft - _filteredForceLeft;
    Eigen::Vector3d errorForceRight = goalForceRight - _filteredForceRight;
    //Controller contact switching
    bool isSwitching = false;
    if (
        !_retargeting.stateContactBase(_retargeting.nameFrameFootLeft()).isEnabled &&
        _controller.getPlane(_retargeting.nameFrameFootLeft()).isEnabled 
    ) {
        _controller.getPlane(_retargeting.nameFrameFootLeft()).isEnabled = false;
        isSwitching = true;
    }
    if (
        !_retargeting.stateContactBase(_retargeting.nameFrameFootRight()).isEnabled &&
        _controller.getPlane(_retargeting.nameFrameFootRight()).isEnabled 
    ) {
        _controller.getPlane(_retargeting.nameFrameFootRight()).isEnabled = false;
        isSwitching = true;
    }
    if (
        !_retargeting.stateContactBase(_retargeting.nameFrameHandLeft()).isEnabled &&
        _controller.getPoint(_retargeting.nameFrameHandLeft()).isEnabled 
    ) {
        _controller.getPoint(_retargeting.nameFrameHandLeft()).isEnabled = false;
        isSwitching = true;
    }
    if (
        !_retargeting.stateContactBase(_retargeting.nameFrameHandRight()).isEnabled &&
        _controller.getPoint(_retargeting.nameFrameHandRight()).isEnabled 
    ) {
        _controller.getPoint(_retargeting.nameFrameHandRight()).isEnabled = false;
        isSwitching = true;
    }
    if (
        _retargeting.stateContactBase(_retargeting.nameFrameFootLeft()).isEnabled &&
        !_controller.getPlane(_retargeting.nameFrameFootLeft()).isEnabled 
    ) {
        _controller.askSwitchingEnable(_retargeting.nameFrameFootLeft());
        isSwitching = true;
    }
    if (
        _retargeting.stateContactBase(_retargeting.nameFrameFootRight()).isEnabled &&
        !_controller.getPlane(_retargeting.nameFrameFootRight()).isEnabled 
    ) {
        _controller.askSwitchingEnable(_retargeting.nameFrameFootRight());
        isSwitching = true;
    }
    if (
        _retargeting.stateContactBase(_retargeting.nameFrameHandLeft()).isEnabled &&
        !_controller.getPoint(_retargeting.nameFrameHandLeft()).isEnabled 
    ) {
        _controller.askSwitchingEnable(_retargeting.nameFrameHandLeft());
        isSwitching = true;
    }
    if (
        _retargeting.stateContactBase(_retargeting.nameFrameHandRight()).isEnabled &&
        !_controller.getPoint(_retargeting.nameFrameHandRight()).isEnabled 
    ) {
        _controller.askSwitchingEnable(_retargeting.nameFrameHandRight());
        isSwitching = true;
    }
    if (isSwitching) {
        for (int i=0;i<3;i++) {
            _controller.runInit();
        }
        _filterInterpolation.reset(_filterInterpolation.value(), 2.0);
    }

    //Assign control effort to flexibility controller
    _controller.getPlane(_retargeting.nameFrameFootLeft()).targetDeltaWrench = 
        _retargeting.solContactWrench(_retargeting.nameFrameFootLeft()) +
        _rhioParamGainP*errorWrenchLeft + 
        _rhioParamGainD*_filterVelFootLeft.value();
    _controller.getPlane(_retargeting.nameFrameFootRight()).targetDeltaWrench = 
        _retargeting.solContactWrench(_retargeting.nameFrameFootRight()) +
        _rhioParamGainP*errorWrenchRight + 
        _rhioParamGainD*_filterVelFootRight.value();
    _controller.getPoint(_retargeting.nameFrameHandLeft()).targetDeltaForce = 
        _retargeting.solContactForce(_retargeting.nameFrameHandLeft()) +
        _rhioParamGainP*errorForceLeft + 
        _rhioParamGainD*_filterVelHandLeft.value();
    _controller.getPoint(_retargeting.nameFrameHandRight()).targetDeltaForce = 
        _retargeting.solContactForce(_retargeting.nameFrameHandRight()) +
        _rhioParamGainP*errorForceRight + 
        _rhioParamGainD*_filterVelHandRight.value();
    //Set controller enable parameter and
    //trigger interpolation filtering when the controller is enabled
    if (_rhioParamControllerEnabled && !_controller.getConfiguration().isIntegrating) {
        _filterInterpolation.reset(_filterInterpolation.value(), 2.0);
    }
    _controller.getConfiguration().isIntegrating = _rhioParamControllerEnabled;
    //Dynamically update controller's joint torque limit 
    //using hysteresis scheme to prevent increasing state joint torque 
    //when a measured torque is reached
    const double limitRatioHight = _rhioParamMaxTauRatio;
    const double limitRatioLow = limitRatioHight - 0.1;
    const double marginTorque = 1.0;
    const double decayVel = 10.0;
    Eigen::VectorXd readTauRatio = 
        state.jointTau.array()/_modelGoal.jointLimitsTorque().array();
    for (size_t i=0;i<_modelGoal.sizeJoint();i++) {
        if (
            std::fabs(readTauRatio(i)) > limitRatioHight && 
            std::fabs(_controller.getConfiguration().torqueJoint(i))+marginTorque < 
                _controller.getConfiguration().limitTauJoint(i)
        ) {
            _controller.getConfiguration().limitTauJoint(i) = 
                std::fabs(_controller.getConfiguration().torqueJoint(i))+marginTorque;
        } else if (
            std::fabs(readTauRatio(i)) < limitRatioLow && 
            _controller.getConfiguration().limitTauJoint(i) < _modelGoal.jointLimitsTorque()(i)
        ) {
            _controller.getConfiguration().limitTauJoint(i) += dt_task*decayVel;
        }
    }

    //Set parameters
    _controller.getConfiguration().limitPosCmdOffset = _rhioParamMaxPosCmdOffset;
    //Run Controller optimization
    _controller.runSEIKO(dt_task);
    //Retrieve wrench delta for complimentary filter
    _lastDeltaWrenchLeft = dt_task*_controller.getPlane(_retargeting.nameFrameFootLeft()).solutionWrench;
    _lastDeltaWrenchRight = dt_task*_controller.getPlane(_retargeting.nameFrameFootRight()).solutionWrench;
    _lastDeltaForceLeft = dt_task*_controller.getPoint(_retargeting.nameFrameHandLeft()).solutionForce;
    _lastDeltaForceRight = dt_task*_controller.getPoint(_retargeting.nameFrameHandRight()).solutionForce;
    //Keep computing flexibility state when the controller is not enabled
    if (!_controller.getConfiguration().isIntegrating) {
        _controller.runInit();
    }
    std::chrono::time_point<std::chrono::steady_clock> timeStopController = 
        std::chrono::steady_clock::now();
    
    //Compute timings
    std::chrono::duration<double> durationRetargeting = 
        timeStopRetargeting - timeStartRetargeting;
    std::chrono::duration<double> durationController = 
        timeStopController - timeStartController;

    //Monitoring
    _rhioIsExperiment = _rhioIsExperiment.get();
    _rhioDurationRetargeting = durationRetargeting.count();
    _rhioDurationController = durationController.count();

    _effectors.at(_retargeting.nameFrameFootLeft()).isEnabled = _retargeting.stateContactBase(_retargeting.nameFrameFootLeft()).isEnabled;
    _effectors.at(_retargeting.nameFrameFootLeft()).targetPos = _retargeting.refTargetPos(_retargeting.nameFrameFootLeft());
    _effectors.at(_retargeting.nameFrameFootLeft()).goalPos = _controller.getPlane(_retargeting.nameFrameFootLeft()).targetPos;
    _effectors.at(_retargeting.nameFrameFootLeft()).contactQuat = Eigen::Quaterniond(_modelGoal.orientation(_retargeting.nameFrameFootLeft(), "ROOT"));
    _effectors.at(_retargeting.nameFrameFootLeft()).deltaTorque = dt_task*_retargeting.solContactWrench(_retargeting.nameFrameFootLeft()).segment(0,3);
    _effectors.at(_retargeting.nameFrameFootLeft()).deltaForce = dt_task*_retargeting.solContactWrench(_retargeting.nameFrameFootLeft()).segment(3,3);
    _effectors.at(_retargeting.nameFrameFootLeft()).goalTorque = goalWrenchLeft.segment(0,3);
    _effectors.at(_retargeting.nameFrameFootLeft()).goalForce = goalWrenchLeft.segment(3,3);
    _effectors.at(_retargeting.nameFrameFootLeft()).readTorque = readWrenchLeft.segment(0,3);
    _effectors.at(_retargeting.nameFrameFootLeft()).readForce = readWrenchLeft.segment(3,3);
    _effectors.at(_retargeting.nameFrameFootLeft()).filteredTorque = _filteredWrenchLeft.segment(0,3);
    _effectors.at(_retargeting.nameFrameFootLeft()).filteredForce = _filteredWrenchLeft.segment(3,3);
    _effectors.at(_retargeting.nameFrameFootLeft()).velTorque = _filterVelFootLeft.value().segment(0,3);
    _effectors.at(_retargeting.nameFrameFootLeft()).velForce = _filterVelFootLeft.value().segment(3,3);
    _effectors.at(_retargeting.nameFrameFootLeft()).errorTorque = errorWrenchLeft.segment(0,3);
    _effectors.at(_retargeting.nameFrameFootLeft()).errorForce = errorWrenchLeft.segment(3,3);
    _effectors.at(_retargeting.nameFrameFootLeft()).effortTorque = _controller.getPlane(_retargeting.nameFrameFootLeft()).targetDeltaWrench.segment(0,3);
    _effectors.at(_retargeting.nameFrameFootLeft()).effortForce = _controller.getPlane(_retargeting.nameFrameFootLeft()).targetDeltaWrench.segment(3,3);
    _effectors.at(_retargeting.nameFrameFootLeft()).solTorque = _controller.getPlane(_retargeting.nameFrameFootLeft()).solutionWrench.segment(0,3);
    _effectors.at(_retargeting.nameFrameFootLeft()).solForce = _controller.getPlane(_retargeting.nameFrameFootLeft()).solutionWrench.segment(3,3);
    _effectors.at(_retargeting.nameFrameFootLeft()).stateTorque = _controller.getPlane(_retargeting.nameFrameFootLeft()).stateWrench.segment(0,3);
    _effectors.at(_retargeting.nameFrameFootLeft()).stateForce = _controller.getPlane(_retargeting.nameFrameFootLeft()).stateWrench.segment(3,3);
    _effectors.at(_retargeting.nameFrameFootRight()).isEnabled = _retargeting.stateContactBase(_retargeting.nameFrameFootRight()).isEnabled;
    _effectors.at(_retargeting.nameFrameFootRight()).targetPos = _retargeting.refTargetPos(_retargeting.nameFrameFootRight());
    _effectors.at(_retargeting.nameFrameFootRight()).goalPos = _controller.getPlane(_retargeting.nameFrameFootRight()).targetPos;
    _effectors.at(_retargeting.nameFrameFootRight()).contactQuat = Eigen::Quaterniond(_modelGoal.orientation(_retargeting.nameFrameFootRight(), "ROOT"));
    _effectors.at(_retargeting.nameFrameFootRight()).deltaTorque = dt_task*_retargeting.solContactWrench(_retargeting.nameFrameFootRight()).segment(0,3);
    _effectors.at(_retargeting.nameFrameFootRight()).deltaForce = dt_task*_retargeting.solContactWrench(_retargeting.nameFrameFootRight()).segment(3,3);
    _effectors.at(_retargeting.nameFrameFootRight()).goalTorque = goalWrenchRight.segment(0,3);
    _effectors.at(_retargeting.nameFrameFootRight()).goalForce = goalWrenchRight.segment(3,3);
    _effectors.at(_retargeting.nameFrameFootRight()).readTorque = readWrenchRight.segment(0,3);
    _effectors.at(_retargeting.nameFrameFootRight()).readForce = readWrenchRight.segment(3,3);
    _effectors.at(_retargeting.nameFrameFootRight()).filteredTorque = _filteredWrenchRight.segment(0,3);
    _effectors.at(_retargeting.nameFrameFootRight()).filteredForce = _filteredWrenchRight.segment(3,3);
    _effectors.at(_retargeting.nameFrameFootRight()).velTorque = _filterVelFootRight.value().segment(0,3);
    _effectors.at(_retargeting.nameFrameFootRight()).velForce = _filterVelFootRight.value().segment(3,3);
    _effectors.at(_retargeting.nameFrameFootRight()).errorTorque = errorWrenchRight.segment(0,3);
    _effectors.at(_retargeting.nameFrameFootRight()).errorForce = errorWrenchRight.segment(3,3);
    _effectors.at(_retargeting.nameFrameFootRight()).effortTorque = _controller.getPlane(_retargeting.nameFrameFootRight()).targetDeltaWrench.segment(0,3);
    _effectors.at(_retargeting.nameFrameFootRight()).effortForce = _controller.getPlane(_retargeting.nameFrameFootRight()).targetDeltaWrench.segment(3,3);
    _effectors.at(_retargeting.nameFrameFootRight()).solTorque = _controller.getPlane(_retargeting.nameFrameFootRight()).solutionWrench.segment(0,3);
    _effectors.at(_retargeting.nameFrameFootRight()).solForce = _controller.getPlane(_retargeting.nameFrameFootRight()).solutionWrench.segment(3,3);
    _effectors.at(_retargeting.nameFrameFootRight()).stateTorque = _controller.getPlane(_retargeting.nameFrameFootRight()).stateWrench.segment(0,3);
    _effectors.at(_retargeting.nameFrameFootRight()).stateForce = _controller.getPlane(_retargeting.nameFrameFootRight()).stateWrench.segment(3,3);
    _effectors.at(_retargeting.nameFrameHandLeft()).isEnabled = _retargeting.stateContactBase(_retargeting.nameFrameHandLeft()).isEnabled;
    _effectors.at(_retargeting.nameFrameHandLeft()).targetPos = _retargeting.refTargetPos(_retargeting.nameFrameHandLeft());
    _effectors.at(_retargeting.nameFrameHandLeft()).goalPos = _controller.getPoint(_retargeting.nameFrameHandLeft()).targetPos;
    _effectors.at(_retargeting.nameFrameHandLeft()).contactQuat = Eigen::Quaterniond(_controller.getPoint(_retargeting.nameFrameHandLeft()).contactMat);
    _effectors.at(_retargeting.nameFrameHandLeft()).deltaForce = dt_task*_retargeting.solContactForce(_retargeting.nameFrameHandLeft());
    _effectors.at(_retargeting.nameFrameHandLeft()).goalForce = goalForceLeft;
    _effectors.at(_retargeting.nameFrameHandLeft()).readForce = readForceLeft;
    _effectors.at(_retargeting.nameFrameHandLeft()).filteredForce = _filteredForceLeft;
    _effectors.at(_retargeting.nameFrameHandLeft()).velForce = _filterVelHandLeft.value();
    _effectors.at(_retargeting.nameFrameHandLeft()).errorForce = errorForceLeft;
    _effectors.at(_retargeting.nameFrameHandLeft()).effortForce = _controller.getPoint(_retargeting.nameFrameHandLeft()).targetDeltaForce;
    _effectors.at(_retargeting.nameFrameHandLeft()).solForce = _controller.getPoint(_retargeting.nameFrameHandLeft()).solutionForce;
    _effectors.at(_retargeting.nameFrameHandLeft()).stateForce = _controller.getPoint(_retargeting.nameFrameHandLeft()).stateForce;
    _effectors.at(_retargeting.nameFrameHandRight()).isEnabled = _retargeting.stateContactBase(_retargeting.nameFrameHandRight()).isEnabled;
    _effectors.at(_retargeting.nameFrameHandRight()).targetPos = _retargeting.refTargetPos(_retargeting.nameFrameHandRight());
    _effectors.at(_retargeting.nameFrameHandRight()).goalPos = _controller.getPoint(_retargeting.nameFrameHandRight()).targetPos;
    _effectors.at(_retargeting.nameFrameHandRight()).contactQuat = Eigen::Quaterniond(_controller.getPoint(_retargeting.nameFrameHandRight()).contactMat);
    _effectors.at(_retargeting.nameFrameHandRight()).deltaForce = dt_task*_retargeting.solContactForce(_retargeting.nameFrameHandRight());
    _effectors.at(_retargeting.nameFrameHandRight()).goalForce = goalForceRight;
    _effectors.at(_retargeting.nameFrameHandRight()).readForce = readForceRight;
    _effectors.at(_retargeting.nameFrameHandRight()).filteredForce = _filteredForceRight;
    _effectors.at(_retargeting.nameFrameHandRight()).velForce = _filterVelHandRight.value();
    _effectors.at(_retargeting.nameFrameHandRight()).errorForce = errorForceRight;
    _effectors.at(_retargeting.nameFrameHandRight()).effortForce = _controller.getPoint(_retargeting.nameFrameHandRight()).targetDeltaForce;
    _effectors.at(_retargeting.nameFrameHandRight()).solForce = _controller.getPoint(_retargeting.nameFrameHandRight()).solutionForce;
    _effectors.at(_retargeting.nameFrameHandRight()).stateForce = _controller.getPoint(_retargeting.nameFrameHandRight()).stateForce;
    _rhioGoalModel = _modelGoal.getDOFPosVect();
    _rhioReadModel = _modelRead.getDOFPosVect();
    _rhioFlexModel = _controller.getConfiguration().statePosture;
    _rhioCmdOffset = _controller.getConfiguration().cmdOffsetJoint;
    _rhioStiffnessRatio = 
        _controller.getConfiguration().stateStiffnessJoint.array()
        / _controller.getConfiguration().stiffnessJoint.array();
    _rhioJointTauGoalRatio = 
        (_retargeting.stateJointTorque().array()
        / _modelGoal.jointLimitsTorque().array()).matrix();
    _rhioJointTauReadRatio = readTauRatio;
    _rhioJointTauLimitRatio = 
        (_controller.getConfiguration().limitTauJoint.array()
        / _modelGoal.jointLimitsTorque().array()).matrix();

    //Send commands
    _filterInterpolation.update(
        _controller.getConfiguration().cmdOffsetJoint +
            _retargeting.getAheadJointPos(), 
        dt_task);
    command.jointCmd = _filterInterpolation.value();
    command.jointIsUsed.setOnes();
}

}

