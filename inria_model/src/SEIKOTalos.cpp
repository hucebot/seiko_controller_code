#include <inria_model/SEIKOTalos.hpp>
#include <inria_model/TalosDOFs.h>

namespace inria {

void SEIKOTalos::setup()
{
    //Frame names
    _nameFootLeft = "left_sole_link";
    _nameFootRight = "right_sole_link";
    _nameHandLeft = "left_hand_frame";
    _nameHandRight = "right_hand_frame";
    
    //Setup contacts
    this->addContact(_nameFootLeft, false);
    this->addContact(_nameFootRight, false);
    this->addContact(_nameHandLeft, true);
    this->addContact(_nameHandRight, true);

    //Call to base setup
    this->SEIKOWrapper::setup();
    
    //Specific configuration for joint angular limit margin
    _jointPosMarginCoef.setOnes();
    if (this->_model->getMappingDOFs().count("head_1_joint") != 0) {
        _jointPosMarginCoef(this->_model->getIndexJoint("head_1_joint")) = 0.1;
    }
    if (this->_model->getMappingDOFs().count("head_2_joint") != 0) {
        _jointPosMarginCoef(this->_model->getIndexJoint("head_2_joint")) = 0.1;
    }
    if (this->_model->getMappingDOFs().count("torso_2_joint") != 0) {
        _jointPosMarginCoef(this->_model->getIndexJoint("torso_2_joint")) = 0.7;
    }
}
        
bool SEIKOTalos::reset()
{
    //Assign SEIKO and filters configuration
    
    //Contact wrench and force weight vectors
    //normalization. Put less (regularization) weight 
    //on normal forces.
    _vectWeightWrench(0) = 1.0;
    _vectWeightWrench(1) = 1.0;
    _vectWeightWrench(2) = 1.0;
    _vectWeightWrench(3) = 1.0;
    _vectWeightWrench(4) = 1.0;
    _vectWeightWrench(5) = 0.01;
    _vectWeightForce(0) = 1.0;
    _vectWeightForce(1) = 1.0;
    _vectWeightForce(2) = 0.01;
    
    //Centroidal momentum regularization
    this->setMomentumWeight(
        1e-6*Eigen::Vector3d::Ones(), 1e-6*Eigen::Vector3d::Ones());
    
    //Joint parameters
    this->setJointLimitPos(
        this->_model->jointLimitsLower() + jointPosMargin*_jointPosMarginCoef,
        this->_model->jointLimitsUpper() - jointPosMargin*_jointPosMarginCoef);
    this->setJointLimitVelAbs(
        jointVelLimit*this->_model->jointLimitsVelocity());
    this->setJointLimitTauAbs(
        jointTauLimitRatio*this->_model->jointLimitsTorque());
    this->setJointWeightPos(
        jointWeightPos*Eigen::VectorXd::Ones(this->_sizeJoint));
    this->setJointWeightVel(
        jointWeightVel*Eigen::VectorXd::Ones(this->_sizeJoint));
    this->setJointWeightTau(
        jointWeightTauOthers*Eigen::VectorXd::Ones(this->_sizeJoint));
    this->setJointClampPos(
        jointClampPos);

    //Set joint tau regularization weight
    Eigen::VectorXd vectJointWeightTau = jointWeightTauOthers*Eigen::VectorXd::Ones(this->_sizeJoint);
    vectJointWeightTau(this->_model->getIndexJoint("arm_left_1_joint")) = jointWeightTauArms;
    vectJointWeightTau(this->_model->getIndexJoint("arm_left_2_joint")) = jointWeightTauArms;
    vectJointWeightTau(this->_model->getIndexJoint("arm_left_3_joint")) = jointWeightTauArms;
    vectJointWeightTau(this->_model->getIndexJoint("arm_left_4_joint")) = jointWeightTauArms;
    vectJointWeightTau(this->_model->getIndexJoint("arm_left_5_joint")) = jointWeightTauArms;
    vectJointWeightTau(this->_model->getIndexJoint("arm_left_6_joint")) = jointWeightTauArms;
    vectJointWeightTau(this->_model->getIndexJoint("arm_left_7_joint")) = jointWeightTauArms;
    vectJointWeightTau(this->_model->getIndexJoint("arm_right_1_joint")) = jointWeightTauArms;
    vectJointWeightTau(this->_model->getIndexJoint("arm_right_2_joint")) = jointWeightTauArms;
    vectJointWeightTau(this->_model->getIndexJoint("arm_right_3_joint")) = jointWeightTauArms;
    vectJointWeightTau(this->_model->getIndexJoint("arm_right_4_joint")) = jointWeightTauArms;
    this->setJointWeightTau(vectJointWeightTau);

    //Set inequality weights
    this->setWeightIneqPos(weightIneqPos);
    this->setWeightIneqVel(weightIneqVel);
    this->setWeightIneqForce(weightIneqForce);
    this->setWeightIneqID(weightIneqID);

    //Cartesian pose parameters
    this->setWeightPose(_nameFootLeft,
        weightPosFoot, weightMatFoot);
    this->setWeightPose(_nameFootRight,
        weightPosFoot, weightMatFoot);
    this->setWeightPose(_nameHandLeft,
        weightPosHand, weightMatHand);
    this->setWeightPose(_nameHandRight,
        weightPosHand, weightMatHand);
    this->setClampPose(_nameFootLeft,
        clampPos, clampMat);
    this->setClampPose(_nameFootRight,
        clampPos, clampMat);
    this->setClampPose(_nameHandLeft,
        clampPos, clampMat);
    this->setClampPose(_nameHandRight,
        clampPos, clampMat);
    this->setContactLimitFriction(_nameFootLeft,
        frictionCoef);
    this->setContactLimitFriction(_nameFootRight,
        frictionCoef);
    this->setContactLimitFriction(_nameHandLeft,
        frictionCoef);
    this->setContactLimitFriction(_nameHandRight,
        frictionCoef);
    this->setNormalForceLimits(_nameFootLeft,
        forceMinFoot, forceMaxFoot);
    this->setNormalForceLimits(_nameFootRight,
        forceMinFoot, forceMaxFoot);
    this->setNormalForceLimits(_nameHandLeft,
        forceMinHand, forceMaxHand);
    this->setNormalForceLimits(_nameHandRight,
        forceMinHand, forceMaxHand);
    this->setLimitVelTorque(_nameFootLeft,
        limitVelTorqueFoot);
    this->setLimitVelTorque(_nameFootRight,
        limitVelTorqueFoot);
    this->setLimitVelForce(_nameFootLeft,
        limitVelForceFoot);
    this->setLimitVelForce(_nameFootRight,
        limitVelForceFoot);
    this->setLimitVelForce(_nameHandLeft,
        limitVelForceHand);
    this->setLimitVelForce(_nameHandRight,
        limitVelForceHand);

    //Plane contact parameters
    this->setContactPlaneLimitCOP(_nameFootLeft,
        limitCOPX, limitCOPY);
    this->setContactPlaneLimitCOP(_nameFootRight,
        limitCOPX, limitCOPY);
    this->setWeightWrench(_nameFootLeft,
        weightWrenchEnabled*_vectWeightWrench);
    this->setWeightWrench(_nameFootRight,
        weightWrenchEnabled*_vectWeightWrench);

    //Point contact parameters
    this->setWeightForce(_nameHandLeft,
        weightForceEnabled*_vectWeightForce);
    this->setWeightForce(_nameHandRight,
        weightForceEnabled*_vectWeightForce);

    //Contact command structure parameters
    for (auto& it : _commands) {
        if (it.second.isPoint) {
            it.second.limitNormalForceMin = forceMinHand;
            it.second.limitNormalForceMax = forceMaxHand;
            it.second.weightForceEnabled = weightForceEnabled;
            it.second.weightForceDisabling = weightForceDisabling;
        } else {
            it.second.limitNormalForceMin = forceMinFoot;
            it.second.limitNormalForceMax = forceMaxFoot;
            it.second.weightForceEnabled = weightWrenchEnabled;
            it.second.weightForceDisabling = weightWrenchDisabling;
        }
        it.second.weightForcePushMode = weightForcePushMode;
        //Configure command pose filters
        it.second.filterPose.setParameters(
            useRawVRPose,
            isLocalFrame,
            scalingLin,
            clampRadiusLin, clampRadiusAng,
            cutoffFreq,
            maxVelLin, maxAccLin,
            maxVelAng, maxAccAng);
        //Configure command force filter
        it.second.filterForce.setParameters(
            clampRadiusForce,
            cutoffFreq,
            maxVelForce, maxAccForce);
    }
    
    //Reset default contact state
    this->toggleContact(_nameFootLeft, true);
    this->toggleContact(_nameFootRight, true);
    this->toggleContact(_nameHandLeft, false);
    this->toggleContact(_nameHandRight, false);

    //Call base reset
    return this->SEIKOWrapper::reset();
}
        
const std::string& SEIKOTalos::nameFrameFootLeft() const
{
    return _nameFootLeft;
}
const std::string& SEIKOTalos::nameFrameFootRight() const
{
    return _nameFootRight;
}
const std::string& SEIKOTalos::nameFrameHandLeft() const
{
    return _nameHandLeft;
}
const std::string& SEIKOTalos::nameFrameHandRight() const
{
    return _nameHandRight;
}

}

