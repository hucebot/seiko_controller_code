#include <stdexcept>
#include <inria_model/SEIKORetargeting.hpp>
#include <eiquadprog/eiquadprog.hpp>
#include <inria_maths/AxisAngle.h>
#include <inria_maths/Clamp.h>
#include <inria_maths/IntegrateDOFVect.h>

namespace inria {

SEIKORetargeting::SEIKORetargeting() :
    _model(nullptr),
    _sizeDOF(0),
    _sizeJoint(0),
    _sizePlaneOn(0),
    _sizePointOn(0),
    _sizePlaneOff(0),
    _sizePointOff(0),
    _contactsPlane(),
    _contactsPoint(),
    _mappingContacts(),
    _isDisabledConstraints(),
    _jointLimitPosLower(),
    _jointLimitPosUpper(),
    _jointLimitVelAbs(),
    _jointLimitTauAbs(),
    _momentumLinWeight(),
    _momentumAngWeight(),
    _jointTargetPos(),
    _jointWeightPos(),
    _jointWeightVel(),
    _jointClampPos(),
    _weightIneqPos(),
    _weightIneqVel(),
    _weightIneqForce(),
    _weightIneqID(),
    _gainMassMatrix(),
    _jointTargetTau(),
    _jointWeightTau(),
    _gravityVector(),
    _jointSolTau(),
    _jointSolDOF(),
    _jointStateTau(),
    _externalBaseWrench(),
    _biasConstraintEquilibrium(),
    _pinocchio()
{
}

SEIKORetargeting::SEIKORetargeting(Model& model) :
    _model(&model),
    _sizeDOF(0),
    _sizeJoint(0),
    _sizePlaneOn(0),
    _sizePointOn(0),
    _sizePlaneOff(0),
    _sizePointOff(0),
    _contactsPlane(),
    _contactsPoint(),
    _mappingContacts(),
    _isDisabledConstraints(),
    _jointLimitPosLower(),
    _jointLimitPosUpper(),
    _jointLimitVelAbs(),
    _jointLimitTauAbs(),
    _momentumLinWeight(),
    _momentumAngWeight(),
    _jointTargetPos(),
    _jointWeightPos(),
    _jointWeightVel(),
    _jointClampPos(),
    _weightIneqPos(),
    _weightIneqVel(),
    _weightIneqForce(),
    _weightIneqID(),
    _gainMassMatrix(),
    _jointTargetTau(),
    _jointWeightTau(),
    _gravityVector(),
    _jointSolTau(),
    _jointSolDOF(),
    _jointStateTau(),
    _externalBaseWrench(),
    _biasConstraintEquilibrium(),
    _pinocchio(model)
{
    init(model);
}
        
void SEIKORetargeting::init(Model& model)
{
    _model = &model;
    _sizeDOF = 0;
    _sizeJoint = 0;
    _sizePlaneOn = 0;
    _sizePointOn = 0;
    _sizePlaneOff = 0;
    _sizePointOff = 0;
    //Reset
    _contactsPlane.clear();
    _contactsPoint.clear();
    _mappingContacts.clear();
    _isDisabledConstraints = false;
    //Joint default initialization
    _sizeDOF = _model->sizeDOF();
    _sizeJoint = _model->sizeJoint();
    //Limits initialization
    _jointLimitPosLower = _model->jointLimitsLower();
    _jointLimitPosUpper = _model->jointLimitsUpper();
    _jointLimitVelAbs = _model->jointLimitsVelocity();
    _jointLimitTauAbs = _model->jointLimitsTorque();
    //Targets and weights default initialization
    _momentumLinWeight = 1e-6*Eigen::Vector3d::Ones();
    _momentumAngWeight = 1e-6*Eigen::Vector3d::Ones();
    _jointTargetPos = _model->getJointPosVect();
    _jointWeightPos = 1.0*Eigen::VectorXd::Ones(_sizeJoint);
    _jointWeightVel = 1.0*Eigen::VectorXd::Ones(_sizeJoint);
    _jointClampPos = 0.2;
    _weightIneqPos = 1.0;
    _weightIneqVel = 1.0;
    _weightIneqForce = 1.0;
    _weightIneqID = 1.0;
    _gainMassMatrix = 0.0;
    _jointTargetTau = Eigen::VectorXd::Zero(_sizeJoint);
    _jointWeightTau = 1.0*Eigen::VectorXd::Ones(_sizeJoint);
    _gravityVector = Eigen::VectorXd::Zero(_sizeDOF);
    _jointSolTau = Eigen::VectorXd::Zero(_sizeJoint);
    _jointSolDOF = Eigen::VectorXd::Zero(_sizeDOF);
    _jointStateTau = Eigen::VectorXd::Zero(_sizeJoint);
    //Equilibrium bias and external wrench
    _externalBaseWrench = Eigen::Vector6d::Zero();
    _biasConstraintEquilibrium = Eigen::Vector6d::Zero();
    //Reset Pinocchio
    _pinocchio = PinocchioInterface(model);
}

void SEIKORetargeting::addContactPlane(const std::string& frameName)
{
    //Check if given name is already registered
    if (_mappingContacts.count(frameName) != 0) {
        throw std::logic_error(
            "inria::SEIKORetargeting::addContactPlane: "
            "Already defined frame name: " 
            + frameName);
    }

    //Default configuration
    ContactPlane_t contact;
    contact.frameId = _model->getIndexFrame(frameName);
    contact.frameName = frameName;
    contact.isEnabled = false;
    contact.targetPos = _model->position(frameName, "ROOT");
    contact.targetMat = _model->orientation(frameName, "ROOT");
    contact.weightPos = 100.0;
    contact.weightMat = 1.0;
    contact.clampPos = 0.01;
    contact.clampMat = 0.01;
    contact.frictionCoef = 0.4;
    contact.normalForceMin = 1e-2;
    contact.normalForceMax = 1e9;
    contact.limitVelForce = 1e9;
    contact.jacobianWorld = Eigen::MatrixXd::Zero(6, _sizeDOF);
    contact.jacobianBody = Eigen::MatrixXd::Zero(6, _sizeDOF);
    contact.limitCOP.x() = 0.01;
    contact.limitCOP.y() = 0.01;
    contact.limitVelTorque = 1e9;
    contact.targetWrench = Eigen::Vector6d::Zero();
    contact.weightWrench = 1e-6*Eigen::Vector6d::Ones();
    contact.solWrench = Eigen::Vector6d::Zero();
    contact.stateWrench = Eigen::Vector6d::Zero();

    //Add new contact plane
    _contactsPlane.push_back(contact);
    _mappingContacts.insert({frameName, _contactsPlane.size()});
    _sizePlaneOff++;
}
void SEIKORetargeting::addContactPoint(const std::string& frameName)
{
    //Check if given name is already registered
    if (_mappingContacts.count(frameName) != 0) {
        throw std::logic_error(
            "inria::SEIKORetargeting::addContactPoint: "
            "Already defined frame name: " 
            + frameName);
    }

    //Default configuration
    ContactPoint_t contact;
    contact.frameId = _model->getIndexFrame(frameName);
    contact.frameName = frameName;
    contact.isEnabled = false;
    contact.targetPos = _model->position(frameName, "ROOT");
    contact.targetMat = _model->orientation(frameName, "ROOT");
    contact.weightPos = 100.0;
    contact.weightMat = 1.0;
    contact.clampPos = 0.01;
    contact.clampMat = 0.01;
    contact.frictionCoef = 0.4;
    contact.normalForceMin = 1e-2;
    contact.normalForceMax = 1e9;
    contact.limitVelForce = 1e9;
    contact.jacobianWorld = Eigen::MatrixXd::Zero(6, _sizeDOF);
    contact.jacobianBody = Eigen::MatrixXd::Zero(6, _sizeDOF);
    contact.contactMat = Eigen::Matrix3d::Identity();
    contact.targetForce = Eigen::Vector3d::Zero();
    contact.weightForce = 1e-6*Eigen::Vector3d::Ones();
    contact.solForce = Eigen::Vector3d::Zero();
    contact.stateForce = Eigen::Vector3d::Zero();

    //Add new contact point
    _contactsPoint.push_back(contact);
    _mappingContacts.insert({frameName, -_contactsPoint.size()});
    _sizePointOff++;
}

void SEIKORetargeting::resetTargetsFromModel()
{
    setJointTargetPos(_model->getJointPosVect());
    for (size_t i=0;i<_contactsPlane.size();i++) {
        _contactsPlane[i].targetPos = 
            _model->position(_contactsPlane[i].frameId, 0);
        _contactsPlane[i].targetMat = 
            _model->orientation(_contactsPlane[i].frameId, 0);
    }
    for (size_t i=0;i<_contactsPoint.size();i++) {
        _contactsPoint[i].targetPos = 
            _model->position(_contactsPoint[i].frameId, 0);
        _contactsPoint[i].targetMat = 
            _model->orientation(_contactsPoint[i].frameId, 0);
    }
}

bool SEIKORetargeting::toggleContact(
    const std::string& frameName,
    bool isEnabled)
{
    if (_mappingContacts.count(frameName) == 0) {
        throw std::logic_error(
            "inria::SEIKORetargeting::toggleContact: "
            "Unknown contact frame name: "
            + frameName);
    }
    int index = _mappingContacts.at(frameName);

    bool isChanged = false;
    if (index > 0) {
        //Plane contact
        index = index-1;
        if (isEnabled && !_contactsPlane.at(index).isEnabled) {
            _contactsPlane.at(index).isEnabled = true;
            _sizePlaneOn++;
            _sizePlaneOff--;
            isChanged = true;
            //Reset position and orientation on plane contact enabling
            _contactsPlane.at(index).targetPos = 
                _model->position(_contactsPlane.at(index).frameId, 0);
            _contactsPlane.at(index).targetMat = 
                _model->orientation(_contactsPlane.at(index).frameId, 0);
        }
        if (!isEnabled && _contactsPlane.at(index).isEnabled) {
            _contactsPlane.at(index).isEnabled = false;
            _sizePlaneOn--;
            _sizePlaneOff++;
            isChanged = true;
        }
    } else if (index < 0) {
        //Point contact
        index = -index-1;
        if (isEnabled && !_contactsPoint.at(index).isEnabled) {
            _contactsPoint.at(index).isEnabled = true;
            _sizePointOn++;
            _sizePointOff--;
            isChanged = true;
            //Reset only position on point contact enabling
            _contactsPoint.at(index).targetPos = 
                _model->position(_contactsPoint.at(index).frameId, 0);
        }
        if (!isEnabled && _contactsPoint.at(index).isEnabled) {
            _contactsPoint.at(index).isEnabled = false;
            _sizePointOn--;
            _sizePointOff++;
            isChanged = true;
        }
    }

    return isChanged;
}

const SEIKORetargeting::ContactBase_t& SEIKORetargeting::stateContactBase(
    const std::string& frameName) const
{
    if (_mappingContacts.count(frameName) == 0) {
        throw std::logic_error(
            "inria::SEIKORetargeting::stateContactBase: "
            "Unknown contact frame name: "
            + frameName);
    }

    int index = _mappingContacts.at(frameName);
    if (index > 0) {
        return _contactsPlane.at(index-1);
    } else {
        return _contactsPoint.at(-index-1);
    }
}
const SEIKORetargeting::ContactPlane_t& SEIKORetargeting::stateContactPlane(
    const std::string& frameName) const
{
    if (_mappingContacts.count(frameName) == 0) {
        throw std::logic_error(
            "inria::SEIKORetargeting::stateContactPlane: "
            "Unknown contact frame name: "
            + frameName);
    }

    int index = _mappingContacts.at(frameName);
    if (index > 0) {
        return _contactsPlane.at(index-1);
    } else {
        throw std::logic_error(
            "inria::SEIKORetargeting::stateContactPlane: "
            "Contact is not plane: "
            + frameName);
    }
}
const SEIKORetargeting::ContactPoint_t& SEIKORetargeting::stateContactPoint(
    const std::string& frameName) const
{
    if (_mappingContacts.count(frameName) == 0) {
        throw std::logic_error(
            "inria::SEIKORetargeting::stateContactPoint: "
            "Unknown contact frame name: "
            + frameName);
    }

    int index = _mappingContacts.at(frameName);
    if (index > 0) {
        throw std::logic_error(
            "inria::SEIKORetargeting::stateContactPoint: "
            "Contact is not point: "
            + frameName);
    } else {
        return _contactsPoint.at(-index-1);
    }
}
        
const std::map<std::string, int>& SEIKORetargeting::getMappingContacts() const
{
    return _mappingContacts;
}

void SEIKORetargeting::setDisableConstraints(bool isDisabled)
{
    _isDisabledConstraints = isDisabled;
}

void SEIKORetargeting::setMomentumWeight(
    const Eigen::Vector3d& weightLin, 
    const Eigen::Vector3d& weightAng)
{
    if (
        weightLin.minCoeff() <= 0.0 ||
        weightAng.minCoeff() <= 0.0
    ) {
        throw std::logic_error(
            "inria::SEIKORetargeting::setMomentumWeight: "
            "Invalid weight");
    }
    _momentumLinWeight = weightLin;
    _momentumAngWeight = weightAng;
}

void SEIKORetargeting::setJointLimitPos(
    const Eigen::VectorXd& limitLower, 
    const Eigen::VectorXd& limitUpper)
{
    if (
        limitLower.size() != _sizeJoint || 
        limitUpper.size() != _sizeJoint || 
        (limitUpper-limitLower).minCoeff() < 0.0
    ) {
        throw std::logic_error(
            "inria::SEIKORetargeting::setJointLimitPos: "
            "Invalid bounds");
    }
    _jointLimitPosLower = limitLower;
    _jointLimitPosUpper = limitUpper;
}
void SEIKORetargeting::setJointLimitVelAbs(const Eigen::VectorXd& maxVel)
{
    if (
        maxVel.size() != _sizeJoint || 
        maxVel.minCoeff() < 0.0
    ) {
        throw std::logic_error(
            "inria::SEIKORetargeting::setJointLimitVelAbs: "
            "Invalid bounds");
    }
    _jointLimitVelAbs = maxVel;
}
void SEIKORetargeting::setJointLimitTauAbs(const Eigen::VectorXd& maxTau)
{
    if (
        maxTau.size() != _sizeJoint || 
        maxTau.minCoeff() < 0.0
    ) {
        throw std::logic_error(
            "inria::SEIKORetargeting::setJointLimitTauAbs: "
            "Invalid bounds");
    }
    _jointLimitTauAbs = maxTau;
}
void SEIKORetargeting::setJointTargetPos(const Eigen::VectorXd& targetPos)
{
    if (targetPos.size() != _sizeJoint) {
        throw std::logic_error(
            "inria::SEIKORetargeting::setJointTargetPos: "
            "Invalid size");
    }
    _jointTargetPos = targetPos;
}
void SEIKORetargeting::setJointWeightPos(const Eigen::VectorXd& weightPos)
{
    if (
        weightPos.size() != _sizeJoint || 
        weightPos.minCoeff() < 0.0
    ) {
        throw std::logic_error(
            "inria::SEIKORetargeting::setJointWeightPos: "
            "Invalid weights");
    }
    _jointWeightPos = weightPos;
}
void SEIKORetargeting::setJointWeightVel(const Eigen::VectorXd& weightVel)
{
    if (
        weightVel.size() != _sizeJoint || 
        weightVel.minCoeff() < 0.0
    ) {
        throw std::logic_error(
            "inria::SEIKORetargeting::setJointWeightVel: "
            "Invalid weights");
    }
    _jointWeightVel = weightVel;
}
void SEIKORetargeting::setJointWeightTau(const Eigen::VectorXd& weightTau)
{
    if (
        weightTau.size() != _sizeJoint || 
        weightTau.minCoeff() < 0.0
    ) {
        throw std::logic_error(
            "inria::SEIKORetargeting::setJointWeightTau: "
            "Invalid weights");
    }
    _jointWeightTau = weightTau;
}
void SEIKORetargeting::setJointClampPos(double clampPos)
{
    if (clampPos < 0.0) {
        throw std::logic_error(
            "inria::SEIKORetargeting::setJointClampPos: "
            "Negative parameter");
    }
    _jointClampPos = clampPos;
}

const Eigen::VectorXd& SEIKORetargeting::getJointLimitPosLower() const
{
    return _jointLimitPosLower;
}
const Eigen::VectorXd& SEIKORetargeting::getJointLimitPosUpper() const
{
    return _jointLimitPosUpper;
}

void SEIKORetargeting::setWeightIneqPos(double weight)
{
    if (weight < 0.0) {
        throw std::logic_error(
            "inria::SEIKORetargeting::setWeightIneqPos: "
            "Negative parameter");
    }
    _weightIneqPos = weight;
}
void SEIKORetargeting::setWeightIneqVel(double weight)
{
    if (weight < 0.0) {
        throw std::logic_error(
            "inria::SEIKORetargeting::setWeightIneqVel: "
            "Negative parameter");
    }
    _weightIneqVel = weight;
}
void SEIKORetargeting::setWeightIneqForce(double weight)
{
    if (weight < 0.0) {
        throw std::logic_error(
            "inria::SEIKORetargeting::setWeightIneqForce: "
            "Negative parameter");
    }
    _weightIneqForce = weight;
}
void SEIKORetargeting::setWeightIneqID(double weight)
{
    if (weight < 0.0) {
        throw std::logic_error(
            "inria::SEIKORetargeting::setWeightIneqID: "
            "Negative parameter");
    }
    _weightIneqID = weight;
}
        
void SEIKORetargeting::setGainMassMatrix(double gain)
{
    _gainMassMatrix = gain;
}
        
Eigen::Vector3d& SEIKORetargeting::refTargetPos(
    const std::string& frameName)
{
    return getContactBase(frameName).targetPos;
}
Eigen::Matrix3d& SEIKORetargeting::refTargetMat(
    const std::string& frameName)
{
    return getContactBase(frameName).targetMat;
}
void SEIKORetargeting::setWeightPose(
    const std::string& frameName,
    double weightPos, double weightMat)
{
    if (weightPos < 0.0 || weightMat < 0.0) {
        throw std::logic_error(
            "inria::SEIKORetargeting::setWeightPose: "
            "Invalid weights");
    }
    getContactBase(frameName).weightPos = weightPos;
    getContactBase(frameName).weightMat = weightMat;
}
void SEIKORetargeting::setClampPose(
    const std::string& frameName,
    double clampPos, double clampMat)
{
    if (clampPos < 0.0 || clampMat < 0.0) {
        throw std::logic_error(
            "inria::SEIKORetargeting::setClampPose: "
            "Invalid clamps");
    }
    getContactBase(frameName).clampPos = clampPos;
    getContactBase(frameName).clampMat = clampMat;
}
void SEIKORetargeting::setContactLimitFriction(
    const std::string& frameName,
    double frictionCoef)
{
    if (frictionCoef < 0.0) {
        throw std::logic_error(
            "inria::SEIKORetargeting::setContactLimitFriction: "
            "Invalid coef");
    }
    getContactBase(frameName).frictionCoef = frictionCoef;
}
void SEIKORetargeting::setNormalForceLimits(
    const std::string& frameName,
    double normalForceMin, double normalForceMax)
{
    if (
        normalForceMax < 0.0 ||
        normalForceMin > normalForceMax
    ) {
        throw std::logic_error(
            "inria::SEIKORetargeting::setNormalForceLimits: "
            "Invalid limits");
    }
    ContactBase_t& contact = getContactBase(frameName);
    contact.normalForceMin = normalForceMin;
    contact.normalForceMax = normalForceMax;
}
void SEIKORetargeting::setLimitVelForce(
    const std::string& frameName,
    double maxVel)
{
    if (maxVel < 0.0) {
        throw std::logic_error(
            "inria::SEIKORetargeting::setLimitVelForce: "
            "Invalid limits");
    }
    getContactBase(frameName).limitVelForce = maxVel;
}
void SEIKORetargeting::setLimitVelTorque(
    const std::string& frameName,
    double maxVel)
{
    if (maxVel < 0.0) {
        throw std::logic_error(
            "inria::SEIKORetargeting::setLimitVelTorque: "
            "Invalid limits");
    }
    getContactPlane(frameName).limitVelTorque = maxVel;
}

void SEIKORetargeting::setContactPlaneLimitCOP(
    const std::string& frameName,
    double limitX, double limitY)
{
    if (limitX < 0.0 || limitY < 0.0) {
        throw std::logic_error(
            "inria::SEIKORetargeting::setContactPlaneLimitCOP: "
            "Invalid limits");
    }
    Eigen::Vector2d limit;
    limit.x() = limitX;
    limit.y() = limitY;
    getContactPlane(frameName).limitCOP = limit;
}
void SEIKORetargeting::setTargetWrench(
    const std::string& frameName,
    const Eigen::Vector6d& targetWrench)
{
    ContactPlane_t& contact = getContactPlane(frameName);
    contact.targetWrench = targetWrench;
}
void SEIKORetargeting::setWeightWrench(
    const std::string& frameName,
    const Eigen::Vector6d& weightWrench)
{
    if (weightWrench.minCoeff() < 0.0) {
        throw std::logic_error(
            "inria::SEIKORetargeting::setWeightWrench: "
            "Invalid weights");
    }
    ContactPlane_t& contact = getContactPlane(frameName);
    contact.weightWrench = weightWrench;
}

void SEIKORetargeting::setContactMat(
    const std::string& frameName,
    const Eigen::Matrix3d& contactMat)
{
    getContactPoint(frameName).contactMat = contactMat;
}
void SEIKORetargeting::setTargetForce(
    const std::string& frameName,
    const Eigen::Vector3d& targetForce)
{
    ContactPoint_t& contact = getContactPoint(frameName);
    contact.targetForce = targetForce;
}
void SEIKORetargeting::setWeightForce(
    const std::string& frameName,
    const Eigen::Vector3d& weightForce)
{
    if (weightForce.minCoeff() < 0.0) {
        throw std::logic_error(
            "inria::SEIKORetargeting::setWeightForce: "
            "Invalid weights");
    }
    ContactPoint_t& contact = getContactPoint(frameName);
    contact.weightForce = weightForce;
}

const Eigen::Vector6d& SEIKORetargeting::refExternalBaseWrench() const
{
    return _externalBaseWrench;
}
Eigen::Vector6d& SEIKORetargeting::refExternalBaseWrench()
{
    return _externalBaseWrench;
}

bool SEIKORetargeting::runInverseDynamics()
{
    //Define problem sizes for 
    //static inverse dynamics
    size_t sizeSol = 
        //Plane contact wrenches
        6*_sizePlaneOn +
        //Point contact forces
        3*_sizePointOn;
    size_t sizeCost = 
        //Targets for joint torques and 
        //contact wrenches and forces
        _sizeJoint +
        6*_sizePlaneOn +
        3*_sizePointOn;
    size_t sizeEq =
        //Floating base upper part of 
        //static equation of motion
        6;
    size_t sizeIneq =
        //Joint torques bounds
        2*_sizeJoint + 
        //Contact plane constraints
        18*_sizePlaneOn +
        //Contact point constraints
        6*_sizePointOn;
    
    //Matrices initialization
    Eigen::DiagonalMatrix<double, Eigen::Dynamic> weights(sizeCost);
    weights.setZero();
    Eigen::MatrixXd costMat = Eigen::MatrixXd::Zero(sizeCost, sizeSol);
    Eigen::VectorXd costVec = Eigen::VectorXd::Zero(sizeCost);
    Eigen::MatrixXd problemCostMat = Eigen::MatrixXd::Zero(sizeSol, sizeSol);
    Eigen::VectorXd problemCostVec = Eigen::VectorXd::Zero(sizeSol);
    Eigen::MatrixXd problemEqMat = Eigen::MatrixXd::Zero(sizeEq, sizeSol);
    Eigen::VectorXd problemEqVec = Eigen::VectorXd::Zero(sizeEq);
    Eigen::MatrixXd problemIneqMat = Eigen::MatrixXd::Zero(sizeIneq, sizeSol);
    Eigen::VectorXd problemIneqVec = Eigen::VectorXd::Zero(sizeIneq);
    Eigen::VectorXd problemSolution = Eigen::VectorXd::Zero(sizeSol);
    
    //Compute joint gravity vector
    _gravityVector = _model->computeGravityVector();

    //Compute plane and point contact Jacobian matrices
    for (size_t i=0;i<_contactsPlane.size();i++) {
        if (_contactsPlane[i].isEnabled) {
            _contactsPlane[i].jacobianBody = _model->pointJacobian(
                _contactsPlane[i].frameId, _contactsPlane[i].frameId);
        }
    }
    for (size_t i=0;i<_contactsPoint.size();i++) {
        if (_contactsPoint[i].isEnabled) {
            _contactsPoint[i].jacobianWorld = _model->pointJacobian(
                _contactsPoint[i].frameId, 0);
        }
    }
    
    //Build the linear relationship from reduced solution 
    //(contact wrenches/forces) to computed joint torques
    //using the static equation of motion.
    //solToTauMat*sol + solToTauVec = S*(dtau+tau)
    Eigen::MatrixXd solToTauMat = Eigen::MatrixXd::Zero(_sizeDOF, sizeSol);
    Eigen::VectorXd solToTauVec = Eigen::VectorXd::Zero(_sizeDOF);
    size_t offsetTmpCol = 0;
    //Contact plane wrenches
    for (size_t i=0;i<_contactsPlane.size();i++) {
        if (_contactsPlane[i].isEnabled) {
            solToTauMat.block(0, offsetTmpCol, _sizeDOF, 6) = 
                -_contactsPlane[i].jacobianBody.transpose();
            offsetTmpCol += 6;
        }
    }
    //Contact point forces
    for (size_t i=0;i<_contactsPoint.size();i++) {
        if (_contactsPoint[i].isEnabled) {
            solToTauMat.block(0, offsetTmpCol, _sizeDOF, 3) =
                -(_contactsPoint[i].jacobianWorld
                    .block(3, 0, 3, _sizeDOF).transpose()
                * _contactsPoint[i].contactMat);
            offsetTmpCol += 3;
        }
    }
    //Gravity vector
    solToTauVec += _gravityVector;
    //Add external wrench on floating base
    solToTauVec.segment(0, 3) -= _externalBaseWrench.segment(3, 3);
    solToTauVec.segment(3, 3) -= _externalBaseWrench.segment(0, 3);
    
    //Static inverse dynamics matrices setup
    //Cost matrix
    size_t offsetCostRow = 0;
    //Joint torques using linear relation from contact 
    //wrenches/forces with lower part of the equation of motion
    costMat.block(offsetCostRow, 0, _sizeJoint, sizeSol) = 
        solToTauMat.block(6, 0, _sizeJoint, sizeSol);
    costVec.segment(offsetCostRow, _sizeJoint) = 
        _jointTargetTau - solToTauVec.segment(6, _sizeJoint);
    weights.diagonal().segment(offsetCostRow, _sizeJoint) = 
        _jointWeightTau;
    offsetCostRow += _sizeJoint;
    //Contact plane wrenches
    size_t offsetCostCol = 0;
    for (size_t i=0;i<_contactsPlane.size();i++) {
        if (_contactsPlane[i].isEnabled) {
            costMat.block(offsetCostRow, offsetCostCol, 6, 6).setIdentity();
            costVec.segment(offsetCostRow, 6) = 
                _contactsPlane[i].targetWrench;
            weights.diagonal().segment(offsetCostRow, 6) = 
                _contactsPlane[i].weightWrench;
            offsetCostRow += 6;
            offsetCostCol += 6;
        }
    }
    //Contact point forces
    for (size_t i=0;i<_contactsPoint.size();i++) {
        if (_contactsPoint[i].isEnabled) {
            costMat.block(offsetCostRow, offsetCostCol, 3, 3).setIdentity();
            costVec.segment(offsetCostRow, 3) = 
                _contactsPoint[i].targetForce;
            weights.diagonal().segment(offsetCostRow, 3) = 
                _contactsPoint[i].weightForce;
            offsetCostRow += 3;
            offsetCostCol += 3;
        }
    }

    //Build equality constraints matrix
    //Upper floating base rows of static equation of motion
    problemEqMat = solToTauMat.block(0, 0, 6, sizeSol);
    problemEqVec = solToTauVec.segment(0, 6);

    //Build inequality constraints matrix
    Eigen::MatrixXd tmpIDIneqMat;
    Eigen::VectorXd tmpIDIneqVec;
    //Build constraints matrix and vector over 
    //joint torques and contact wrenches/forces
    buildStaticIDInequalities(tmpIDIneqMat, tmpIDIneqVec);
    //Build mapping from solution (only contact wrenches/forces)
    //to extented vector with joint torques and contacts
    Eigen::MatrixXd tmpMappingMat = 
        Eigen::MatrixXd::Zero(_sizeJoint + sizeSol, sizeSol);
    Eigen::VectorXd tmpMappingVec = 
        Eigen::VectorXd::Zero(_sizeJoint + sizeSol);
    tmpMappingMat.block(0, 0, _sizeJoint, sizeSol) = 
        solToTauMat.block(6, 0, _sizeJoint, sizeSol);
    tmpMappingMat.block(_sizeJoint, 0, sizeSol, sizeSol).setIdentity();
    tmpMappingVec.segment(0, _sizeJoint) = solToTauVec.segment(6, _sizeJoint);
    //Build problem inequalities
    problemIneqMat = tmpIDIneqMat*tmpMappingMat;
    problemIneqVec = tmpIDIneqMat*tmpMappingVec + tmpIDIneqVec;

    //Build the weighted distance of linear target costs
    problemCostMat = costMat.transpose()*weights*costMat;
    problemCostVec = -costMat.transpose()*weights*costVec;

    //Solve the QP problem
    double cost;
    Eigen::VectorXi activeSet;
    size_t activeSetSize;
    if (_isDisabledConstraints) {
        cost = eiquadprog::solvers::solve_quadprog(
            problemCostMat,
            problemCostVec,
            problemEqMat.transpose(),
            problemEqVec,
            Eigen::MatrixXd::Zero(sizeSol, 0),
            Eigen::VectorXd::Zero(0),
            problemSolution,
            activeSet, activeSetSize);
    } else {
        cost = eiquadprog::solvers::solve_quadprog(
            problemCostMat,
            problemCostVec,
            problemEqMat.transpose(),
            problemEqVec,
            problemIneqMat.transpose(),
            problemIneqVec,
            problemSolution,
            activeSet, activeSetSize);
    }
    bool isSuccess = !(std::isnan(cost) || std::isinf(cost));

    //Check solver success
    if (!isSuccess) {
        //Return failure
        return false;
    } else {
        //Copy computed solution to internal state
        size_t offsetSolRow = 0;
        //Contact wrenches
        for (size_t i=0;i<_contactsPlane.size();i++) {
            if (_contactsPlane[i].isEnabled) {
                _contactsPlane[i].stateWrench = 
                    problemSolution.segment(offsetSolRow, 6);
                offsetSolRow += 6;
            } else {
                _contactsPlane[i].stateWrench.setZero();
            }
        }
        //Contact forces
        for (size_t i=0;i<_contactsPoint.size();i++) {
            if (_contactsPoint[i].isEnabled) {
                _contactsPoint[i].stateForce = 
                    problemSolution.segment(offsetSolRow, 3);
                offsetSolRow += 3;
            } else {
                _contactsPoint[i].stateForce.setZero();
            }
        }
        //Compute the torque from botton joint 
        //part of static equation of motion
        _jointStateTau = 
            solToTauMat.block(6, 0, _sizeJoint, sizeSol)*problemSolution 
            + solToTauVec.segment(6, _sizeJoint);
        //Return success
        return true;
    }
}

bool SEIKORetargeting::runSEIKO(double dt)
{
    //Define problem sizes
    size_t sizeSolID = 
        //Plane contact wrenches velocity
        6*_sizePlaneOn + 
        //Point contact forces velocity
        3*_sizePointOn;
    size_t sizeSol = 
        //Velocity for each DOF
        _sizeDOF +
        //Static wrenches and forces velocity
        sizeSolID;
    size_t sizeCost = 
        //Centroidal momentum regularization 
        6 +
        //Joint velocity regularization
        _sizeJoint +
        //Joint position target
        _sizeJoint +
        //Disabled pose and point contact target
        6*(_sizePlaneOff + _sizePointOff) +
        //Enabled point contact orientation target
        3*_sizePointOn +
        //Joint torque minimization
        _sizeJoint +
        //Contact plane wrenches minimization
        6*_sizePlaneOn +
        //Contact point forces minimization
        3*_sizePointOn;
    size_t sizeEq =
        //Enabled pose contact constraint
        6*_sizePlaneOn +
        //Enabled point contact constraint
        3*_sizePointOn +
        //Upper floating base part of 
        //the static equation of motion
        6;
    size_t sizeIDIneq = 
        //Joint torques bounds
        2*_sizeJoint + 
        //Contact plane constraints
        18*_sizePlaneOn +
        //Contact point constraints
        6*_sizePointOn;
    size_t sizeIneq =
        //Joint position bounds
        2*_sizeJoint +
        //Joint velocity bounds
        2*_sizeJoint +
        //Torque change (velocity) bounds
        2*3*_sizePlaneOn +
        //Force change (velocity) bounds
        2*3*(_sizePlaneOn +_sizePointOn) +
        //Static inverse dynamics feasibility
        sizeIDIneq;
    
    //Matrices initialization
    Eigen::DiagonalMatrix<double, Eigen::Dynamic> weights = 
        Eigen::DiagonalMatrix<double, Eigen::Dynamic>(sizeCost);
    weights.setZero();
    Eigen::MatrixXd costMat = Eigen::MatrixXd::Zero(sizeCost, sizeSol);
    Eigen::VectorXd costVec = Eigen::VectorXd::Zero(sizeCost);
    Eigen::MatrixXd problemCostMat = Eigen::MatrixXd::Zero(sizeSol, sizeSol);
    Eigen::VectorXd problemCostVec = Eigen::VectorXd::Zero(sizeSol);
    Eigen::MatrixXd problemEqMat = Eigen::MatrixXd::Zero(sizeEq, sizeSol);
    Eigen::VectorXd problemEqVec = Eigen::VectorXd::Zero(sizeEq);
    Eigen::MatrixXd problemIneqMat = Eigen::MatrixXd::Zero(sizeIneq, sizeSol);
    Eigen::VectorXd problemIneqVec = Eigen::VectorXd::Zero(sizeIneq);
    Eigen::VectorXd problemSolution = Eigen::VectorXd::Zero(sizeSol);

    //Update pinocchio kinematics
    _pinocchio.updateKinematics(_model->getDOFPosVect());

    //Compute plane and point contact Jacobian matrices
    for (size_t i=0;i<_contactsPlane.size();i++) {
        _contactsPlane[i].jacobianWorld = _model->pointJacobian(
            _contactsPlane[i].frameId, 0);
        if (_contactsPlane[i].isEnabled) {
            _contactsPlane[i].jacobianBody = _model->pointJacobian(
                _contactsPlane[i].frameId, _contactsPlane[i].frameId);
        }
    }
    for (size_t i=0;i<_contactsPoint.size();i++) {
        _contactsPoint[i].jacobianWorld = _model->pointJacobian(
            _contactsPoint[i].frameId, 0);
    }
    
    //Compute mass dynamic equation terms
    Eigen::MatrixXd massMatrix;
    Eigen::VectorXd coreolis;
    _model->computeEquationOfMotion(
        massMatrix, coreolis, &_gravityVector);

    //Compute centroidal dynamic matrix
    Eigen::MatrixXd CMM;
    Eigen::VectorXd CMMDotQDot;
    _model->computeCentroidalDynamics(
        massMatrix, coreolis, _gravityVector,
        CMM, CMMDotQDot);

    //Build current ID state vector
    Eigen::VectorXd currentIDVect = 
        Eigen::VectorXd::Zero(_sizeJoint + sizeSolID);
    size_t offsetSolID = 0;
    currentIDVect.segment(offsetSolID, _sizeJoint) = _jointStateTau;
    offsetSolID += _sizeJoint;
    for (size_t i=0;i<_contactsPlane.size();i++) {
        if (_contactsPlane[i].isEnabled) {
            currentIDVect.segment(offsetSolID, 6) = 
                _contactsPlane[i].stateWrench;
            offsetSolID += 6;
        }
    }
    for (size_t i=0;i<_contactsPoint.size();i++) {
        if (_contactsPoint[i].isEnabled) {
            currentIDVect.segment(offsetSolID, 3) = 
                _contactsPoint[i].stateForce;
            offsetSolID += 3;
        }
    }

    //Build the linear relationship from reduced solution 
    //(joint velocity and contact wrenches/forces velocity)
    //to computed joint torques (after integration)
    //using the differentiated static equation of motion.
    //dt*solToTauMat*sol + solToTauVec = S*(dt*dottau+tau)
    //S*dottau = solToTauMat*sol
    //S*tau = solToTauVec
    Eigen::MatrixXd solToTauMat = Eigen::MatrixXd::Zero(_sizeDOF, sizeSol);
    Eigen::VectorXd solToTauVec = Eigen::VectorXd::Zero(_sizeDOF);
    size_t offsetTmpCol = _sizeDOF;
    for (size_t i=0;i<_contactsPlane.size();i++) {
        if (_contactsPlane[i].isEnabled) {
            solToTauMat.block(0, 0, _sizeDOF, _sizeDOF) -= 
                _pinocchio.diffHessianWrenchInLocalProduct(
                    _contactsPlane[i].frameName, 
                    _contactsPlane[i].stateWrench);
            solToTauMat.block(0, offsetTmpCol, _sizeDOF, 6) = 
                -_contactsPlane[i].jacobianBody.transpose();
            solToTauVec -= 
                _contactsPlane[i].jacobianBody.transpose()
                * _contactsPlane[i].stateWrench;
            offsetTmpCol += 6;
        }
    }
    for (size_t i=0;i<_contactsPoint.size();i++) {
        if (_contactsPoint[i].isEnabled) {
            solToTauMat.block(0, 0, _sizeDOF, _sizeDOF) -= 
                _pinocchio.diffHessianForceInWorldProduct(
                    _contactsPoint[i].frameName, 
                    (_contactsPoint[i].contactMat*_contactsPoint[i].stateForce));
            solToTauMat.block(0, offsetTmpCol, _sizeDOF, 3) = 
                -_contactsPoint[i].jacobianWorld
                    .block(3, 0, 3, _sizeDOF).transpose()
                * _contactsPoint[i].contactMat;
            solToTauVec -=
                _contactsPoint[i].jacobianWorld.block(3, 0, 3, _sizeDOF).transpose()
                * _contactsPoint[i].contactMat
                * _contactsPoint[i].stateForce;
            offsetTmpCol += 3;
        }
    }
    solToTauMat.block(0, 0, _sizeDOF, _sizeDOF) += 
        _pinocchio.diffGravityVector();
    solToTauVec += _gravityVector;
    //Apply optional quasi-dynamic mass matrix term
    solToTauMat.block(0, 0, _sizeDOF, _sizeDOF) += 
        _gainMassMatrix*massMatrix;
    //Add external wrench on floating base
    solToTauVec.segment(0, 3) -= _externalBaseWrench.segment(3, 3);
    solToTauVec.segment(3, 3) -= _externalBaseWrench.segment(0, 3);

    //Build cost
    size_t offsetCostRow = 0;
    //Centroidal momentum regularization
    costMat.block(offsetCostRow, 0, 6, _sizeDOF) = 
        CMM;
    costVec.segment(offsetCostRow, 6) = 
        Eigen::VectorXd::Zero(6);
    weights.diagonal().segment(offsetCostRow+0, 3) = 
        _momentumAngWeight;
    weights.diagonal().segment(offsetCostRow+3, 3) = 
        _momentumLinWeight;
    offsetCostRow += 6;
    //Joint velocity regularization
    costMat.block(offsetCostRow, 6, _sizeJoint, _sizeJoint) = 
        Eigen::MatrixXd::Identity(_sizeJoint, _sizeJoint);
    costVec.segment(offsetCostRow, _sizeJoint) = 
        Eigen::VectorXd::Zero(_sizeJoint);
    weights.diagonal().segment(offsetCostRow, _sizeJoint) = 
        _jointWeightVel;
    offsetCostRow += _sizeJoint;
    //Joint position target
    costMat.block(offsetCostRow, 6, _sizeJoint, _sizeJoint) = 
        Eigen::MatrixXd::Identity(_sizeJoint, _sizeJoint);
    costVec.segment(offsetCostRow, _sizeJoint) = (1.0/dt)*ClampVectorAllComponent(
        _jointTargetPos - _model->getJointPosVect(), _jointClampPos);
    weights.diagonal().segment(offsetCostRow, _sizeJoint) = 
        _jointWeightPos;
    offsetCostRow += _sizeJoint;
    //Disable pose contact target
    for (size_t i=0;i<_contactsPlane.size();i++) {
        if (!_contactsPlane[i].isEnabled) {
            writePoseTarget(
                dt,
                _contactsPlane[i],
                costMat, costVec, weights, 
                offsetCostRow,
                true);
            offsetCostRow += 6;
        }
    }
    //Disable point contact target
    for (size_t i=0;i<_contactsPoint.size();i++) {
        if (!_contactsPoint[i].isEnabled) {
            writePoseTarget(
                dt,
                _contactsPoint[i],
                costMat, costVec, weights, 
                offsetCostRow,
                true);
            offsetCostRow += 6;
        }
    }
    //Enabled point contact orientation target
    for (size_t i=0;i<_contactsPoint.size();i++) {
        if (_contactsPoint[i].isEnabled) {
            writePoseTarget(
                dt,
                _contactsPoint[i],
                costMat, costVec, weights, 
                offsetCostRow,
                false);
            offsetCostRow += 3;
        }
    }
    //Torques wrenches and forces velocity.
    //Joint torques using linear relation from contact 
    //wrenches/forces with lower part of the equation of motion
    costMat.block(offsetCostRow, 0, _sizeJoint, sizeSol) = 
        solToTauMat.block(6, 0, _sizeJoint, sizeSol);
    costVec.segment(offsetCostRow, _sizeJoint) = 
        (1.0/dt)*
        (_jointTargetTau - solToTauVec.segment(6, _sizeJoint));
    weights.diagonal().segment(offsetCostRow, _sizeJoint) = 
        _jointWeightTau;
    offsetCostRow += _sizeJoint;
    //Contact plane wrenches
    size_t offsetCostCol = _sizeDOF;
    for (size_t i=0;i<_contactsPlane.size();i++) {
        if (_contactsPlane[i].isEnabled) {
            costMat.block(offsetCostRow, offsetCostCol, 6, 6) = 
                Eigen::MatrixXd::Identity(6, 6);
            costVec.segment(offsetCostRow, 6) = 
                (1.0/dt)*
                (_contactsPlane[i].targetWrench - _contactsPlane[i].stateWrench);
            weights.diagonal().segment(offsetCostRow, 6) = 
                _contactsPlane[i].weightWrench;
            offsetCostRow += 6;
            offsetCostCol += 6;
        }
    }
    //Contact point forces
    for (size_t i=0;i<_contactsPoint.size();i++) {
        if (_contactsPoint[i].isEnabled) {
            costMat.block(offsetCostRow, offsetCostCol, 3, 3) = 
                Eigen::MatrixXd::Identity(3, 3);
            costVec.segment(offsetCostRow, 3) = 
                (1.0/dt)*
                (_contactsPoint[i].targetForce - _contactsPoint[i].stateForce);
            weights.diagonal().segment(offsetCostRow, 3) = 
                _contactsPoint[i].weightForce;
            offsetCostRow += 3;
            offsetCostCol += 3;
        }
    }

    //Build equality constraints
    size_t offsetEq = 0;
    //Enabled pose contacts
    for (size_t i=0;i<_contactsPlane.size();i++) {
        if (_contactsPlane[i].isEnabled) {
            const size_t& frameId = _contactsPlane[i].frameId;
            Eigen::Vector3d deltaMat = MatrixToAxis(
                    _contactsPlane[i].targetMat
                    * _model->orientation(frameId, 0).transpose());
            Eigen::Vector3d deltaPos = 
                _contactsPlane[i].targetPos
                - _model->position(frameId, 0);
            problemEqMat.block(offsetEq, 0, 6, _sizeDOF) = 
                _contactsPlane[i].jacobianWorld;
            problemEqVec.segment(offsetEq+0, 3) = -(1.0/dt)*deltaMat;
            problemEqVec.segment(offsetEq+3, 3) = -(1.0/dt)*deltaPos;
            offsetEq += 6;
        }
    }
    //Enabled point contacts
    for (size_t i=0;i<_contactsPoint.size();i++) {
        if (_contactsPoint[i].isEnabled) {
            const size_t& frameId = _contactsPoint[i].frameId;
            Eigen::Vector3d deltaPos = 
                _contactsPoint[i].targetPos
                - _model->position(frameId, 0);
            problemEqMat.block(offsetEq, 0, 3, _sizeDOF) = 
                _contactsPoint[i].jacobianWorld.block(3, 0, 3, _sizeDOF);
            problemEqVec.segment(offsetEq, 3) = -(1.0/dt)*deltaPos;
            offsetEq += 3;
        }
    }
    //Static equation of motion
    problemEqMat.block(offsetEq, 0, 6, sizeSol) = 
        solToTauMat.block(0, 0, 6, sizeSol);
    _biasConstraintEquilibrium = solToTauVec.segment(0, 6);
    problemEqVec.segment(offsetEq, 6) = 
        (1.0/dt)*_biasConstraintEquilibrium;
    
    //Build inequality constraints 
    size_t offsetIneq = 0;
    //Joint kinematics bounds
    for (size_t i=0;i<_sizeJoint;i++) {
        problemIneqMat(offsetIneq+0, 6+i) = dt*_weightIneqPos;
        problemIneqMat(offsetIneq+1, 6+i) = -dt*_weightIneqPos;
        problemIneqVec(offsetIneq+0) = 
            _weightIneqPos*
            (_model->getDOFPos(i+6) - _jointLimitPosLower(i));
        problemIneqVec(offsetIneq+1) = 
            _weightIneqPos*
            (-_model->getDOFPos(i+6) + _jointLimitPosUpper(i));
        offsetIneq += 2;
    }
    //Joint velocity bounds
    for (size_t i=0;i<_sizeJoint;i++) {
        problemIneqMat(offsetIneq+0, 6+i) = 1.0*_weightIneqVel;
        problemIneqMat(offsetIneq+1, 6+i) = -1.0*_weightIneqVel;
        problemIneqVec(offsetIneq+0) = 
            _weightIneqVel*_jointLimitVelAbs(i);
        problemIneqVec(offsetIneq+1) = 
            _weightIneqVel*_jointLimitVelAbs(i);
        offsetIneq += 2;
    }
    //Torque velocity bounds
    size_t offsetIneqCol = _sizeDOF;
    for (size_t i=0;i<_contactsPlane.size();i++) {
        if (_contactsPlane[i].isEnabled) {
            problemIneqMat.block(offsetIneq+0, offsetIneqCol+0, 3, 3) = 
                _weightIneqForce*Eigen::MatrixXd::Identity(3, 3);
            problemIneqMat.block(offsetIneq+3, offsetIneqCol+0, 3, 3) = 
                -_weightIneqForce*Eigen::MatrixXd::Identity(3, 3);
            problemIneqVec.segment(offsetIneq+0, 3) = 
                _weightIneqForce*_contactsPlane[i].limitVelTorque
                * Eigen::VectorXd::Ones(3);
            problemIneqVec.segment(offsetIneq+3, 3) = 
                _weightIneqForce*_contactsPlane[i].limitVelTorque
                * Eigen::VectorXd::Ones(3);
            offsetIneq += 2*3;
            offsetIneqCol += 6;
        }
    }
    //Force velocity bounds
    offsetIneqCol = _sizeDOF;
    for (size_t i=0;i<_contactsPlane.size();i++) {
        if (_contactsPlane[i].isEnabled) {
            problemIneqMat.block(offsetIneq+0, offsetIneqCol+3, 3, 3) = 
                _weightIneqForce*Eigen::MatrixXd::Identity(3, 3);
            problemIneqMat.block(offsetIneq+3, offsetIneqCol+3, 3, 3) = 
                -_weightIneqForce*Eigen::MatrixXd::Identity(3, 3);
            problemIneqVec.segment(offsetIneq+0, 3) = 
                _weightIneqForce*_contactsPlane[i].limitVelForce
                * Eigen::VectorXd::Ones(3);
            problemIneqVec.segment(offsetIneq+3, 3) = 
                _weightIneqForce*_contactsPlane[i].limitVelForce
                * Eigen::VectorXd::Ones(3);
            offsetIneq += 2*3;
            offsetIneqCol += 6;
        }
    }
    for (size_t i=0;i<_contactsPoint.size();i++) {
        if (_contactsPoint[i].isEnabled) {
            problemIneqMat.block(offsetIneq+0, offsetIneqCol, 3, 3) = 
                _weightIneqForce*Eigen::MatrixXd::Identity(3, 3);
            problemIneqMat.block(offsetIneq+3, offsetIneqCol, 3, 3) = 
                -_weightIneqForce*Eigen::MatrixXd::Identity(3, 3);
            problemIneqVec.segment(offsetIneq+0, 3) = 
                _weightIneqForce*_contactsPoint[i].limitVelForce
                * Eigen::VectorXd::Ones(3);
            problemIneqVec.segment(offsetIneq+3, 3) = 
                _weightIneqForce*_contactsPoint[i].limitVelForce
                * Eigen::VectorXd::Ones(3);
            offsetIneq += 2*3;
            offsetIneqCol += 3;
        }
    }
    //Static torques wrenches forces
    Eigen::MatrixXd tmpIDIneqMat;
    Eigen::VectorXd tmpIDIneqVec;
    //Build constraints matrix and vector over 
    //joint torques and contact wrenches/forces
    buildStaticIDInequalities(tmpIDIneqMat, tmpIDIneqVec);
    //Build mapping from ID solution (only contact wrenches/forces)
    //to extented vector with joint torques and contacts
    Eigen::MatrixXd tmpMappingMat = 
        Eigen::MatrixXd::Zero(_sizeJoint + sizeSolID, sizeSol);
    Eigen::VectorXd tmpMappingVec = 
        Eigen::VectorXd::Zero(_sizeJoint + sizeSolID);
    tmpMappingMat.block(0, 0, _sizeJoint, sizeSol) = 
        solToTauMat.block(6, 0, _sizeJoint, sizeSol);
    tmpMappingMat.block(_sizeJoint, _sizeDOF, sizeSolID, sizeSolID).setIdentity();
    tmpMappingVec.segment(0, _sizeJoint) = solToTauVec.segment(6, _sizeJoint);
    tmpMappingVec.segment(_sizeJoint, sizeSolID) = 
        currentIDVect.segment(_sizeJoint, sizeSolID);
    //Build problem inequalities
    problemIneqMat.block(offsetIneq, 0, sizeIDIneq, sizeSol) = 
        _weightIneqID*dt*tmpIDIneqMat*tmpMappingMat;
    problemIneqVec.segment(offsetIneq, sizeIDIneq) = 
        _weightIneqID*(tmpIDIneqMat*tmpMappingVec + tmpIDIneqVec);
    offsetIneq += sizeIDIneq;

    //Build the weighted distance of linear target costs
    problemCostMat = costMat.transpose()*weights*costMat;
    problemCostVec = -costMat.transpose()*weights*costVec;
    
    //Solve the QP problem
    bool isSuccess = false;
    double cost;
    Eigen::VectorXi activeSet;
    size_t activeSetSize;
    if (_isDisabledConstraints) {
        cost = eiquadprog::solvers::solve_quadprog(
            problemCostMat,
            problemCostVec,
            problemEqMat.transpose(),
            problemEqVec,
            Eigen::MatrixXd::Zero(sizeSol, 0),
            Eigen::VectorXd::Zero(0),
            problemSolution,
            activeSet, activeSetSize);
    } else {
        cost = eiquadprog::solvers::solve_quadprog(
            problemCostMat,
            problemCostVec,
            problemEqMat.transpose(),
            problemEqVec,
            problemIneqMat.transpose(),
            problemIneqVec,
            problemSolution,
            activeSet, activeSetSize);
    }
    isSuccess = !(std::isnan(cost) || std::isinf(cost));
    
    //Check solver success
    if (!isSuccess) {
        //Set all changes to zero
        //DOF position
        _jointSolDOF.setZero();
        //Joint torques
        _jointSolTau.setZero();
        //Contact wrenches
        for (size_t i=0;i<_contactsPlane.size();i++) {
            _contactsPlane[i].solWrench.setZero();
        }
        //Contact forces
        for (size_t i=0;i<_contactsPoint.size();i++) {
            _contactsPoint[i].solForce.setZero();
        }
        //Return failure
        return false;
    } else {
        //Copy the computed changes
        size_t offsetSolRow = 0;
        //DOF velocity
        _jointSolDOF = 
            problemSolution.segment(offsetSolRow, _sizeDOF);
        for (size_t i=0;i<_sizeJoint;i++) {
            _jointSolDOF(6+i) = ClampAbsolute(
                _jointSolDOF(6+i), _jointLimitVelAbs(i));
        }
        offsetSolRow += _sizeDOF;
        //Contact wrenches
        for (size_t i=0;i<_contactsPlane.size();i++) {
            if (_contactsPlane[i].isEnabled) {
                _contactsPlane[i].solWrench = 
                    problemSolution.segment(offsetSolRow, 6);
                offsetSolRow += 6;
            } else {
                _contactsPlane[i].stateWrench.setZero();
                _contactsPlane[i].solWrench.setZero();
            }
            for (int j=0;j<3;j++) {
                _contactsPlane[i].solWrench(0+j) = ClampAbsolute(
                    _contactsPlane[i].solWrench(0+j), 
                    _contactsPlane[i].limitVelTorque);
                _contactsPlane[i].solWrench(3+j) = ClampAbsolute(
                    _contactsPlane[i].solWrench(3+j), 
                    _contactsPlane[i].limitVelForce);
            }
        }
        //Contact forces
        for (size_t i=0;i<_contactsPoint.size();i++) {
            if (_contactsPoint[i].isEnabled) {
                _contactsPoint[i].solForce = 
                    problemSolution.segment(offsetSolRow, 3);
                offsetSolRow += 3;
            } else {
                _contactsPoint[i].stateForce.setZero();
                _contactsPoint[i].solForce.setZero();
            }
            for (int j=0;j<3;j++) {
                _contactsPoint[i].solForce(j) = ClampAbsolute(
                    _contactsPoint[i].solForce(j), 
                    _contactsPoint[i].limitVelForce);
            }
        }
        //Reset joint torque state (before integration) to 
        //be coherent with state contact forces
        _jointStateTau = solToTauVec.segment(6, _sizeJoint);
        //Joint torques
        _jointSolTau = 
            solToTauMat.block(6, 0, _sizeJoint, sizeSol)*problemSolution; 
        //Return success
        return true;
    }
}

void SEIKORetargeting::integrateSolution(double dt)
{
    //Integrate position
    Eigen::VectorXd newPos = IntegrateDOFVect(
        _model->getDOFPosVect(), dt*_jointSolDOF);
    for (size_t i=0;i<_sizeJoint;i++) {
        newPos(6+i) = ClampRange(
            newPos(6+i), _jointLimitPosLower(i), _jointLimitPosUpper(i));
    }
    _model->setDOFPosVect(newPos);
    //Assign velocity
    _model->setDOFVelVect(_jointSolDOF);
    //Model update
    _model->updateState();

    //Integrate joint torques
    _jointStateTau += dt*_jointSolTau;
    
    //Integrate contact wrenches
    for (size_t i=0;i<_contactsPlane.size();i++) {
        _contactsPlane[i].stateWrench += 
            dt*_contactsPlane[i].solWrench;
    }
    //Integrate contact forces
    for (size_t i=0;i<_contactsPoint.size();i++) {
        _contactsPoint[i].stateForce += 
            dt*_contactsPoint[i].solForce;
    }
}        

const Eigen::VectorXd& SEIKORetargeting::solDOFPosition() const
{
    return _jointSolDOF;
}
const Eigen::VectorXd& SEIKORetargeting::solJointTorque() const
{
    return _jointSolTau;
}
const Eigen::Vector6d& SEIKORetargeting::solContactWrench(
    const std::string& frameName) const
{
    return stateContactPlane(frameName).solWrench;
}
const Eigen::Vector3d& SEIKORetargeting::solContactForce(
    const std::string& frameName) const
{
    return stateContactPoint(frameName).solForce;
}

const Eigen::VectorXd& SEIKORetargeting::stateJointTorque() const
{
    return _jointStateTau;
}
const Eigen::Vector6d& SEIKORetargeting::stateContactWrench(
    const std::string& frameName) const
{
    return stateContactPlane(frameName).stateWrench;
}
const Eigen::Vector3d& SEIKORetargeting::stateContactForce(
    const std::string& frameName) const
{
    return stateContactPoint(frameName).stateForce;
}

double SEIKORetargeting::stateNormalForce(
    const std::string& frameName) const
{
    if (_mappingContacts.count(frameName) == 0) {
        throw std::logic_error(
            "inria::SEIKORetargeting::stateNormalForce: "
            "Unknown contact frame name: "
            + frameName);
    }
    int index = _mappingContacts.at(frameName);

    if (index > 0) {
        return _contactsPlane.at(index-1).stateWrench(5);
    } else {
        return _contactsPoint.at(-index-1).stateForce(2);
    }
}

const Eigen::Vector6d& SEIKORetargeting::getErrorConstraintEquilibrium() const
{
    return _biasConstraintEquilibrium;
}

std::map<std::string, double> SEIKORetargeting::computeConstraintRatios() const
{
    //Container for computed ratio
    std::map<std::string, double> container;

    //Joint position
    Eigen::VectorXd neutral = 
        0.5*_jointLimitPosLower + 0.5*_jointLimitPosUpper;
    Eigen::VectorXd range = 
        _jointLimitPosUpper - neutral;
    Eigen::VectorXd position = 
        _model->getJointPosVect();
    for (size_t i=0;i<_sizeJoint;i++) {
        double ratio = std::fabs(neutral(i)-position(i))/range(i);
        container.insert(std::make_pair(
            "pos_" + _model->getNameJoint(i), 
            ratio));
    }

    //Joint torque
    for (size_t i=0;i<_sizeJoint;i++) {
        double ratio = 
            std::fabs(_jointStateTau(i))/_jointLimitTauAbs(i);
        container.insert(std::make_pair(
            "tau_" + _model->getNameJoint(i), 
            ratio));
    }

    //Total applied normal force
    double sumNormalForce = 0.0;
    for (size_t i=0;i<_contactsPlane.size();i++) {
        if (_contactsPlane[i].isEnabled) {
            sumNormalForce += _contactsPlane[i].stateWrench(5);
        }
    }
    for (size_t i=0;i<_contactsPoint.size();i++) {
        if (_contactsPoint[i].isEnabled) {
            sumNormalForce += _contactsPoint[i].stateForce(2);
        }
    }
    
    //Contact wrench
    for (size_t i=0;i<_contactsPlane.size();i++) {
        if (_contactsPlane[i].isEnabled) {
            const Eigen::Vector6d& wrench = _contactsPlane[i].stateWrench;
            double ratioCOPX = 0.0;
            double ratioCOPY = 0.0;
            double ratioFrictionX = 0.0;
            double ratioFrictionY = 0.0;
            if (std::fabs(wrench(5)) > 1e-3) {
                ratioCOPX = 
                    std::fabs(wrench(1)) /
                    std::fabs(wrench(5)*_contactsPlane[i].limitCOP.x());
                ratioCOPY = 
                    std::fabs(wrench(0)) /
                    std::fabs(wrench(5)*_contactsPlane[i].limitCOP.y());
                ratioFrictionX = 
                    std::fabs(wrench(3)) /
                    std::fabs(wrench(5)*_contactsPlane[i].frictionCoef);
                ratioFrictionY = 
                    std::fabs(wrench(4)) /
                    std::fabs(wrench(5)*_contactsPlane[i].frictionCoef);
            }
            container.insert(std::make_pair(
                "copX_" + _contactsPlane[i].frameName,
                ratioCOPX));
            container.insert(std::make_pair(
                "copY_" + _contactsPlane[i].frameName,
                ratioCOPY));
            container.insert(std::make_pair(
                "frictionX_" + _contactsPlane[i].frameName,
                ratioFrictionX));
            container.insert(std::make_pair(
                "frictionY_" + _contactsPlane[i].frameName,
                ratioFrictionY));
            container.insert(std::make_pair(
                "forceNormal_" + _contactsPlane[i].frameName,
                wrench(5)/sumNormalForce));
        } 
    }
    
    //Contact force
    for (size_t i=0;i<_contactsPoint.size();i++) {
        if (_contactsPoint[i].isEnabled) {
            const Eigen::Vector3d& force = _contactsPoint[i].stateForce;
            sumNormalForce += force(2);
            double ratioFrictionX = 0.0;
            double ratioFrictionY = 0.0;
            if (std::fabs(force(2)) > 1e-3) {
                ratioFrictionX = 
                    std::fabs(force(0)) /
                    std::fabs(force(2)*_contactsPoint[i].frictionCoef);
                ratioFrictionY = 
                    std::fabs(force(1)) /
                    std::fabs(force(2)*_contactsPoint[i].frictionCoef);
            }
            container.insert(std::make_pair(
                "frictionX_" + _contactsPoint[i].frameName,
                ratioFrictionX));
            container.insert(std::make_pair(
                "frictionY_" + _contactsPoint[i].frameName,
                ratioFrictionY));
            container.insert(std::make_pair(
                "forceNormal_" + _contactsPoint[i].frameName,
                force(2)/sumNormalForce));
        } 
    }

    return container;
}

void SEIKORetargeting::buildStaticIDInequalities(
    Eigen::MatrixXd& problemIneqMat,
    Eigen::VectorXd& problemIneqVec)
{
    //Define problem sizes
    size_t sizeIDSol = 
        //Joint torques
        _sizeJoint +
        //Plane contact wrenches
        6*_sizePlaneOn +
        //Point contact forces
        3*_sizePointOn;
    size_t sizeIDIneq =
        //Joint torques bounds
        2*_sizeJoint + 
        //Contact plane constraints
        18*_sizePlaneOn +
        //Contact point constraints
        6*_sizePointOn;

    //Matrix and vector reset
    if (
        (size_t)problemIneqMat.rows() == sizeIDIneq && 
        (size_t)problemIneqMat.cols() == sizeIDSol
    ) {
        problemIneqMat.setZero();
    } else {
        problemIneqMat = Eigen::MatrixXd::Zero(sizeIDIneq, sizeIDSol);
    }
    if ((size_t)problemIneqVec.size() == sizeIDIneq) {
        problemIneqVec.setZero();
    } else {
        problemIneqVec = Eigen::VectorXd::Zero(sizeIDIneq);
    }

    //Build inequality constraints
    size_t offsetRow = 0;
    size_t offsetCol = 0;
    //Joint torque limits
    for (size_t i=0;i<_sizeJoint;i++) {
        problemIneqMat(offsetRow+0, offsetCol) = 1.0;
        problemIneqMat(offsetRow+1, offsetCol) = -1.0;
        problemIneqVec(offsetRow+0) = _jointLimitTauAbs(i);
        problemIneqVec(offsetRow+1) = _jointLimitTauAbs(i);
        offsetRow += 2;
        offsetCol += 1;
    }
    //Contact plane constraints
    for (size_t i=0;i<_contactsPlane.size();i++) {
        if (_contactsPlane[i].isEnabled) {
            writeContactConstraints(
                problemIneqMat, problemIneqVec,
                true, i,
                offsetRow, offsetCol);
            offsetRow += 18;
            offsetCol += 6;
        }
    }
    //Contact point constraints
    for (size_t i=0;i<_contactsPoint.size();i++) {
        if (_contactsPoint[i].isEnabled) {
            writeContactConstraints(
                problemIneqMat, problemIneqVec,
                false, i,
                offsetRow, offsetCol);
            offsetRow += 6;
            offsetCol += 3;
        }
    }
}

SEIKORetargeting::ContactBase_t& SEIKORetargeting::getContactBase(
    const std::string& frameName)
{
    if (_mappingContacts.count(frameName) == 0) {
        throw std::logic_error(
            "inria::SEIKORetargeting::getContactBase: "
            "Unknown contact frame name: "
            + frameName);
    }

    int index = _mappingContacts.at(frameName);
    if (index > 0) {
        return _contactsPlane.at(index-1);
    } else {
        return _contactsPoint.at(-index-1);
    }
}
SEIKORetargeting::ContactPlane_t& SEIKORetargeting::getContactPlane(
    const std::string& frameName)
{
    if (_mappingContacts.count(frameName) == 0) {
        throw std::logic_error(
            "inria::SEIKORetargeting::getContactPlane: "
            "Unknown contact frame name: "
            + frameName);
    }

    int index = _mappingContacts.at(frameName);
    if (index > 0) {
        return _contactsPlane.at(index-1);
    } else {
        throw std::logic_error(
            "inria::SEIKORetargeting::getContactPlane: "
            "Contact is not plane: "
            + frameName);
    }
}
SEIKORetargeting::ContactPoint_t& SEIKORetargeting::getContactPoint(
    const std::string& frameName)
{
    if (_mappingContacts.count(frameName) == 0) {
        throw std::logic_error(
            "inria::SEIKORetargeting::getContactPoint: "
            "Unknown contact frame name: "
            + frameName);
    }

    int index = _mappingContacts.at(frameName);
    if (index > 0) {
        throw std::logic_error(
            "inria::SEIKORetargeting::getContactPoint: "
            "Contact is not point: "
            + frameName);
    } else {
        return _contactsPoint.at(-index-1);
    }
}

void SEIKORetargeting::writeContactConstraints(
    Eigen::MatrixXd& problemIneqMat,
    Eigen::VectorXd& problemIneqVec,
    bool isContactPlane,
    size_t indexContact,
    size_t offsetRow,
    size_t offsetCol)
{
    //Check input matrix and vector sizes
    if (
        (isContactPlane && offsetRow+18 > (size_t)problemIneqMat.rows()) ||
        (isContactPlane && offsetRow+18 > (size_t)problemIneqVec.size()) ||
        (isContactPlane && offsetCol+6 > (size_t)problemIneqMat.cols()) ||
        (!isContactPlane && offsetRow+6 > (size_t)problemIneqMat.rows()) ||
        (!isContactPlane && offsetRow+6 > (size_t)problemIneqVec.size()) ||
        (!isContactPlane && offsetCol+3 > (size_t)problemIneqMat.cols())
    ) {
        throw std::logic_error(
            "inria::SEIKORetargeting::writeContactConstraints: "
            "Invalid input matrix or vector sizes.");
    }
    //Check contact container index
    if (
        (isContactPlane && indexContact >= _contactsPlane.size()) ||
        (!isContactPlane && indexContact >= _contactsPoint.size())
    ) {
        throw std::logic_error(
            "inria::SEIKORetargeting::writeContactConstraints: "
            "Invalid contact index: " 
            + std::to_string(indexContact));
    }

    //Retrieve limits
    double normalForceMin = 0.0;
    double normalForceMax = 0.0;
    double frictionCoef = 0.0;
    double copMaxX = 0.0;
    double copMaxY = 0.0;
    if (isContactPlane) {
        normalForceMin = 
            _contactsPlane[indexContact].normalForceMin;
        normalForceMax = 
            _contactsPlane[indexContact].normalForceMax;
        frictionCoef = 
            _contactsPlane[indexContact].frictionCoef;
        copMaxX = 
            _contactsPlane[indexContact].limitCOP.x();
        copMaxY = 
            _contactsPlane[indexContact].limitCOP.y();
    } else {
        normalForceMin = 
            _contactsPoint[indexContact].normalForceMin;
        normalForceMax = 
            _contactsPoint[indexContact].normalForceMax;
        frictionCoef = 
            _contactsPoint[indexContact].frictionCoef;
    }
    double frictionCoefX = frictionCoef;
    double frictionCoefY = frictionCoef;

    //Contact plane constraints
    if (isContactPlane) {
        //Normal contact force bounds
        problemIneqMat(offsetRow+0, offsetCol+5) = 1.0;
        problemIneqVec(offsetRow+0) = -normalForceMin;
        problemIneqMat(offsetRow+1, offsetCol+5) = -1.0;
        problemIneqVec(offsetRow+1) = normalForceMax;
        //Contact friction pyramid
        problemIneqMat(offsetRow+2, offsetCol+3) = 1.0;
        problemIneqMat(offsetRow+2, offsetCol+5) = frictionCoefX;
        problemIneqMat(offsetRow+3, offsetCol+4) = 1.0;
        problemIneqMat(offsetRow+3, offsetCol+5) = frictionCoefY;
        problemIneqMat(offsetRow+4, offsetCol+3) = -1.0;
        problemIneqMat(offsetRow+4, offsetCol+5) = frictionCoefX;
        problemIneqMat(offsetRow+5, offsetCol+4) = -1.0;
        problemIneqMat(offsetRow+5, offsetCol+5) = frictionCoefY;
        //Plane center of pressure bounds
        problemIneqMat(offsetRow+6, offsetCol+1) = -1.0;
        problemIneqMat(offsetRow+6, offsetCol+5) = copMaxX;
        problemIneqMat(offsetRow+7, offsetCol+0) = 1.0;
        problemIneqMat(offsetRow+7, offsetCol+5) = copMaxY;
        problemIneqMat(offsetRow+8, offsetCol+1) = 1.0;
        problemIneqMat(offsetRow+8, offsetCol+5) = copMaxX;
        problemIneqMat(offsetRow+9, offsetCol+0) = -1.0;
        problemIneqMat(offsetRow+9, offsetCol+5) = copMaxY;
        //Plane torsional torque bounds
        //
        //Inequality over contact torque yaw no friction bounds.
        //See Stéphane Caron paper: 
        //"Stability of Surface Contacts for Humanoid Robots:
        //Closed-Form Formulae of the Contact Wrench Cone
        //for Rectangular Support Areas"
        //
        //tau_min <= tau_z <= tau_max
        //tau_min = -mu.(X+Y).f_z + |Y.f_x - mu.tau_x| + |X.f_y - mu.tau_y|
        //tau_max = +mu.(X+Y).f_z - |Y.f_x + mu.tau_x| - |X.f_y + mu.tau_y|
        //
        //Minimum bound
        problemIneqMat.block(offsetRow+10, offsetCol+0, 1, 6) <<
            frictionCoef, frictionCoef, 1.0, 
            -copMaxY, -copMaxX, frictionCoef*(copMaxX+copMaxY);
        problemIneqMat.block(offsetRow+11, offsetCol+0, 1, 6) <<
            frictionCoef, -frictionCoef, 1.0, 
            -copMaxY, copMaxX, frictionCoef*(copMaxX+copMaxY);
        problemIneqMat.block(offsetRow+12, offsetCol+0, 1, 6) <<
            -frictionCoef, -frictionCoef, 1.0, 
            copMaxY, copMaxX, frictionCoef*(copMaxX+copMaxY);
        problemIneqMat.block(offsetRow+13, offsetCol+0, 1, 6) <<
            -frictionCoef, frictionCoef, 1.0, 
            copMaxY, -copMaxX, frictionCoef*(copMaxX+copMaxY);
        //Maximum bound
        problemIneqMat.block(offsetRow+14, offsetCol+0, 1, 6) <<
            -frictionCoef, -frictionCoef, -1.0, 
            -copMaxY, -copMaxX, frictionCoef*(copMaxX+copMaxY);
        problemIneqMat.block(offsetRow+15, offsetCol+0, 1, 6) <<
            -frictionCoef, frictionCoef, -1.0, 
            -copMaxY, copMaxX, frictionCoef*(copMaxX+copMaxY);
        problemIneqMat.block(offsetRow+16, offsetCol+0, 1, 6) <<
            frictionCoef, frictionCoef, -1.0, 
            copMaxY, copMaxX, frictionCoef*(copMaxX+copMaxY);
        problemIneqMat.block(offsetRow+17, offsetCol+0, 1, 6) <<
            frictionCoef, -frictionCoef, -1.0, 
            copMaxY, -copMaxX, frictionCoef*(copMaxX+copMaxY);
    } 
    //Contact point constraints
    else {
        //Normal contact force bounds
        problemIneqMat(offsetRow+0, offsetCol+2) = 1.0;
        problemIneqVec(offsetRow+0) = -normalForceMin;
        problemIneqMat(offsetRow+1, offsetCol+2) = -1.0;
        problemIneqVec(offsetRow+1) = normalForceMax;
        //Contact friction pyramid
        problemIneqMat(offsetRow+2, offsetCol+0) = 1.0;
        problemIneqMat(offsetRow+2, offsetCol+2) = frictionCoefX;
        problemIneqMat(offsetRow+3, offsetCol+1) = 1.0;
        problemIneqMat(offsetRow+3, offsetCol+2) = frictionCoefY;
        problemIneqMat(offsetRow+4, offsetCol+0) = -1.0;
        problemIneqMat(offsetRow+4, offsetCol+2) = frictionCoefX;
        problemIneqMat(offsetRow+5, offsetCol+1) = -1.0;
        problemIneqMat(offsetRow+5, offsetCol+2) = frictionCoefY;
    }
}

void SEIKORetargeting::writePoseTarget(
    double dt,
    const ContactBase_t& contactRef,
    Eigen::MatrixXd& costMat,
    Eigen::VectorXd& costVec,
    Eigen::DiagonalMatrix<double, Eigen::Dynamic>& weights,
    size_t offsetRow,
    bool isAddPosition)
{
    const size_t& frameId = contactRef.frameId;
    //Compute clamped Cartesian orientation error vector
    Eigen::Vector3d deltaMat = ClampVectorNorm(
        MatrixToAxis(
            contactRef.targetMat
            * _model->orientation(frameId, 0).transpose()), 
        contactRef.clampMat);
    //Position and orientation case
    if (isAddPosition) {
        //Compute clamped Cartesian position 
        Eigen::Vector3d deltaPos = ClampVectorNorm(
            contactRef.targetPos - _model->position(frameId, 0), 
            contactRef.clampPos);
        //Assign matrix, vector and weight
        costMat.block(offsetRow, 0, 6, _sizeDOF) =
            contactRef.jacobianWorld;
        costVec.segment(offsetRow+0, 3) = (1.0/dt)*deltaMat;
        costVec.segment(offsetRow+3, 3) = (1.0/dt)*deltaPos;
        weights.diagonal().segment(offsetRow+0, 3) = 
            contactRef.weightMat*Eigen::VectorXd::Ones(3);
        weights.diagonal().segment(offsetRow+3, 3) = 
            contactRef.weightPos*Eigen::VectorXd::Ones(3);
    }
    //Only orientation case
    else {
        //Assign matrix, vector and weight
        costMat.block(offsetRow, 0, 3, _sizeDOF) =
            contactRef.jacobianWorld.block(0, 0, 3, _sizeDOF);
        costVec.segment(offsetRow+0, 3) = (1.0/dt)*deltaMat;
        weights.diagonal().segment(offsetRow+0, 3) = 
            contactRef.weightMat*Eigen::VectorXd::Ones(3);
    }
}

}

