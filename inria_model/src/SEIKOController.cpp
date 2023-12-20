#include <inria_model/SEIKOController.hpp>
#include <eiquadprog/eiquadprog.hpp>
#include <inria_maths/Angle.h>
#include <inria_maths/AxisAngle.h>
#include <inria_maths/Clamp.h>
#include <inria_maths/IntegrateDOFVect.h>

namespace inria {
        
SEIKOController::SEIKOController() :
    _model(nullptr),
    _pinocchio(),
    _contactPoint(),
    _contactPlane(),
    _mappingPoint(),
    _mappingPlane(),
    _configuration()
{
}

SEIKOController::SEIKOController(Model& model) :
    _model(&model),
    _pinocchio(model),
    _contactPoint(),
    _contactPlane(),
    _mappingPoint(),
    _mappingPlane(),
    _configuration()
{
    _configuration.isIntegrating = true;
    _configuration.gainPose = 1.0;
    _configuration.weightRegBase = 1e-6;
    _configuration.weightElasticEnergy = 1.0;
    _configuration.weightRegCmdChange = 1e1*Eigen::VectorXd::Ones(_model->sizeJoint());
    _configuration.weightRegCmdOffset = 1e1*Eigen::VectorXd::Ones(_model->sizeJoint());
    _configuration.stiffnessJoint = Eigen::VectorXd::Ones(_model->sizeJoint());
    _configuration.limitPosLowerJoint = _model->jointLimitsLower();
    _configuration.limitPosUpperJoint = _model->jointLimitsUpper();
    _configuration.limitPosCmdOffset = M_PI;
    _configuration.limitTauJoint = _model->jointLimitsTorque();
    _configuration.limitRatioStiffness = 0.5;
    _configuration.statePosture = Eigen::VectorXd::Zero(_model->sizeVectPos());
    _configuration.cmdOffsetJoint = Eigen::VectorXd::Zero(_model->sizeJoint());
    _configuration.stateStiffnessJoint = _configuration.stiffnessJoint;
    _configuration.stateExternalWrench = Eigen::Vector6d::Zero();
    _configuration.targetPosture = Eigen::VectorXd::Zero(_model->sizeVectPos());
    _configuration.solutionPosture = Eigen::VectorXd::Zero(_model->sizeVectVel());
    _configuration.solutionCommand = Eigen::VectorXd::Zero(_model->sizeJoint());
    _configuration.commandJoint = Eigen::VectorXd::Zero(_model->sizeJoint());
    _configuration.torqueJoint = Eigen::VectorXd::Zero(_model->sizeJoint());
    _configuration.solutionStiffnessJoint = Eigen::VectorXd::Zero(_model->sizeJoint());
    _configuration.solutionExternalWrench = Eigen::Vector6d::Zero();
    _configuration.biasBase = 0.0;
    _configuration.biasJoint = 0.0;
    reset();
}

void SEIKOController::addContactPoint(const std::string& frameName)
{
    //Check if given name is already registered
    if (_mappingPoint.count(frameName) != 0) {
        throw std::logic_error(
            "inria::SEIKOController::addContactPoint: "
            "Already defined frame name: " 
            + frameName);
    }

    _contactPoint.push_back(ContactPoint_t{
        _model->getIndexFrame(frameName),
        frameName,
        1.0*Eigen::Vector3d::Ones(),
        1.0,
        1.0,
        false,
        Eigen::Matrix3d::Identity(),
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero(),
        Eigen::Matrix3d::Identity(),
        Eigen::MatrixXd(),
        Eigen::Vector3d::Zero(),
    });
    _mappingPoint.insert(std::make_pair(frameName, _contactPoint.size()-1));
}
void SEIKOController::addContactPlane(const std::string& frameName)
{
    //Check if given name is already registered
    if (_mappingPlane.count(frameName) != 0) {
        throw std::logic_error(
            "inria::SEIKOController::addContactPlane: "
            "Already defined frame name: " 
            + frameName);
    }
    
    _contactPlane.push_back(ContactPlane_t{
        _model->getIndexFrame(frameName),
        frameName,
        1.0*Eigen::Vector6d::Ones(),
        1.0, 1.0,
        false,
        Eigen::Vector6d::Zero(),
        Eigen::Vector6d::Zero(),
        Eigen::Vector3d::Zero(),
        Eigen::Matrix3d::Identity(),
        Eigen::MatrixXd(),
        Eigen::MatrixXd(),
        Eigen::Vector6d::Zero(),
    });
    _mappingPlane.insert(std::make_pair(frameName, _contactPlane.size()-1));
}
        
void SEIKOController::reset()
{
    for (size_t i=0;i<_contactPoint.size();i++) {
        _contactPoint[i].isEnabled = false;
        _contactPoint[i].contactMat.setIdentity();
        _contactPoint[i].targetDeltaForce.setZero();
        _contactPoint[i].stateForce.setZero();
        _contactPoint[i].targetPos = 
            _model->position(_contactPoint[i].frameId, 0);
        _contactPoint[i].targetMat = 
            _model->orientation(_contactPoint[i].frameId, 0);
        _contactPoint[i].solutionForce.setZero();
    }
    for (size_t i=0;i<_contactPlane.size();i++) {
        _contactPlane[i].isEnabled = false;
        _contactPlane[i].targetDeltaWrench.setZero();
        _contactPlane[i].stateWrench.setZero();
        _contactPlane[i].targetPos = 
            _model->position(_contactPlane[i].frameId, 0);
        _contactPlane[i].targetMat = 
            _model->orientation(_contactPlane[i].frameId, 0);
        _contactPlane[i].solutionWrench.setZero();
    }
    _configuration.statePosture = _model->getDOFPosVect();
    _configuration.cmdOffsetJoint.setZero();
    _configuration.stateStiffnessJoint = _configuration.stiffnessJoint;
    _configuration.stateExternalWrench.setZero();
    _configuration.targetPosture = _model->getDOFPosVect();
    _configuration.solutionPosture.setZero();
    _configuration.solutionCommand.setZero();
    _configuration.commandJoint = _model->getJointPosVect();
    _configuration.torqueJoint.setZero();
    _configuration.solutionStiffnessJoint.setZero();
    _configuration.solutionExternalWrench.setZero();
    _configuration.biasBase = 0.0;
    _configuration.biasJoint = 0.0;
}
        
void SEIKOController::askSwitchingEnable(const std::string& frameName)
{
    if (
        _mappingPoint.count(frameName) != 0 &&
        _mappingPlane.count(frameName) != 0
    ) {
        throw std::logic_error(
            "inria::SEIKOController::askSwitchingEnable: "
            "Contact does not exist: " + frameName);
    }
    
    //Retrieve desired posture from model
    Eigen::VectorXd tmpPosture = _model->getDOFPosVect();
    
    //Assign integrated state to the model
    _model->setDOFPosVect(_configuration.statePosture);
    _model->updateState();

    //Save predicted deflected position and orientation at switching
    if (_mappingPoint.count(frameName) != 0) {
        ContactPoint_t& ref = getPoint(frameName);
        if (!ref.isEnabled) {
            ref.isEnabled = true;
            ref.targetPos = _model->position(ref.frameId, 0);
        }
    }
    if (_mappingPlane.count(frameName) != 0) {
        ContactPlane_t& ref = getPlane(frameName);
        if (!ref.isEnabled) {
            ref.isEnabled = true;
            ref.targetPos = _model->position(ref.frameId, 0);
            ref.targetMat = _model->orientation(ref.frameId, 0);
        }
    }
    
    //Restore back the desired posture to the model
    _model->setDOFPosVect(tmpPosture);
    _model->updateState();
}

const SEIKOController::ContactPoint_t& SEIKOController::getPoint(
    const std::string& frameName) const
{
    if (_mappingPoint.count(frameName) == 0) {
        throw std::logic_error(
            "inria::SEIKOController::getPoint: "
            "Contact does not exist: " + frameName);
    }
    return _contactPoint[_mappingPoint.at(frameName)];
}
SEIKOController::ContactPoint_t& SEIKOController::getPoint(
    const std::string& frameName)
{
    if (_mappingPoint.count(frameName) == 0) {
        throw std::logic_error(
            "inria::SEIKOController::getPoint: "
            "Contact does not exist: " + frameName);
    }
    return _contactPoint[_mappingPoint.at(frameName)];
}
const SEIKOController::ContactPlane_t& SEIKOController::getPlane(
    const std::string& frameName) const
{
    if (_mappingPlane.count(frameName) == 0) {
        throw std::logic_error(
            "inria::SEIKOController::getPlane: "
            "Contact does not exist: " + frameName);
    }
    return _contactPlane[_mappingPlane.at(frameName)];
}
SEIKOController::ContactPlane_t& SEIKOController::getPlane(
    const std::string& frameName)
{
    if (_mappingPlane.count(frameName) == 0) {
        throw std::logic_error(
            "inria::SEIKOController::getPlane: "
            "Contact does not exist: " + frameName);
    }
    return _contactPlane[_mappingPlane.at(frameName)];
}

const SEIKOController::Configuration_t& 
SEIKOController::getConfiguration() const
{
    return _configuration;
}
SEIKOController::Configuration_t& SEIKOController::getConfiguration()
{
    return _configuration;
}

bool SEIKOController::runInit()
{
    //Retrieve desired posture from model
    _configuration.targetPosture = _model->getDOFPosVect();
    
    //Assign integrated state to the model
    _model->setDOFPosVect(_configuration.statePosture);
    _model->updateState();
    
    //Retrieve sizes
    size_t sizeJoint = _model->sizeJoint();
    size_t sizeDOF = 6 + sizeJoint;
    size_t sizePointOn = 0;
    size_t sizePlaneOn = 0;
    for (size_t i=0;i<_contactPoint.size();i++) {
        if (_contactPoint[i].isEnabled) {
            sizePointOn++;
        } 
    }
    for (size_t i=0;i<_contactPlane.size();i++) {
        if (_contactPlane[i].isEnabled) {
            sizePlaneOn++;
        } 
    }
    size_t sizeSolWrench = 3*sizePointOn + 6*sizePlaneOn;
    
    //Problem sizes
    //Decision variables
    size_t sizeSol = 
        //Floating base and joint position change
        sizeDOF +
        //Enabled points force and planes wrench change
        sizeSolWrench;
    //Cost function
    size_t sizeCost = 
        //Floating base change regularization
        6 +
        //Elastic energy minimization
        sizeJoint +
        //Enabled point force and plane wrench regularization
        sizeSolWrench;
    //Equality constraint
    size_t sizeEq = 
        //No motion for all enabled points and planes 
        //contact effectors with bias corrective term
        3*sizePointOn + 6*sizePlaneOn +
        //Upper and lower part of differentiated equilibrium 
        //equation with bias corrective term
        sizeDOF;
    //Inequality constraints
    size_t sizeIneq = 0;
    
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
    
    //Compute resulting joint command
    _configuration.commandJoint =
        _configuration.targetPosture.segment(6,sizeJoint)
        + _configuration.cmdOffsetJoint;
    
    //Compute Jacobian matrices
    for (size_t i=0;i<_contactPoint.size();i++) {
        _contactPoint[i].jacWorld = _model->pointJacobian(
            _contactPoint[i].frameId, 0);
    }
    for (size_t i=0;i<_contactPlane.size();i++) {
        _contactPlane[i].jacBody = _model->pointJacobian(
            _contactPlane[i].frameId, _contactPlane[i].frameId);
        _contactPlane[i].jacWorld = _model->pointJacobian(
            _contactPlane[i].frameId, 0);
    }

    //Update Pinocchio kinematics
    _pinocchio.updateKinematics(_model->getDOFPosVect());

    //Build cost
    size_t offsetCostRow = 0;
    //Floating base change regularization
    costMat.block(offsetCostRow,0,6,6) = 
        Eigen::MatrixXd::Identity(6,6);
    costVec.segment(offsetCostRow,6) = 
        Eigen::VectorXd::Zero(6);
    weights.diagonal().segment(offsetCostRow,6) = 
        _configuration.weightRegBase*Eigen::VectorXd::Ones(6);
    offsetCostRow += 6;
    //Elastic energy minimization
    costMat.block(offsetCostRow,6,sizeJoint,sizeJoint) = 
        Eigen::MatrixXd::Identity(sizeJoint,sizeJoint);
    costVec.segment(offsetCostRow,sizeJoint) = 
        _configuration.commandJoint
        - _model->getJointPosVect();
    weights.diagonal().segment(offsetCostRow,sizeJoint) = 
        _configuration.weightElasticEnergy*_configuration.stateStiffnessJoint;
    offsetCostRow += sizeJoint;
    //Enabled point force and plane wrench regularization
    size_t offsetCostCol = sizeDOF;
    for (size_t i=0;i<_contactPoint.size();i++) {
        if (_contactPoint[i].isEnabled) {
            costMat.block(offsetCostRow,offsetCostCol,3,3) = 
                Eigen::MatrixXd::Identity(3,3);
            costVec.segment(offsetCostRow,3) = 
                Eigen::Vector3d::Zero() - _contactPoint[i].stateForce;
            weights.diagonal().segment(offsetCostRow,3) = 
                _contactPoint[i].weightForce;
            offsetCostRow += 3;
            offsetCostCol += 3;
        }
    }
    for (size_t i=0;i<_contactPlane.size();i++) {
        if (_contactPlane[i].isEnabled) {
            costMat.block(offsetCostRow,offsetCostCol,6,6) = 
                Eigen::MatrixXd::Identity(6,6);
            costVec.segment(offsetCostRow,6) = 
                Eigen::Vector6d::Zero() - _contactPlane[i].stateWrench;
            weights.diagonal().segment(offsetCostRow,6) = 
                _contactPlane[i].weightWrench;
            offsetCostRow += 6;
            offsetCostCol += 6;
        }
    }
    
    //Build equality constraints
    size_t offsetEqRow = 0;
    //No motion for enabled effectors
    for (size_t i=0;i<_contactPoint.size();i++) {
        if (_contactPoint[i].isEnabled) {
            const size_t& frameId = _contactPoint[i].frameId;
            Eigen::Vector3d deltaPos = 
                _contactPoint[i].targetPos 
                - _model->position(frameId, 0);
            problemEqMat.block(offsetEqRow,0,3,sizeDOF) = 
                _contactPoint[i].jacWorld.block(3,0,3,sizeDOF);
            problemEqVec.segment(offsetEqRow,3) = -deltaPos;
            offsetEqRow += 3;
        }
    }
    for (size_t i=0;i<_contactPlane.size();i++) {
        if (_contactPlane[i].isEnabled) {
            const size_t& frameId = _contactPlane[i].frameId;
            Eigen::Vector3d deltaMat = MatrixToAxis(
                _contactPlane[i].targetMat
                * _model->orientation(frameId, 0).transpose());
            Eigen::Vector3d deltaPos = 
                _contactPlane[i].targetPos
                - _model->position(frameId, 0);
            problemEqMat.block(offsetEqRow,0,6,sizeDOF) = 
                _contactPlane[i].jacWorld.block(0,0,6,sizeDOF);
            problemEqVec.segment(offsetEqRow+0,3) = -deltaMat;
            problemEqVec.segment(offsetEqRow+3,3) = -deltaPos;
            offsetEqRow += 6;
        }
    }
    //Differentiated static equation of motion
    size_t offsetEqCol = sizeDOF;
    for (size_t i=0;i<_contactPoint.size();i++) {
        if (_contactPoint[i].isEnabled) {
            problemEqMat.block(offsetEqRow,0,sizeDOF,sizeDOF) -= 
                _pinocchio.diffHessianForceInWorldProduct(
                    _contactPoint[i].frameName, 
                    (_contactPoint[i].contactMat*_contactPoint[i].stateForce));
            problemEqMat.block(offsetEqRow,offsetEqCol,sizeDOF,3) -= 
                (_contactPoint[i].jacWorld.block(3,0,3,sizeDOF).transpose()
                * _contactPoint[i].contactMat);
            problemEqVec.segment(offsetEqRow,sizeDOF) -=
                (_contactPoint[i].jacWorld.block(3,0,3,sizeDOF).transpose()
                * _contactPoint[i].contactMat
                * _contactPoint[i].stateForce);
            offsetEqCol += 3;
        }
    }
    for (size_t i=0;i<_contactPlane.size();i++) {
        if (_contactPlane[i].isEnabled) {
            problemEqMat.block(offsetEqRow,0,sizeDOF,sizeDOF) -= 
                _pinocchio.diffHessianWrenchInLocalProduct(
                    _contactPlane[i].frameName, 
                    _contactPlane[i].stateWrench);
            problemEqMat.block(offsetEqRow,offsetEqCol,sizeDOF,6) -= 
                _contactPlane[i].jacBody.transpose();
            problemEqVec.segment(offsetEqRow,sizeDOF) -= 
                _contactPlane[i].jacBody.transpose()
                * _contactPlane[i].stateWrench;
            offsetEqCol += 6;
        }
    }
    problemEqMat.block(offsetEqRow,0,sizeDOF,sizeDOF) += 
        _pinocchio.diffGravityVector();
    problemEqMat.block(offsetEqRow+6,6,sizeJoint,sizeJoint) +=
        _configuration.stateStiffnessJoint.asDiagonal();
    problemEqVec.segment(offsetEqRow,sizeDOF) += 
        _pinocchio.gravityVector();
    problemEqVec.segment(offsetEqRow+6,sizeJoint) -=
        (_configuration.stateStiffnessJoint.array()*
        (_configuration.commandJoint - _model->getJointPosVect()).array()).matrix();
    problemEqVec.segment(offsetEqRow,6) -= 
        _configuration.stateExternalWrench;
    _configuration.biasBase = problemEqVec.segment(offsetEqRow+0,6).norm();
    _configuration.biasJoint = problemEqVec.segment(offsetEqRow+6,sizeJoint).norm();
    offsetEqRow += sizeDOF;
    
    //Build inequality constraints 
    //None
    
    //Build the weighted distance of linear target costs
    problemCostMat = costMat.transpose()*weights*costMat;
    problemCostVec = -costMat.transpose()*weights*costVec;

    //Solve the QP problem
    Eigen::VectorXi activeSet;
    size_t activeSetSize;
    double cost = eiquadprog::solvers::solve_quadprog(
        problemCostMat,
        problemCostVec,
        problemEqMat.transpose(),
        problemEqVec,
        problemIneqMat.transpose(),
        problemIneqVec,
        problemSolution,
        activeSet, activeSetSize);
    bool isSuccess = !(std::isnan(cost) || std::isinf(cost));
    if (!isSuccess) {
        throw std::logic_error(
            "inria::SEIKOController::runInit: QP failed.");
    }

    //Retrieve solution
    _configuration.solutionPosture = problemSolution.segment(0,sizeDOF);
    size_t offsetSolRow = sizeDOF;
    for (size_t i=0;i<_contactPoint.size();i++) {
        if (_contactPoint[i].isEnabled) {
            _contactPoint[i].solutionForce = 
                problemSolution.segment(offsetSolRow,3);
            offsetSolRow += 3;
        } else {
            _contactPoint[i].solutionForce.setZero();
        }
    }
    for (size_t i=0;i<_contactPlane.size();i++) {
        if (_contactPlane[i].isEnabled) {
            _contactPlane[i].solutionWrench = 
                problemSolution.segment(offsetSolRow,6);
            offsetSolRow += 6;
        } else {
            _contactPlane[i].solutionWrench.setZero();
        }
    }
    _configuration.solutionCommand.setZero();
    _configuration.solutionStiffnessJoint.setZero();
    _configuration.solutionExternalWrench.setZero();

    //Integrate solution state
    _configuration.statePosture = IntegrateDOFVect(
        _model->getDOFPosVect(), _configuration.solutionPosture);
    for (size_t i=0;i<_contactPoint.size();i++) {
        _contactPoint[i].stateForce += 
            _contactPoint[i].solutionForce;
        if (!_contactPoint[i].isEnabled) {
            _contactPoint[i].stateForce.setZero();
        }
    }
    for (size_t i=0;i<_contactPlane.size();i++) {
        _contactPlane[i].stateWrench += 
            _contactPlane[i].solutionWrench;
        if (!_contactPlane[i].isEnabled) {
            _contactPlane[i].stateWrench.setZero();
        }
    }

    //Compute resulting joint torque
    _configuration.torqueJoint = 
        (_configuration.stateStiffnessJoint.array()*
        (_configuration.commandJoint - _configuration.statePosture.segment(6,sizeJoint)).array()).matrix();
    
    //Restore back the desired posture to the model
    _model->setDOFPosVect(_configuration.targetPosture);
    _model->updateState();

    return true;
}

bool SEIKOController::runSEIKO(double dt)
{
    //Retrieve desired posture from model
    _configuration.targetPosture = _model->getDOFPosVect();
    
    //Compute effectors desired pose in world frame
    //(before model state update) for disabled effectors
    for (size_t i=0;i<_contactPoint.size();i++) {
        if (!_contactPoint[i].isEnabled) {
            _contactPoint[i].targetPos = 
                _model->position(_contactPoint[i].frameId, 0);
        }
        _contactPoint[i].targetMat = 
            _model->orientation(_contactPoint[i].frameId, 0);
    }
    for (size_t i=0;i<_contactPlane.size();i++) {
        if (!_contactPlane[i].isEnabled) {
            _contactPlane[i].targetPos = 
                _model->position(_contactPlane[i].frameId, 0);
            _contactPlane[i].targetMat = 
                _model->orientation(_contactPlane[i].frameId, 0);
        }
    }
    
    //Assign integrated state to the model
    _model->setDOFPosVect(_configuration.statePosture);
    _model->updateState();
    
    //Retrieve sizes
    size_t sizeJoint = _model->sizeJoint();
    size_t sizeDOF = 6 + sizeJoint;
    size_t sizePointOn = 0;
    size_t sizePlaneOn = 0;
    size_t sizePointOff = 0;
    size_t sizePlaneOff = 0;
    for (size_t i=0;i<_contactPoint.size();i++) {
        if (_contactPoint[i].isEnabled) {
            sizePointOn++;
        } else {
            sizePointOff++;
        }
    }
    for (size_t i=0;i<_contactPlane.size();i++) {
        if (_contactPlane[i].isEnabled) {
            sizePlaneOn++;
        } else {
            sizePlaneOff++;
        }
    }
    size_t sizeSolWrench = 3*sizePointOn + 6*sizePlaneOn;
    
    //Problem sizes
    //Decision variables
    size_t sizeSol = 
        //Floating base and joint velocity
        sizeDOF +
        //Enabled points force and planes wrench velocity
        sizeSolWrench;
    //Cost function
    size_t sizeCost = 
        //Floating base velocity regularization
        6 +
        //Enabled point force and plane wrench target
        sizeSolWrench +
        //Command offset velocity regularization
        sizeJoint +
        //Command offset regularization
        sizeJoint +
        //Pose reaching for disabled point and plane effectors
        3*sizePointOff + 6*sizePlaneOff +
        //Orientation task for enabled point contact;
        3*sizePointOn;
    //Equality constraint
    size_t sizeEq = 
        //No motion for all enabled points and planes 
        //contact effectors with bias corrective term
        3*sizePointOn + 6*sizePlaneOn +
        //Upper part of differentiated equilibrium 
        //equation with bias corrective term
        6;
    //Inequality constraints
    size_t sizeIneq =
        //Joint position bounds
        2*sizeJoint +
        //Command position offset bounds
        2*sizeJoint +
        //Joint torque bounds
        2*sizeJoint;
    
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
    
    //Compute resulting joint command
    _configuration.commandJoint =
        _configuration.targetPosture.segment(6,sizeJoint)
        + _configuration.cmdOffsetJoint;
    //Compute resulting joint torque
    _configuration.torqueJoint = 
        (_configuration.stateStiffnessJoint.array()*
        (_configuration.commandJoint - _model->getJointPosVect()).array()).matrix();
    
    //Compute Jacobian matrices
    for (size_t i=0;i<_contactPoint.size();i++) {
        _contactPoint[i].jacWorld = _model->pointJacobian(
            _contactPoint[i].frameId, 0);
    }
    for (size_t i=0;i<_contactPlane.size();i++) {
        _contactPlane[i].jacBody = _model->pointJacobian(
            _contactPlane[i].frameId, _contactPlane[i].frameId);
        _contactPlane[i].jacWorld = _model->pointJacobian(
            _contactPlane[i].frameId, 0);
    }

    //Update Pinocchio kinematics
    _pinocchio.updateKinematics(_model->getDOFPosVect());

    //Build the linear relationship from reduced solution 
    //to computed joint command offset change (times joint stiffness)
    //using the static equation of motion.
    //dt*solToTauMat*sol + solToTauVec = dt*S*K*d(cmdOffset)
    Eigen::MatrixXd solToTauMat = Eigen::MatrixXd::Zero(sizeDOF,sizeSol);
    Eigen::VectorXd solToTauVec = Eigen::VectorXd::Zero(sizeDOF);
    size_t offsetTmpCol = sizeDOF;
    //Contact points
    for (size_t i=0;i<_contactPoint.size();i++) {
        if (_contactPoint[i].isEnabled) {
            solToTauMat.block(0,0,sizeDOF,sizeDOF) -= 
                _pinocchio.diffHessianForceInWorldProduct(
                    _contactPoint[i].frameName, 
                    (_contactPoint[i].contactMat*_contactPoint[i].stateForce));
            solToTauMat.block(0,offsetTmpCol,sizeDOF,3) -= 
                (_contactPoint[i].jacWorld.block(3,0,3,sizeDOF).transpose()
                * _contactPoint[i].contactMat);
            solToTauVec.segment(0,sizeDOF) -=
                (_contactPoint[i].jacWorld.block(3,0,3,sizeDOF).transpose()
                * _contactPoint[i].contactMat
                * _contactPoint[i].stateForce);
            offsetTmpCol += 3;
        }
    }
    //Contact planes
    for (size_t i=0;i<_contactPlane.size();i++) {
        if (_contactPlane[i].isEnabled) {
            solToTauMat.block(0,0,sizeDOF,sizeDOF) -= 
                _pinocchio.diffHessianWrenchInLocalProduct(
                    _contactPlane[i].frameName, 
                    _contactPlane[i].stateWrench);
            solToTauMat.block(0,offsetTmpCol,sizeDOF,6) -= 
                _contactPlane[i].jacBody.transpose();
            solToTauVec.segment(0,sizeDOF) -= 
                _contactPlane[i].jacBody.transpose()
                * _contactPlane[i].stateWrench;
            offsetTmpCol += 6;
        }
    }
    //Joint space
    solToTauMat.block(0,0,sizeDOF,sizeDOF) += 
        _pinocchio.diffGravityVector();
    solToTauMat.block(6,6,sizeJoint,sizeJoint) +=
        _configuration.stateStiffnessJoint.asDiagonal();
    solToTauVec.segment(0,sizeDOF) += 
        _pinocchio.gravityVector();
    solToTauVec.segment(6,sizeJoint) -=
        (_configuration.stateStiffnessJoint.array()*
        (_configuration.commandJoint - _model->getJointPosVect()).array()).matrix();
    solToTauVec.segment(0,6) -= 
        _configuration.stateExternalWrench;
    _configuration.biasBase = solToTauVec.segment(0,6).norm();
    _configuration.biasJoint = 
        (_configuration.stateStiffnessJoint.asDiagonal().inverse()
            *solToTauVec.segment(6,sizeJoint)).norm();

    //Build cost
    size_t offsetCostRow = 0;
    //Floating base velocity regularization
    costMat.block(offsetCostRow,0,6,6) = 
        Eigen::MatrixXd::Identity(6,6);
    costVec.segment(offsetCostRow,6) = 
        Eigen::VectorXd::Zero(6);
    weights.diagonal().segment(offsetCostRow,6) = 
        _configuration.weightRegBase*Eigen::VectorXd::Ones(6);
    offsetCostRow += 6;
    //Enabled point force and plane wrench velocity target
    size_t offsetCostCol = sizeDOF;
    for (size_t i=0;i<_contactPoint.size();i++) {
        if (_contactPoint[i].isEnabled) {
            costMat.block(offsetCostRow,offsetCostCol,3,3) = 
                Eigen::MatrixXd::Identity(3,3);
            costVec.segment(offsetCostRow,3) = 
                _contactPoint[i].targetDeltaForce;
            weights.diagonal().segment(offsetCostRow,3) = 
                _contactPoint[i].weightForce;
            offsetCostRow += 3;
            offsetCostCol += 3;
        }
    }
    for (size_t i=0;i<_contactPlane.size();i++) {
        if (_contactPlane[i].isEnabled) {
            costMat.block(offsetCostRow,offsetCostCol,6,6) = 
                Eigen::MatrixXd::Identity(6,6);
            costVec.segment(offsetCostRow,6) = 
                _contactPlane[i].targetDeltaWrench;
            weights.diagonal().segment(offsetCostRow,6) = 
                _contactPlane[i].weightWrench;
            offsetCostRow += 6;
            offsetCostCol += 6;
        }
    }
    //Joint command offset velocity regularization
    costMat.block(offsetCostRow,0,sizeJoint,sizeSol) = 
        _configuration.stateStiffnessJoint.asDiagonal().inverse()*
        solToTauMat.block(6,0,sizeJoint,sizeSol);
    costVec.segment(offsetCostRow,sizeJoint) = 
        -(1.0/dt)*(_configuration.stateStiffnessJoint.asDiagonal().inverse()
        *solToTauVec.segment(6,sizeJoint));
    weights.diagonal().segment(offsetCostRow,sizeJoint) = 
        _configuration.weightRegCmdChange;
    offsetCostRow += sizeJoint;
    //Joint command offset regularization
    costMat.block(offsetCostRow,0,sizeJoint,sizeSol) = 
        dt*_configuration.stateStiffnessJoint.asDiagonal().inverse()*
        solToTauMat.block(6,0,sizeJoint,sizeSol);
    costVec.segment(offsetCostRow,sizeJoint) = 
        Eigen::VectorXd::Zero(sizeJoint) - _configuration.cmdOffsetJoint
        - (_configuration.stateStiffnessJoint.asDiagonal().inverse()
            *solToTauVec.segment(6,sizeJoint));
    weights.diagonal().segment(offsetCostRow,sizeJoint) = 
        _configuration.weightRegCmdOffset;
    offsetCostRow += sizeJoint;
    //Disabled effector pose target
    for (size_t i=0;i<_contactPoint.size();i++) {
        if (!_contactPoint[i].isEnabled) {
            const size_t& frameId = _contactPoint[i].frameId;
            Eigen::Vector3d deltaPos = 
                _contactPoint[i].targetPos 
                - _model->position(frameId, 0);
            costMat.block(offsetCostRow,0,3,sizeDOF) = 
                _contactPoint[i].jacWorld.block(3,0,3,sizeDOF);
            costVec.segment(offsetCostRow,3) = 
                (1.0/dt)*_configuration.gainPose*deltaPos;
            weights.diagonal().segment(offsetCostRow,3) = 
                _contactPoint[i].weightPos*Eigen::Vector3d::Ones();
            offsetCostRow += 3;
        }
    }
    for (size_t i=0;i<_contactPlane.size();i++) {
        if (!_contactPlane[i].isEnabled) {
            const size_t& frameId = _contactPlane[i].frameId;
            Eigen::Vector3d deltaMat = MatrixToAxis(
                _contactPlane[i].targetMat
                * _model->orientation(frameId, 0).transpose());
            Eigen::Vector3d deltaPos = 
                _contactPlane[i].targetPos
                - _model->position(frameId, 0);
            costMat.block(offsetCostRow,0,6,sizeDOF) = 
                _contactPlane[i].jacWorld.block(0,0,6,sizeDOF);
            costVec.segment(offsetCostRow+0,3) = 
                (1.0/dt)*_configuration.gainPose*deltaMat;
            costVec.segment(offsetCostRow+3,3) = 
                (1.0/dt)*_configuration.gainPose*deltaPos;
            weights.diagonal().segment(offsetCostRow+0,3) = 
                _contactPlane[i].weightMat*Eigen::Vector3d::Ones();
            weights.diagonal().segment(offsetCostRow+3,3) = 
                _contactPlane[i].weightPos*Eigen::Vector3d::Ones();
            offsetCostRow += 6;
        }
    }
    //Enabled point effector orientation target
    for (size_t i=0;i<_contactPoint.size();i++) {
        if (_contactPoint[i].isEnabled) {
            const size_t& frameId = _contactPoint[i].frameId;
            Eigen::Vector3d deltaMat = MatrixToAxis(
                _contactPoint[i].targetMat
                * _model->orientation(frameId, 0).transpose());
            costMat.block(offsetCostRow,0,3,sizeDOF) = 
                _contactPoint[i].jacWorld.block(0,0,3,sizeDOF);
            costVec.segment(offsetCostRow,3) = 
                (1.0/dt)*_configuration.gainPose*deltaMat;
            weights.diagonal().segment(offsetCostRow,3) = 
                _contactPoint[i].weightMat*Eigen::Vector3d::Ones();
            offsetCostRow += 3;
        }
    }
    
    //Build equality constraints
    size_t offsetEqRow = 0;
    //No motion for enabled effectors
    for (size_t i=0;i<_contactPoint.size();i++) {
        if (_contactPoint[i].isEnabled) {
            const size_t& frameId = _contactPoint[i].frameId;
            Eigen::Vector3d deltaPos = 
                _contactPoint[i].targetPos 
                - _model->position(frameId, 0);
            problemEqMat.block(offsetEqRow,0,3,sizeDOF) = 
                _contactPoint[i].jacWorld.block(3,0,3,sizeDOF);
            problemEqVec.segment(offsetEqRow,3) = -(1.0/dt)*deltaPos;
            offsetEqRow += 3;
        }
    }
    for (size_t i=0;i<_contactPlane.size();i++) {
        if (_contactPlane[i].isEnabled) {
            const size_t& frameId = _contactPlane[i].frameId;
            Eigen::Vector3d deltaMat = MatrixToAxis(
                _contactPlane[i].targetMat
                * _model->orientation(frameId, 0).transpose());
            Eigen::Vector3d deltaPos = 
                _contactPlane[i].targetPos
                - _model->position(frameId, 0);
            problemEqMat.block(offsetEqRow,0,6,sizeDOF) = 
                _contactPlane[i].jacWorld.block(0,0,6,sizeDOF);
            problemEqVec.segment(offsetEqRow+0,3) = -(1.0/dt)*deltaMat;
            problemEqVec.segment(offsetEqRow+3,3) = -(1.0/dt)*deltaPos;
            offsetEqRow += 6;
        }
    }
    //Differentiated upper static equation of motion
    problemEqMat.block(offsetEqRow,0,6,sizeSol) = solToTauMat.block(0,0,6,sizeSol);
    problemEqVec.segment(offsetEqRow,6) = (1.0/dt)*solToTauVec.segment(0,6);
    offsetEqRow += 6;
    
    //Build inequality constraints 
    size_t offsetIneq = 0;
    //Joint kinematics bounds
    const double weightIneq = 10.0;
    //Lower position
    problemIneqMat.block(offsetIneq,0,sizeJoint,sizeSol) = weightIneq*(
        dt*_configuration.stateStiffnessJoint.asDiagonal().inverse()*
        solToTauMat.block(6,0,sizeJoint,sizeSol));
    problemIneqVec.segment(offsetIneq,sizeJoint) = weightIneq*(
        (_configuration.stateStiffnessJoint.asDiagonal().inverse()
            *solToTauVec.segment(6,sizeJoint))
        + _configuration.commandJoint - _configuration.limitPosLowerJoint);
    offsetIneq += sizeJoint;
    //Upper position
    problemIneqMat.block(offsetIneq,0,sizeJoint,sizeSol) = -weightIneq*(
        dt*_configuration.stateStiffnessJoint.asDiagonal().inverse()*
        solToTauMat.block(6,0,sizeJoint,sizeSol));
    problemIneqVec.segment(offsetIneq,sizeJoint) = weightIneq*(
        -(_configuration.stateStiffnessJoint.asDiagonal().inverse()
            *solToTauVec.segment(6,sizeJoint))
        + _configuration.limitPosUpperJoint - _configuration.commandJoint);
    offsetIneq += sizeJoint;
    //Command offset position bounds
    //Lower position
    problemIneqMat.block(offsetIneq,0,sizeJoint,sizeSol) = weightIneq*(
        dt*_configuration.stateStiffnessJoint.asDiagonal().inverse()*
        solToTauMat.block(6,0,sizeJoint,sizeSol));
    problemIneqVec.segment(offsetIneq,sizeJoint) = weightIneq*(
        (_configuration.stateStiffnessJoint.asDiagonal().inverse()
            *solToTauVec.segment(6,sizeJoint))
        + _configuration.cmdOffsetJoint 
        + _configuration.limitPosCmdOffset*Eigen::VectorXd::Ones(sizeJoint));
    offsetIneq += sizeJoint;
    //Upper position
    problemIneqMat.block(offsetIneq,0,sizeJoint,sizeSol) = -weightIneq*(
        dt*_configuration.stateStiffnessJoint.asDiagonal().inverse()*
        solToTauMat.block(6,0,sizeJoint,sizeSol));
    problemIneqVec.segment(offsetIneq,sizeJoint) = weightIneq*(
        -(_configuration.stateStiffnessJoint.asDiagonal().inverse()
            *solToTauVec.segment(6,sizeJoint))
        + _configuration.limitPosCmdOffset*Eigen::VectorXd::Ones(sizeJoint) 
        - _configuration.cmdOffsetJoint);
    offsetIneq += sizeJoint;
    //Joint torque limits
    problemIneqMat.block(offsetIneq,0,sizeJoint,sizeSol) = 
        dt*weightIneq*solToTauMat.block(6,0,sizeJoint,sizeSol);
    problemIneqMat.block(offsetIneq,6,sizeJoint,sizeJoint) +=
        -dt*weightIneq*_configuration.stateStiffnessJoint.asDiagonal();
    problemIneqVec.segment(offsetIneq,sizeJoint) = weightIneq*(
        _configuration.limitTauJoint
        + solToTauVec.segment(6,sizeJoint)
        + _configuration.torqueJoint);
    offsetIneq += sizeJoint;
    problemIneqMat.block(offsetIneq,0,sizeJoint,sizeSol) = 
        -dt*weightIneq*solToTauMat.block(6,0,sizeJoint,sizeSol);
    problemIneqMat.block(offsetIneq,6,sizeJoint,sizeJoint) +=
        dt*weightIneq*_configuration.stateStiffnessJoint.asDiagonal();
    problemIneqVec.segment(offsetIneq,sizeJoint) = weightIneq*(
        _configuration.limitTauJoint
        - solToTauVec.segment(6,sizeJoint)
        - _configuration.torqueJoint);
    offsetIneq += sizeJoint;
    
    //Build the weighted distance of linear target costs
    problemCostMat = costMat.transpose()*weights*costMat;
    problemCostVec = -costMat.transpose()*weights*costVec;

    //Solve the QP problem
    Eigen::VectorXi activeSet;
    size_t activeSetSize;
    double cost = eiquadprog::solvers::solve_quadprog(
        problemCostMat,
        problemCostVec,
        problemEqMat.transpose(),
        problemEqVec,
        problemIneqMat.transpose(),
        problemIneqVec,
        problemSolution,
        activeSet, activeSetSize);
    bool isSuccess = !(std::isnan(cost) || std::isinf(cost));
    if (!isSuccess) {
        throw std::logic_error(
            "inria::SEIKOController::runSEIKO: QP failed.");
    }

    //Retrieve solution
    _configuration.solutionPosture = problemSolution.segment(0,sizeDOF);
    size_t offsetSolRow = sizeDOF;
    for (size_t i=0;i<_contactPoint.size();i++) {
        if (_contactPoint[i].isEnabled) {
            _contactPoint[i].solutionForce = 
                problemSolution.segment(offsetSolRow,3);
            offsetSolRow += 3;
        } else {
            _contactPoint[i].solutionForce.setZero();
        }
    }
    for (size_t i=0;i<_contactPlane.size();i++) {
        if (_contactPlane[i].isEnabled) {
            _contactPlane[i].solutionWrench = 
                problemSolution.segment(offsetSolRow,6);
            offsetSolRow += 6;
        } else {
            _contactPlane[i].solutionWrench.setZero();
        }
    }
    _configuration.solutionCommand = 
        _configuration.stateStiffnessJoint.asDiagonal().inverse()*
        (solToTauMat.block(6,0,sizeJoint,sizeSol)*problemSolution 
            + (1.0/dt)*solToTauVec.segment(6,sizeJoint));
    _configuration.solutionStiffnessJoint.setZero();
    _configuration.solutionExternalWrench.setZero();

    //Integrate solution state
    if (_configuration.isIntegrating) {
        _configuration.statePosture = IntegrateDOFVect(
            _model->getDOFPosVect(), dt*_configuration.solutionPosture);
        for (size_t i=0;i<_contactPoint.size();i++) {
            _contactPoint[i].stateForce += 
                dt*_contactPoint[i].solutionForce;
            if (!_contactPoint[i].isEnabled) {
                _contactPoint[i].stateForce.setZero();
            }
        }
        for (size_t i=0;i<_contactPlane.size();i++) {
            _contactPlane[i].stateWrench += 
                dt*_contactPlane[i].solutionWrench;
            if (!_contactPlane[i].isEnabled) {
                _contactPlane[i].stateWrench.setZero();
            }
        }
        _configuration.cmdOffsetJoint +=
            dt*_configuration.solutionCommand;
        for (size_t i=0;i<sizeJoint;i++) {
            _configuration.cmdOffsetJoint(i) = ClampRange(
                _configuration.cmdOffsetJoint(i), 
                -_configuration.limitPosCmdOffset, 
                _configuration.limitPosCmdOffset);
            _configuration.cmdOffsetJoint(i) = ClampRange(
                _configuration.cmdOffsetJoint(i), 
                _configuration.limitPosLowerJoint(i) - _configuration.targetPosture(6+i),
                _configuration.limitPosUpperJoint(i) - _configuration.targetPosture(6+i));
        }
    }

    //Recompute resulting joint command
    _configuration.commandJoint =
        _configuration.targetPosture.segment(6,sizeJoint)
        + _configuration.cmdOffsetJoint;
    //Compute resulting joint torque
    _configuration.torqueJoint = 
        (_configuration.stateStiffnessJoint.array()*
        (_configuration.commandJoint - _configuration.statePosture.segment(6,sizeJoint)).array()).matrix();
    
    //Restore back the desired posture to the model
    _model->setDOFPosVect(_configuration.targetPosture);
    _model->updateState();
    
    return true;
}

}

