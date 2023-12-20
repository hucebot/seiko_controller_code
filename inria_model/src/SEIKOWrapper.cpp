#include <inria_model/SEIKOWrapper.hpp>
#include <inria_maths/Angle.h>

namespace inria {

void SEIKOWrapper::setup()
{
    //Reset internal state
    _jointPosMarginCoef = Eigen::VectorXd::Ones(this->_sizeJoint);
    _vectWeightWrench.setZero();
    _vectWeightForce.setZero();
    _noFiltersAndCommands = false;
    _aheadJointPos = Eigen::VectorXd::Zero(this->_sizeJoint);
}
        
void SEIKOWrapper::disableFiltersAndCommands(bool noFiltersAndCommands)
{
    _noFiltersAndCommands = noFiltersAndCommands;
}
        
bool SEIKOWrapper::reset()
{
    //Reset SEIKO to underlying model
    this->resetTargetsFromModel();
    _aheadJointPos = _model->getJointPosVect();

    //Reset forces and wrenches target
    for (auto& it : _commands) {
        if (it.second.isPoint) {
            this->setTargetForce(it.first,
                Eigen::Vector3d::Zero());
        } else {
            this->setTargetWrench(it.first,
                Eigen::Vector6d::Zero());
        }
    }
    
    //Reset default point contact orientation
    for (auto& it : _commands) {
        if (it.second.isPoint) {
            this->setContactMat(it.first,
                this->_model->orientation(it.first, "ROOT"));
        } 
    }

    //Reset command pose filters
    //and current velocity commands
    for (auto& it : _commands) {
        it.second.isSwitchingDisable = false;
        it.second.isPushMode = false;
        it.second.isClutch = false;
        it.second.velLin.setZero();
        it.second.velAng.setZero();
        it.second.velForceNormal = 0.0;
        it.second.filterPose.reset(
            this->_model->position(it.first, "ROOT"),
            this->_model->orientation(it.first, "ROOT"));
        it.second.filterForce.reset();
        it.second.targetTorque.setZero();
        it.second.targetForce.setZero();
    }
    
    //Run static ID to reset SEIKO 
    //internal forces and wrenches
    bool isSuccess = this->runInverseDynamics();

    //Reset smoothing init ratio
    _initRatio = 0.0;

    return isSuccess;
}

void SEIKOWrapper::setTargetPose(
    const std::string& frameName,
    bool isClutch,
    const Eigen::Vector3d& rawPos, 
    const Eigen::Matrix3d& rawMat)
{
    if (_commands.count(frameName) == 0) {
        throw std::logic_error(
            "inria::SEIKOWrapper::setTargetPose: "
            "Undeclared frame name: " + frameName);
    }
    _commands.at(frameName).isClutch = isClutch;
    _commands.at(frameName).rawPos = rawPos;
    _commands.at(frameName).rawMat = rawMat;
}
        
void SEIKOWrapper::setTargetVel(
    const std::string& frameName,
    const Eigen::Vector3d& velLin, 
    const Eigen::Vector3d& velAng)
{
    if (_commands.count(frameName) == 0) {
        throw std::logic_error(
            "inria::SEIKOWrapper::setTargetVel: "
            "Undeclared frame name: " + frameName);
    }
    _commands.at(frameName).velLin = velLin;
    _commands.at(frameName).velAng = velAng;
}

void SEIKOWrapper::setPushMode(
    const std::string& frameName,
    bool isEnabled)
{
    if (_commands.count(frameName) == 0) {
        throw std::logic_error(
            "inria::SEIKOWrapper::setPushMode: "
            "Undeclared frame name: " + frameName);
    }
    EndEffector_t& contact = _commands.at(frameName);
    //Push mode disable always possible
    if (!isEnabled) {
        contact.isPushMode = false;
        contact.targetTorque.setZero();
        contact.targetForce.setZero();
        contact.filterForce.reset();
    } 
    //Only enable push mode if the contact is enabled and not being disabled
    else if (
        this->stateContactBase(frameName).isEnabled &&
        !contact.isSwitchingDisable &&
        isEnabled && 
        !contact.isPushMode
    ) {
        //Set current wrench state as target on pushing mode enabling
        if (contact.isPoint) {
            contact.targetTorque.setZero();
            contact.targetForce = this->stateContactForce(frameName);
        } else {
            Eigen::Vector6d wrench = this->stateContactWrench(frameName);
            contact.targetTorque = wrench.segment(0, 3);
            contact.targetForce = wrench.segment(3, 3);
        }
        contact.isPushMode = true;
        contact.filterForce.reset();
    }
}

void SEIKOWrapper::setForceNormalVel(
    const std::string& frameName,
    double vel)
{
    if (_commands.count(frameName) == 0) {
        throw std::logic_error(
            "inria::SEIKOWrapper::setForceNormalVel: "
            "Undeclared frame name: " + frameName);
    }
    _commands.at(frameName).velForceNormal = vel;
}

Eigen::Vector3d& SEIKOWrapper::refTargetTorque(
    const std::string& frameName)
{
    if (_commands.count(frameName) == 0) {
        throw std::logic_error(
            "inria::SEIKOWrapper::refTargetWrench: "
            "Undeclared frame name: " + frameName);
    }
    EndEffector_t& contact = _commands.at(frameName);
    if (contact.isPoint) {
        throw std::logic_error(
            "inria::SEIKOWrapper::refTargetWrench: "
            "Frame is not a plane: " + frameName);
    }
    return contact.targetTorque;
}
Eigen::Vector3d& SEIKOWrapper::refTargetForce(
    const std::string& frameName)
{
    if (_commands.count(frameName) == 0) {
        throw std::logic_error(
            "inria::SEIKOWrapper::refTargetForce: "
            "Undeclared frame name: " + frameName);
    }
    EndEffector_t& contact = _commands.at(frameName);
    return contact.targetForce;
}

void SEIKOWrapper::askSwitching(
    const std::string& frameName,
    bool isEnabled)
{
    if (_commands.count(frameName) == 0) {
        throw std::logic_error(
            "inria::SEIKOWrapper::askSwitching: "
            "Undeclared frame name: " + frameName);
    }
    EndEffector_t& contact = _commands.at(frameName);
    if (isEnabled) {
        //Ask for enabling contact
        if (
            !contact.isSwitchingDisable && 
            !this->stateContactBase(frameName).isEnabled
        ) {
            if (contact.isPoint) {
                //Reset default surface orientation
                this->setContactMat(frameName,
                    this->_model->orientation(frameName, "ROOT"));
                //Reset force weight
                this->setWeightForce(frameName,
                    contact.weightForceEnabled*_vectWeightForce);
            } else {
                //Reset wrench weight
                this->setWeightWrench(frameName,
                    contact.weightForceEnabled*_vectWeightWrench);
            }
            //Set min force to zero
            this->setNormalForceLimits(frameName,
                0.0, contact.limitNormalForceMax);
            //Toggle SEIKO contact
            this->toggleContact(frameName, true);
            //Reset push mode
            contact.isPushMode = false;
        }
    } else {
        //Ask for disabling procedure
        if (
            !contact.isSwitchingDisable && 
            this->stateContactBase(frameName).isEnabled
        ) {
            //Start switching procedure
            contact.isSwitchingDisable = true;
            if (contact.isPoint) {
                //Set hight force weight
                this->setWeightForce(frameName,
                    contact.weightForceDisabling*_vectWeightForce);
            } else {
                //Set hight wrench weight
                this->setWeightWrench(frameName,
                    contact.weightForceDisabling*_vectWeightWrench);
            }
            //Set min force to zero
            this->setNormalForceLimits(frameName,
                0.0, contact.limitNormalForceMax);
            //Reset push mode
            contact.isPushMode = false;
        }
    }
}

bool SEIKOWrapper::update(double dt_task, double dt_ahead)
{
    //Switching state update
    for (auto& it : _commands) {
        const ContactBase_t& contact = this->stateContactBase(it.first);
        double forceNormal = this->stateNormalForce(it.first);
        //For enabled contact, check if their min force limit 
        //can be raised to desired one
        if (
            !it.second.isSwitchingDisable &&
            contact.isEnabled && 
            contact.normalForceMin < it.second.limitNormalForceMin &&
            forceNormal > contact.normalForceMin+1e-2
        ) {
            this->setNormalForceLimits(
                it.first,
                std::min(forceNormal-1e-2, it.second.limitNormalForceMin), 
                it.second.limitNormalForceMax);
        }
        //Check if disabling contacts can be fully disabled
        if (
            it.second.isSwitchingDisable &&
            contact.isEnabled && 
            forceNormal < 1.0*dt_task*contact.limitVelForce
        ) {
            it.second.isSwitchingDisable = false;
            this->toggleContact(it.first, false);
        }
        //Update normal force target from velocity command
        if (it.second.isPushMode) {
            it.second.filterForce.update(
                dt_task, it.second.velForceNormal, 
                forceNormal);
            it.second.targetForce.z() = it.second.filterForce.value();
        } else {
            it.second.filterForce.reset();
        }
        //Pushing mode update wrench/force weight
        if (
            !it.second.isSwitchingDisable &&
            contact.isEnabled
        ) {
            if (it.second.isPushMode) {
                if (it.second.isPoint) {
                    this->setWeightForce(
                        it.first, 
                        it.second.weightForcePushMode*Eigen::Vector3d::Ones());
                } else {
                    this->setWeightWrench(
                        it.first, 
                        it.second.weightForcePushMode*Eigen::Vector6d::Ones());
                }
            } else {
                if (it.second.isPoint) {
                    this->setWeightForce(
                        it.first,
                        it.second.weightForceEnabled*_vectWeightForce);
                } else {
                    this->setWeightWrench(
                        it.first,
                        it.second.weightForceEnabled*_vectWeightWrench);
                }
            }
        }
        //Pushing mode update wrench/force target
        if (!_noFiltersAndCommands) {
            if (it.second.isPushMode) {
                if (it.second.isPoint) {
                    this->setTargetForce(
                        it.first, 
                        it.second.targetForce);
                } else {
                    Eigen::Vector6d wrench;
                    wrench.segment(0, 3) = it.second.targetTorque;
                    wrench.segment(3, 3) = it.second.targetForce;
                    this->setTargetWrench(
                        it.first, 
                        wrench);
                }
            } else {
                if (it.second.isPoint) {
                    this->setTargetForce(
                        it.first, 
                        Eigen::Vector3d::Zero());
                } else {
                    this->setTargetWrench(
                        it.first, 
                        Eigen::Vector6d::Zero());
                }
            }
            //Reset force velocity command
            it.second.velForceNormal = 0.0;
        }
    }

    //Update command filtering and send command to SEIKO
    if (!_noFiltersAndCommands) {
        for (auto& it : _commands) {
            const ContactBase_t& contact = this->stateContactBase(it.first);
            if (contact.isEnabled) {
                //Reset filters to current pose for enabled contact
                it.second.filterPose.reset(
                    this->refTargetPos(it.first),
                    this->refTargetMat(it.first));
            } else {
                //Update filtering and send command 
                //to SEIKO for enabled contact
                it.second.filterPose.update(
                    dt_task, 
                    it.second.isClutch,
                    it.second.rawPos, it.second.rawMat,
                    it.second.velLin, it.second.velAng,
                    this->_model->position(it.first, "ROOT"),
                    this->_model->orientation(it.first, "ROOT"));
                this->refTargetPos(it.first) = it.second.filterPose.valuePos();
                this->refTargetMat(it.first) = it.second.filterPose.valueMat();
            }
            it.second.velLin.setZero();
            it.second.velAng.setZero();
        }
    }

    //Compute SEIKO step
    bool isSuccess = this->runSEIKO(dt_task);

    if (isSuccess) {
        //Apply smoothing ratio on all (linear) computed delta
        _initRatio = ClampRange(_initRatio, 0.0, 1.0);
        this->_jointSolDOF *= _initRatio;
        this->_jointSolTau *= _initRatio;
        for (size_t i=0;i<this->_contactsPlane.size();i++) {
            this->_contactsPlane[i].solWrench *= _initRatio;
        }
        for (size_t i=0;i<this->_contactsPoint.size();i++) {
            this->_contactsPoint[i].solForce *= _initRatio;
        }
        //Update initialization smoothing
        _initRatio += dt_task/2.0;
        _initRatio = ClampRange(_initRatio, 0.0, 1.0);
        //Integrate computed deltas
        this->integrateSolution(dt_task);
        //Update underlying model
        this->_model->updateState();
        //Compute ahead command from current delta as velocity
        computeAheadJointPos(dt_ahead);
    }

    return isSuccess;
}

const Eigen::VectorXd& SEIKOWrapper::getAheadJointPos() const
{
    return _aheadJointPos;
}

const SEIKOWrapper::EndEffector_t& SEIKOWrapper::stateEffector(
    const std::string& frameName) const
{
    if (_commands.count(frameName) == 0) {
        throw std::logic_error(
            "inria::SEIKOWrapper::stateEffector: "
            "Undeclared frame name: " + frameName);
    }
    return _commands.at(frameName);
}

bool SEIKOWrapper::stateIsContactSwitching(
    const std::string& frameName) const
{
    if (_commands.count(frameName) == 0) {
        throw std::logic_error(
            "inria::SEIKOWrapper::stateIsContactSwitching: "
            "Undeclared frame name: " + frameName);
    }
    const ContactBase_t& contact = this->stateContactBase(frameName);
    const EndEffector_t& effector = _commands.at(frameName);
    bool isDisabling = 
        contact.isEnabled && 
        effector.isSwitchingDisable;
    bool isEnabling = 
        contact.isEnabled && 
        !effector.isSwitchingDisable && 
        contact.normalForceMin < effector.limitNormalForceMin;
    return isDisabling || isEnabling;
}

void SEIKOWrapper::addContact(
    const std::string nameFrame, 
    bool isPoint)
{
    if (isPoint) {
        this->addContactPoint(nameFrame);
    } else {
        this->addContactPlane(nameFrame);
    }
    _commands.insert(std::make_pair(nameFrame, EndEffector_t{
        nameFrame, isPoint,
        false, false,
        0.0, 0.0, 0.0, 0.0, 0.0,
        false, 
        Eigen::Vector3d::Zero(),
        Eigen::Matrix3d::Identity(),
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero(),
        0.0,
        FilterPoseCommand(),
        FilterVelCommand<double>(),
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero(),
    }));
}

void SEIKOWrapper::computeAheadJointPos(double dt)
{
    //Reset to last computed state
    _aheadJointPos = this->_model->getJointPosVect();
    //Retrieve computed joint velocity
    const auto& jointVel = this->_model->getJointVelVect();
    //Integrate time step
    _aheadJointPos += dt*jointVel;
}        

}

