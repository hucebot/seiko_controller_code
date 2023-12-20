#ifndef INRIA_CONTROLLER_TASKBASETALOS_HPP
#define INRIA_CONTROLLER_TASKBASETALOS_HPP

#include <inria_controller/TaskBase.hpp>

namespace inria {

/**
 * TaskBaseTalos
 *
 * Specialized State, Command structures and 
 * TaskBase class for Talos robot.
 */
struct ControllerTalosState_t : public ControllerBaseState_t {
    //Estimated floating base orientation
    Eigen::Quaterniond quatBase;
    //Measured and estimated feet wrenches
    Eigen::Vector6d wrenchFootLeft;
    Eigen::Vector6d wrenchFootRight;
    //Measured and estimated hand wrenches
    Eigen::Vector6d wrenchHandLeft;
    Eigen::Vector6d wrenchHandRight;
    //Measured torso IMU orientation
    Eigen::Quaterniond quatTorsoIMU;
    
    /**
     * Import Base constructor
     */
    using ControllerBaseState_t::ControllerBaseState_t;

    /**
     * Reset overload
     */
    void reset()
    {
        this->ControllerBaseState_t::reset();
        quatBase.setIdentity();
        wrenchFootLeft.setZero();
        wrenchFootRight.setZero();
        wrenchHandLeft.setZero();
        wrenchHandRight.setZero();
        quatTorsoIMU.setIdentity();
    }
};
struct ControllerTalosCommand_t : public ControllerBaseCommand_t {
    using ControllerBaseCommand_t::ControllerBaseCommand_t;
};
class TaskBaseTalos : public TaskBase<ControllerTalosState_t, ControllerTalosCommand_t> {};

}

#endif

