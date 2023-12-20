#ifndef INRIA_CONTROLLER_TASKBASE_HPP
#define INRIA_CONTROLLER_TASKBASE_HPP

#include <string>
#include <chrono>
#include <memory>
#include <Eigen/Dense>
#include <RhIO.hpp>
#include <inria_model/Model.hpp>
#include <inria_utils/BufferReaderWriter.hpp>
#include <inria_controller/RhIOEigen.hpp>

namespace inria {

/**
 * State structure for communication from RT controller 
 * to non RT tasks.
 * The joint order is defined by the URDF model used.
 */
struct ControllerBaseState_t {
    //If false, this state structure is not valid
    bool isValid;
    //Time at RT controller estimation (in ROS time)
    double time;
    //Is the task running
    bool isRunning;
    //Measured joint position, velocity 
    //and torque in model order
    Eigen::VectorXd jointPos;
    Eigen::VectorXd jointVel;
    Eigen::VectorXd jointTau;
    //Send joint position command
    Eigen::VectorXd jointCmd;

    /**
     * Constructor with memory allocation.
     * @param sizeJoint Number of joint in model.
     */
    ControllerBaseState_t(size_t sizeJoint) :
        isValid(false),
        time(0.0),
        isRunning(false),
        jointPos(Eigen::VectorXd::Zero(sizeJoint)),
        jointVel(Eigen::VectorXd::Zero(sizeJoint)),
        jointTau(Eigen::VectorXd::Zero(sizeJoint)),
        jointCmd(Eigen::VectorXd::Zero(sizeJoint))
    {
        reset();
    }

    /**
     * Reset to invalid
     */
    void reset()
    {
        isValid = false;
        time = 0.0;
        isRunning = false;
        jointPos.setZero();
        jointVel.setZero();
        jointTau.setZero();
        jointCmd.setZero();
    }
};

/**
 * Command structure for communication from non RT 
 * tasks to RT controller.
 * The joint order is defined by the URDF model used.
 */
struct ControllerBaseCommand_t {
    //Time associated to RT controller state timestamp 
    //used to compute this command (in ROS time)
    double timeState;
    //Future time (after timeState) for which 
    //this command is computed (in ROS time)
    double timeCmd;
    //Position command for each joint in model order
    Eigen::VectorXd jointCmd;
    //Boolean indicating if associated joint command is valid
    Eigen::VectorXi jointIsUsed;
    
    /**
     * Constructor with memory allocation.
     * @param sizeJoint Number of joint in model.
     */
    ControllerBaseCommand_t(size_t sizeJoint) :
        timeState(0.0),
        timeCmd(0.0),
        jointCmd(Eigen::VectorXd::Zero(sizeJoint)),
        jointIsUsed(Eigen::VectorXi::Zero(sizeJoint))
    {
    }
    
    /**
     * Reset to invalid
     */
    void reset()
    {
        timeState = 0.0;
        timeCmd = 0.0;
        jointCmd.setZero();
        jointIsUsed.setZero();
    }
};

template <typename T_State, typename T_Command>
class ThreadManager;

/**
 * TaskBase
 *
 * Base class for control tasks running 
 * in a non RT thread.
 * @param T_State Type for controller state data.
 * @param T_Command Type for controller command data;
 */
template <typename T_State, typename T_Command>
class TaskBase
{
    public:

        /**
         * Empty initialization
         */
        TaskBase();

        /**
         * Virtual destructor
         */
        virtual ~TaskBase();

        /**
         * Return task name
         */
        std::string getName() const;

        /**
         * Return task scheduling frequency
         */
        double getFrequency() const;

        /**
         * Initialize the task with the 
         * declared model
         */
        void doInit(const Model& model);

        /**
         * Run one update step of the task
         */
        void doUpdate();
        
        /**
         * Lock free RT interface to write 
         * state from controller to task and 
         * read command from task to controller.
         */
        T_State& refState();
        void pushState();
        void pullCommand();
        const T_Command& refCommand() const;

    protected:

        /**
         * Task name.
         *
         * @return a unique constant 
         * string name for the task.
         */
        virtual const char* name() const = 0;

        /**
         * Task defined task scheduling frequency
         * (by thread manager)
         *
         * @return desire frequency in Hertz
         */
        virtual double schedulingFrequency() const = 0;

        /**
         * Task defined prediction ahead time step 
         * for lowlevel interpolation.
         *
         * @return time step in seconds.
         */
        virtual double aheadTimeStep() const = 0;

        /**
         * Task defined no RT task initialization
         * run at controller initialization.
         *
         * @param rhioNode Task specific RhIO node.
         * @param pathModel Path to used URDF model.
         */
        virtual void init(
            RhIO::IONode& rhioNode,
            const std::string& pathModel) = 0;

        /**
         * Task defined RT update step.
         *
         * @param dt_task Duration since last call in second.
         * @param dt_ahead Time step after state time for which to 
         * compute the command in second.
         * @param state Last (const) estimated measured state 
         * from RT controller.
         * @param command Joint command structure to be send
         * to RT controller.
         */
        virtual void update(
            double dt_task,
            double dt_ahead,
            const T_State& state,
            T_Command& command) = 0;

    private:

        /**
         * Lock free buffers for state and command communication
         * between RT controller and the non RT task
         */
        std::unique_ptr<BufferReaderWriter<T_State>> 
            _bufferState;
        std::unique_ptr<BufferReaderWriter<T_Command>> 
            _bufferCommand;

        /**
         * RhIO node associated to the task
         */
        RhIO::IONode* _rhioNode;

        /**
         * Task computation time and period statistics
         */
        RhIO::WrapperFloat _rhioTimingDuration;
        RhIO::WrapperFloat _rhioTimingPeriodTask;
        RhIO::WrapperFloat _rhioTimingPeriodReal;
        double _timeState;
        std::chrono::time_point<std::chrono::steady_clock> _timeUpdate;
        bool _isTimeInit;
};

}

#endif

