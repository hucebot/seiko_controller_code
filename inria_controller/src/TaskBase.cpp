#include <inria_controller/TaskBaseTalos.hpp>
#include <inria_controller/TaskBase.hpp>

namespace inria {

template <typename T_State, typename T_Command>
TaskBase<T_State, T_Command>::TaskBase() :
    _bufferState(),
    _bufferCommand(),
    _rhioNode(nullptr),
    _rhioTimingDuration(),
    _rhioTimingPeriodTask(),
    _rhioTimingPeriodReal(),
    _timeState(0.0),
    _timeUpdate(),
    _isTimeInit(false)
{
}

template <typename T_State, typename T_Command>
TaskBase<T_State, T_Command>::~TaskBase()
{
}
        
template <typename T_State, typename T_Command>
std::string TaskBase<T_State, T_Command>::getName() const
{
    return std::string(this->name());
}

template <typename T_State, typename T_Command>
double TaskBase<T_State, T_Command>::getFrequency() const
{
    return this->schedulingFrequency();
}

template <typename T_State, typename T_Command>
void TaskBase<T_State, T_Command>::doInit(const Model& model)
{
    //Allocate lock free buffers
    size_t sizeJoint = model.sizeJoint();
    _bufferState = std::make_unique<
        BufferReaderWriter<T_State>>(sizeJoint);
    _bufferCommand = std::make_unique<
        BufferReaderWriter<T_Command>>(sizeJoint);
    //Model configuration
    std::string pathModel = model.getPathURDF();
    //RhIO configuration
    RhIO::Root.newChild("/tasks/" + std::string(this->name()));
    _rhioNode = &RhIO::Root.child("/tasks/" + std::string(this->name()));
    _rhioTimingDuration.create(*_rhioNode, "timing_duration")
        ->comment("Task computation time")
        ->defaultValue(0.0);
    _rhioTimingPeriodTask.create(*_rhioNode, "timing_period_task")
        ->comment("Task update elapsed time")
        ->defaultValue(0.0);
    _rhioTimingPeriodReal.create(*_rhioNode, "timing_period_real")
        ->comment("Task update computed period time")
        ->defaultValue(0.0);
    //Call specific task initialization
    this->init(*_rhioNode, pathModel);
    _isTimeInit = false;
}
 
template <typename T_State, typename T_Command>
void TaskBase<T_State, T_Command>::doUpdate()
{
    std::chrono::time_point<std::chrono::steady_clock> timeStart = 
        std::chrono::steady_clock::now();
    //Retrieve state from controller
    _bufferState->getFromReader()->reset();
    _bufferState->pullFromReader();
    const T_State& state = *(_bufferState->getFromReader());
    //Abord update of the latest state is not valid
    if (!state.isValid) {
        return;
    }
    //Reset command structure
    T_Command& command = *(_bufferCommand->getFromWriter());
    command.reset();
    //Get state timing
    if (!_isTimeInit) {
        _timeState = state.time;
        _timeUpdate = std::chrono::steady_clock::now();
        _isTimeInit = true;
    }
    double dt_task = state.time - _timeState;
    if (dt_task < 0.0) {
        throw std::logic_error(
            "inria::TaskBase::doUpdate: "
            "Negative state time step: " 
            + std::to_string(dt_task));
    }
    //Call specific task update
    double dt_ahead = this->aheadTimeStep();
    if (dt_ahead < 0.0) {
        throw std::logic_error(
            "inria::TaskBase::doUpdate: "
            "Negative ahead time step: " 
            + std::to_string(dt_task));
    }
    this->update(dt_task, dt_ahead, state, command);
    //Send command structure to RT controller
    command.timeState = state.time;
    command.timeCmd = state.time + dt_ahead;
    _bufferCommand->pushFromWriter();
    //Timing statistics
    std::chrono::time_point<std::chrono::steady_clock> timeEnd = 
        std::chrono::steady_clock::now();
    std::chrono::duration<double> duration2 = 
        timeEnd - timeStart;
    std::chrono::duration<double> duration3 = 
        timeStart - _timeUpdate;
    _rhioTimingDuration = duration2.count();
    _rhioTimingPeriodTask = dt_task;
    _rhioTimingPeriodReal = duration3.count();
    _timeState = state.time;
    _timeUpdate = timeStart;
}

template <typename T_State, typename T_Command>
T_State& TaskBase<T_State, T_Command>::refState()
{
    return *_bufferState->getFromWriter();
}
template <typename T_State, typename T_Command>
void TaskBase<T_State, T_Command>::pushState()
{
    _bufferState->pushFromWriter();
}
template <typename T_State, typename T_Command>
void TaskBase<T_State, T_Command>::pullCommand()
{
    _bufferCommand->getFromReader()->reset();
    _bufferCommand->pullFromReader();
}
template <typename T_State, typename T_Command>
const T_Command& TaskBase<T_State, T_Command>::refCommand() const
{
    return *_bufferCommand->getFromReader();
}

//Explicit template instantiation for robot specialisations
template class TaskBase<ControllerTalosState_t, ControllerTalosCommand_t>;

}

