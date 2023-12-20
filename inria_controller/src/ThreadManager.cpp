#include <inria_controller/TaskBaseTalos.hpp>
#include <stdexcept>
#include <fenv.h>
#include <inria_controller/ThreadManager.hpp>
#include <inria_utils/Scheduling.h>
#include <ros/ros.h>

namespace inria {

//Private static variable initialization
template <typename T_State, typename T_Command>
std::atomic<bool> ThreadManager<T_State, T_Command>::_isError(false);

template <typename T_State, typename T_Command>
ThreadManager<T_State, T_Command>::ThreadManager() :
    _containerTasks(),
    _defaultState(),
    _thread(nullptr),
    _task(),
    _rhioState(),
    _isRunning()
{
    //Reset error flag
    _isError.store(false);
}

template <typename T_State, typename T_Command>
ThreadManager<T_State, T_Command>::~ThreadManager()
{
    //Stop thread if still running
    stopThread();
    //Deallocate tasks
    for (size_t i=0;i<_containerTasks.size();i++) {
        if (_containerTasks[i] != nullptr) {
            delete _containerTasks[i];
            _containerTasks[i] = nullptr;
        }
    }
    _containerTasks.clear();
}
        
template <typename T_State, typename T_Command>
void ThreadManager<T_State, T_Command>::initThread(
    const Model& model,
    const std::vector<TaskBase<T_State,T_Command>*>& tasks)
{
    //Configure error signal handling
    setupSignals();
    
    //Retrieve pre-allocated tasks
    _containerTasks = tasks;

    //Retrieve wrapper on controller state
    _rhioState.bind(RhIO::Root.child("/controller"), "state");

    //Default no task initialization
    _defaultState = std::make_unique<T_State>(model.sizeJoint());

    for (size_t i=0;i<_containerTasks.size();i++) {
        //Call tasks initialization
        ROS_INFO_STREAM(
            "inria::ThreadManager::initThread: "
            "Task initialization: " << _containerTasks[i]->getName());
        _containerTasks[i]->doInit(model);
        //Checks
        if (_containerTasks[i]->getFrequency() <= 0.0) {
            throw std::logic_error(
                "inria::ThreadManager::initThread: "
                "Invalid task scheduling frequency: " + 
                _containerTasks[i]->getName());
        }
        //Create RhIO command to start each task
        RhIO::Root.newCommand(
            "task_" + _containerTasks[i]->getName(),
            "Use the task: " + _containerTasks[i]->getName(),
            [this, i](const std::vector<std::string>& args) -> std::string
            {
                if (args.size() != 0) {
                    return "Invalid usage";
                } 
                if (this->_rhioState.get() != 0 && this->_rhioState.get() != 2) {
                    return "Error controller is still running";
                }
                this->_task.exchange(this->_containerTasks[i]);
                return "Task changed to: " + this->_task.load()->getName();
            });
    }
    //Create RhIO command to unset to none current task
    RhIO::Root.newCommand(
        "task_unset",
        "Unset current task to none",
        [this](const std::vector<std::string>& args) -> std::string
        {
            if (args.size() != 0) {
                return "Invalid usage";
            } 
            if (this->_rhioState.get() != 0 && this->_rhioState.get() != 2) {
                return "Error controller is still running";
            }
            this->_task.exchange(nullptr);
            return "Task changed to: none";
        });
    //Create RhIO command to give current task name
    RhIO::Root.newCommand(
        "task_get",
        "Print current task name",
        [this](const std::vector<std::string>& args) -> std::string
        {
            if (args.size() != 0) {
                return "Invalid usage";
            }
            if (this->_task.load() == nullptr) {
                return "none";
            } else {
                return this->_task.load()->getName();
            }
        });

    //Reset current task
    _task.store(nullptr);
}

template <typename T_State, typename T_Command>
void ThreadManager<T_State, T_Command>::startThread()
{
    //Ask thread to run
    _isRunning.store(true);
    //Allocate and start non RT thread
    if (_thread == nullptr) {
        _thread = new std::thread(
            [this](){
                this->threadMain();
            }
        );
    }
}

template <typename T_State, typename T_Command>
void ThreadManager<T_State, T_Command>::stopThread()
{
    //Ask thread to stop
    _isRunning.store(false);
    //Wait for thread to stop and deallocate thread
    if (_thread != nullptr) {
        _thread->join();
        delete _thread;
        _thread = nullptr;
    }
}

template <typename T_State, typename T_Command>
bool ThreadManager<T_State, T_Command>::isError() const
{
    return _isError.load();
}

template <typename T_State, typename T_Command>
bool ThreadManager<T_State, T_Command>::isTask() const
{
    return (_task.load() != nullptr);
}
template <typename T_State, typename T_Command>
std::string ThreadManager<T_State, T_Command>::nameTask() const
{
    if (_task.load() != nullptr) {
        return _task.load()->getName();
    } else {
        return "";
    }
}
template <typename T_State, typename T_Command>
T_State& ThreadManager<T_State, T_Command>::refState()
{
    if (isTask()) {
        return _task.load()->refState();
    } else {
        return *_defaultState;
    }
}
template <typename T_State, typename T_Command>
void ThreadManager<T_State, T_Command>::pushState()
{
    _task.load()->pushState();
}
template <typename T_State, typename T_Command>
void ThreadManager<T_State, T_Command>::pullCommand()
{
    _task.load()->pullCommand();
}
template <typename T_State, typename T_Command>
const T_Command& ThreadManager<T_State, T_Command>::refCommand() const
{
    return _task.load()->refCommand();
}

template <typename T_State, typename T_Command>
void ThreadManager<T_State, T_Command>::threadMain()
{
    //Configure error signal handling
    setupSignals();

    //Set thread name and configuration
    SystemSetThreadName("inria_task");
    SystemSetThreadRealTime();

    std::chrono::time_point<std::chrono::steady_clock> timeLast = 
        std::chrono::steady_clock::now();
    //Thread main loop
    while (_isRunning.load() && !_isError.load()) {
        std::chrono::time_point<std::chrono::steady_clock> timeNow = 
            std::chrono::steady_clock::now();
        //Retrieve current task
        TaskBase<T_State, T_Command>* taskPtr = _task.load();
        //Call to task implementation
        try {
            if (taskPtr != nullptr) {
                taskPtr->doUpdate();
            }
        //If any exception is caught, 
        //go in emergency mode 
        } catch (const std::string& e) {
            std::string name = 
                std::string("inria_task_") + 
                ((taskPtr == nullptr) ? "none" : taskPtr->getName());
            ROS_ERROR_STREAM(
                "inria::ThreadManager: "
                "String exception in thread "
                << name << " with message: " << e);
            _isError.store(true);
            return;
        } catch (const std::exception& e) {
            std::string name = 
                std::string("inria_task_") + 
                ((taskPtr == nullptr) ? "none" : taskPtr->getName());
            ROS_ERROR_STREAM(
                "inria::ThreadManager: "
                "Exception in thread "
                << name << " with message: " << e.what());
            _isError.store(true);
            return;
        } catch (...) {
            std::string name = 
                std::string("inria_task_") + 
                ((taskPtr == nullptr) ? "none" : taskPtr->getName());
            ROS_ERROR_STREAM(
                "inria::ThreadManager: "
                "Unknown exception in thread "
                << name);
            _isError.store(true);
            return;
        }
        //Fixed desired frequency scheduling
        //(inspired by ros::WallRate)
        double schedulingFrequency = 100.0;
        if (taskPtr != nullptr) {
            schedulingFrequency = taskPtr->getFrequency();
        }
        auto durationTarget = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(1.0/schedulingFrequency));
        std::chrono::time_point<std::chrono::steady_clock> timeExpected = 
            timeLast + durationTarget;
        auto durationSleep = timeExpected - timeNow;
        timeLast = timeExpected;
        if (durationSleep.count() > 0.0) {
            std::this_thread::sleep_for(durationSleep);
        } else {
            if (timeNow > timeExpected + durationTarget) {
                timeLast = timeNow;
            }
        }
    }
}
 
template <typename T_State, typename T_Command>
void ThreadManager<T_State, T_Command>::setupSignals()
{
    //Main OROCOS thread in which the initializations are 
    //done seems to block some signals...
    //Be sure to unblock the signals so we can 
    //catch and handle them
    sigset_t setSignal;
    sigemptyset(&setSignal);
    sigaddset(&setSignal, SIGSEGV);
    sigaddset(&setSignal, SIGILL);
    sigaddset(&setSignal, SIGFPE);
    sigaddset(&setSignal, SIGABRT);
    sigprocmask(SIG_UNBLOCK, &setSignal, NULL);

    //Setup the signal handler over
    //segfault and FPE errors
    struct sigaction act;
    memset(&act, '\0', sizeof(act));
    act.sa_sigaction = signalHandler;
    act.sa_flags = SA_SIGINFO;
    if (
        sigaction(SIGSEGV, &act, NULL) < 0 ||
        sigaction(SIGILL, &act, NULL) < 0 ||
        sigaction(SIGFPE, &act, NULL) < 0 ||
        sigaction(SIGABRT, &act, NULL) < 0
    ) {
        throw std::logic_error(
            "inria::ThreadManager::initThread: "
            "Unable to setup signal handler");
    }

    //Enable floating point pedantic exceptions
    feenableexcept(FE_DIVBYZERO| FE_INVALID | FE_OVERFLOW);
}

template <typename T_State, typename T_Command>
void ThreadManager<T_State, T_Command>::signalHandler(
    int sig, siginfo_t* siginfo, void* context)
{
    (void)siginfo;
    (void)context;

    if (sig == SIGSEGV) {
        ROS_ERROR_STREAM(
            "inria::ThreadManager: "
            "Segfault error signal catched in thread: " 
            << SystemGetThreadName());
        _isError.store(true);
    }
    if (sig == SIGILL) {
        ROS_ERROR_STREAM(
            "inria::ThreadManager: "
            "Invalid instruction signal catched in thread: " 
            << SystemGetThreadName());
        _isError.store(true);
    }
    if (sig == SIGFPE) {
        ROS_ERROR_STREAM(
            "inria::ThreadManager: "
            "FPE error signal catched in thread: "
            << SystemGetThreadName());
        _isError.store(true);
    }
    if (sig == SIGABRT) {
        ROS_ERROR_STREAM(
            "inria::ThreadManager: "
            "Abord error signal catched in thread: "
            << SystemGetThreadName());
        _isError.store(true);
    }
    
    //The thread which triggered the signal 
    //is then trapped not to repeat the signal
    //continuously
    while (true) {
        usleep(1000000);
    }
}

//Explicit template instantiation for robot specialisations
template class ThreadManager<ControllerTalosState_t, ControllerTalosCommand_t>;

}

