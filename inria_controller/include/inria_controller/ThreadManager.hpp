#ifndef INRIA_CONTROLLER_THREADMANAGER_HPP
#define INRIA_CONTROLLER_THREADMANAGER_HPP

#include <vector>
#include <thread>
#include <atomic>
#include <memory>
#include <signal.h>
#include <RhIO.hpp>
#include <inria_controller/TaskBase.hpp>
#include <inria_model/Model.hpp>

namespace inria {

/**
 * ThreadManager
 *
 * Class creating and
 * managing non real-time thread
 * within RT controller process.
 * @param T_State Type for controller state data.
 * @param T_Command Type for controller command data;
 */
template <typename T_State, typename T_Command>
class ThreadManager
{
    public:
        
        /**
         * Empty initialization 
         */
        ThreadManager();

        /**
         * Stop thread
         * if still running
         */
        ~ThreadManager();
        
        /**
         * Copy and assignment are disabled
         * to prevent memory management issues
         */
        ThreadManager(const ThreadManager&) = delete;
        ThreadManager& operator=(const ThreadManager&) = delete;

        /**
         * Initialization of non RT thread
         * 
         * @param model Model instance used.
         * @param tasks Container of allocated TaskBase*
         * to be initialized and managed.
         */
        void initThread(
            const Model& model, 
            const std::vector<TaskBase<T_State,T_Command>*>& tasks);

        /**
         * Start non RT thread
         */
        void startThread();

        /**
         * Ask non RT thread to stop
         */
        void stopThread();

        /**
         * If true, error has occurred in task and 
         * the controller must be fully stopped
         */
        bool isError() const;

        /**
         * Lock free RT interface to write state from controller 
         * to task and read command from task to controller.
         * WARNING: isTask() must be true before calling
         * the other task getters.
         * NOTE: refState() is always valid even if no task
         * is currently running.
         */
        bool isTask() const;
        std::string nameTask() const;
        T_State& refState();
        void pushState();
        void pullCommand();
        const T_Command& refCommand() const;

    private:

        /**
         * Container for polymorphic allocated
         * tasks instance
         */
        std::vector<TaskBase<T_State, T_Command>*> _containerTasks;

        /**
         * Default state structure when no task is selected
         */
        std::unique_ptr<T_State> _defaultState;

        /**
         * System non RT thread
         */
        std::thread* _thread;

        /**
         * Currently enabled task instance 
         * (stored in _containerTasks) or nullptr
         */
        std::atomic<TaskBase<T_State,T_Command>*> _task;

        /**
         * RhIO wrapper to controller state
         */
        RhIO::WrapperInt _rhioState;

        /**
         * If false, non RT thread should stop
         */
        std::atomic<bool> _isRunning;
        
        /**
         * True if an error has occurred
         * in one running threads.
         * Static to be accessed in signalHandler.
         */
        static std::atomic<bool> _isError;
        
        /**
         * Non RT thread main function
         */
        void threadMain();

        /**
         * Configure signal catching
         */
        void setupSignals();
        
        /**
         * Catch segmentation fault and
         * FPE signal error to be able to 
         * trigger the emergency mode
         */
        static void signalHandler(
            int sig, siginfo_t* siginfo, void* context);
};

}

#endif

