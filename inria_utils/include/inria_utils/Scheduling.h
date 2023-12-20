#ifndef INRIA_UTILS_SCHEDULING_H
#define INRIA_UTILS_SCHEDULING_H

#include <stdexcept>
#include <unistd.h>
#include <sys/types.h>
#include <sys/syscall.h>
#include <sys/prctl.h>
#include <sched.h>
#include <string.h>
#include <errno.h>

namespace inria {

/**
 * @return Linux system process 
 * and thread unique id
 */
inline pid_t SystemGetProcessId()
{
    return getpid();
}
inline pid_t SystemGetThreadId()
{
    return (pid_t)syscall(SYS_gettid);
}

/**
 * Configure the calling thread as
 * real-time FIFO scheduling with 
 * max priority
 */
inline void SystemSetThreadRealTime()
{
    struct sched_param param;
    param.sched_priority = 99;
    int isError = sched_setscheduler(
        0, SCHED_FIFO, &param);
    if (isError != 0) {
        throw std::runtime_error(
            "inria::SystemSetThreadRealTime: "
            "Enabling real time thread failed: " +
            std::string(strerror(errno)));
    }
}

/**
 * Configure the calling thread CPU affinity
 * to the single given core number.
 *
 * @param num Core number on which the 
 * calling thread must run (in 0..CPU_COUNT-1)
 */
inline void SystemBindThreadToCPU(unsigned int num)
{
    cpu_set_t mask;
    CPU_ZERO(&mask);
    CPU_SET(num, &mask);
    int isError = sched_setaffinity(
        0, sizeof(mask), &mask);
    if (isError != 0) {
        throw std::runtime_error(
            "inria::SystemBindThreadToCPU: "
            "Binding thread to CPU failed: " +
            std::string(strerror(errno)));
    }
}

/**
 * Set the calling thread name.
 *
 * @param name The name to be assigned 
 * to the calling thread. If longer than
 * 15 characters, the name will be truncated.
 */
inline void SystemSetThreadName(const std::string& name)
{
    int isError = prctl(
        PR_SET_NAME,
        name.c_str(), 0, 0, 0);
    if (isError != 0) {
        throw std::runtime_error(
            "inria::SystemSetThread: "
            "Set thread name failed: " +
            name);
    }
}

/**
 * @return the name of the calling thread
 */
inline std::string SystemGetThreadName()
{
    char buffer[16];
    prctl(PR_GET_NAME,buffer, 0, 0, 0);
    return std::string(buffer);
}
    
}

#endif

