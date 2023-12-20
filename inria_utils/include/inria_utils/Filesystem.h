#ifndef INRIA_UTILS_FILESYSTEM_H
#define INRIA_UTILS_FILESYSTEM_H

#include <string>

namespace inria {

/**
 * Resolve system absolute or relative path
 * to handle finding ROS package path.
 * Thread safe but non RT safe.
 *
 * @param path Absolute, relative or
 * ROS package path to be resolved.
 * @return the resolved system path or 
 * the identity path if no ROS package 
 * can be found.
 */
std::string SystemResolvePath(
    const std::string& path);

}

#endif

