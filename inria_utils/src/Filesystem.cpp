#include <iostream>
#include <vector>
#include <mutex>
#include <inria_utils/Filesystem.h>
#include <rospack/rospack.h>
#include <cstdlib>

namespace inria {

/**
 * Global instance of the ROS pack 
 * crawler protected from thread access
 */
static std::mutex globalMutex;
static bool globalIsInitialized = false;
static rospack::Rospack globalRP;

/**
 * Use ROSPack to retrieve the absolute 
 * name of a ROS package. 
 * Thread safe.
 *
 * @param name ROS name to be search.
 * @return the absolute package path 
 * or an empty string if not found.
 */
static std::string ROSGetPackagePath(const std::string& name)
{
    //Thread safe global access
    globalMutex.lock();
    if (!globalIsInitialized) {
        //Global ROSpack crawler 
        //initialization on the first call
        globalIsInitialized = true;
        globalRP.setQuiet(true);
        //Get search paths from environment
        std::vector<std::string> searchPaths;
        globalRP.getSearchPathFromEnv(searchPaths);
        //Explore every ROS packages
        globalRP.crawl(searchPaths, false);
    }
    //Find the package path
    std::string path;
    bool isSuccess = globalRP.find(name, path);
    globalMutex.unlock();

    if (isSuccess) {
        return path;
    } else {
        return "";
    }
}

std::string SystemResolvePath(
    const std::string& path)
{
    //Empty case
    if (path.length() == 0) {
        return "";
    }

    //Absolute or local file path
    if (path[0] == '/' || path[0] == '.') {
        return path;
    }

    //Local home case
    if (path[0] == '~') {
        const char* pathHome = std::getenv("HOME");
        if (pathHome != nullptr) {
            return std::string(pathHome) + path.substr(1);
        } else {
            return path;
        }
    }

    //URDF path convention
    if (path.substr(0, 7) == "file://") {
        return path.substr(7);
    }
    std::string tmpPath = path;
    if (path.substr(0, 10) == "package://") {
        tmpPath = path.substr(10);
    }
    if (path.substr(0, 8) == "model://") {
        tmpPath = path.substr(8);
    }

    //Try to retrieve from ROS the absolute path of 
    //a package from its name if possible or 
    //return the original path else
    size_t index = tmpPath.find_first_of("/");
    std::string partPackageName = tmpPath;
    std::string partLast = "";
    if (index != std::string::npos) {
        partPackageName = tmpPath.substr(0, index);
        partLast = tmpPath.substr(index+1);
    }

    //Retrieve the ROS absolute path 
    //of a package name if it exists
    std::string partPackagePath = 
        ROSGetPackagePath(partPackageName);

    if (partPackagePath.length() > 0) {
        return partPackagePath + "/" + partLast;
    } else {
        return tmpPath;
    }
}

}

