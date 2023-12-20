cmake_minimum_required(VERSION 2.8.12)
project(inria_controller)

find_package(catkin REQUIRED
    controller_interface
    hardware_interface
    pluginlib
    realtime_tools
    urdf
    rhio_common
    rhio_server
    inria_utils
    inria_maths
    inria_model
)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS 
        controller_interface 
        hardware_interface 
        pluginlib 
        realtime_tools 
        urdf
        rhio_common
        rhio_server
        inria_utils
        inria_maths
        inria_model
)

include_directories(
    include 
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${pinocchio_INCLUDE_DIRS}
)

add_compile_options(-std=c++17 -Wno-invalid-partial-specialization)

add_library(${PROJECT_NAME}
    src/ControllerBase.cpp
    src/ControllerTalos.cpp
    src/TaskBase.cpp
    src/ThreadManager.cpp
    src/TaskSEIKO.cpp
)
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME} DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(FILES plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
install(
    FILES 
        scripts/startTalos.sh
        scripts/stopTalos.sh
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} 
    PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE
)

