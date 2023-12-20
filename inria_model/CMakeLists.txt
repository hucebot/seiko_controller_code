cmake_minimum_required(VERSION 2.8.12)
project(inria_model)

find_package(catkin REQUIRED
    urdf
    rbdl_catkin
    inria_maths
    inria_utils
)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS 
        urdf
        rbdl_catkin
        eiquadprog
        inria_maths
        inria_utils
)

include_directories(
    include 
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${pinocchio_INCLUDE_DIRS}
)
link_directories(
    ${CMAKE_INSTALL_PREFIX}/lib/
)

add_compile_options(-std=c++17 -Wno-invalid-partial-specialization)

add_library(${PROJECT_NAME}
    src/Model.cpp
    src/RBDLRootUpdate.cpp
    src/PinocchioInterface.cpp
    src/SEIKORetargeting.cpp
    src/SEIKOWrapper.cpp
    src/SEIKOTalos.cpp
    src/SEIKOController.cpp
)
target_link_libraries(${PROJECT_NAME} 
    ${catkin_LIBRARIES} 
    ${pinocchio_LIBRARIES} 
)

install(TARGETS ${PROJECT_NAME} DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

