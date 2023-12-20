cmake_minimum_required(VERSION 2.8.12)
project(inria_talos_description)

find_package(catkin REQUIRED)
catkin_package()

install(DIRECTORY urdf/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf/)
install(DIRECTORY meshes/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes/)

