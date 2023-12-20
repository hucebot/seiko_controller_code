#!/bin/bash

#Stop controller
rosrun controller_manager controller_manager stop inria_controller/ControllerTalos
sleep 1

#Unload controller
rosrun controller_manager controller_manager unload inria_controller/ControllerTalos

