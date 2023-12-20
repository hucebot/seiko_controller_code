#!/bin/bash

#Configure network
if [ "$HOSTNAME" = talos-5c ]; then
    sudo sync
    sudo ip route add 224.0.0.1 dev eth0
fi

#Stop default Pal controllers
echo "Stopping default PAL controllers"
rosrun controller_manager controller_manager stop torso_current_limit_controller > /dev/null
rosrun controller_manager controller_manager stop left_leg_current_limit_controller > /dev/null
rosrun controller_manager controller_manager stop right_leg_current_limit_controller > /dev/null
rosrun controller_manager controller_manager stop head_current_limit_controller > /dev/null
rosrun controller_manager controller_manager stop left_gripper_current_limit_controller > /dev/null
rosrun controller_manager controller_manager stop right_gripper_current_limit_controller > /dev/null
rosrun controller_manager controller_manager stop left_arm_current_limit_controller > /dev/null
rosrun controller_manager controller_manager stop right_arm_current_limit_controller > /dev/null

#Disable PAL statistic on real hardware
if [ "$HOSTNAME" = talos-5c ]; then
    pal-stop statsdcc
fi

#Load plugin config
rosparam load `rospack find inria_controller`/config/inria_controller.yaml
sleep 1

#Load controller
rosrun controller_manager controller_manager load inria_controller/ControllerTalos
sleep 1

#Start controller
rosrun controller_manager controller_manager start inria_controller/ControllerTalos
sleep 1

#Enabled RT thread CPU isolation on real hardware
if [ "$HOSTNAME" = talos-5c ]; then
    sudo cset shield --reset
    sudo cset shield --cpu=0,2
    LWP1=`ps -eLo lwp,comm | grep inria_control | sed 's/^ *//g' | cut -d " " -f1`
    LWP2=`ps -eLo lwp,comm | grep inria_task | sed 's/^ *//g' | cut -d " " -f1`
    sudo cset shield --shield --pid $LWP1
    sudo cset shield --shield --pid $LWP2
fi

