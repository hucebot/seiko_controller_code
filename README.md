# SEIKO Controller:<br>Multi-Contact Whole-Body Force Control for Position-Controlled Robots

**Website**: https://hucebot.github.io/seiko_controller_website \
**Video**: https://youtu.be/RGkZS57_6Nk \
**Paper**: https://ieeexplore.ieee.org/abstract/document/10517393 \
**ArXiv**: https://arxiv.org/abs/2312.16465 \
**HAL**: https://hal.univ-lorraine.fr/hal-04362547 \
**Authors**: Quentin Rouxel, [Serena Ivaldi](https://members.loria.fr/SIvaldi), [Jean-Baptiste Mouret](https://members.loria.fr/JBMouret)

![SEIKO Talos Multi-Contact](/assets/seiko_talos_multicontact.png)

This repository contains the implementation of **SEIKO** (Sequential Equilibrium Inverse Kinematic Optimization) associated with the paper
*Multi-Contact Whole-Body Force Control for Position-Controlled Robots*, developed at [INRIA Nancy](https://www.inria.fr/en/inria-centre-universite-lorraine), [Larsen Team](https://www.inria.fr/en/larsen), in 2023 ; and the paper 
*Multicontact Motion Retargeting Using Whole-Body Optimization of Full Kinematics and Sequential Force Equilibrium*, developed at the [University of Edinburgh](https://groups.inf.ed.ac.uk/advr/) in 2021.
This is the code that was run on the [Talos humanoid robot](https://pal-robotics.com/robots/talos) during multi-contact experiments.
This repository does NOT include the implementation of various tests, simulation environment, viewer and human-interface used to teleoperate the robot.

## Implementation Overview

![SEIKO Pipeline Architecture](/assets/architecture.png)

* Implementation of [SEIKO Retargeting](https://ieeexplore.ieee.org/abstract/document/9728754) QP, Cartesian input commands processing and contact switch procedure:
```
inria_model/include/inria_model/SEIKORetargeting.hpp
inria_model/src/SEIKORetargeting.cpp
inria_model/include/inria_model/SEIKOWrapper.hpp
inria_model/src/SEIKOWrapper.cpp
```
* Parameters of SEIKO Retargeting for Talos robot:
```
inria_model/include/inria_model/SEIKOTalos.hpp
inria_model/src/SEIKOTalos.cpp
```
* Implementation of SEIKO Controller QP:
```
inria_model/include/inria_model/SEIKOController.hpp
inria_model/src/SEIKOController.cpp
```
* Main controller pipeline thread running at 500 Hz:
```
inria_controller/include/inria_controller/TaskSEIKO.hpp
inria_controller/src/TaskSEIKO.cpp
```
* [ROS_Control](http://wiki.ros.org/ros_control) lowlevel controller running in position-control mode at 2 kHz and performing joint position commands interpolation and safety checks:
```
inria_controller/include/inria_controller/ControllerBase.hpp
inria_controller/src/ControllerBase.cpp
inria_controller/include/inria_controller/ControllerTalos.hpp
inria_controller/src/ControllerTalos.cpp
```
* Wrappers around *RBDL* and *Pinocchio* libraries to manipulate the robot's model:
```
inria_model/include/inria_model/Model.hpp
inria_model/src/Model.cpp
inria_model/include/inria_model/PinocchioInterface.hpp
inria_model/src/PinocchioInterface.cpp
```
* URDF model of the Talos robot with 3d-printed ball-shaped right hand effector: 
```
inria_talos_description/urdf/talos_stump.urdf
```
* Starting script to load and run the *ROS_Control* controller on the robot:
```
inria_controller/scripts/startTalos.sh
```

## Dependencies

This implementation is intended  to be compiled with [catkin](http://wiki.ros.org/catkin) under [ROS Melodic](http://wiki.ros.org/melodic).
It requires the following catkin packages:
* **RBDL** (rigid body dynamics) \
https://gitlab.inria.fr/locolearn/rbdl_catkin
* **Pinocchio** (rigid body dynamics for analytical derivatives) \
https://github.com/stack-of-tasks/pinocchio
* **eiquadprog** (QP solver) \
https://github.com/stack-of-tasks/eiquadprog
* **RhIO** (user and command interface, monitoring, configuration, logging) \
https://gitlab.inria.fr/locolearn/rhio
* **ZMQ** (network transport for RhIO) \
https://gitlab.inria.fr/locolearn/libzmq_catkin

## Citation

**[SEIKO Controller](https://ieeexplore.ieee.org/abstract/document/10517393):**
```
@article{rouxel2024multicontact,
  title={Multi-Contact Whole-Body Force Control for Position-Controlled Robots},
  author={Rouxel, Quentin and Ivaldi, Serena and Mouret, Jean-Baptiste},
  journal={IEEE Robotics and Automation Letters},
  year={2024},
  publisher={IEEE}
}

```
**[SEIKO Retargeting](https://ieeexplore.ieee.org/abstract/document/9728754):**
```
@article{rouxel2022multicontact,
    title={Multicontact motion retargeting using whole-body optimization of full kinematics and sequential force equilibrium},
    author={Rouxel, Quentin and Yuan, Kai and Wen, Ruoshi and Li, Zhibin},
    journal={IEEE/ASME Transactions on Mechatronics},
    volume={27},
    number={5},
    pages={4188--4198},
    year={2022},
    publisher={IEEE}
}
```

## License

Licensed under the [BSD License](LICENSE)

