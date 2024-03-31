# Installation Guide

OnOrbitROS framework can be found on [this](https://github.com/OnOrbitROS) GitHub. There you will find the packages to install in your ROS environment as well as different guided examples to discover the functionalities OnOrbitROS offers. With this framework it is also possible to create new projects from the existing ones or from scrath.

---

## ROS Distribution
The project has been developed in ROS Noetic (Ubuntu 20.04). The Ubuntu intallation guidelines can be found [here](https://wiki.ros.org/noetic/Installation/Ubuntu). ROS offers many tutorials and guides to help you set up and work with your environments and projects, that can be accessed through [ROS Wiki](https://wiki.ros.org/).

---

## Gazebo Simulator

Gazebo is an open-source and free simulation tool specifically designed for fast and efficient test development. It has been selected as it can simulate complex 3D environments where each element possesses properties such as mass, velocity, and friction, and can accurately represent the dynamics and interations of space robots in complex 3D settings, so it is possible to create a realistic simulation of on-orbit conditions. Install from [here](https://classic.gazebosim.org/tutorials?tut=install_ubuntu).

There is high interation of Gazebo within ROS: simulations are generated from an XML file based on the SDF description language (extension of URDF that ROS uses for robot description) for quick and easy creations and configurations; and it is possible to include additional plugins to extend its functionalities. 

OnOrbitROS includes different plugins to simulate the on-orbit-specific conditions (using existing standard and available ROS/Gazebo packages to develop complex tasks). Also, Gazebo integrates interfaces to use multiple physics engines such as Open Dynamics Engine, Bullet, Simbody, Dynamic Animation and Robotics Toolkit (DART).

---

## OnOrbitROS Framework
Import the ROS framework from the GitHub (found [here](https://github.com/OnOrbitROS)). 

0. Install ROS and create an ROS environment (or use a previous one). The tutorial for doing so can be found [here](https://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment).
1. Download the framework with `git clone` or manually into your desired ROS environment.
2. Build the environment with `catkin_make` or `catkin_make_isolated`.
3. Solve any dependency problems that might occur. See [Other packages](#other-packages) for help.

You can now start working with the framework. 

---

## Examples of Applications
In case you would like to install the example cases: 

1. Download the example you want with `git clone` or manually into the folder `orbit_ws/src`.
2. Build the environment with `catkin_make` or `catkin_make_isolated`.
3. Execute the `.launch` corresponding to that example. See the specific example documentation for that (found in [Examples of Applications](/examples))

---

## Other packages
In addition to the Ubuntu installation with its packages, it might be necessary to manually install these other packages. This is because OnOrbitROS takes advantages of packages already developed in ROS to improve its functionalities.

Packages for the controllers:

- [Controller Manager](https://wiki.ros.org/controller_manager): `$ sudo apt-get install ros-noetic-controller-manager`
- [Joint State Controller](https://wiki.ros.org/joint_state_controller):    `$ sudo apt-get install ros-noetic-joint-state-controller`
- [Effort Controller](https://wiki.ros.org/effort_controllers): `$ sudo apt-get install ros-noetic-effort-controller`
- [Position Controllers](https://wiki.ros.org/position_controllers):    `$ sudo apt-get install ros-noetic-position-controllers`  
