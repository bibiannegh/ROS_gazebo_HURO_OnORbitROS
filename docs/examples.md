# Examples of Applications using OnOrbitROS

In this section you can find different examples that exploit OnOrbitROS functionalities. They can serve as a guide to develop your own application, based on these or from scratch.

---

## Template

A template package, called **package_template** has been created to allocate the basic functionalities of a simulation using OnOrbitROS. It serves as a guide to create custom applications from it. 

When the launch file is loaded the world with a 1U cubesat in the world, describing the ETS VII's orbit. 

    roslaunch package_template default.launch

---

## ETS VII

The package **ets_vii** contains the simulation of the Japanese satellite ETS VII presented in 
<span style="color:gray">
    Ramón, J. L., Pomares, J., & Felicetti, L. (2023). Task space control for on-orbit space robotics using a new ROS-based framework. Simulation Modelling Practice and Theory, 127(102790), 102790. [https://doi.org/10.1016/j.simpat.2023.102790](https://doi.org/10.1016/j.simpat.2023.102790)
</span>

The project is launched with the following command, that loads the satellite with a robotic manipulator performing a trajectory after 10 minutes. It includes cartesian controllers to perform On Orbit Servicing tasks (for more information please refer to the previous article).

    roslaunch ets_vii effort_controllers_wgg.launch 
    

<img src="/images/etsvii.jpg" alt="ETS VII" style="width: 40%;" />

