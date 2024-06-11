# Troubleshooting

In this page you will find the collection of tips for solving possible errors mentioned throughout this documentation. There are some other comments included that might be of help when using the framework.

### World File not loading
If you encounter any errors when launching the world, for example that the default Gazebo world loads instead of your own, create manually the `.world` file from the `.xacro` and leave it in the worlds forlder by executing the following command: 

    $ rosrun xacro xacro <path_to_world/world_name>.world.xacro > <path_to_world/world_name>.world
---

### Plugin Library not updating
If you have modified a plugin library, such as the `laserPlugin.cpp` or `Orbit_robot_pkg_plugin.cpp` and they do not update, try removing the `.so` file from the workspace `devel` or `devel_isolated` folders and the folder `orbit_robot_pkg` from the `build` or `build_isolated` folders. Those `.so` files should be located as this example: `>> <workspace_name>/devel/orbit_robot_pkg/lib/`. Then do another `catkin_make` and `source devel/setup.bash` so it generates the files again. 

---

### Robot model drifting when launching the simulation
If you launch the simulation in Gazebo and the robot or spacecraft model located at the origin of the LVLH drifts away it is because the default world's LVLH link has a visual cube with physical properties. When the simulation loads and some part of your model intersects with the LVLH visual link, it produces a collision that makes your model to drift away. 

In this case you have 2 options: 

- Change the origin of your model in the `.launch` file, NOT in the `.xacro` file, so they don't collide: `<arg name="world_pose" value="-x 0 -y 0 -z 1" />`
- Hide the LVLH base link removing its visual components to this declaration: `<link name="referenceSpaceCraft"/>`

---

### Missing Controllers' packages
In addition to the Ubuntu installation with its packages, it might be necessary to manually install these other packages. This is because OnOrbitROS takes advantages of packages already developed in ROS to improve its functionalities.

Packages for the controllers:

- [Controller Manager](https://wiki.ros.org/controller_manager): `$ sudo apt-get install ros-noetic-controller-manager`
- [Joint State Controller](https://wiki.ros.org/joint_state_controller):    `$ sudo apt-get install ros-noetic-joint-state-controller`
- [Effort Controller](https://wiki.ros.org/effort_controllers): `$ sudo apt-get install ros-noetic-effort-controller`
- [Position Controllers](https://wiki.ros.org/position_controllers):    `$ sudo apt-get install ros-noetic-position-controllers`  

---

### Rosbag missing
To save the information in a file use the following command on your launch. If the folder you have specified in the command does not exist, the rosbag will not be saved. By default it is located under the folder `log` in the project.

    <node pkg="rosbag" type="record" name="rosbag_record" args="record -o $(find <your_package_name>/log/data /<your_topics>"/>