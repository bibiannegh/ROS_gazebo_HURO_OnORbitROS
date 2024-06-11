# Launch OnOrbitROS

All the functionalities of OnOrbitROS are launched simply by launching the project's launch file:

    roslaunch <your_project_name> <your_launch_file>


As presented in other sections, from this file the following functionalities are loaded: 

- Gazebo simulation parameters.
- Launch of the orbit and orbit publisher package.
- The world file (converting the .xacro to .world) and then loading with then the gazebo_ros launcher.
- Spacecraft / robot URDF loaded to the param server.
- World position of the spacecraft / robot model in the world.
- Loads of the ROS controllers.

When creating a new application, it is recommended to follow the same structure as the presented in the previously made applications to ensure all the functionalities are launched simply from one file. 

Under [Examples of Applications](examples.md) you will find the specific command used to launch each of the applications included.
