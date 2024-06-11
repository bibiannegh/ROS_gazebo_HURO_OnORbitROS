/*** Orbit Publisher Node ***/
/* 
    Reads the information of the orbit defined in the .yaml file. 
    Publishes the information needed for the simulation, as the position, velocity, air density and more. 
    Here is the LVLH information in reference to the ECI's frame. 

    The .yaml file is loaded in this package's .launch (fix_basic.launch) as follows: 
      <arg name="name" value="orbit_IIS" />
      <group ns="$(arg name)">
        <rosparam command="load" file="$(find orbit_publisher_pkg)/config/dynamic_orbit.yaml" />
        <!-- <node name="orbit" pkg="orbit_publisher_pkg" type="orbit_publisher_pkg_node" launch-prefix="xterm -e gdb -(to delete)-args"/> -->
        <node name="orbit" pkg="orbit_publisher_pkg" type="orbit_publisher_pkg_node" />
      </group>

    In case more information is needed from the other plugins it should be published from this script. 
    If additional (new) information is needed must be defined in "Orbit.h", declared in "<orbit>.yaml", read by "Orbit.cpp" and published here. 

    Node personalized for the simulation of the orbits. 

    Developed by: Human Robotics Laboratoy (University of Alicante)
    https://www.huro.ua.es/index.php/research/research-lines/space-robotics 
*/
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"
#include "orbit_publisher_pkg/Orbit.h"

#include <sstream>

// Calculated LVLH
int main(int argc, char **argv)
{
  int rate;
  ros::init(argc, argv, "OrbitPositionPublisher");    // Published the orbital information required for the simulation
  ros::NodeHandle n;
  std::string node_name = ros::this_node::getNamespace();
  ros::Publisher orbit_position_pub = n.advertise<geometry_msgs::Pose>("/OrbitPosition", 1000);
  ros::Publisher orbit_velocity_pub = n.advertise<geometry_msgs::Vector3>("/OrbitVelocity", 1000);
  

  if (!n.getParam("/" + node_name + "/publish_rate",rate))
  {
      rate = 10; 
  }
  ros::Rate loop_rate(rate);

  geometry_msgs::Pose pos;

  if (!n.getParam("/" + node_name + "/pos_x",pos.position.x))
  {
    pos.position.x = 0.0;
  }
  if (!n.getParam("/" + node_name + "/pos_y",pos.position.y))
  {
    pos.position.y = 0.0;
  }
  if (!n.getParam("/" + node_name + "/pos_z",pos.position.z))
  {
    pos.position.z = 0.0;
  }
  if (!n.getParam("/" + node_name + "/orientation_x",pos.orientation.x))
  {
    pos.orientation.x = 0.0;
  }
  if (!n.getParam("/" + node_name + "/orientation_y",pos.orientation.y))
  {
    pos.orientation.y = 0.0;
  }
  if (!n.getParam("/" + node_name + "/orientation_z",pos.orientation.z))
  {
    pos.orientation.z = 0.0;
  }
  if (!n.getParam("/" + node_name + "/orientation_w",pos.orientation.w))
  {
    pos.orientation.w = 1.0;
  }

  while (ros::ok())
  {  
    orbit_position_pub.publish(pos);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}