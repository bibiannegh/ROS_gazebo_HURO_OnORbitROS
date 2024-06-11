/*** Orbit Publisher Node ***/
/* 
    Reads the information of the orbit defined in the .yaml file. 
    Publishes the information needed for the simulation, as the position, velocity, air density and more. 
    Here is the LVLH information in reference to the ECI's frame. 

    The .yaml file is loaded in this package's .launch (basic.launch) as follows: 
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
#include "std_msgs/Float64.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"
#include "orbit_publisher_pkg/Orbit.h"

#include "rosgraph_msgs/Clock.h"
#include <sstream>

double gazebo_sim_time = 0.0;   // Gazebo simulation time

// Callback function for the /clock topic
void clockCallback(const rosgraph_msgs::Clock::ConstPtr& msg) {
    gazebo_sim_time = msg->clock.toSec();
}

int main(int argc, char **argv)
{
  int rate;  
  ros::init(argc, argv, "OrbitPositionPublisher");  // Published the orbital information required for the simulation
  ros::NodeHandle n;
  std::string node_name = ros::this_node::getNamespace();
  // ros::Publisher orbit_position_pub = n.advertise<geometry_msgs::Pose>("/"+ node_name +"/OrbitPosition", 1000);
  // ros::Publisher orbit_velocity_pub = n.advertise<geometry_msgs::Vector3>("/" + node_name +"/OrbitVelocity", 1000);
  ros::Publisher orbit_position_pub = n.advertise<geometry_msgs::Pose>("/OrbitPosition", 1000);
  ros::Publisher orbit_velocity_pub = n.advertise<geometry_msgs::Vector3>("/OrbitVelocity", 1000);
  ros::Publisher orbit_altitude_pub = n.advertise<std_msgs::Float64>("/OrbitAltitude", 1000);

  // Orbit parameters
  ros::Publisher orbit_sim_ecc_pub = n.advertise<std_msgs::Float64>("/OrbitEccentricity", 1000);
  ros::Publisher orbit_sim_sa_pub = n.advertise<std_msgs::Float64>("/OrbitSemiMajorAxis", 1000);
  ros::Publisher orbit_sim_incl_pub = n.advertise<std_msgs::Float64>("/OrbitInclination", 1000);
  ros::Publisher orbit_sim_raan_pub = n.advertise<std_msgs::Float64>("/OrbitRAAN", 1000);
  ros::Publisher orbit_sim_aop_pub = n.advertise<std_msgs::Float64>("/OrbitArgumentOfPerigee", 1000);
  ros::Publisher orbit_sim_ta_pub = n.advertise<std_msgs::Float64>("/OrbitTrueAnomaly", 1000);
  ros::Publisher orbit_sim_period_pub = n.advertise<std_msgs::Float64>("/OrbitPeriod", 1000);

  ros::Subscriber clock_sub = n.subscribe("/clock", 10, clockCallback);

  if (!n.getParam("/" + node_name + "/publish_rate",rate))
  {
      rate = 10; 
  }
  ros::Rate loop_rate(rate);

  Orbit orbital(n, node_name);    // Create the orbit
  orbital.SetName(node_name); 
  Eigen::Quaternion<double> q; 
  geometry_msgs::Pose pos;
  geometry_msgs::Vector3 vel; 
  std_msgs::Float64 altitude; 

  // Other orbit parameters
  std_msgs::Float64 sim_ecc; 
  std_msgs::Float64 sim_sa;
  std_msgs::Float64 sim_incl;
  std_msgs::Float64 sim_raan;
  std_msgs::Float64 sim_aop;
  std_msgs::Float64 sim_ta;
  std_msgs::Float64 sim_period;
  bool publish_orbit_parameters;    // Publish or not the keplerian orbital values
  if (!n.getParam("/" + node_name + "/orbit/publish_orbit_parameters", publish_orbit_parameters))
  {
    publish_orbit_parameters = true;
  }
  
  /* +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
  while (ros::ok())
  {
    // *** Continous publish *** //
    orbital.KeplerianToEci(gazebo_sim_time);  
    q = orbital.GetLvLhRotationToEci();

    // Pose
    pos.position.x = orbital.GetPositionEciX();   
    pos.position.y = orbital.GetPositionEciY();
    pos.position.z = orbital.GetPositionEciZ();
    pos.orientation.w = q.w();
    pos.orientation.x = q.x();
    pos.orientation.y = q.y();
    pos.orientation.z = q.z();
    orbit_position_pub.publish(pos);

    // Velocity
    vel.x = orbital.GetVelocityEciX();
    vel.y = orbital.GetVelocityEciY();
    vel.z = orbital.GetVelocityEciZ();
    orbit_velocity_pub.publish(vel);

    // Altitude
    altitude.data = orbital.CalcAltitude(gazebo_sim_time); 
    orbit_altitude_pub.publish(altitude);

    // Keplerian Orbit Elements
    if( publish_orbit_parameters ){
      orbital.CalcOrbitParamsFromSV(gazebo_sim_time);
      sim_ecc.data = orbital.GetSimEccentricity();
      orbit_sim_ecc_pub.publish(sim_ecc);
      sim_sa.data = orbital.GetSimSemiMajorAxis();
      orbit_sim_sa_pub.publish(sim_sa);
      sim_incl.data = orbital.GetSimInclination();
      orbit_sim_incl_pub.publish(sim_incl);
      sim_raan.data = orbital.GetSimRateOfRightAscension();
      orbit_sim_raan_pub.publish(sim_raan);
      sim_aop.data = orbital.GetSimArgumentOfPerigee();
      orbit_sim_aop_pub.publish(sim_aop);
      sim_ta.data = orbital.GetSimTrueAnomaly();
      orbit_sim_ta_pub.publish(sim_ta);
      sim_period.data = orbital.GetSimPeriod();
      orbit_sim_period_pub.publish(sim_period);
    }

    // *** Publish only once *** /
    // if( gazebo_sim_time < 2 ){    // publish until x simulation seconds
      
    // }

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}