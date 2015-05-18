#ifndef RADIAL_ACTUATOR_CONTROLLER_HPP
#define RADIAL_ACTUATOR_CONTROLLER_HPP

#include "ros/ros.h"
#include "Component.hpp"
#include "high_level_control/controlInputs.h"
#include "motor_control/radialPos.h"
//# Start User Includes Marker
//# End User Includes Marker

//# Start User Globals Marker
//# End User Globals Marker

class radial_actuator_controller : public Component
{
public:
  // Constructor
  radial_actuator_controller(ComponentConfig& config, int argc, char **argv) : Component(config, argc, argv) {}

  // Initialization
  void Init(const ros::TimerEvent& event);

  // Subscriber Callback - controlInputs_sub
  void controlInputs_sub_OnOneData(const high_level_control::controlInputs::ConstPtr& received_data); 
 
  // Server Callback - radialPos_server
  bool radialPosCallback(motor_control::radialPos::Request &req, 
    motor_control::radialPos::Response &res);

  // Timer Callback - radialPosTimer
  void radialPosTimerCallback(const ros::TimerEvent& event);

  // Start up
  void startUp();

  // Destructor
  ~radial_actuator_controller();

private:

  // Timer
  ros::Timer radialPosTimer;

  // Subscriber
  ros::Subscriber controlInputs_sub; 

  // Server 
  ros::ServiceServer radialPos_server;

  //# Start User Private Variables Marker
  //# End User Private Variables Marker
};


#endif
