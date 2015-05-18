#ifndef VERTICAL_ACTUATOR_CONTROLLER_HPP
#define VERTICAL_ACTUATOR_CONTROLLER_HPP

#include "ros/ros.h"
#include "Component.hpp"
#include "high_level_control/controlInputs.h"
#include "motor_control/verticalPos.h"
//# Start User Includes Marker
//# End User Includes Marker

//# Start User Globals Marker
//# End User Globals Marker

class vertical_actuator_controller : public Component
{
public:
  // Constructor
  vertical_actuator_controller(ComponentConfig& config, int argc, char **argv) : Component(config, argc, argv) {}

  // Initialization
  void Init(const ros::TimerEvent& event);

  // Subscriber Callback - controlInputs_sub
  void controlInputs_sub_OnOneData(const high_level_control::controlInputs::ConstPtr& received_data); 
 
  // Server Callback - verticalPos_server
  bool verticalPosCallback(motor_control::verticalPos::Request &req, 
    motor_control::verticalPos::Response &res);

  // Timer Callback - verticalPosTimer
  void verticalPosTimerCallback(const ros::TimerEvent& event);

  // Start up
  void startUp();

  // Destructor
  ~vertical_actuator_controller();

private:

  // Timer
  ros::Timer verticalPosTimer;

  // Subscriber
  ros::Subscriber controlInputs_sub; 

  // Server 
  ros::ServiceServer verticalPos_server;

  //# Start User Private Variables Marker

  //# End User Private Variables Marker
};


#endif
