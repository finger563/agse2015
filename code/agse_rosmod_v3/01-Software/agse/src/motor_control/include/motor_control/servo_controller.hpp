#ifndef SERVO_CONTROLLER_HPP
#define SERVO_CONTROLLER_HPP

#include "ros/ros.h"
#include "Component.hpp"
#include "high_level_control/controlInputs.h"
#include "motor_control/gripperRotation.h"
#include "motor_control/armRotation.h"
#include "motor_control/gripperPos.h"
//# Start User Includes Marker
//# End User Includes Marker

//# Start User Globals Marker
//# End User Globals Marker

class servo_controller : public Component
{
public:
  // Constructor
  servo_controller(ComponentConfig& config, int argc, char **argv) : Component(config, argc, argv) {}

  // Initialization
  void Init(const ros::TimerEvent& event);

  // Subscriber Callback - controlInputs_sub
  void controlInputs_sub_OnOneData(const high_level_control::controlInputs::ConstPtr& received_data); 
 
  // Server Callback - gripperRotation_server
  bool gripperRotationCallback(motor_control::gripperRotation::Request &req, 
    motor_control::gripperRotation::Response &res);

  // Server Callback - armRotation_server
  bool armRotationCallback(motor_control::armRotation::Request &req, 
    motor_control::armRotation::Response &res);

  // Server Callback - gripperPos_server
  bool gripperPosCallback(motor_control::gripperPos::Request &req, 
    motor_control::gripperPos::Response &res);

  // Timer Callback - servoTimer
  void servoTimerCallback(const ros::TimerEvent& event);

  // Start up
  void startUp();

  // Destructor
  ~servo_controller();

private:

  // Timer
  ros::Timer servoTimer;

  // Subscriber
  ros::Subscriber controlInputs_sub; 

  // Server 
  ros::ServiceServer gripperRotation_server;

  // Server 
  ros::ServiceServer armRotation_server;

  // Server 
  ros::ServiceServer gripperPos_server;

  //# Start User Private Variables Marker

  //# End User Private Variables Marker
};


#endif
