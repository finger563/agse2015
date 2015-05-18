#ifndef USER_INPUT_CONTROLLER_HPP
#define USER_INPUT_CONTROLLER_HPP

#include "ros/ros.h"
#include "Component.hpp"
#include "high_level_control/controlInputs.h"
#include "high_level_control/armState.h"
//# Start User Includes Marker
//# End User Includes Marker

//# Start User Globals Marker
//# End User Globals Marker

class user_input_controller : public Component
{
public:
  // Constructor
  user_input_controller(ComponentConfig& config, int argc, char **argv) : Component(config, argc, argv) {}

  // Initialization
  void Init(const ros::TimerEvent& event);

  // Subscriber Callback - armState_sub
  void armState_sub_OnOneData(const high_level_control::armState::ConstPtr& received_data); 
 
  // Timer Callback - userInputTimer
  void userInputTimerCallback(const ros::TimerEvent& event);

  // Start up
  void startUp();

  // Destructor
  ~user_input_controller();

private:

  // Timer
  ros::Timer userInputTimer;

  // Subscriber
  ros::Subscriber armState_sub; 

  // Publisher 
  ros::Publisher controlInputs_pub;

  //# Start User Private Variables Marker
  //# End User Private Variables Marker
};


#endif
