#ifndef USER_INPUT_CONTROLLER_HPP
#define USER_INPUT_CONTROLLER_HPP

#include "ros/ros.h"
#include "Component.hpp"

#include "agse_package/controlInputs.h"

#include "agse_package/gpio.h"

// --------------------------------
//      USER INCLUDES GO HERE
// --------------------------------

class user_input_controller : public Component
{
    public:

        // These functions' business logic will be filled in by the user:

	// Init() is always generated
	void Init(const ros::TimerEvent& event);

	// Callback for userInputTimer timer
	void userInputTimerCallback(const ros::TimerEvent& event);


	// these functions' business logic will be auto-generated:

	// startUp() is used to configure timers, publishers, & service providers
	void startUp();

	// required for clean shutdown
	~user_input_controller();

    private:

  bool paused;
  // pin for pause switch
  unsigned int pauseSwitchPin;
  // variable to keep track of switch state
  unsigned int pauseSwitchState;

	// ROS Timer - userInputTimer
	ros::Timer userInputTimer;


	// ROS Publisher - controlInputs_pub
	ros::Publisher controlInputs_pub;


};


#endif
