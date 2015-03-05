#ifndef USER_INPUT_CONTROLLER_HPP
#define USER_INPUT_CONTROLLER_HPP

#include "ros/ros.h"
#include "Component.hpp"

// --------------------------------
//      USER INCLUDES GO HERE
// --------------------------------
//# Start User Includes Marker
#include "agse_package/gpio.h"
//# End User Includes Marker

#include "agse_package/controlInputs.h"

//# Start User Globals Marker

//# End User Globals Marker

class user_input_controller : public Component
{
    public:
        // Component user_input_controller Constructor
        user_input_controller(std::string nodeName, std::string compName, int argc, char **argv) : Component(nodeName, compName, argc, argv) {}

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

	// ROS Timer - userInputTimer
	ros::Timer userInputTimer;


	// ROS Publisher - controlInputs_pub
	ros::Publisher controlInputs_pub;


        //# Start User Private Variables Marker
  bool paused;
  // pin for pause switch
  unsigned int pauseSwitchPin;
  // variable to keep track of switch state
  unsigned int pauseSwitchState;
        //# End User Private Variables Marker
};


#endif
