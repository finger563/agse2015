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
#include "agse_package/sampleState.h"
#include "agse_package/payloadBayState.h"

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

	// OnOneData Subscription handler for sampleState_sub subscriber 
	void sampleState_sub_OnOneData(const agse_package::sampleState::ConstPtr& received_data); 
 
	// OnOneData Subscription handler for payloadBayState_sub subscriber 
	void payloadBayState_sub_OnOneData(const agse_package::payloadBayState::ConstPtr& received_data); 
 

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


	// ROS Subscriber - sampleState_sub
	ros::Subscriber sampleState_sub; 

	// ROS Subscriber - payloadBayState_sub
	ros::Subscriber payloadBayState_sub; 


	// ROS Publisher - controlInputs_pub
	ros::Publisher controlInputs_pub;


        //# Start User Private Variables Marker
        bool paused;

        // Pins for Pause (AMBER) Missile Switch
  unsigned int pauseSwitchPin; //63
  unsigned int pauseSwitch_LEDPin; //62

        // Pins for Manual Override (RED) Missile Switch
  unsigned int manualSwitchPin; //37
  unsigned int manualSwitch_LEDPin; //36
 
        // Pins for halt (BLUE) Missile Switch
  unsigned int haltSwitchPin; //33
  unsigned int haltSwitch_LEDPin; //32

        // Pin for Pause LED
  unsigned int pauseLED; //76
        // Pin for Alarm LED
        unsigned int alarmLED;
        // Pin for Sample LED
        unsigned int sampleLED[3];
        // Pin for Bay LED
        unsigned int bayLED[3];
        // Pin for Init LED
        unsigned int initLED[3];

        // variable to keep track of switch state
        unsigned int pauseSwitchState;
        //# End User Private Variables Marker
};


#endif
