#ifndef VERTICAL_ACTUATOR_CONTROLLER_HPP
#define VERTICAL_ACTUATOR_CONTROLLER_HPP

#include "ros/ros.h"
#include "Component.hpp"

// --------------------------------
//      USER INCLUDES GO HERE
// --------------------------------
//# Start User Includes Marker
#include "agse_package/gpio.h"
#include "agse_package/eqep.h"
//# End User Includes Marker

#include "agse_package/controlInputs.h"
#include "agse_package/verticalPos.h"

//# Start User Globals Marker

//# End User Globals Marker

class vertical_actuator_controller : public Component
{
    public:
        // Component vertical_actuator_controller Constructor
        vertical_actuator_controller(std::string hostName, std::string nodeName, std::string compName, int argc, char **argv) : Component(hostName, nodeName, compName, argc, argv) {}

        // These functions' business logic will be filled in by the user:

	// Init() is always generated
	void Init(const ros::TimerEvent& event);

	// OnOneData Subscription handler for controlInputs_sub subscriber 
	void controlInputs_sub_OnOneData(const agse_package::controlInputs::ConstPtr& received_data); 
 

	// Component Service Callback
	bool verticalPosCallback(agse_package::verticalPos::Request &req,
		agse_package::verticalPos::Response &res);


	// Callback for verticalPosTimer timer
	void verticalPosTimerCallback(const ros::TimerEvent& event);


	// these functions' business logic will be auto-generated:

	// startUp() is used to configure timers, publishers, & service providers
	void startUp();

	// required for clean shutdown
	~vertical_actuator_controller();

    private:

	// ROS Timer - verticalPosTimer
	ros::Timer verticalPosTimer;


	// ROS Subscriber - controlInputs_sub
	ros::Subscriber controlInputs_sub; 


	// ROS Service Server - verticalPos_server
	ros::ServiceServer verticalPos_server;


        //# Start User Private Variables Marker
  bool paused;
  // epsion value for minimum actionable difference between goal and current
  int epsilon;
  // goal position for the vertical linear actuator
  int verticalGoal;
  // current position of the vertical linear actuator
  int verticalCurrent;
  // pin that motor forward is connected to
  unsigned int motorForwardPin;
  // pin that motor backward is connected to
  unsigned int motorBackwardPin;
  // ADC the motor potentiometer is connected to (for the prototype)
  int adcPin;
  // enhanced Quadrature Encoder Pulse eQEP module for the vertical actuator
  eQEP verticalMotoreQEP;
  long vm_eqep_period;
        //# End User Private Variables Marker
};


#endif
