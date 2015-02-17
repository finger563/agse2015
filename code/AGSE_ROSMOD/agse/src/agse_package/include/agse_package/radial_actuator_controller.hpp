#ifndef RADIAL_ACTUATOR_CONTROLLER_HPP
#define RADIAL_ACTUATOR_CONTROLLER_HPP

#include "ros/ros.h"
#include "Component.hpp"

// --------------------------------
//      USER INCLUDES GO HERE
// --------------------------------
//# Start User Includes Marker
#include "agse_package/gpio.h"
//# End User Includes Marker

#include "agse_package/controlInputs.h"
#include "agse_package/radialPos.h"

//# Start User Globals Marker

//# End User Globals Marker

class radial_actuator_controller : public Component
{
    public:

        // These functions' business logic will be filled in by the user:

	// Init() is always generated
	void Init(const ros::TimerEvent& event);

	// OnOneData Subscription handler for controlInputs_sub subscriber 
	void controlInputs_sub_OnOneData(const agse_package::controlInputs::ConstPtr& received_data); 
 

	// Component Service Callback
	bool radialPosCallback(agse_package::radialPos::Request  &req,
		agse_package::radialPos::Response &res);


	// Callback for radialPosTimer timer
	void radialPosTimerCallback(const ros::TimerEvent& event);


	// these functions' business logic will be auto-generated:

	// startUp() is used to configure timers, publishers, & service providers
	void startUp();

	// required for clean shutdown
	~radial_actuator_controller();

    private:

	// ROS Timer - radialPosTimer
	ros::Timer radialPosTimer;


	// ROS Subscriber - controlInputs_sub
	ros::Subscriber controlInputs_sub; 


	// ROS Service Server - radialPos_server
	ros::ServiceServer radialPos_server;


        //# Start User Private Variables Marker
  // paused variable which is controlled by the pause switch
  bool paused;
  // epsion value for minimum actionable difference between goal and current
  int epsilon;
  // goal position for the radial linear actuator
  int radialGoal;
  // current position of the radial linear actuator
  int radialCurrent;
  // pin that motor forward is connected to
  unsigned int motorForwardPin;
  // pin that motor backward is connected to
  unsigned int motorBackwardPin;
  // pin that encoder pin 0 is connected to
  unsigned int radialEncoderPin0;
  // pin that encoder pin 1 is connected to
  unsigned int radialEncoderPin1;
        //# End User Private Variables Marker
};


#endif
