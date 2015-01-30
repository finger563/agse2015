#ifndef RADIAL_ACTUATOR_CONTROLLER_HPP
#define RADIAL_ACTUATOR_CONTROLLER_HPP

#include "ros/ros.h"
#include "Component.hpp"

#include "agse_package/controlInputs.h"
#include "agse_package/radialPos.h"

// --------------------------------
//      USER INCLUDES GO HERE
// --------------------------------

class radial_actuator_controller : public Component
{
    public:

        // These functions' business logic will be filled in by the user:

	// Init() is always generated
	void Init(const ros::TimerEvent& event);

	// OnOneData Subscription handler for controlInputs_sub subscriber 
	void controlInputs_sub_OnOneData(const agse_package::controlInputs::ConstPtr& received_data); 
 

	// Component Service Callback
	bool radialPos_serverCallback(agse_package::radialPos::Request  &req,
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


	// ROS Service Server - radialPos_server_server
	ros::ServiceServer radialPos_server_server;


};


#endif
