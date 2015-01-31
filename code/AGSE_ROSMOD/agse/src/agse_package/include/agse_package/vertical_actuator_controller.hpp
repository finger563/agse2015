#ifndef VERTICAL_ACTUATOR_CONTROLLER_HPP
#define VERTICAL_ACTUATOR_CONTROLLER_HPP

#include "ros/ros.h"
#include "Component.hpp"

#include "agse_package/controlInputs.h"
#include "agse_package/verticalPos.h"

// --------------------------------
//      USER INCLUDES GO HERE
// --------------------------------

class vertical_actuator_controller : public Component
{
    public:

        // These functions' business logic will be filled in by the user:

	// Init() is always generated
	void Init(const ros::TimerEvent& event);

	// OnOneData Subscription handler for controlInputs_sub subscriber 
	void controlInputs_sub_OnOneData(const agse_package::controlInputs::ConstPtr& received_data); 
 

	// Component Service Callback
	bool verticalPos_serverCallback(agse_package::verticalPos::Request  &req,
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


	// ROS Service Server - verticalPos_server_server
	ros::ServiceServer verticalPos_server_server;


};


#endif