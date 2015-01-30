#ifndef GRIPPER_CONTROLLER_HPP
#define GRIPPER_CONTROLLER_HPP

#include "ros/ros.h"
#include "Component.hpp"

#include "agse_package/controlInputs.h"
#include "agse_package/gripperRotation.h"
#include "agse_package/gripperPos.h"

// --------------------------------
//      USER INCLUDES GO HERE
// --------------------------------

class gripper_controller : public Component
{
    public:

        // These functions' business logic will be filled in by the user:

	// Init() is always generated
	void Init(const ros::TimerEvent& event);

	// OnOneData Subscription handler for controlInputs_sub subscriber 
	void controlInputs_sub_OnOneData(const agse_package::controlInputs::ConstPtr& received_data); 
 

	// Component Service Callback
	bool gripperRotation_serverCallback(agse_package::gripperRotation::Request  &req,
		agse_package::gripperRotation::Response &res);

	// Component Service Callback
	bool gripperPos_serverCallback(agse_package::gripperPos::Request  &req,
		agse_package::gripperPos::Response &res);


	// Callback for gripperTimer timer
	void gripperTimerCallback(const ros::TimerEvent& event);


	// these functions' business logic will be auto-generated:

	// startUp() is used to configure timers, publishers, & service providers
	void startUp();

	// required for clean shutdown
	~gripper_controller();

    private:

	// ROS Timer - gripperTimer
	ros::Timer gripperTimer;


	// ROS Subscriber - controlInputs_sub
	ros::Subscriber controlInputs_sub; 


	// ROS Service Server - gripperRotation_server_server
	ros::ServiceServer gripperRotation_server_server;

	// ROS Service Server - gripperPos_server_server
	ros::ServiceServer gripperPos_server_server;


};


#endif
