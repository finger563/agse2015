#ifndef ARM_ROTATION_CONTROLLER_HPP
#define ARM_ROTATION_CONTROLLER_HPP

#include "ros/ros.h"
#include "Component.hpp"

#include "agse_package/controlInputs.h"
#include "agse_package/armRotation.h"

// --------------------------------
//      USER INCLUDES GO HERE
// --------------------------------

class arm_rotation_controller : public Component
{
    public:

        // These functions' business logic will be filled in by the user:

	// Init() is always generated
	void Init(const ros::TimerEvent& event);

	// OnOneData Subscription handler for controlInputs_sub subscriber 
	void controlInputs_sub_OnOneData(const agse_package::controlInputs::ConstPtr& received_data); 
 

	// Component Service Callback
	bool armRotation_serverCallback(agse_package::armRotation::Request  &req,
		agse_package::armRotation::Response &res);


	// Callback for armRotationTimer timer
	void armRotationTimerCallback(const ros::TimerEvent& event);


	// these functions' business logic will be auto-generated:

	// startUp() is used to configure timers, publishers, & service providers
	void startUp();

	// required for clean shutdown
	~arm_rotation_controller();

    private:

	// ROS Timer - armRotationTimer
	ros::Timer armRotationTimer;


	// ROS Subscriber - controlInputs_sub
	ros::Subscriber controlInputs_sub; 


	// ROS Service Server - armRotation_server_server
	ros::ServiceServer armRotation_server_server;


};


#endif
