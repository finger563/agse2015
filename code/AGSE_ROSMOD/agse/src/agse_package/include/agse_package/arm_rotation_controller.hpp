#ifndef ARM_ROTATION_CONTROLLER_HPP
#define ARM_ROTATION_CONTROLLER_HPP

#include "ros/ros.h"
#include "Component.hpp"

#include "agse_package/setArmRotation.h"

// --------------------------------
//      USER INCLUDES GO HERE
// --------------------------------

class arm_rotation_controller : public Component
{
    public:

        // These functions' business logic will be filled in by the user:

	// Init() is always generated
	void Init(const ros::TimerEvent& event);

	// Component Service Callback
	bool setArmRotation_serverCallback(agse_package::setArmRotation::Request  &req,
		agse_package::setArmRotation::Response &res);


	// these functions' business logic will be auto-generated:

	// startUp() is used to configure timers, publishers, & service providers
	void startUp();

	// required for clean shutdown
	~arm_rotation_controller();

    private:

	// ROS Service Server - setArmRotation_server_server
	ros::ServiceServer setArmRotation_server_server;


};


#endif
