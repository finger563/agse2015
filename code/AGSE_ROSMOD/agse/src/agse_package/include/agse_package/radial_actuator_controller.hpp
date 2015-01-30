#ifndef RADIAL_ACTUATOR_CONTROLLER_HPP
#define RADIAL_ACTUATOR_CONTROLLER_HPP

#include "ros/ros.h"
#include "Component.hpp"

#include "agse_package/setRadialPos.h"

// --------------------------------
//      USER INCLUDES GO HERE
// --------------------------------

class radial_actuator_controller : public Component
{
    public:

        // These functions' business logic will be filled in by the user:

	// Init() is always generated
	void Init(const ros::TimerEvent& event);

	// Component Service Callback
	bool setRadialPos_serverCallback(agse_package::setRadialPos::Request  &req,
		agse_package::setRadialPos::Response &res);


	// these functions' business logic will be auto-generated:

	// startUp() is used to configure timers, publishers, & service providers
	void startUp();

	// required for clean shutdown
	~radial_actuator_controller();

    private:

	// ROS Service Server - setRadialPos_server_server
	ros::ServiceServer setRadialPos_server_server;


};


#endif
