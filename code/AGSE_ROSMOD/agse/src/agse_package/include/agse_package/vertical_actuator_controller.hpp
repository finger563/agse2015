#ifndef VERTICAL_ACTUATOR_CONTROLLER_HPP
#define VERTICAL_ACTUATOR_CONTROLLER_HPP

#include "ros/ros.h"
#include "Component.hpp"

#include "agse_package/setVerticalPos.h"

// --------------------------------
//      USER INCLUDES GO HERE
// --------------------------------

class vertical_actuator_controller : public Component
{
    public:

        // These functions' business logic will be filled in by the user:

	// Init() is always generated
	void Init(const ros::TimerEvent& event);

	// Component Service Callback
	bool setVerticalPos_serverCallback(agse_package::setVerticalPos::Request  &req,
		agse_package::setVerticalPos::Response &res);


	// these functions' business logic will be auto-generated:

	// startUp() is used to configure timers, publishers, & service providers
	void startUp();

	// required for clean shutdown
	~vertical_actuator_controller();

    private:

	// ROS Service Server - setVerticalPos_server_server
	ros::ServiceServer setVerticalPos_server_server;


};


#endif
