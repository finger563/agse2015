#ifndef GRIPPER_CONTROLLER_HPP
#define GRIPPER_CONTROLLER_HPP

#include "ros/ros.h"
#include "Component.hpp"

#include "agse_package/setGripperPos.h"
#include "agse_package/setGripperRotation.h"

// --------------------------------
//      USER INCLUDES GO HERE
// --------------------------------

class gripper_controller : public Component
{
    public:

        // These functions' business logic will be filled in by the user:

	// Init() is always generated
	void Init(const ros::TimerEvent& event);

	// Component Service Callback
	bool setGripperPos_serverCallback(agse_package::setGripperPos::Request  &req,
		agse_package::setGripperPos::Response &res);

	// Component Service Callback
	bool setGripperRotation_serverCallback(agse_package::setGripperRotation::Request  &req,
		agse_package::setGripperRotation::Response &res);


	// these functions' business logic will be auto-generated:

	// startUp() is used to configure timers, publishers, & service providers
	void startUp();

	// required for clean shutdown
	~gripper_controller();

    private:

	// ROS Service Server - setGripperPos_server_server
	ros::ServiceServer setGripperPos_server_server;

	// ROS Service Server - setGripperRotation_server_server
	ros::ServiceServer setGripperRotation_server_server;


};


#endif
