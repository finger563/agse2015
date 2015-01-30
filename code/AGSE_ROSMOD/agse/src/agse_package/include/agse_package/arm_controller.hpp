#ifndef ARM_CONTROLLER_HPP
#define ARM_CONTROLLER_HPP

#include "ros/ros.h"
#include "Component.hpp"

#include "agse_package/setRadialPos.h"
#include "agse_package/setArmRotation.h"
#include "agse_package/setVerticalPos.h"
#include "agse_package/sampleStateFromImage.h"
#include "agse_package/setGripperPos.h"
#include "agse_package/setGripperRotation.h"

// --------------------------------
//      USER INCLUDES GO HERE
// --------------------------------

class arm_controller : public Component
{
    public:

        // These functions' business logic will be filled in by the user:

	// Init() is always generated
	void Init(const ros::TimerEvent& event);

	// Callback for armTimer timer
	void armTimerCallback(const ros::TimerEvent& event);


	// these functions' business logic will be auto-generated:

	// startUp() is used to configure timers, publishers, & service providers
	void startUp();

	// required for clean shutdown
	~arm_controller();

    private:

	// ROS Timer - armTimer
	ros::Timer armTimer;


	// ROS Service Client - setRadialPos_client_client
	ros::ServiceClient setRadialPos_client_client;

	// ROS Service Client - setVerticalPos_client_client
	ros::ServiceClient setVerticalPos_client_client;

	// ROS Service Client - setGripperPos_client_client
	ros::ServiceClient setGripperPos_client_client;

	// ROS Service Client - setGripperRotation_client_client
	ros::ServiceClient setGripperRotation_client_client;

	// ROS Service Client - setArmRotation_client_client
	ros::ServiceClient setArmRotation_client_client;

	// ROS Service Client - sampleStateFromImage_client_client
	ros::ServiceClient sampleStateFromImage_client_client;


};


#endif
