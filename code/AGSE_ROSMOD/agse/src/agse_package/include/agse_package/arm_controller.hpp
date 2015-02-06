#ifndef ARM_CONTROLLER_HPP
#define ARM_CONTROLLER_HPP

#include "ros/ros.h"
#include "Component.hpp"

// --------------------------------
//      USER INCLUDES GO HERE
// --------------------------------
//# Start User Includes Marker

//# End User Includes Marker

#include "agse_package/controlInputs.h"
#include "agse_package/sampleStateFromImage.h"
#include "agse_package/radialPos.h"
#include "agse_package/armRotation.h"
#include "agse_package/gripperRotation.h"
#include "agse_package/verticalPos.h"
#include "agse_package/gripperPos.h"

//# Start User Globals Marker

//# End User Globals Marker

class arm_controller : public Component
{
    public:

        // These functions' business logic will be filled in by the user:

	// Init() is always generated
	void Init(const ros::TimerEvent& event);

	// OnOneData Subscription handler for controlInputs_sub subscriber 
	void controlInputs_sub_OnOneData(const agse_package::controlInputs::ConstPtr& received_data); 
 

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


	// ROS Subscriber - controlInputs_sub
	ros::Subscriber controlInputs_sub; 


	// ROS Service Client - sampleStateFromImage_client_client
	ros::ServiceClient sampleStateFromImage_client_client;

	// ROS Service Client - radialPos_client_client
	ros::ServiceClient radialPos_client_client;

	// ROS Service Client - armRotation_client_client
	ros::ServiceClient armRotation_client_client;

	// ROS Service Client - gripperRotation_client_client
	ros::ServiceClient gripperRotation_client_client;

	// ROS Service Client - verticalPos_client_client
	ros::ServiceClient verticalPos_client_client;

	// ROS Service Client - gripperPos_client_client
	ros::ServiceClient gripperPos_client_client;

        //# Start User Private Variables Marker

        //# End User Private Variables Marker
};


#endif
