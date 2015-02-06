#ifndef IMAGE_PROCESSOR_HPP
#define IMAGE_PROCESSOR_HPP

#include "ros/ros.h"
#include "Component.hpp"

// --------------------------------
//      USER INCLUDES GO HERE
// --------------------------------
//# Start User Includes Marker

//# End User Includes Marker

#include "agse_package/controlInputs.h"
#include "agse_package/sampleStateFromImage.h"

class image_processor : public Component
{
    public:

        // These functions' business logic will be filled in by the user:

	// Init() is always generated
	void Init(const ros::TimerEvent& event);

	// OnOneData Subscription handler for controlInputs_sub subscriber 
	void controlInputs_sub_OnOneData(const agse_package::controlInputs::ConstPtr& received_data); 
 

	// Component Service Callback
	bool sampleStateFromImage_serverCallback(agse_package::sampleStateFromImage::Request  &req,
		agse_package::sampleStateFromImage::Response &res);


	// Callback for imageTimer timer
	void imageTimerCallback(const ros::TimerEvent& event);


	// these functions' business logic will be auto-generated:

	// startUp() is used to configure timers, publishers, & service providers
	void startUp();

	// required for clean shutdown
	~image_processor();

    private:

	// ROS Timer - imageTimer
	ros::Timer imageTimer;


	// ROS Subscriber - controlInputs_sub
	ros::Subscriber controlInputs_sub; 


	// ROS Service Server - sampleStateFromImage_server_server
	ros::ServiceServer sampleStateFromImage_server_server;


        //# Start User Private Variables Marker

        //# End User Private Variables Marker
};


#endif
