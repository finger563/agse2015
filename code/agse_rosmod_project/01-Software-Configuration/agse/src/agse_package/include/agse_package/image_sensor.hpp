#ifndef IMAGE_SENSOR_HPP
#define IMAGE_SENSOR_HPP

#include "ros/ros.h"
#include "Component.hpp"

// --------------------------------
//      USER INCLUDES GO HERE
// --------------------------------
//# Start User Includes Marker

//# End User Includes Marker

#include "agse_package/captureImage.h"

//# Start User Globals Marker

//# End User Globals Marker

class image_sensor : public Component
{
    public:

        // These functions' business logic will be filled in by the user:

	// Init() is always generated
	void Init(const ros::TimerEvent& event);

	// Component Service Callback
	bool captureImageCallback(agse_package::captureImage::Request &req,
		agse_package::captureImage::Response &res);


	// these functions' business logic will be auto-generated:

	// startUp() is used to configure timers, publishers, & service providers
	void startUp();

	// required for clean shutdown
	~image_sensor();

    private:

	// ROS Service Server - captureImage_server
	ros::ServiceServer captureImage_server;


        //# Start User Private Variables Marker

        //# End User Private Variables Marker
};


#endif
