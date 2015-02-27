#ifndef IMAGE_SENSOR_HPP
#define IMAGE_SENSOR_HPP

#include "ros/ros.h"
#include "Component.hpp"

// --------------------------------
//      USER INCLUDES GO HERE
// --------------------------------
//# Start User Includes Marker
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/wait.h>
#include <unistd.h>
#include <jpeglib.h>
#include <time.h>
#include <linux/videodev2.h>
#include <libv4l2.h>
//# End User Includes Marker

#include "agse_package/controlInputs.h"
#include "agse_package/captureImage.h"

//# Start User Globals Marker

//# End User Globals Marker

class image_sensor : public Component
{
    public:
        // Component image_sensor Constructor
        image_sensor(std::string nodeName, int argc, char **argv) : Component(nodeName, argc, argv) {}

        // These functions' business logic will be filled in by the user:

	// Init() is always generated
	void Init(const ros::TimerEvent& event);

	// OnOneData Subscription handler for controlInputs_sub subscriber 
	void controlInputs_sub_OnOneData(const agse_package::controlInputs::ConstPtr& received_data); 
 

	// Component Service Callback
	bool captureImageCallback(agse_package::captureImage::Request &req,
		agse_package::captureImage::Response &res);


	// these functions' business logic will be auto-generated:

	// startUp() is used to configure timers, publishers, & service providers
	void startUp();

	// required for clean shutdown
	~image_sensor();

    private:

	// ROS Subscriber - controlInputs_sub
	ros::Subscriber controlInputs_sub; 


	// ROS Service Server - captureImage_server
	ros::ServiceServer captureImage_server;


        //# Start User Private Variables Marker
        bool paused;
        char videoDevice[50];
        int width;
        int height;
        //# End User Private Variables Marker
};


#endif
