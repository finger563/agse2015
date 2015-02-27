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
#include <signal.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <jpeglib.h>
#include <time.h>
#include <linux/videodev2.h>
extern "C" {
#include "agse_package/v4l2uvc.h"
}
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
        bool paused;
        char videoDevice[50];
        int format;
        int width;
        int height;
        int brightness;
        int contrast;
        int saturation;
        int gain;
        int quality;
        struct vdIn *videoIn;
        int grabMethod;
        //# End User Private Variables Marker
};


#endif
