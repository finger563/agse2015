#ifndef IMAGE_PROCESSOR_HPP
#define IMAGE_PROCESSOR_HPP

#include "ros/ros.h"
#include "Component.hpp"

// --------------------------------
//      USER INCLUDES GO HERE
// --------------------------------
//# Start User Includes Marker
#include "agse_package/sample_detector.hpp"
#include "agse_package/payloadbay_detector.hpp"
//# End User Includes Marker

#include "agse_package/controlInputs.h"
#include "agse_package/captureImage.h"
#include "agse_package/sampleStateFromImage.h"
#include "agse_package/payloadBayStateFromImage.h"

//# Start User Globals Marker

//# End User Globals Marker

class image_processor : public Component
{
    public:
        // Component image_processor Constructor
        image_processor(std::string nodeName, int argc, char **argv) : Component(nodeName, argc, argv) {}

        // These functions' business logic will be filled in by the user:

	// Init() is always generated
	void Init(const ros::TimerEvent& event);

	// OnOneData Subscription handler for controlInputs_sub subscriber 
	void controlInputs_sub_OnOneData(const agse_package::controlInputs::ConstPtr& received_data); 
 

	// Component Service Callback
	bool sampleStateFromImageCallback(agse_package::sampleStateFromImage::Request &req,
		agse_package::sampleStateFromImage::Response &res);

	// Component Service Callback
	bool payloadBayStateFromImageCallback(agse_package::payloadBayStateFromImage::Request &req,
		agse_package::payloadBayStateFromImage::Response &res);


	// these functions' business logic will be auto-generated:

	// startUp() is used to configure timers, publishers, & service providers
	void startUp();

	// required for clean shutdown
	~image_processor();

    private:

	// ROS Subscriber - controlInputs_sub
	ros::Subscriber controlInputs_sub; 


	// ROS Service Server - sampleStateFromImage_server
	ros::ServiceServer sampleStateFromImage_server;

	// ROS Service Server - payloadBayStateFromImage_server
	ros::ServiceServer payloadBayStateFromImage_server;


	// ROS Service Client - captureImage_client
	ros::ServiceClient captureImage_client;

        //# Start User Private Variables Marker
  bool paused;
  Sample_Detector sampleDetector;
  PayloadBay_Detector payloadBayDetector;
        //# End User Private Variables Marker
};


#endif
