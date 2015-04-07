#ifndef USER_INPUT_IMAGER_HPP
#define USER_INPUT_IMAGER_HPP

#include "ros/ros.h"
#include "Component.hpp"

// --------------------------------
//      USER INCLUDES GO HERE
// --------------------------------
//# Start User Includes Marker
#include "agse_package/uip.h"
using namespace cv;
//# End User Includes Marker

#include "agse_package/payloadBayDetectionImages.h"
#include "agse_package/sampleDetectionImages.h"
#include "agse_package/payloadBayState.h"
#include "agse_package/sampleState.h"
#include "agse_package/captureImage.h"

//# Start User Globals Marker

//# End User Globals Marker

class user_input_imager : public Component
{
    public:
        // Component user_input_imager Constructor
        user_input_imager(std::string hostName, std::string nodeName, std::string compName, int argc, char **argv) : Component(hostName, nodeName, compName, argc, argv) {}

        // These functions' business logic will be filled in by the user:

	// Init() is always generated
	void Init(const ros::TimerEvent& event);

	// OnOneData Subscription handler for payloadBayDetectionImages_sub subscriber 
	void payloadBayDetectionImages_sub_OnOneData(const agse_package::payloadBayDetectionImages::ConstPtr& received_data); 
 
	// OnOneData Subscription handler for sampleDetectionImages_sub subscriber 
	void sampleDetectionImages_sub_OnOneData(const agse_package::sampleDetectionImages::ConstPtr& received_data); 
 
	// OnOneData Subscription handler for payloadBayState_sub subscriber 
	void payloadBayState_sub_OnOneData(const agse_package::payloadBayState::ConstPtr& received_data); 
 
	// OnOneData Subscription handler for sampleState_sub subscriber 
	void sampleState_sub_OnOneData(const agse_package::sampleState::ConstPtr& received_data); 
 
	// Callback for uiImage_timer timer
	void uiImage_timerCallback(const ros::TimerEvent& event);

	// these functions' business logic will be auto-generated:

	// startUp() is used to configure timers, publishers, & service providers
	void startUp();

	// required for clean shutdown
	~user_input_imager();

    private:

	// ROS Timer - uiImage_timer
	ros::Timer uiImage_timer;


	// ROS Subscriber - payloadBayDetectionImages_sub
	ros::Subscriber payloadBayDetectionImages_sub; 

	// ROS Subscriber - sampleDetectionImages_sub
	ros::Subscriber sampleDetectionImages_sub; 

	// ROS Subscriber - payloadBayState_sub
	ros::Subscriber payloadBayState_sub; 

	// ROS Subscriber - sampleState_sub
	ros::Subscriber sampleState_sub; 

	// ROS Service Client - captureImage_client
	ros::ServiceClient captureImage_client;

        //# Start User Private Variables Marker

  // Received Images
  Mat pb_rawImage;
  Mat pb_hsvImage;
  Mat pb_gsImage;
  Mat pb_bitwise;

  Mat sample_rawImage;
  Mat sample_hsvImage;
  Mat sample_gsImage;
  Mat sample_bitwise;

  // Four Images to show in UIP
  IplImage * top_left;
  IplImage * top_right;
  IplImage * bottom_left;
  IplImage * bottom_right;

  // Fullscreen single images
  IplImage * camera_feed;
  IplImage * processed_image;

  // Keyboard interrupt
  int key;

        //# End User Private Variables Marker
};


#endif
