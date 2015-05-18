#ifndef IMAGE_PROCESSOR_HPP
#define IMAGE_PROCESSOR_HPP

#include "ros/ros.h"
#include "Component.hpp"
#include "image_processing/sampleDetectionImages.h"
#include "image_processing/payloadBayDetectionImages.h"
#include "high_level_control/controlInputs.h"
#include "image_processing/captureImage.h"
#include "image_processing/sampleStateFromImage.h"
#include "image_processing/payloadBayStateFromImage.h"
//# Start User Includes Marker
//# End User Includes Marker

//# Start User Globals Marker
//# End User Globals Marker

class image_processor : public Component
{
public:
  // Constructor
  image_processor(ComponentConfig& config, int argc, char **argv) : Component(config, argc, argv) {}

  // Initialization
  void Init(const ros::TimerEvent& event);

  // Subscriber Callback - controlInputs_sub
  void controlInputs_sub_OnOneData(const high_level_control::controlInputs::ConstPtr& received_data); 
 
  // Server Callback - sampleStateFromImage_server
  bool sampleStateFromImageCallback(image_processing::sampleStateFromImage::Request &req, 
    image_processing::sampleStateFromImage::Response &res);

  // Server Callback - payloadBayStateFromImage_server
  bool payloadBayStateFromImageCallback(image_processing::payloadBayStateFromImage::Request &req, 
    image_processing::payloadBayStateFromImage::Response &res);

  // Start up
  void startUp();

  // Destructor
  ~image_processor();

private:

  // Subscriber
  ros::Subscriber controlInputs_sub; 

  // Publisher 
  ros::Publisher sampleDetectionImages_pub;

  // Publisher 
  ros::Publisher payloadBayDetectionImages_pub;

  // Server 
  ros::ServiceServer sampleStateFromImage_server;

  // Server 
  ros::ServiceServer payloadBayStateFromImage_server;

  // Client 
  ros::ServiceClient captureImage_client;

  //# Start User Private Variables Marker
  //# End User Private Variables Marker
};


#endif
