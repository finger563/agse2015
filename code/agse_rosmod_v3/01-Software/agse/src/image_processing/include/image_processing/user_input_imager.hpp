#ifndef USER_INPUT_IMAGER_HPP
#define USER_INPUT_IMAGER_HPP

#include "ros/ros.h"
#include "Component.hpp"
#include "image_processing/sampleDetectionImages.h"
#include "image_processing/payloadBayDetectionImages.h"
#include "high_level_control/sampleState.h"
#include "high_level_control/payloadBayState.h"
#include "image_processing/captureImage.h"
//# Start User Includes Marker
//# End User Includes Marker

//# Start User Globals Marker
//# End User Globals Marker

class user_input_imager : public Component
{
public:
  // Constructor
  user_input_imager(ComponentConfig& config, int argc, char **argv) : Component(config, argc, argv) {}

  // Initialization
  void Init(const ros::TimerEvent& event);

  // Subscriber Callback - sampleDetectionImages_sub
  void sampleDetectionImages_sub_OnOneData(const image_processing::sampleDetectionImages::ConstPtr& received_data); 
 
  // Subscriber Callback - payloadBayDetectionImages_sub
  void payloadBayDetectionImages_sub_OnOneData(const image_processing::payloadBayDetectionImages::ConstPtr& received_data); 
 
  // Subscriber Callback - sampleState_sub
  void sampleState_sub_OnOneData(const high_level_control::sampleState::ConstPtr& received_data); 
 
  // Subscriber Callback - payloadBayState_sub
  void payloadBayState_sub_OnOneData(const high_level_control::payloadBayState::ConstPtr& received_data); 
 
  // Timer Callback - uiImage_timer
  void uiImage_timerCallback(const ros::TimerEvent& event);

  // Start up
  void startUp();

  // Destructor
  ~user_input_imager();

private:

  // Timer
  ros::Timer uiImage_timer;

  // Subscriber
  ros::Subscriber sampleDetectionImages_sub; 

  // Subscriber
  ros::Subscriber payloadBayDetectionImages_sub; 

  // Subscriber
  ros::Subscriber sampleState_sub; 

  // Subscriber
  ros::Subscriber payloadBayState_sub; 

  // Client 
  ros::ServiceClient captureImage_client;

  //# Start User Private Variables Marker

  //# End User Private Variables Marker
};


#endif
