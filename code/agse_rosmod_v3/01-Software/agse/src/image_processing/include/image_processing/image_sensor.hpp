#ifndef IMAGE_SENSOR_HPP
#define IMAGE_SENSOR_HPP

#include "ros/ros.h"
#include "Component.hpp"
#include "high_level_control/controlInputs.h"
#include "image_processing/captureImage.h"
//# Start User Includes Marker
//# End User Includes Marker

//# Start User Globals Marker
//# End User Globals Marker

class image_sensor : public Component
{
public:
  // Constructor
  image_sensor(ComponentConfig& config, int argc, char **argv) : Component(config, argc, argv) {}

  // Initialization
  void Init(const ros::TimerEvent& event);

  // Subscriber Callback - controlInputs_sub
  void controlInputs_sub_OnOneData(const high_level_control::controlInputs::ConstPtr& received_data); 
 
  // Server Callback - captureImage_server
  bool captureImageCallback(image_processing::captureImage::Request &req, 
    image_processing::captureImage::Response &res);

  // Start up
  void startUp();

  // Destructor
  ~image_sensor();

private:

  // Subscriber
  ros::Subscriber controlInputs_sub; 

  // Server 
  ros::ServiceServer captureImage_server;

  //# Start User Private Variables Marker
  //# End User Private Variables Marker
};


#endif
