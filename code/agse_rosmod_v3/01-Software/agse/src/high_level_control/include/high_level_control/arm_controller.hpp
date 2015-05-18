#ifndef ARM_CONTROLLER_HPP
#define ARM_CONTROLLER_HPP

#include "ros/ros.h"
#include "Component.hpp"
#include "high_level_control/armState.h"
#include "high_level_control/sampleState.h"
#include "high_level_control/payloadBayState.h"
#include "high_level_control/controlInputs.h"
#include "motor_control/verticalPos.h"
#include "motor_control/radialPos.h"
#include "motor_control/armRotation.h"
#include "motor_control/gripperRotation.h"
#include "motor_control/gripperPos.h"
#include "image_processing/sampleStateFromImage.h"
#include "image_processing/payloadBayStateFromImage.h"
//# Start User Includes Marker
//# End User Includes Marker

//# Start User Globals Marker
//# End User Globals Marker

class arm_controller : public Component
{
public:
  // Constructor
  arm_controller(ComponentConfig& config, int argc, char **argv) : Component(config, argc, argv) {}

  // Initialization
  void Init(const ros::TimerEvent& event);

  // Subscriber Callback - controlInputs_sub
  void controlInputs_sub_OnOneData(const high_level_control::controlInputs::ConstPtr& received_data); 
 
  // Timer Callback - armTimer
  void armTimerCallback(const ros::TimerEvent& event);

  // Start up
  void startUp();

  // Destructor
  ~arm_controller();

private:

  // Timer
  ros::Timer armTimer;

  // Subscriber
  ros::Subscriber controlInputs_sub; 

  // Publisher 
  ros::Publisher armState_pub;

  // Publisher 
  ros::Publisher sampleState_pub;

  // Publisher 
  ros::Publisher payloadBayState_pub;

  // Client 
  ros::ServiceClient verticalPos_client;

  // Client 
  ros::ServiceClient radialPos_client;

  // Client 
  ros::ServiceClient armRotation_client;

  // Client 
  ros::ServiceClient gripperRotation_client;

  // Client 
  ros::ServiceClient gripperPos_client;

  // Client 
  ros::ServiceClient sampleStateFromImage_client;

  // Client 
  ros::ServiceClient payloadBayStateFromImage_client;

  //# Start User Private Variables Marker

  //# End User Private Variables Marker
};


#endif
