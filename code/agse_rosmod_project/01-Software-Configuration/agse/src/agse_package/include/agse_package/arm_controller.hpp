#ifndef ARM_CONTROLLER_HPP
#define ARM_CONTROLLER_HPP

#include "ros/ros.h"
#include "Component.hpp"

// --------------------------------
//      USER INCLUDES GO HERE
// --------------------------------
//# Start User Includes Marker
#include "agse_package/detected_object.hpp"
#include "agse_package/gpio.h"
//# End User Includes Marker

#include "agse_package/sampleState.h"
#include "agse_package/payloadBayState.h"
#include "agse_package/controlInputs.h"
#include "agse_package/sampleStateFromImage.h"
#include "agse_package/radialPos.h"
#include "agse_package/armRotation.h"
#include "agse_package/gripperRotation.h"
#include "agse_package/verticalPos.h"
#include "agse_package/gripperPos.h"
#include "agse_package/payloadBayStateFromImage.h"

//# Start User Globals Marker

//# End User Globals Marker

class arm_controller : public Component
{
    public:
        // Component arm_controller Constructor
        arm_controller(std::string hostName, std::string nodeName, std::string compName, int argc, char **argv) : Component(hostName, nodeName, compName, argc, argv) {}

        // These functions' business logic will be filled in by the user:

	// Init() is always generated
	void Init(const ros::TimerEvent& event);

	// OnOneData Subscription handler for controlInputs_sub subscriber 
	void controlInputs_sub_OnOneData(const agse_package::controlInputs::ConstPtr& received_data); 
 

	// Callback for armTimer timer
	void armTimerCallback(const ros::TimerEvent& event);


	// these functions' business logic will be auto-generated:

	// startUp() is used to configure timers, publishers, & service providers
	void startUp();

	// required for clean shutdown
	~arm_controller();

    private:

	// ROS Timer - armTimer
	ros::Timer armTimer;


	// ROS Subscriber - controlInputs_sub
	ros::Subscriber controlInputs_sub; 


	// ROS Publisher - sampleState_pub
	ros::Publisher sampleState_pub;

	// ROS Publisher - payloadBayState_pub
	ros::Publisher payloadBayState_pub;


	// ROS Service Client - sampleStateFromImage_client
	ros::ServiceClient sampleStateFromImage_client;

	// ROS Service Client - radialPos_client
	ros::ServiceClient radialPos_client;

	// ROS Service Client - armRotation_client
	ros::ServiceClient armRotation_client;

	// ROS Service Client - gripperRotation_client
	ros::ServiceClient gripperRotation_client;

	// ROS Service Client - verticalPos_client
	ros::ServiceClient verticalPos_client;

	// ROS Service Client - gripperPos_client
	ros::ServiceClient gripperPos_client;

	// ROS Service Client - payloadBayStateFromImage_client
	ros::ServiceClient payloadBayStateFromImage_client;

        //# Start User Private Variables Marker
  bool paused;
  
  enum ArmState
    {
      INIT,
      FINDING_PB,
      OPENING_PB,
      FINDING_SAMPLE,
      GRABBING_SAMPLE,
      CARRYING_SAMPLE,
      INSERTING_SAMPLE,
      CLOSING_PB,
      MOVING_AWAY
    };

  void UpdateSensorData();
  void UpdateArmPosition();
  bool CheckGoals();

  ArmState currentState;
  agse_package::sampleState sample;
  agse_package::payloadBayState payloadBay;

  void Init_StateFunc();
  void Finding_PB_StateFunc();
  void Opening_PB_StateFunc();
  void Finding_Sample_StateFunc();
  void Grabbing_Sample_StateFunc();
  void Carrying_Sample_StateFunc();
  void Inserting_Sample_StateFunc();
  void Closing_PB_StateFunc();
  void Moving_Away_StateFunc();
  
  float maxRadialPos;
  float maxVerticalPos;
  float maxArmRotation;
  float maxGripperRotation;
  float maxGripperPos;

  float minRadialPos;
  float minVerticalPos;
  float minArmRotation;
  float minGripperRotation;
  float minGripperPos;

  float gripperPosOpened;
  float gripperPosClosed;
  
  float currentRadialPos;
  float currentVerticalPos;
  float currentArmRotation;
  float currentGripperRotation;
  float currentGripperPos;

  float goalRadialPos;
  float goalVerticalPos;
  float goalArmRotation;
  float goalGripperRotation;
  float goalGripperPos;

  float radialEpsilon;
  float verticalEpsilon;
  float armRotationEpsilon;
  float gripperRotationEpsilon;
  float gripperPosEpsilon;

  float radialOffset;   // should be 0
  float verticalOffset; // should be 0
  float armRotationOffset; // difference between our 0 angle (arm frame) and servo's 0 angle
  float gripperRotationOffset; // difference between our 0 angle (plane of radius) and servo's 0 angle
  float gripperPosOffset; // difference between our 0 angle (open) and the servo's 0 angle

  bool call_at_timer; // We want Init_StateFunc to be called at each timer tick (not till CheckGoals = true)

        //# End User Private Variables Marker
};


#endif
