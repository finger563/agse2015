#ifndef SERVO_CONTROLLER_HPP
#define SERVO_CONTROLLER_HPP

#include "ros/ros.h"
#include "Component.hpp"

// --------------------------------
//      USER INCLUDES GO HERE
// --------------------------------
//# Start User Includes Marker
#include "agse_package/Dynamixel.h"
#include "agse_package/SerialPort.h"
//# End User Includes Marker

#include "agse_package/controlInputs.h"
#include "agse_package/armRotation.h"
#include "agse_package/gripperPos.h"
#include "agse_package/gripperRotation.h"

//# Start User Globals Marker

//# End User Globals Marker

class servo_controller : public Component
{
    public:
        // Component servo_controller Constructor
        servo_controller(std::string hostName, std::string nodeName, std::string compName, int argc, char **argv) : Component(hostName, nodeName, compName, argc, argv) {}

        // These functions' business logic will be filled in by the user:

	// Init() is always generated
	void Init(const ros::TimerEvent& event);

	// OnOneData Subscription handler for controlInputs_sub subscriber 
	void controlInputs_sub_OnOneData(const agse_package::controlInputs::ConstPtr& received_data); 
 

	// Component Service Callback
	bool armRotationCallback(agse_package::armRotation::Request &req,
		agse_package::armRotation::Response &res);

	// Component Service Callback
	bool gripperPosCallback(agse_package::gripperPos::Request &req,
		agse_package::gripperPos::Response &res);

	// Component Service Callback
	bool gripperRotationCallback(agse_package::gripperRotation::Request &req,
		agse_package::gripperRotation::Response &res);


	// Callback for servoTimer timer
	void servoTimerCallback(const ros::TimerEvent& event);


	// these functions' business logic will be auto-generated:

	// startUp() is used to configure timers, publishers, & service providers
	void startUp();

	// required for clean shutdown
	~servo_controller();

    private:

	// ROS Timer - servoTimer
	ros::Timer servoTimer;


	// ROS Subscriber - controlInputs_sub
	ros::Subscriber controlInputs_sub; 


	// ROS Service Server - armRotation_server
	ros::ServiceServer armRotation_server;

	// ROS Service Server - gripperPos_server
	ros::ServiceServer gripperPos_server;

	// ROS Service Server - gripperRotation_server
	ros::ServiceServer gripperRotation_server;


        //# Start User Private Variables Marker
  bool paused;

  // serial port we use on the Jetson TK1
  SerialPort serialPort;
  char portName[50];
  // object for reading/writing to AX-12A servos
  Dynamixel dynamixel;

  // IDs for the servo motors
  int armServoID;
  int gripperRotationID;
  int gripperPositionID;

  // goal Position for the arm servo
  float armRotationGoal;
  // current position of the arm servo
  float armRotationCurrent;

  // goal Position for the gripperRotation servo
  float gripperRotationGoal;
  // current position of the gripperRotation servo
  float gripperRotationCurrent;

  // goal Position for the gripperPos servo
  float gripperPosGoal;
  // current position of the gripperPos servo
  float gripperPosCurrent;
        //# End User Private Variables Marker
};


#endif
