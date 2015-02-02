#include "agse_package/arm_rotation_controller.hpp"
#include "agse_package/SerialPort.h"
#include "agse_package/Dynamixel.h"

// -------------------------------------------------------
// BUSINESS LOGIC OF THESE FUNCTIONS SUPPLIED BY DEVELOPER
// -------------------------------------------------------

int ax12_base_id=1;
int ax12_gripper_id=10;

bool myLedState = true;
int myPosition1 = 100;
int myPosition2 = 500;
int pos = myPosition1;

char portName[] = "//dev//ttyTHS0";

SerialPort serialPort;
Dynamixel dynamixel;

// Init Function
void arm_rotation_controller::Init(const ros::TimerEvent& event)
{
    // Initialize Component

  if (serialPort.connect(portName)!=0) {
  }
  else {
    ROS_INFO ("Can't open serial port");
  }


    // Stop Init Timer
    initOneShotTimer.stop();
}

// OnOneData Subscription handler for controlInputs_sub subscriber
void arm_rotation_controller::controlInputs_sub_OnOneData(const agse_package::controlInputs::ConstPtr& received_data)
{
    // Business Logic for controlInputs_sub subscriber callback 
}

// Component Service Callback
bool arm_rotation_controller::armRotation_serverCallback(agse_package::armRotation::Request  &req,
    agse_package::armRotation::Response &res)
{
    // Business Logic for <listener.ROS_Server instance at 0xb53effa8> Service
}

// Callback for armRotationTimer timer
void arm_rotation_controller::armRotationTimerCallback(const ros::TimerEvent& event)
{
  myLedState = !myLedState;
  if (pos==myPosition1)
    pos = myPosition2;
  else
    pos = myPosition1;
  // Business Logic for armRotationTimer 
  int retVal;
  ROS_INFO("\nSERVO ID %d:",ax12_base_id);
  int pos1=dynamixel.getPosition(&serialPort, ax12_base_id);
  retVal = dynamixel.getSetLedCommand(&serialPort, ax12_base_id, !myLedState);
  dynamixel.setPosition(&serialPort, ax12_base_id, pos+100);
  ROS_INFO("\nSERVO ID %d:",ax12_gripper_id);
  int pos2=dynamixel.getPosition(&serialPort, ax12_gripper_id);
  retVal = dynamixel.getSetLedCommand(&serialPort, ax12_gripper_id, myLedState);
  dynamixel.setPosition(&serialPort, ax12_gripper_id, pos);
}

// ---------------------------------------------
// EVERYTHING BELOW HERE IS COMPLETELY GENERATED
// ---------------------------------------------

// Destructor - required for clean shutdown when process is killed
arm_rotation_controller::~arm_rotation_controller()
{
    armRotationTimer.stop();
    controlInputs_sub.shutdown();
    armRotation_server_server.shutdown();
    serialPort.disconnect();
}

void arm_rotation_controller::startUp()
{
    ros::NodeHandle nh;

    // Configure all subscribers associated with this component
    ros::SubscribeOptions controlInputs_sub_options;
    controlInputs_sub_options = 
	ros::SubscribeOptions::create<agse_package::controlInputs>
	    ("controlInputs",
	     1000,
	     boost::bind(&arm_rotation_controller::controlInputs_sub_OnOneData, this, _1),
	     ros::VoidPtr(),
             &this->compQueue);
    this->controlInputs_sub = nh.subscribe(controlInputs_sub_options);

    // Configure all provided services associated with this component
    ros::AdvertiseServiceOptions armRotation_server_server_options;
    armRotation_server_server_options = 
	ros::AdvertiseServiceOptions::create<agse_package::armRotation>
	    ("armRotation",
             boost::bind(&arm_rotation_controller::armRotation_serverCallback, this, _1, _2),
	     ros::VoidPtr(),
             &this->compQueue);
    this->armRotation_server_server = nh.advertiseService(armRotation_server_server_options);
 
    // Create Init Timer
    ros::TimerOptions timer_options;
    timer_options = 
	ros::TimerOptions
	    (ros::Duration(-1),
	     boost::bind(&arm_rotation_controller::Init, this, _1),
	     &this->compQueue,
             true);
    this->initOneShotTimer = nh.createTimer(timer_options);  
  
    // Create all component timers
    timer_options = 
	ros::TimerOptions
             (ros::Duration(1),
	     boost::bind(&arm_rotation_controller::armRotationTimerCallback, this, _1),
	     &this->compQueue);
    this->armRotationTimer = nh.createTimer(timer_options);

}
