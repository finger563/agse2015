#include "agse_package/servo_controller.hpp"
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
void servo_controller::Init(const ros::TimerEvent& event)
{
    // Initialize Component
  if (serialPort.connect(portName)!=0) {
  }
  else {
    ROS_INFO ("Can't open serial port");
  }
  paused = true;
    // Stop Init Timer
    initOneShotTimer.stop();
}

// OnOneData Subscription handler for controlInputs_sub subscriber
void servo_controller::controlInputs_sub_OnOneData(const agse_package::controlInputs::ConstPtr& received_data)
{
    // Business Logic for controlInputs_sub subscriber subscribing to topic controlInputs callback 
  paused = received_data->paused;
}

// Component Service Callback
bool servo_controller::armRotation_serverCallback(agse_package::armRotation::Request  &req,
    agse_package::armRotation::Response &res)
{
    // Business Logic for armRotation_server Server providing armRotation Service

}
// Component Service Callback
bool servo_controller::gripperPos_serverCallback(agse_package::gripperPos::Request  &req,
    agse_package::gripperPos::Response &res)
{
    // Business Logic for gripperPos_server Server providing gripperPos Service

}
// Component Service Callback
bool servo_controller::gripperRotation_serverCallback(agse_package::gripperRotation::Request  &req,
    agse_package::gripperRotation::Response &res)
{
    // Business Logic for gripperRotation_server Server providing gripperRotation Service

}

// Callback for servoTimer timer
void servo_controller::servoTimerCallback(const ros::TimerEvent& event)
{
    // Business Logic for servoTimer 
  if (!paused) {
    myLedState = !myLedState;
    if (pos==myPosition1)
      pos = myPosition2;
    else
      pos = myPosition1;
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
}

// ---------------------------------------------
// EVERYTHING BELOW HERE IS COMPLETELY GENERATED
// ---------------------------------------------

// Destructor - required for clean shutdown when process is killed
servo_controller::~servo_controller()
{
    servoTimer.stop();
    controlInputs_sub.shutdown();
    armRotation_server_server.shutdown();
    gripperPos_server_server.shutdown();
    gripperRotation_server_server.shutdown();
}

void servo_controller::startUp()
{
    ros::NodeHandle nh;

    // Configure all subscribers associated with this component
    // subscriber: controlInputs_sub
    ros::SubscribeOptions controlInputs_sub_options;
    controlInputs_sub_options = 
	ros::SubscribeOptions::create<agse_package::controlInputs>
	    ("controlInputs",
	     1000,
	     boost::bind(&servo_controller::controlInputs_sub_OnOneData, this, _1),
	     ros::VoidPtr(),
             &this->compQueue);
    this->controlInputs_sub = nh.subscribe(controlInputs_sub_options);

    // Configure all provided services associated with this component
    // server: armRotation_server
    ros::AdvertiseServiceOptions armRotation_server_server_options;
    armRotation_server_server_options = 
	ros::AdvertiseServiceOptions::create<agse_package::armRotation>
	    ("armRotation",
             boost::bind(&servo_controller::armRotation_serverCallback, this, _1, _2),
	     ros::VoidPtr(),
             &this->compQueue);
    this->armRotation_server_server = nh.advertiseService(armRotation_server_server_options);
    // server: gripperPos_server
    ros::AdvertiseServiceOptions gripperPos_server_server_options;
    gripperPos_server_server_options = 
	ros::AdvertiseServiceOptions::create<agse_package::gripperPos>
	    ("gripperPos",
             boost::bind(&servo_controller::gripperPos_serverCallback, this, _1, _2),
	     ros::VoidPtr(),
             &this->compQueue);
    this->gripperPos_server_server = nh.advertiseService(gripperPos_server_server_options);
    // server: gripperRotation_server
    ros::AdvertiseServiceOptions gripperRotation_server_server_options;
    gripperRotation_server_server_options = 
	ros::AdvertiseServiceOptions::create<agse_package::gripperRotation>
	    ("gripperRotation",
             boost::bind(&servo_controller::gripperRotation_serverCallback, this, _1, _2),
	     ros::VoidPtr(),
             &this->compQueue);
    this->gripperRotation_server_server = nh.advertiseService(gripperRotation_server_server_options);
 
    // Create Init Timer
    ros::TimerOptions timer_options;
    timer_options = 
	ros::TimerOptions
	    (ros::Duration(-1),
	     boost::bind(&servo_controller::Init, this, _1),
	     &this->compQueue,
             true);
    this->initOneShotTimer = nh.createTimer(timer_options);  
  
    // Create all component timers
    // timer: timer.name
    timer_options = 
	ros::TimerOptions
             (ros::Duration(1.0),
	     boost::bind(&servo_controller::servoTimerCallback, this, _1),
	     &this->compQueue);
    this->servoTimer = nh.createTimer(timer_options);

}
