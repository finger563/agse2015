#include "agse_package/gripper_controller.hpp"

// -------------------------------------------------------
// BUSINESS LOGIC OF THESE FUNCTIONS SUPPLIED BY DEVELOPER
// -------------------------------------------------------

// Init Function
void gripper_controller::Init(const ros::TimerEvent& event)
{
    // Initialize Component

    // Stop Init Timer
    initOneShotTimer.stop();
}

// Component Service Callback
bool gripper_controller::setGripperPos_serverCallback(agse_package::setGripperPos::Request  &req,
    agse_package::setGripperPos::Response &res)
{
    // Business Logic for <listener.ROS_Server instance at 0xb5429328> Service
}
// Component Service Callback
bool gripper_controller::setGripperRotation_serverCallback(agse_package::setGripperRotation::Request  &req,
    agse_package::setGripperRotation::Response &res)
{
    // Business Logic for <listener.ROS_Server instance at 0xb54293a0> Service
}

// ---------------------------------------------
// EVERYTHING BELOW HERE IS COMPLETELY GENERATED
// ---------------------------------------------

// Destructor - required for clean shutdown when process is killed
gripper_controller::~gripper_controller()
{
    setGripperPos_server_server.shutdown();
    setGripperRotation_server_server.shutdown();
}

void gripper_controller::startUp()
{
    ros::NodeHandle nh;

    // Configure all provided services associated with this component
    ros::AdvertiseServiceOptions setGripperPos_server_server_options;
    setGripperPos_server_server_options = 
	ros::AdvertiseServiceOptions::create<agse_package::setGripperPos>
	    ("setGripperPos",
             boost::bind(&gripper_controller::setGripperPos_serverCallback, this, _1, _2),
	     ros::VoidPtr(),
             &this->compQueue);
    this->setGripperPos_server_server = nh.advertiseService(setGripperPos_server_server_options);
    // Configure all provided services associated with this component
    ros::AdvertiseServiceOptions setGripperRotation_server_server_options;
    setGripperRotation_server_server_options = 
	ros::AdvertiseServiceOptions::create<agse_package::setGripperRotation>
	    ("setGripperRotation",
             boost::bind(&gripper_controller::setGripperRotation_serverCallback, this, _1, _2),
	     ros::VoidPtr(),
             &this->compQueue);
    this->setGripperRotation_server_server = nh.advertiseService(setGripperRotation_server_server_options);
 
    // Create Init Timer
    ros::TimerOptions timer_options;
    timer_options = 
	ros::TimerOptions
	    (ros::Duration(-1),
	     boost::bind(&gripper_controller::Init, this, _1),
	     &this->compQueue,
             true);
    this->initOneShotTimer = nh.createTimer(timer_options);  
  
}
