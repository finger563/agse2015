#include "agse_package/arm_rotation_controller.hpp"

// -------------------------------------------------------
// BUSINESS LOGIC OF THESE FUNCTIONS SUPPLIED BY DEVELOPER
// -------------------------------------------------------

// Init Function
void arm_rotation_controller::Init(const ros::TimerEvent& event)
{
    // Initialize Component

    // Stop Init Timer
    initOneShotTimer.stop();
}

// Component Service Callback
bool arm_rotation_controller::setArmRotation_serverCallback(agse_package::setArmRotation::Request  &req,
    agse_package::setArmRotation::Response &res)
{
    // Business Logic for <listener.ROS_Server instance at 0xb5429f08> Service
}

// ---------------------------------------------
// EVERYTHING BELOW HERE IS COMPLETELY GENERATED
// ---------------------------------------------

// Destructor - required for clean shutdown when process is killed
arm_rotation_controller::~arm_rotation_controller()
{
    setArmRotation_server_server.shutdown();
}

void arm_rotation_controller::startUp()
{
    ros::NodeHandle nh;

    // Configure all provided services associated with this component
    ros::AdvertiseServiceOptions setArmRotation_server_server_options;
    setArmRotation_server_server_options = 
	ros::AdvertiseServiceOptions::create<agse_package::setArmRotation>
	    ("setArmRotation",
             boost::bind(&arm_rotation_controller::setArmRotation_serverCallback, this, _1, _2),
	     ros::VoidPtr(),
             &this->compQueue);
    this->setArmRotation_server_server = nh.advertiseService(setArmRotation_server_server_options);
 
    // Create Init Timer
    ros::TimerOptions timer_options;
    timer_options = 
	ros::TimerOptions
	    (ros::Duration(-1),
	     boost::bind(&arm_rotation_controller::Init, this, _1),
	     &this->compQueue,
             true);
    this->initOneShotTimer = nh.createTimer(timer_options);  
  
}
