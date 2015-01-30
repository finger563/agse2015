#include "agse_package/arm_controller.hpp"

// -------------------------------------------------------
// BUSINESS LOGIC OF THESE FUNCTIONS SUPPLIED BY DEVELOPER
// -------------------------------------------------------

// Init Function
void arm_controller::Init(const ros::TimerEvent& event)
{
    // Initialize Component

    // Stop Init Timer
    initOneShotTimer.stop();
}

// Callback for armTimer timer
void arm_controller::armTimerCallback(const ros::TimerEvent& event)
{
    // Business Logic for armTimer 
}

// ---------------------------------------------
// EVERYTHING BELOW HERE IS COMPLETELY GENERATED
// ---------------------------------------------

// Destructor - required for clean shutdown when process is killed
arm_controller::~arm_controller()
{
    armTimer.stop();
    setRadialPos_client_client.shutdown();
    setVerticalPos_client_client.shutdown();
    setGripperPos_client_client.shutdown();
    setGripperRotation_client_client.shutdown();
    setArmRotation_client_client.shutdown();
    sampleStateFromImage_client_client.shutdown();
}

void arm_controller::startUp()
{
    ros::NodeHandle nh;

    // Configure all required services associated with this component
    this->setRadialPos_client_client = nh.serviceClient<agse_package::setRadialPos>
	("setRadialPos"); 
    // Configure all required services associated with this component
    this->setVerticalPos_client_client = nh.serviceClient<agse_package::setVerticalPos>
	("setVerticalPos"); 
    // Configure all required services associated with this component
    this->setGripperPos_client_client = nh.serviceClient<agse_package::setGripperPos>
	("setGripperPos"); 
    // Configure all required services associated with this component
    this->setGripperRotation_client_client = nh.serviceClient<agse_package::setGripperRotation>
	("setGripperRotation"); 
    // Configure all required services associated with this component
    this->setArmRotation_client_client = nh.serviceClient<agse_package::setArmRotation>
	("setArmRotation"); 
    // Configure all required services associated with this component
    this->sampleStateFromImage_client_client = nh.serviceClient<agse_package::sampleStateFromImage>
	("sampleStateFromImage"); 

    // Create Init Timer
    ros::TimerOptions timer_options;
    timer_options = 
	ros::TimerOptions
	    (ros::Duration(-1),
	     boost::bind(&arm_controller::Init, this, _1),
	     &this->compQueue,
             true);
    this->initOneShotTimer = nh.createTimer(timer_options);  
  
    // Create all component timers
    timer_options = 
	ros::TimerOptions
             (ros::Duration(0.02),
	     boost::bind(&arm_controller::armTimerCallback, this, _1),
	     &this->compQueue);
    this->armTimer = nh.createTimer(timer_options);

}
