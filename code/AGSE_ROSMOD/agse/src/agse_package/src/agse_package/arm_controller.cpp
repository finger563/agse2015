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

// OnOneData Subscription handler for controlInputs_sub subscriber
void arm_controller::controlInputs_sub_OnOneData(const agse_package::controlInputs::ConstPtr& received_data)
{
    // Business Logic for controlInputs_sub subscriber callback 
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
    controlInputs_sub.shutdown();
    sampleStateFromImage_client_client.shutdown();
    radialPos_client_client.shutdown();
    armRotation_client_client.shutdown();
    gripperRotation_client_client.shutdown();
    verticalPos_client_client.shutdown();
    gripperPos_client_client.shutdown();
}

void arm_controller::startUp()
{
    ros::NodeHandle nh;

    // Configure all subscribers associated with this component
    ros::SubscribeOptions controlInputs_sub_options;
    controlInputs_sub_options = 
	ros::SubscribeOptions::create<agse_package::controlInputs>
	    ("controlInputs",
	     1000,
	     boost::bind(&arm_controller::controlInputs_sub_OnOneData, this, _1),
	     ros::VoidPtr(),
             &this->compQueue);
    this->controlInputs_sub = nh.subscribe(controlInputs_sub_options);

    // Configure all required services associated with this component
    this->sampleStateFromImage_client_client = nh.serviceClient<agse_package::sampleStateFromImage>
	("sampleStateFromImage"); 
    // Configure all required services associated with this component
    this->radialPos_client_client = nh.serviceClient<agse_package::radialPos>
	("radialPos"); 
    // Configure all required services associated with this component
    this->armRotation_client_client = nh.serviceClient<agse_package::armRotation>
	("armRotation"); 
    // Configure all required services associated with this component
    this->gripperRotation_client_client = nh.serviceClient<agse_package::gripperRotation>
	("gripperRotation"); 
    // Configure all required services associated with this component
    this->verticalPos_client_client = nh.serviceClient<agse_package::verticalPos>
	("verticalPos"); 
    // Configure all required services associated with this component
    this->gripperPos_client_client = nh.serviceClient<agse_package::gripperPos>
	("gripperPos"); 

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