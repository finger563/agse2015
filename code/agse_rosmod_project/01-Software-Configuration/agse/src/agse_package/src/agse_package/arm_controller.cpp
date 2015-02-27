#include "agse_package/arm_controller.hpp"

//# Start User Globals Marker

//# End User Globals Marker

// -------------------------------------------------------
// BUSINESS LOGIC OF THESE FUNCTIONS SUPPLIED BY DEVELOPER
// -------------------------------------------------------

// Init Function
//# Start Init Marker
void arm_controller::Init(const ros::TimerEvent& event)
{
    // Initialize Component

    // Stop Init Timer
    initOneShotTimer.stop();
}
//# End Init Marker

// OnOneData Subscription handler for controlInputs_sub subscriber
//# Start controlInputs_sub_OnOneData Marker
void arm_controller::controlInputs_sub_OnOneData(const agse_package::controlInputs::ConstPtr& received_data)
{
    // Business Logic for controlInputs_sub subscriber subscribing to topic controlInputs callback 

}
//# End controlInputs_sub_OnOneData Marker

// Callback for armTimer timer
//# Start armTimerCallback Marker
void arm_controller::armTimerCallback(const ros::TimerEvent& event)
{
    // Business Logic for armTimer 

}
//# End armTimerCallback Marker

// ---------------------------------------------
// EVERYTHING BELOW HERE IS COMPLETELY GENERATED
// ---------------------------------------------

// Destructor - required for clean shutdown when process is killed
arm_controller::~arm_controller()
{
    armTimer.stop();
    controlInputs_sub.shutdown();
    sampleStateFromImage_client.shutdown();
    radialPos_client.shutdown();
    armRotation_client.shutdown();
    gripperRotation_client.shutdown();
    verticalPos_client.shutdown();
    gripperPos_client.shutdown();
//# Start Destructor Marker

//# End Destructor Marker
}

void arm_controller::startUp()
{
    ros::NodeHandle nh;

    // Configure all subscribers associated with this component
    // subscriber: controlInputs_sub
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
    // client: sampleStateFromImage_client
    this->sampleStateFromImage_client = nh.serviceClient<agse_package::sampleStateFromImage>
	("sampleStateFromImage"); 
    // client: radialPos_client
    this->radialPos_client = nh.serviceClient<agse_package::radialPos>
	("radialPos"); 
    // client: armRotation_client
    this->armRotation_client = nh.serviceClient<agse_package::armRotation>
	("armRotation"); 
    // client: gripperRotation_client
    this->gripperRotation_client = nh.serviceClient<agse_package::gripperRotation>
	("gripperRotation"); 
    // client: verticalPos_client
    this->verticalPos_client = nh.serviceClient<agse_package::verticalPos>
	("verticalPos"); 
    // client: gripperPos_client
    this->gripperPos_client = nh.serviceClient<agse_package::gripperPos>
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
    // timer: timer.properties["name"]
    timer_options = 
	ros::TimerOptions
             (ros::Duration(0.02),
	     boost::bind(&arm_controller::armTimerCallback, this, _1),
	     &this->compQueue);
    this->armTimer = nh.createTimer(timer_options);

}
