#include "agse_package/radial_actuator_controller.hpp"

// -------------------------------------------------------
// BUSINESS LOGIC OF THESE FUNCTIONS SUPPLIED BY DEVELOPER
// -------------------------------------------------------

// Init Function
void radial_actuator_controller::Init(const ros::TimerEvent& event)
{
    // Initialize Component

    // Stop Init Timer
    initOneShotTimer.stop();
}

// OnOneData Subscription handler for controlInputs_sub subscriber
void radial_actuator_controller::controlInputs_sub_OnOneData(const agse_package::controlInputs::ConstPtr& received_data)
{
    // Business Logic for controlInputs_sub subscriber subscribing to topic controlInputs callback 

}

// Component Service Callback
bool radial_actuator_controller::radialPos_serverCallback(agse_package::radialPos::Request  &req,
    agse_package::radialPos::Response &res)
{
    // Business Logic for radialPos_server Server providing radialPos Service

}

// Callback for radialPosTimer timer
void radial_actuator_controller::radialPosTimerCallback(const ros::TimerEvent& event)
{
    // Business Logic for radialPosTimer 

}

// ---------------------------------------------
// EVERYTHING BELOW HERE IS COMPLETELY GENERATED
// ---------------------------------------------

// Destructor - required for clean shutdown when process is killed
radial_actuator_controller::~radial_actuator_controller()
{
    radialPosTimer.stop();
    controlInputs_sub.shutdown();
    radialPos_server_server.shutdown();
}

void radial_actuator_controller::startUp()
{
    ros::NodeHandle nh;

    // Configure all subscribers associated with this component
    // subscriber: controlInputs_sub
    ros::SubscribeOptions controlInputs_sub_options;
    controlInputs_sub_options = 
	ros::SubscribeOptions::create<agse_package::controlInputs>
	    ("controlInputs",
	     1000,
	     boost::bind(&radial_actuator_controller::controlInputs_sub_OnOneData, this, _1),
	     ros::VoidPtr(),
             &this->compQueue);
    this->controlInputs_sub = nh.subscribe(controlInputs_sub_options);

    // Configure all provided services associated with this component
    // server: radialPos_server
    ros::AdvertiseServiceOptions radialPos_server_server_options;
    radialPos_server_server_options = 
	ros::AdvertiseServiceOptions::create<agse_package::radialPos>
	    ("radialPos",
             boost::bind(&radial_actuator_controller::radialPos_serverCallback, this, _1, _2),
	     ros::VoidPtr(),
             &this->compQueue);
    this->radialPos_server_server = nh.advertiseService(radialPos_server_server_options);
 
    // Create Init Timer
    ros::TimerOptions timer_options;
    timer_options = 
	ros::TimerOptions
	    (ros::Duration(-1),
	     boost::bind(&radial_actuator_controller::Init, this, _1),
	     &this->compQueue,
             true);
    this->initOneShotTimer = nh.createTimer(timer_options);  
  
    // Create all component timers
    // timer: timer.name
    timer_options = 
	ros::TimerOptions
             (ros::Duration(0.01),
	     boost::bind(&radial_actuator_controller::radialPosTimerCallback, this, _1),
	     &this->compQueue);
    this->radialPosTimer = nh.createTimer(timer_options);

}
