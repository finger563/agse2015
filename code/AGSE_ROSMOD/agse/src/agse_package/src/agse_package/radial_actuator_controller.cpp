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

// Component Service Callback
bool radial_actuator_controller::setRadialPos_serverCallback(agse_package::setRadialPos::Request  &req,
    agse_package::setRadialPos::Response &res)
{
    // Business Logic for <listener.ROS_Server instance at 0xb5425e68> Service
}

// ---------------------------------------------
// EVERYTHING BELOW HERE IS COMPLETELY GENERATED
// ---------------------------------------------

// Destructor - required for clean shutdown when process is killed
radial_actuator_controller::~radial_actuator_controller()
{
    setRadialPos_server_server.shutdown();
}

void radial_actuator_controller::startUp()
{
    ros::NodeHandle nh;

    // Configure all provided services associated with this component
    ros::AdvertiseServiceOptions setRadialPos_server_server_options;
    setRadialPos_server_server_options = 
	ros::AdvertiseServiceOptions::create<agse_package::setRadialPos>
	    ("setRadialPos",
             boost::bind(&radial_actuator_controller::setRadialPos_serverCallback, this, _1, _2),
	     ros::VoidPtr(),
             &this->compQueue);
    this->setRadialPos_server_server = nh.advertiseService(setRadialPos_server_server_options);
 
    // Create Init Timer
    ros::TimerOptions timer_options;
    timer_options = 
	ros::TimerOptions
	    (ros::Duration(-1),
	     boost::bind(&radial_actuator_controller::Init, this, _1),
	     &this->compQueue,
             true);
    this->initOneShotTimer = nh.createTimer(timer_options);  
  
}
