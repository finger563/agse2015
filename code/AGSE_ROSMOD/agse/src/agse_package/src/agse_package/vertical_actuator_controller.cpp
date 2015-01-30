#include "agse_package/vertical_actuator_controller.hpp"

// -------------------------------------------------------
// BUSINESS LOGIC OF THESE FUNCTIONS SUPPLIED BY DEVELOPER
// -------------------------------------------------------

// Init Function
void vertical_actuator_controller::Init(const ros::TimerEvent& event)
{
    // Initialize Component

    // Stop Init Timer
    initOneShotTimer.stop();
}

// Component Service Callback
bool vertical_actuator_controller::setVerticalPos_serverCallback(agse_package::setVerticalPos::Request  &req,
    agse_package::setVerticalPos::Response &res)
{
    // Business Logic for <listener.ROS_Server instance at 0xb54290f8> Service
}

// ---------------------------------------------
// EVERYTHING BELOW HERE IS COMPLETELY GENERATED
// ---------------------------------------------

// Destructor - required for clean shutdown when process is killed
vertical_actuator_controller::~vertical_actuator_controller()
{
    setVerticalPos_server_server.shutdown();
}

void vertical_actuator_controller::startUp()
{
    ros::NodeHandle nh;

    // Configure all provided services associated with this component
    ros::AdvertiseServiceOptions setVerticalPos_server_server_options;
    setVerticalPos_server_server_options = 
	ros::AdvertiseServiceOptions::create<agse_package::setVerticalPos>
	    ("setVerticalPos",
             boost::bind(&vertical_actuator_controller::setVerticalPos_serverCallback, this, _1, _2),
	     ros::VoidPtr(),
             &this->compQueue);
    this->setVerticalPos_server_server = nh.advertiseService(setVerticalPos_server_server_options);
 
    // Create Init Timer
    ros::TimerOptions timer_options;
    timer_options = 
	ros::TimerOptions
	    (ros::Duration(-1),
	     boost::bind(&vertical_actuator_controller::Init, this, _1),
	     &this->compQueue,
             true);
    this->initOneShotTimer = nh.createTimer(timer_options);  
  
}
