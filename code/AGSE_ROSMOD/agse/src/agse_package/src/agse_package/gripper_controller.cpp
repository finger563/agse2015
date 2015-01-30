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

// OnOneData Subscription handler for controlInputs_sub subscriber
void gripper_controller::controlInputs_sub_OnOneData(const agse_package::controlInputs::ConstPtr& received_data)
{
    // Business Logic for controlInputs_sub subscriber callback 
}

// Component Service Callback
bool gripper_controller::gripperRotation_serverCallback(agse_package::gripperRotation::Request  &req,
    agse_package::gripperRotation::Response &res)
{
    // Business Logic for <listener.ROS_Server instance at 0xb5417288> Service
}
// Component Service Callback
bool gripper_controller::gripperPos_serverCallback(agse_package::gripperPos::Request  &req,
    agse_package::gripperPos::Response &res)
{
    // Business Logic for <listener.ROS_Server instance at 0xb5417148> Service
}

// Callback for gripperTimer timer
void gripper_controller::gripperTimerCallback(const ros::TimerEvent& event)
{
    // Business Logic for gripperTimer 
}

// ---------------------------------------------
// EVERYTHING BELOW HERE IS COMPLETELY GENERATED
// ---------------------------------------------

// Destructor - required for clean shutdown when process is killed
gripper_controller::~gripper_controller()
{
    gripperTimer.stop();
    controlInputs_sub.shutdown();
    gripperRotation_server_server.shutdown();
    gripperPos_server_server.shutdown();
}

void gripper_controller::startUp()
{
    ros::NodeHandle nh;

    // Configure all subscribers associated with this component
    ros::SubscribeOptions controlInputs_sub_options;
    controlInputs_sub_options = 
	ros::SubscribeOptions::create<agse_package::controlInputs>
	    ("controlInputs",
	     1000,
	     boost::bind(&gripper_controller::controlInputs_sub_OnOneData, this, _1),
	     ros::VoidPtr(),
             &this->compQueue);
    this->controlInputs_sub = nh.subscribe(controlInputs_sub_options);

    // Configure all provided services associated with this component
    ros::AdvertiseServiceOptions gripperRotation_server_server_options;
    gripperRotation_server_server_options = 
	ros::AdvertiseServiceOptions::create<agse_package::gripperRotation>
	    ("gripperRotation",
             boost::bind(&gripper_controller::gripperRotation_serverCallback, this, _1, _2),
	     ros::VoidPtr(),
             &this->compQueue);
    this->gripperRotation_server_server = nh.advertiseService(gripperRotation_server_server_options);
    // Configure all provided services associated with this component
    ros::AdvertiseServiceOptions gripperPos_server_server_options;
    gripperPos_server_server_options = 
	ros::AdvertiseServiceOptions::create<agse_package::gripperPos>
	    ("gripperPos",
             boost::bind(&gripper_controller::gripperPos_serverCallback, this, _1, _2),
	     ros::VoidPtr(),
             &this->compQueue);
    this->gripperPos_server_server = nh.advertiseService(gripperPos_server_server_options);
 
    // Create Init Timer
    ros::TimerOptions timer_options;
    timer_options = 
	ros::TimerOptions
	    (ros::Duration(-1),
	     boost::bind(&gripper_controller::Init, this, _1),
	     &this->compQueue,
             true);
    this->initOneShotTimer = nh.createTimer(timer_options);  
  
    // Create all component timers
    timer_options = 
	ros::TimerOptions
             (ros::Duration(0.01),
	     boost::bind(&gripper_controller::gripperTimerCallback, this, _1),
	     &this->compQueue);
    this->gripperTimer = nh.createTimer(timer_options);

}
