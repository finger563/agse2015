#include "agse_package/image_processor.hpp"

// -------------------------------------------------------
// BUSINESS LOGIC OF THESE FUNCTIONS SUPPLIED BY DEVELOPER
// -------------------------------------------------------

// Init Function
void image_processor::Init(const ros::TimerEvent& event)
{
    // Initialize Component

    // Stop Init Timer
    initOneShotTimer.stop();
}

// Component Service Callback
bool image_processor::sampleStateFromImage_serverCallback(agse_package::sampleStateFromImage::Request  &req,
    agse_package::sampleStateFromImage::Response &res)
{
    // Business Logic for <listener.ROS_Server instance at 0xb5429c10> Service
}

// Callback for imageTimer timer
void image_processor::imageTimerCallback(const ros::TimerEvent& event)
{
    // Business Logic for imageTimer 
}

// ---------------------------------------------
// EVERYTHING BELOW HERE IS COMPLETELY GENERATED
// ---------------------------------------------

// Destructor - required for clean shutdown when process is killed
image_processor::~image_processor()
{
    imageTimer.stop();
    sampleStateFromImage_server_server.shutdown();
}

void image_processor::startUp()
{
    ros::NodeHandle nh;

    // Configure all provided services associated with this component
    ros::AdvertiseServiceOptions sampleStateFromImage_server_server_options;
    sampleStateFromImage_server_server_options = 
	ros::AdvertiseServiceOptions::create<agse_package::sampleStateFromImage>
	    ("sampleStateFromImage",
             boost::bind(&image_processor::sampleStateFromImage_serverCallback, this, _1, _2),
	     ros::VoidPtr(),
             &this->compQueue);
    this->sampleStateFromImage_server_server = nh.advertiseService(sampleStateFromImage_server_server_options);
 
    // Create Init Timer
    ros::TimerOptions timer_options;
    timer_options = 
	ros::TimerOptions
	    (ros::Duration(-1),
	     boost::bind(&image_processor::Init, this, _1),
	     &this->compQueue,
             true);
    this->initOneShotTimer = nh.createTimer(timer_options);  
  
    // Create all component timers
    timer_options = 
	ros::TimerOptions
             (ros::Duration(0.04),
	     boost::bind(&image_processor::imageTimerCallback, this, _1),
	     &this->compQueue);
    this->imageTimer = nh.createTimer(timer_options);

}
