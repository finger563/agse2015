#include "agse_package/image_sensor.hpp"

//# Start User Globals Marker

//# End User Globals Marker

// -------------------------------------------------------
// BUSINESS LOGIC OF THESE FUNCTIONS SUPPLIED BY DEVELOPER
// -------------------------------------------------------

// Init Function
//# Start Init Marker
void image_sensor::Init(const ros::TimerEvent& event)
{
    // Initialize Component

    // Stop Init Timer
    initOneShotTimer.stop();
}
//# End Init Marker

// Component Service Callback
//# Start captureImageCallback Marker
bool image_sensor::captureImageCallback(agse_package::captureImage::Request  &req,
    agse_package::captureImage::Response &res)
{
    // Business Logic for captureImage_server Server providing captureImage Service

}
//# End captureImageCallback Marker

// ---------------------------------------------
// EVERYTHING BELOW HERE IS COMPLETELY GENERATED
// ---------------------------------------------

// Destructor - required for clean shutdown when process is killed
image_sensor::~image_sensor()
{
    captureImage_server.shutdown();
}

void image_sensor::startUp()
{
    ros::NodeHandle nh;

    // Configure all provided services associated with this component
    // server: captureImage_server
    ros::AdvertiseServiceOptions captureImage_server_options;
    captureImage_server_options = 
	ros::AdvertiseServiceOptions::create<agse_package::captureImage>
	    ("captureImage",
             boost::bind(&image_sensor::captureImageCallback, this, _1, _2),
	     ros::VoidPtr(),
             &this->compQueue);
    this->captureImage_server = nh.advertiseService(captureImage_server_options);
 
    // Create Init Timer
    ros::TimerOptions timer_options;
    timer_options = 
	ros::TimerOptions
	    (ros::Duration(-1),
	     boost::bind(&image_sensor::Init, this, _1),
	     &this->compQueue,
             true);
    this->initOneShotTimer = nh.createTimer(timer_options);  
  
}
