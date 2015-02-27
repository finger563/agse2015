#include "agse_package/image_processor.hpp"

//# Start User Globals Marker
#include "agse_package/Image_Processor.h"
//# End User Globals Marker

// -------------------------------------------------------
// BUSINESS LOGIC OF THESE FUNCTIONS SUPPLIED BY DEVELOPER
// -------------------------------------------------------

// Init Function
//# Start Init Marker
void image_processor::Init(const ros::TimerEvent& event)
{
    // Initialize 
    imgproc_instance.init();
    // Stop Init Timer
    initOneShotTimer.stop();
}
//# End Init Marker

// OnOneData Subscription handler for controlInputs_sub subscriber
//# Start controlInputs_sub_OnOneData Marker
void image_processor::controlInputs_sub_OnOneData(const agse_package::controlInputs::ConstPtr& received_data)
{
    // Business Logic for controlInputs_sub subscriber subscribing to topic controlInputs callback 
    paused = received_data->paused;
    ROS_INFO( paused ? "Image Processor paused!" : "Image Processor Unpaused" );
}
//# End controlInputs_sub_OnOneData Marker

// Component Service Callback
//# Start sampleStateFromImageCallback Marker
bool image_processor::sampleStateFromImageCallback(agse_package::sampleStateFromImage::Request  &req,
    agse_package::sampleStateFromImage::Response &res)
{
    // Business Logic for sampleStateFromImage_server Server providing sampleStateFromImage Service

  if(!paused) {

    agse_package::captureImage arg;

    if (this->captureImage_client.call(arg)) {
      ROS_INFO("Obtaining new image frame!");
    }
   
    else {
      ROS_INFO("ERROE!!~!!!@R");
    }

  }

    imgproc_instance.run(image_byte_array); 
      
}
//# End sampleStateFromImageCallback Marker

// Callback for imageTimer timer
//# Start imageTimerCallback Marker
void image_processor::imageTimerCallback(const ros::TimerEvent& event)
{
    // Business Logic for imageTimer 

}
//# End imageTimerCallback Marker

// ---------------------------------------------
// EVERYTHING BELOW HERE IS COMPLETELY GENERATED
// ---------------------------------------------

// Destructor - required for clean shutdown when process is killed
image_processor::~image_processor()
{
    imageTimer.stop();
    controlInputs_sub.shutdown();
    sampleStateFromImage_server.shutdown();
    captureImage_client.shutdown();
}

void image_processor::startUp()
{
    ros::NodeHandle nh;

    // Configure all subscribers associated with this component
    // subscriber: controlInputs_sub
    ros::SubscribeOptions controlInputs_sub_options;
    controlInputs_sub_options = 
	ros::SubscribeOptions::create<agse_package::controlInputs>
	    ("controlInputs",
	     1000,
	     boost::bind(&image_processor::controlInputs_sub_OnOneData, this, _1),
	     ros::VoidPtr(),
             &this->compQueue);
    this->controlInputs_sub = nh.subscribe(controlInputs_sub_options);

    // Configure all provided services associated with this component
    // server: sampleStateFromImage_server
    ros::AdvertiseServiceOptions sampleStateFromImage_server_options;
    sampleStateFromImage_server_options = 
	ros::AdvertiseServiceOptions::create<agse_package::sampleStateFromImage>
	    ("sampleStateFromImage",
             boost::bind(&image_processor::sampleStateFromImageCallback, this, _1, _2),
	     ros::VoidPtr(),
             &this->compQueue);
    this->sampleStateFromImage_server = nh.advertiseService(sampleStateFromImage_server_options);
 
    // Configure all required services associated with this component
    // client: captureImage_client
    this->captureImage_client = nh.serviceClient<agse_package::captureImage>
	("captureImage"); 

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
    // timer: timer.properties["name"]
    timer_options = 
	ros::TimerOptions
             (ros::Duration(0.04),
	     boost::bind(&image_processor::imageTimerCallback, this, _1),
	     &this->compQueue);
    this->imageTimer = nh.createTimer(timer_options);

}
