#include "agse_package/image_processor.hpp"

//# Start User Globals Marker

//# End User Globals Marker

// -------------------------------------------------------
// BUSINESS LOGIC OF THESE FUNCTIONS SUPPLIED BY DEVELOPER
// -------------------------------------------------------

// Init Function
//# Start Init Marker
void image_processor::Init(const ros::TimerEvent& event)
{
    // Initialize 
  // initialize both the sample detector code
  // and the payload bay (marker) detector code
    sampleDetector.init();
    payloadBayDetector.init( 1.0f, "camera.yml");
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
  if (!paused)
    {
      agse_package::captureImage arg;
      if (this->captureImage_client.call(arg)) {
	ROS_INFO("Image width: %d, height: %d, size: %d", 
		 arg.response.width,
		 arg.response.height,
		 arg.response.imgVector.size());
	// NEED TO GET RETURN VALUES ABOUT DETECTED SAMPLE HERE
	sampleDetector.run(arg.response.imgVector, arg.response.width, arg.response.height); 
	// NEED TO SET REAL RESPONSE HERE
	res.foundSample = false;
	res.sample.pos.r     = 0.0f;
	res.sample.pos.theta = 0.0f;
	res.sample.pos.z     = 0.0f;
	res.sample.orientation.theta = 0.0f;
	res.sample.orientation.phi   = 0.0f;
	return true;
      }
      else {
	ROS_INFO("ERROR: Client call failed; couldn't get image.");
      }
    }
  return false;
}
//# End sampleStateFromImageCallback Marker
// Component Service Callback
//# Start payloadBayStateFromImageCallback Marker
bool image_processor::payloadBayStateFromImageCallback(agse_package::payloadBayStateFromImage::Request  &req,
    agse_package::payloadBayStateFromImage::Response &res)
{
    // Business Logic for payloadBayStateFromImage_server Server providing payloadBayStateFromImage Service
  if (!paused)
    {
      agse_package::captureImage arg;
      if (this->captureImage_client.call(arg)) {
	ROS_INFO("Image width: %d, height: %d, size: %d", 
		 arg.response.width,
		 arg.response.height,
		 arg.response.imgVector.size());
	// NEED TO GET RETURN VALUES ABOUT DETECTED PAYLOAD BAY HERE
	std::vector<aruco::Marker> markers = 
	  payloadBayDetector.run(arg.response.imgVector, arg.response.width, arg.response.height); 
	// NEED TO SET REAL RESPONSE HERE
	res.foundPayloadBay = false;
	res.payloadBay.pos.r     = 0.0f;
	res.payloadBay.pos.theta = 0.0f;
	res.payloadBay.pos.z     = 0.0f;
	res.payloadBay.orientation.theta = 0.0f;
	res.payloadBay.orientation.phi   = 0.0f;
	return true;
      }
      else {
	ROS_INFO("ERROR: Client call failed; couldn't get image.");
      }
    }
  return false;
}
//# End payloadBayStateFromImageCallback Marker

// ---------------------------------------------
// EVERYTHING BELOW HERE IS COMPLETELY GENERATED
// ---------------------------------------------

// Destructor - required for clean shutdown when process is killed
image_processor::~image_processor()
{
    controlInputs_sub.shutdown();
    sampleStateFromImage_server.shutdown();
    payloadBayStateFromImage_server.shutdown();
    captureImage_client.shutdown();
//# Start Destructor Marker

//# End Destructor Marker
}

void image_processor::startUp()
{
    ros::NodeHandle nh;

    // Need to read in and parse the group configuration xml if it exists
    GroupXMLParser groupParser;
    std::string configFileName = nodeName + "." + compName + ".xml";
    if ( boost::filesystem::exists(configFileName) )
    {
        groupParser.Parse(configFileName);
	groupParser.Print();
    }

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
    // server: payloadBayStateFromImage_server
    ros::AdvertiseServiceOptions payloadBayStateFromImage_server_options;
    payloadBayStateFromImage_server_options = 
	ros::AdvertiseServiceOptions::create<agse_package::payloadBayStateFromImage>
	    ("payloadBayStateFromImage",
             boost::bind(&image_processor::payloadBayStateFromImageCallback, this, _1, _2),
	     ros::VoidPtr(),
             &this->compQueue);
    this->payloadBayStateFromImage_server = nh.advertiseService(payloadBayStateFromImage_server_options);
 
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
  
}
