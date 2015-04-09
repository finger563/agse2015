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
  paused = true;
  // initialize both the sample detector code
  // and the payload bay (marker) detector code
    sampleDetector.init();
    payloadBayDetector.init( 0.75f, "/home/ubuntu/camera.yml");

    // Command line args for image processor
    for (int i = 0; i < node_argc; i++)
      {
	if (!strcmp(node_argv[i], "-unpaused"))
	  {
	    paused = false;
	  }
	if (!strcmp(node_argv[i], "-detect"))
	  {
	    agse_package::captureImage arg;
	    if (this->captureImage_client.call(arg)) {
	      //	      ROS_INFO("Image width: %d, height: %d, size: %d", 
	      //		       arg.response.width,
	      //	       arg.response.height,
	      //       arg.response.imgVector.size());
	      Mat image = Mat( arg.response.height, 
			       arg.response.width, 
			       CV_8UC3, 
			       arg.response.imgVector.data());
	      Mat detectedObjectsMask = Mat::zeros(image.size(), CV_8UC3);
	      DetectedObject sample = 
		sampleDetector.run( image,
				    detectedObjectsMask); 
	      DetectedObject payloadBay =
		payloadBayDetector.run( image,
					detectedObjectsMask); 
	      cv::imwrite("Sample-01-Raw.png", image+detectedObjectsMask);
	      ROS_INFO("Sample: %d, (%f,%f), %f",sample.state, sample.x, sample.y, sample.angle);
	      ROS_INFO("PayloadBay: %d, (%f,%f), %f",payloadBay.state, payloadBay.x, payloadBay.y, payloadBay.angle);
	    }
	  }
      }

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
	//	ROS_INFO("Image width: %d, height: %d, size: %d", 
	//		 arg.response.width,
	//		 arg.response.height,
	//		 arg.response.imgVector.size());
	Mat image = Mat( arg.response.height, 
			 arg.response.width, 
			 CV_8UC3, 
			 arg.response.imgVector.data());
	Mat detectedObjectsMask = Mat::zeros(image.size(), CV_8UC3);
	// NEED TO GET RETURN VALUES ABOUT DETECTED SAMPLE HERE
	DetectedObject sample =
	  sampleDetector.run(image,detectedObjectsMask,"autonomous_S"); 
	//	ROS_INFO("Sample: %d, (%f,%f), %f",sample.state, sample.x, sample.y, sample.angle);
	cv::imwrite("Sample-01-Raw.png", image+detectedObjectsMask);
	if ( sample.x >= 0 && sample.x <= arg.response.width &&
	     sample.y >= 0 && sample.y <= arg.response.height )
	  {
	    res.status = sample.state;
	    res.x = sample.x - arg.response.width / 2;   // convert [0,w] -> [-w/2,w/2]
	    res.y = sample.y - arg.response.height / 2;  // convert [0,h] -> [-h/2,h/2]
	    res.angle = sample.angle;
	  }
	else
	  res.status = HIDDEN;
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
	//	ROS_INFO("Image width: %d, height: %d, size: %d", 
	//	 arg.response.width,
	//	 arg.response.height,
	//	 arg.response.imgVector.size());
	Mat image = Mat( arg.response.height, 
			 arg.response.width, 
			 CV_8UC3, 
			 arg.response.imgVector.data());
	Mat detectedObjectsMask = Mat::zeros(image.size(), CV_8UC3);
	// NEED TO GET RETURN VALUES ABOUT DETECTED PAYLOAD BAY HERE
	DetectedObject payloadBay =
	  payloadBayDetector.run(image,detectedObjectsMask,"autonomous_PB"); 
	ROS_INFO("PayloadBay: %d, (%f,%f), %f",payloadBay.state, payloadBay.x, payloadBay.y, payloadBay.angle);
	cv::imwrite("PayloadBay-01-Raw.png", image+detectedObjectsMask);
	if ( payloadBay.x >= 0 && payloadBay.x <= arg.response.width &&
	     payloadBay.y >= 0 && payloadBay.y <= arg.response.height )
	  {
	    res.status = payloadBay.state;
	    res.x = payloadBay.x - arg.response.width / 2;   // convert [0,w] -> [-w/2,w/2]
	    res.y = payloadBay.y - arg.response.height / 2;  // convert [0,h] -> [-h/2,h/2]
	    res.angle = payloadBay.angle;
	  }
	else
	  res.status = HIDDEN;
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
    payloadBayDetectionImages_pub.shutdown();
    sampleDetectionImages_pub.shutdown();
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
    std::map<std::string,std::string> *portGroupMap = NULL;
    std::string configFileName = nodeName + "." + compName + ".xml";
    if (groupParser.Parse(configFileName))
    {
	portGroupMap = &groupParser.portGroupMap;
    }

    std::string advertiseName;

    // Configure all subscribers associated with this component
    // subscriber: controlInputs_sub
    advertiseName = "controlInputs";
    if ( portGroupMap != NULL && portGroupMap->find("controlInputs_sub") != portGroupMap->end() )
        advertiseName += "_" + (*portGroupMap)["controlInputs_sub"];
    ros::SubscribeOptions controlInputs_sub_options;
    controlInputs_sub_options = 
	ros::SubscribeOptions::create<agse_package::controlInputs>
	    (advertiseName.c_str(),
	     1000,
	     boost::bind(&image_processor::controlInputs_sub_OnOneData, this, _1),
	     ros::VoidPtr(),
             &this->compQueue);
    this->controlInputs_sub = nh.subscribe(controlInputs_sub_options);

    // Configure all publishers associated with this component
    // publisher: payloadBayDetectionImages_pub
    advertiseName = "payloadBayDetectionImages";
    if ( portGroupMap != NULL && portGroupMap->find("payloadBayDetectionImages_pub") != portGroupMap->end() )
        advertiseName += "_" + (*portGroupMap)["payloadBayDetectionImages_pub"];
    this->payloadBayDetectionImages_pub = nh.advertise<agse_package::payloadBayDetectionImages>
	(advertiseName.c_str(), 1000);	
    // publisher: sampleDetectionImages_pub
    advertiseName = "sampleDetectionImages";
    if ( portGroupMap != NULL && portGroupMap->find("sampleDetectionImages_pub") != portGroupMap->end() )
        advertiseName += "_" + (*portGroupMap)["sampleDetectionImages_pub"];
    this->sampleDetectionImages_pub = nh.advertise<agse_package::sampleDetectionImages>
	(advertiseName.c_str(), 1000);	

    // Configure all provided services associated with this component
    // server: sampleStateFromImage_server
    advertiseName = "sampleStateFromImage";
    if ( portGroupMap != NULL && portGroupMap->find("sampleStateFromImage_server") != portGroupMap->end() )
        advertiseName += "_" + (*portGroupMap)["sampleStateFromImage_server"];
    ros::AdvertiseServiceOptions sampleStateFromImage_server_options;
    sampleStateFromImage_server_options = 
	ros::AdvertiseServiceOptions::create<agse_package::sampleStateFromImage>
	    (advertiseName.c_str(),
             boost::bind(&image_processor::sampleStateFromImageCallback, this, _1, _2),
	     ros::VoidPtr(),
             &this->compQueue);
    this->sampleStateFromImage_server = nh.advertiseService(sampleStateFromImage_server_options);
    // server: payloadBayStateFromImage_server
    advertiseName = "payloadBayStateFromImage";
    if ( portGroupMap != NULL && portGroupMap->find("payloadBayStateFromImage_server") != portGroupMap->end() )
        advertiseName += "_" + (*portGroupMap)["payloadBayStateFromImage_server"];
    ros::AdvertiseServiceOptions payloadBayStateFromImage_server_options;
    payloadBayStateFromImage_server_options = 
	ros::AdvertiseServiceOptions::create<agse_package::payloadBayStateFromImage>
	    (advertiseName.c_str(),
             boost::bind(&image_processor::payloadBayStateFromImageCallback, this, _1, _2),
	     ros::VoidPtr(),
             &this->compQueue);
    this->payloadBayStateFromImage_server = nh.advertiseService(payloadBayStateFromImage_server_options);
 
    // Configure all required services associated with this component
    // client: captureImage_client
    advertiseName = "captureImage";
    if ( portGroupMap != NULL && portGroupMap->find(advertiseName+"_client") != portGroupMap->end() )
        advertiseName += "_" + (*portGroupMap)[advertiseName+"_client"];
    this->captureImage_client = nh.serviceClient<agse_package::captureImage>
	(advertiseName.c_str()); 

    // Create Init Timer
    ros::TimerOptions timer_options;
    timer_options = 
	ros::TimerOptions
	    (ros::Duration(-1),
	     boost::bind(&image_processor::Init, this, _1),
	     &this->compQueue,
             true);
    this->initOneShotTimer = nh.createTimer(timer_options);  
  

    /*
     * Identify present working directory of node executable
     */
    std::string s = node_argv[0];
    std::string exec_path = s;
    std::string delimiter = "/";
    std::string exec, pwd;
    size_t pos = 0;
    while ((pos = s.find(delimiter)) != std::string::npos) {
        exec = s.substr(0, pos);
        s.erase(0, pos + delimiter.length());
    }
    exec = s.substr(0, pos);
    pwd = exec_path.erase(exec_path.find(exec), exec.length());
    // Establish the log file name
    std::string log_file_path = pwd + nodeName + "." + compName + ".log"; 

    // Create the log file & open file stream
    LOGGER.CREATE_FILE(log_file_path);

    // Establish log levels of LOGGER
    LOGGER.SET_LOG_LEVELS(groupParser.logging);
}
