#include "image_processing/image_processor.hpp"

//# Start User Globals Marker
//# End User Globals Marker

// Initialization Function
//# Start Init Marker
void image_processor::Init(const ros::TimerEvent& event)
{
  // Initialize Here

  // Stop Init Timer
  initOneShotTimer.stop();
}
//# End Init Marker

// Subscriber Callback - controlInputs_sub
//# Start controlInputs_sub_OnOneData Marker
void image_processor::controlInputs_sub_OnOneData(const high_level_control::controlInputs::ConstPtr& received_data)
{
  // Business Logic for controlInputs_sub Subscriber
}
//# End controlInputs_sub_OnOneData Marker

// Server Callback - sampleStateFromImage_server
//# Start sampleStateFromImageCallback Marker
bool image_processor::sampleStateFromImageCallback(image_processing::sampleStateFromImage::Request  &req,
  image_processing::sampleStateFromImage::Response &res)
{
  // Business Logic for sampleStateFromImage_server Server

  return true;
}
//# End sampleStateFromImageCallback Marker
// Server Callback - payloadBayStateFromImage_server
//# Start payloadBayStateFromImageCallback Marker
bool image_processor::payloadBayStateFromImageCallback(image_processing::payloadBayStateFromImage::Request  &req,
  image_processing::payloadBayStateFromImage::Response &res)
{
  // Business Logic for payloadBayStateFromImage_server Server

  return true;
}
//# End payloadBayStateFromImageCallback Marker


// Destructor - Cleanup Ports & Timers
image_processor::~image_processor()
{
  sampleDetectionImages_pub.shutdown();
  payloadBayDetectionImages_pub.shutdown();
  controlInputs_sub.shutdown();
  sampleStateFromImage_server.shutdown();
  payloadBayStateFromImage_server.shutdown();
  captureImage_client.shutdown();
  //# Start Destructor Marker
  //# End Destructor Marker
}

// Startup - Setup Component Ports & Timers
void image_processor::startUp()
{
  ros::NodeHandle nh;
  std::string advertiseName;

  // Component Subscriber - controlInputs_sub
  advertiseName = "controlInputs";
  if (portGroupMap.find("controlInputs_sub") != portGroupMap.end())
    advertiseName += "_" + portGroupMap["controlInputs_sub"];
  ros::SubscribeOptions controlInputs_sub_options;
  controlInputs_sub_options = ros::SubscribeOptions::create<high_level_control::controlInputs>
      (advertiseName.c_str(),
       1000,
       boost::bind(&image_processor::controlInputs_sub_OnOneData, this, _1),
       ros::VoidPtr(),
       &this->compQueue);
  this->controlInputs_sub = nh.subscribe(controlInputs_sub_options);

  // Component Publisher - sampleDetectionImages_pub
  advertiseName = "sampleDetectionImages";
  if (portGroupMap.find("sampleDetectionImages_pub") != portGroupMap.end())
    advertiseName += "_" + portGroupMap["sampleDetectionImages_pub"];
  this->sampleDetectionImages_pub = nh.advertise<image_processing::sampleDetectionImages>(advertiseName.c_str(), 1000);
  // Component Publisher - payloadBayDetectionImages_pub
  advertiseName = "payloadBayDetectionImages";
  if (portGroupMap.find("payloadBayDetectionImages_pub") != portGroupMap.end())
    advertiseName += "_" + portGroupMap["payloadBayDetectionImages_pub"];
  this->payloadBayDetectionImages_pub = nh.advertise<image_processing::payloadBayDetectionImages>(advertiseName.c_str(), 1000);

  // Component Server - sampleStateFromImage_server
  advertiseName = "sampleStateFromImage";
  if (portGroupMap.find("sampleStateFromImage_server") != portGroupMap.end())
    advertiseName += "_" + portGroupMap["sampleStateFromImage_server"];
  ros::AdvertiseServiceOptions sampleStateFromImage_server_server_options;
  sampleStateFromImage_server_server_options = ros::AdvertiseServiceOptions::create<image_processing::sampleStateFromImage>
      (advertiseName.c_str(),
       boost::bind(&image_processor::sampleStateFromImageCallback, this, _1, _2),
       ros::VoidPtr(),
       &this->compQueue);
  this->sampleStateFromImage_server = nh.advertiseService(sampleStateFromImage_server_server_options);
  // Component Server - payloadBayStateFromImage_server
  advertiseName = "payloadBayStateFromImage";
  if (portGroupMap.find("payloadBayStateFromImage_server") != portGroupMap.end())
    advertiseName += "_" + portGroupMap["payloadBayStateFromImage_server"];
  ros::AdvertiseServiceOptions payloadBayStateFromImage_server_server_options;
  payloadBayStateFromImage_server_server_options = ros::AdvertiseServiceOptions::create<image_processing::payloadBayStateFromImage>
      (advertiseName.c_str(),
       boost::bind(&image_processor::payloadBayStateFromImageCallback, this, _1, _2),
       ros::VoidPtr(),
       &this->compQueue);
  this->payloadBayStateFromImage_server = nh.advertiseService(payloadBayStateFromImage_server_server_options);
 
  // Configure all required services associated with this component
  // Component Client - captureImage_client
  advertiseName = "captureImage";
  if (portGroupMap.find("captureImage_client") != portGroupMap.end())
    advertiseName += "_" + portGroupMap["captureImage_client"];
      this->captureImage_client = nh.serviceClient<image_processing::captureImage>(advertiseName.c_str()); 

  // Init Timer
  ros::TimerOptions timer_options;
  timer_options = 
    ros::TimerOptions
    (ros::Duration(-1),
     boost::bind(&image_processor::Init, this, _1),
     &this->compQueue,
     true);
  this->initOneShotTimer = nh.createTimer(timer_options);  
  
  // Identify the pwd of Node Executable
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
  std::string log_file_path = pwd + nodeName + "." + compName + ".log"; 
  
  // Create the log file & open file stream
  LOGGER.CREATE_FILE(log_file_path);
  
  // Establish log levels of LOGGER
  LOGGER.SET_LOG_LEVELS(logLevels);
}

extern "C" {
  Component *maker(ComponentConfig &config, int argc, char **argv) {
    return new image_processor(config,argc,argv);
  }
}
