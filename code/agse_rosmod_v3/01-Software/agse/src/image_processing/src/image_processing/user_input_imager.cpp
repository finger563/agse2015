#include "image_processing/user_input_imager.hpp"

//# Start User Globals Marker
//# End User Globals Marker

// Initialization Function
//# Start Init Marker
void user_input_imager::Init(const ros::TimerEvent& event)
{
  // Initialize Here

  // Stop Init Timer
  initOneShotTimer.stop();
}
//# End Init Marker

// Subscriber Callback - sampleDetectionImages_sub
//# Start sampleDetectionImages_sub_OnOneData Marker
void user_input_imager::sampleDetectionImages_sub_OnOneData(const image_processing::sampleDetectionImages::ConstPtr& received_data)
{
  // Business Logic for sampleDetectionImages_sub Subscriber
}
//# End sampleDetectionImages_sub_OnOneData Marker
// Subscriber Callback - payloadBayDetectionImages_sub
//# Start payloadBayDetectionImages_sub_OnOneData Marker
void user_input_imager::payloadBayDetectionImages_sub_OnOneData(const image_processing::payloadBayDetectionImages::ConstPtr& received_data)
{
  // Business Logic for payloadBayDetectionImages_sub Subscriber
}
//# End payloadBayDetectionImages_sub_OnOneData Marker
// Subscriber Callback - sampleState_sub
//# Start sampleState_sub_OnOneData Marker
void user_input_imager::sampleState_sub_OnOneData(const high_level_control::sampleState::ConstPtr& received_data)
{
  // Business Logic for sampleState_sub Subscriber
}
//# End sampleState_sub_OnOneData Marker
// Subscriber Callback - payloadBayState_sub
//# Start payloadBayState_sub_OnOneData Marker
void user_input_imager::payloadBayState_sub_OnOneData(const high_level_control::payloadBayState::ConstPtr& received_data)
{
  // Business Logic for payloadBayState_sub Subscriber
}
//# End payloadBayState_sub_OnOneData Marker

// Timer Callback - uiImage_timer
//# Start uiImage_timerCallback Marker
void user_input_imager::uiImage_timerCallback(const ros::TimerEvent& event)
{
  // Business Logic for uiImage_timer Timer
}
//# End uiImage_timerCallback Marker


// Destructor - Cleanup Ports & Timers
user_input_imager::~user_input_imager()
{
  uiImage_timer.stop();
  sampleDetectionImages_sub.shutdown();
  payloadBayDetectionImages_sub.shutdown();
  sampleState_sub.shutdown();
  payloadBayState_sub.shutdown();
  captureImage_client.shutdown();
  //# Start Destructor Marker
  //# End Destructor Marker
}

// Startup - Setup Component Ports & Timers
void user_input_imager::startUp()
{
  ros::NodeHandle nh;
  std::string advertiseName;

  // Component Subscriber - sampleDetectionImages_sub
  advertiseName = "sampleDetectionImages";
  if (portGroupMap.find("sampleDetectionImages_sub") != portGroupMap.end())
    advertiseName += "_" + portGroupMap["sampleDetectionImages_sub"];
  ros::SubscribeOptions sampleDetectionImages_sub_options;
  sampleDetectionImages_sub_options = ros::SubscribeOptions::create<image_processing::sampleDetectionImages>
      (advertiseName.c_str(),
       1000,
       boost::bind(&user_input_imager::sampleDetectionImages_sub_OnOneData, this, _1),
       ros::VoidPtr(),
       &this->compQueue);
  this->sampleDetectionImages_sub = nh.subscribe(sampleDetectionImages_sub_options);
  // Component Subscriber - payloadBayDetectionImages_sub
  advertiseName = "payloadBayDetectionImages";
  if (portGroupMap.find("payloadBayDetectionImages_sub") != portGroupMap.end())
    advertiseName += "_" + portGroupMap["payloadBayDetectionImages_sub"];
  ros::SubscribeOptions payloadBayDetectionImages_sub_options;
  payloadBayDetectionImages_sub_options = ros::SubscribeOptions::create<image_processing::payloadBayDetectionImages>
      (advertiseName.c_str(),
       1000,
       boost::bind(&user_input_imager::payloadBayDetectionImages_sub_OnOneData, this, _1),
       ros::VoidPtr(),
       &this->compQueue);
  this->payloadBayDetectionImages_sub = nh.subscribe(payloadBayDetectionImages_sub_options);
  // Component Subscriber - sampleState_sub
  advertiseName = "sampleState";
  if (portGroupMap.find("sampleState_sub") != portGroupMap.end())
    advertiseName += "_" + portGroupMap["sampleState_sub"];
  ros::SubscribeOptions sampleState_sub_options;
  sampleState_sub_options = ros::SubscribeOptions::create<high_level_control::sampleState>
      (advertiseName.c_str(),
       1000,
       boost::bind(&user_input_imager::sampleState_sub_OnOneData, this, _1),
       ros::VoidPtr(),
       &this->compQueue);
  this->sampleState_sub = nh.subscribe(sampleState_sub_options);
  // Component Subscriber - payloadBayState_sub
  advertiseName = "payloadBayState";
  if (portGroupMap.find("payloadBayState_sub") != portGroupMap.end())
    advertiseName += "_" + portGroupMap["payloadBayState_sub"];
  ros::SubscribeOptions payloadBayState_sub_options;
  payloadBayState_sub_options = ros::SubscribeOptions::create<high_level_control::payloadBayState>
      (advertiseName.c_str(),
       1000,
       boost::bind(&user_input_imager::payloadBayState_sub_OnOneData, this, _1),
       ros::VoidPtr(),
       &this->compQueue);
  this->payloadBayState_sub = nh.subscribe(payloadBayState_sub_options);

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
     boost::bind(&user_input_imager::Init, this, _1),
     &this->compQueue,
     true);
  this->initOneShotTimer = nh.createTimer(timer_options);  
  
  // Component Timer - timer.properties["name"]
  timer_options = 
    ros::TimerOptions
    (ros::Duration(0.5),
     boost::bind(&user_input_imager::uiImage_timerCallback, this, _1),
     &this->compQueue);
  this->uiImage_timer = nh.createTimer(timer_options);

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
    return new user_input_imager(config,argc,argv);
  }
}
