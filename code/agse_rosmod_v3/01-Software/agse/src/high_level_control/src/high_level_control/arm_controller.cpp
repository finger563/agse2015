#include "high_level_control/arm_controller.hpp"

//# Start User Globals Marker
//# End User Globals Marker

// Initialization Function
//# Start Init Marker
void arm_controller::Init(const ros::TimerEvent& event)
{
  // Initialize Here

  // Stop Init Timer
  initOneShotTimer.stop();
}
//# End Init Marker

// Subscriber Callback - controlInputs_sub
//# Start controlInputs_sub_OnOneData Marker
void arm_controller::controlInputs_sub_OnOneData(const high_level_control::controlInputs::ConstPtr& received_data)
{
  // Business Logic for controlInputs_sub Subscriber
}
//# End controlInputs_sub_OnOneData Marker

// Timer Callback - armTimer
//# Start armTimerCallback Marker
void arm_controller::armTimerCallback(const ros::TimerEvent& event)
{
  // Business Logic for armTimer Timer
}
//# End armTimerCallback Marker


// Destructor - Cleanup Ports & Timers
arm_controller::~arm_controller()
{
  armTimer.stop();
  armState_pub.shutdown();
  sampleState_pub.shutdown();
  payloadBayState_pub.shutdown();
  controlInputs_sub.shutdown();
  verticalPos_client.shutdown();
  radialPos_client.shutdown();
  armRotation_client.shutdown();
  gripperRotation_client.shutdown();
  gripperPos_client.shutdown();
  sampleStateFromImage_client.shutdown();
  payloadBayStateFromImage_client.shutdown();
  //# Start Destructor Marker
  //# End Destructor Marker
}

// Startup - Setup Component Ports & Timers
void arm_controller::startUp()
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
       boost::bind(&arm_controller::controlInputs_sub_OnOneData, this, _1),
       ros::VoidPtr(),
       &this->compQueue);
  this->controlInputs_sub = nh.subscribe(controlInputs_sub_options);

  // Component Publisher - armState_pub
  advertiseName = "armState";
  if (portGroupMap.find("armState_pub") != portGroupMap.end())
    advertiseName += "_" + portGroupMap["armState_pub"];
  this->armState_pub = nh.advertise<high_level_control::armState>(advertiseName.c_str(), 1000);
  // Component Publisher - sampleState_pub
  advertiseName = "sampleState";
  if (portGroupMap.find("sampleState_pub") != portGroupMap.end())
    advertiseName += "_" + portGroupMap["sampleState_pub"];
  this->sampleState_pub = nh.advertise<high_level_control::sampleState>(advertiseName.c_str(), 1000);
  // Component Publisher - payloadBayState_pub
  advertiseName = "payloadBayState";
  if (portGroupMap.find("payloadBayState_pub") != portGroupMap.end())
    advertiseName += "_" + portGroupMap["payloadBayState_pub"];
  this->payloadBayState_pub = nh.advertise<high_level_control::payloadBayState>(advertiseName.c_str(), 1000);

  // Configure all required services associated with this component
  // Component Client - verticalPos_client
  advertiseName = "verticalPos";
  if (portGroupMap.find("verticalPos_client") != portGroupMap.end())
    advertiseName += "_" + portGroupMap["verticalPos_client"];
      this->verticalPos_client = nh.serviceClient<motor_control::verticalPos>(advertiseName.c_str()); 
  // Component Client - radialPos_client
  advertiseName = "radialPos";
  if (portGroupMap.find("radialPos_client") != portGroupMap.end())
    advertiseName += "_" + portGroupMap["radialPos_client"];
      this->radialPos_client = nh.serviceClient<motor_control::radialPos>(advertiseName.c_str()); 
  // Component Client - armRotation_client
  advertiseName = "armRotation";
  if (portGroupMap.find("armRotation_client") != portGroupMap.end())
    advertiseName += "_" + portGroupMap["armRotation_client"];
      this->armRotation_client = nh.serviceClient<motor_control::armRotation>(advertiseName.c_str()); 
  // Component Client - gripperRotation_client
  advertiseName = "gripperRotation";
  if (portGroupMap.find("gripperRotation_client") != portGroupMap.end())
    advertiseName += "_" + portGroupMap["gripperRotation_client"];
      this->gripperRotation_client = nh.serviceClient<motor_control::gripperRotation>(advertiseName.c_str()); 
  // Component Client - gripperPos_client
  advertiseName = "gripperPos";
  if (portGroupMap.find("gripperPos_client") != portGroupMap.end())
    advertiseName += "_" + portGroupMap["gripperPos_client"];
      this->gripperPos_client = nh.serviceClient<motor_control::gripperPos>(advertiseName.c_str()); 
  // Component Client - sampleStateFromImage_client
  advertiseName = "sampleStateFromImage";
  if (portGroupMap.find("sampleStateFromImage_client") != portGroupMap.end())
    advertiseName += "_" + portGroupMap["sampleStateFromImage_client"];
      this->sampleStateFromImage_client = nh.serviceClient<image_processing::sampleStateFromImage>(advertiseName.c_str()); 
  // Component Client - payloadBayStateFromImage_client
  advertiseName = "payloadBayStateFromImage";
  if (portGroupMap.find("payloadBayStateFromImage_client") != portGroupMap.end())
    advertiseName += "_" + portGroupMap["payloadBayStateFromImage_client"];
      this->payloadBayStateFromImage_client = nh.serviceClient<image_processing::payloadBayStateFromImage>(advertiseName.c_str()); 

  // Init Timer
  ros::TimerOptions timer_options;
  timer_options = 
    ros::TimerOptions
    (ros::Duration(-1),
     boost::bind(&arm_controller::Init, this, _1),
     &this->compQueue,
     true);
  this->initOneShotTimer = nh.createTimer(timer_options);  
  
  // Component Timer - timer.properties["name"]
  timer_options = 
    ros::TimerOptions
    (ros::Duration(0.2),
     boost::bind(&arm_controller::armTimerCallback, this, _1),
     &this->compQueue);
  this->armTimer = nh.createTimer(timer_options);

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
    return new arm_controller(config,argc,argv);
  }
}
