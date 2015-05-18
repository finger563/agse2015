#include "high_level_control/user_input_controller.hpp"

//# Start User Globals Marker
//# End User Globals Marker

// Initialization Function
//# Start Init Marker
void user_input_controller::Init(const ros::TimerEvent& event)
{
  // Initialize Here

  // Stop Init Timer
  initOneShotTimer.stop();
}
//# End Init Marker

// Subscriber Callback - armState_sub
//# Start armState_sub_OnOneData Marker
void user_input_controller::armState_sub_OnOneData(const high_level_control::armState::ConstPtr& received_data)
{
  // Business Logic for armState_sub Subscriber
}
//# End armState_sub_OnOneData Marker

// Timer Callback - userInputTimer
//# Start userInputTimerCallback Marker
void user_input_controller::userInputTimerCallback(const ros::TimerEvent& event)
{
  // Business Logic for userInputTimer Timer
}
//# End userInputTimerCallback Marker


// Destructor - Cleanup Ports & Timers
user_input_controller::~user_input_controller()
{
  userInputTimer.stop();
  controlInputs_pub.shutdown();
  armState_sub.shutdown();
  //# Start Destructor Marker
  //# End Destructor Marker
}

// Startup - Setup Component Ports & Timers
void user_input_controller::startUp()
{
  ros::NodeHandle nh;
  std::string advertiseName;

  // Component Subscriber - armState_sub
  advertiseName = "armState";
  if (portGroupMap.find("armState_sub") != portGroupMap.end())
    advertiseName += "_" + portGroupMap["armState_sub"];
  ros::SubscribeOptions armState_sub_options;
  armState_sub_options = ros::SubscribeOptions::create<high_level_control::armState>
      (advertiseName.c_str(),
       1000,
       boost::bind(&user_input_controller::armState_sub_OnOneData, this, _1),
       ros::VoidPtr(),
       &this->compQueue);
  this->armState_sub = nh.subscribe(armState_sub_options);

  // Component Publisher - controlInputs_pub
  advertiseName = "controlInputs";
  if (portGroupMap.find("controlInputs_pub") != portGroupMap.end())
    advertiseName += "_" + portGroupMap["controlInputs_pub"];
  this->controlInputs_pub = nh.advertise<high_level_control::controlInputs>(advertiseName.c_str(), 1000);

  // Init Timer
  ros::TimerOptions timer_options;
  timer_options = 
    ros::TimerOptions
    (ros::Duration(-1),
     boost::bind(&user_input_controller::Init, this, _1),
     &this->compQueue,
     true);
  this->initOneShotTimer = nh.createTimer(timer_options);  
  
  // Component Timer - timer.properties["name"]
  timer_options = 
    ros::TimerOptions
    (ros::Duration(0.1),
     boost::bind(&user_input_controller::userInputTimerCallback, this, _1),
     &this->compQueue);
  this->userInputTimer = nh.createTimer(timer_options);

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
    return new user_input_controller(config,argc,argv);
  }
}
