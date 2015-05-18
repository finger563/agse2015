#include "motor_control/vertical_actuator_controller.hpp"

//# Start User Globals Marker
//# End User Globals Marker

// Initialization Function
//# Start Init Marker
void vertical_actuator_controller::Init(const ros::TimerEvent& event)
{
  // Initialize Here

  // Stop Init Timer
  initOneShotTimer.stop();
}
//# End Init Marker

// Subscriber Callback - controlInputs_sub
//# Start controlInputs_sub_OnOneData Marker
void vertical_actuator_controller::controlInputs_sub_OnOneData(const high_level_control::controlInputs::ConstPtr& received_data)
{
  // Business Logic for controlInputs_sub Subscriber
}
//# End controlInputs_sub_OnOneData Marker

// Server Callback - verticalPos_server
//# Start verticalPosCallback Marker
bool vertical_actuator_controller::verticalPosCallback(motor_control::verticalPos::Request  &req,
  motor_control::verticalPos::Response &res)
{
  // Business Logic for verticalPos_server Server

  return true;
}
//# End verticalPosCallback Marker

// Timer Callback - verticalPosTimer
//# Start verticalPosTimerCallback Marker
void vertical_actuator_controller::verticalPosTimerCallback(const ros::TimerEvent& event)
{
  // Business Logic for verticalPosTimer Timer
}
//# End verticalPosTimerCallback Marker


// Destructor - Cleanup Ports & Timers
vertical_actuator_controller::~vertical_actuator_controller()
{
  verticalPosTimer.stop();
  controlInputs_sub.shutdown();
  verticalPos_server.shutdown();
  //# Start Destructor Marker
  //# End Destructor Marker
}

// Startup - Setup Component Ports & Timers
void vertical_actuator_controller::startUp()
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
       boost::bind(&vertical_actuator_controller::controlInputs_sub_OnOneData, this, _1),
       ros::VoidPtr(),
       &this->compQueue);
  this->controlInputs_sub = nh.subscribe(controlInputs_sub_options);

  // Component Server - verticalPos_server
  advertiseName = "verticalPos";
  if (portGroupMap.find("verticalPos_server") != portGroupMap.end())
    advertiseName += "_" + portGroupMap["verticalPos_server"];
  ros::AdvertiseServiceOptions verticalPos_server_server_options;
  verticalPos_server_server_options = ros::AdvertiseServiceOptions::create<motor_control::verticalPos>
      (advertiseName.c_str(),
       boost::bind(&vertical_actuator_controller::verticalPosCallback, this, _1, _2),
       ros::VoidPtr(),
       &this->compQueue);
  this->verticalPos_server = nh.advertiseService(verticalPos_server_server_options);
 
  // Init Timer
  ros::TimerOptions timer_options;
  timer_options = 
    ros::TimerOptions
    (ros::Duration(-1),
     boost::bind(&vertical_actuator_controller::Init, this, _1),
     &this->compQueue,
     true);
  this->initOneShotTimer = nh.createTimer(timer_options);  
  
  // Component Timer - timer.properties["name"]
  timer_options = 
    ros::TimerOptions
    (ros::Duration(0.01),
     boost::bind(&vertical_actuator_controller::verticalPosTimerCallback, this, _1),
     &this->compQueue);
  this->verticalPosTimer = nh.createTimer(timer_options);

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
    return new vertical_actuator_controller(config,argc,argv);
  }
}
