#include "motor_control/servo_controller.hpp"

//# Start User Globals Marker
//# End User Globals Marker

// Initialization Function
//# Start Init Marker
void servo_controller::Init(const ros::TimerEvent& event)
{
  // Initialize Here

  // Stop Init Timer
  initOneShotTimer.stop();
}
//# End Init Marker

// Subscriber Callback - controlInputs_sub
//# Start controlInputs_sub_OnOneData Marker
void servo_controller::controlInputs_sub_OnOneData(const high_level_control::controlInputs::ConstPtr& received_data)
{
  // Business Logic for controlInputs_sub Subscriber
}
//# End controlInputs_sub_OnOneData Marker

// Server Callback - gripperRotation_server
//# Start gripperRotationCallback Marker
bool servo_controller::gripperRotationCallback(motor_control::gripperRotation::Request  &req,
  motor_control::gripperRotation::Response &res)
{
  // Business Logic for gripperRotation_server Server

  return true;
}
//# End gripperRotationCallback Marker
// Server Callback - armRotation_server
//# Start armRotationCallback Marker
bool servo_controller::armRotationCallback(motor_control::armRotation::Request  &req,
  motor_control::armRotation::Response &res)
{
  // Business Logic for armRotation_server Server

  return true;
}
//# End armRotationCallback Marker
// Server Callback - gripperPos_server
//# Start gripperPosCallback Marker
bool servo_controller::gripperPosCallback(motor_control::gripperPos::Request  &req,
  motor_control::gripperPos::Response &res)
{
  // Business Logic for gripperPos_server Server

  return true;
}
//# End gripperPosCallback Marker

// Timer Callback - servoTimer
//# Start servoTimerCallback Marker
void servo_controller::servoTimerCallback(const ros::TimerEvent& event)
{
  // Business Logic for servoTimer Timer
}
//# End servoTimerCallback Marker


// Destructor - Cleanup Ports & Timers
servo_controller::~servo_controller()
{
  servoTimer.stop();
  controlInputs_sub.shutdown();
  gripperRotation_server.shutdown();
  armRotation_server.shutdown();
  gripperPos_server.shutdown();
  //# Start Destructor Marker

  //# End Destructor Marker
}

// Startup - Setup Component Ports & Timers
void servo_controller::startUp()
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
       boost::bind(&servo_controller::controlInputs_sub_OnOneData, this, _1),
       ros::VoidPtr(),
       &this->compQueue);
  this->controlInputs_sub = nh.subscribe(controlInputs_sub_options);

  // Component Server - gripperRotation_server
  advertiseName = "gripperRotation";
  if (portGroupMap.find("gripperRotation_server") != portGroupMap.end())
    advertiseName += "_" + portGroupMap["gripperRotation_server"];
  ros::AdvertiseServiceOptions gripperRotation_server_server_options;
  gripperRotation_server_server_options = ros::AdvertiseServiceOptions::create<motor_control::gripperRotation>
      (advertiseName.c_str(),
       boost::bind(&servo_controller::gripperRotationCallback, this, _1, _2),
       ros::VoidPtr(),
       &this->compQueue);
  this->gripperRotation_server = nh.advertiseService(gripperRotation_server_server_options);
  // Component Server - armRotation_server
  advertiseName = "armRotation";
  if (portGroupMap.find("armRotation_server") != portGroupMap.end())
    advertiseName += "_" + portGroupMap["armRotation_server"];
  ros::AdvertiseServiceOptions armRotation_server_server_options;
  armRotation_server_server_options = ros::AdvertiseServiceOptions::create<motor_control::armRotation>
      (advertiseName.c_str(),
       boost::bind(&servo_controller::armRotationCallback, this, _1, _2),
       ros::VoidPtr(),
       &this->compQueue);
  this->armRotation_server = nh.advertiseService(armRotation_server_server_options);
  // Component Server - gripperPos_server
  advertiseName = "gripperPos";
  if (portGroupMap.find("gripperPos_server") != portGroupMap.end())
    advertiseName += "_" + portGroupMap["gripperPos_server"];
  ros::AdvertiseServiceOptions gripperPos_server_server_options;
  gripperPos_server_server_options = ros::AdvertiseServiceOptions::create<motor_control::gripperPos>
      (advertiseName.c_str(),
       boost::bind(&servo_controller::gripperPosCallback, this, _1, _2),
       ros::VoidPtr(),
       &this->compQueue);
  this->gripperPos_server = nh.advertiseService(gripperPos_server_server_options);
 
  // Init Timer
  ros::TimerOptions timer_options;
  timer_options = 
    ros::TimerOptions
    (ros::Duration(-1),
     boost::bind(&servo_controller::Init, this, _1),
     &this->compQueue,
     true);
  this->initOneShotTimer = nh.createTimer(timer_options);  
  
  // Component Timer - timer.properties["name"]
  timer_options = 
    ros::TimerOptions
    (ros::Duration(0.25),
     boost::bind(&servo_controller::servoTimerCallback, this, _1),
     &this->compQueue);
  this->servoTimer = nh.createTimer(timer_options);

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
    return new servo_controller(config,argc,argv);
  }
}
