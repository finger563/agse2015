#include "agse_package/radial_actuator_controller.hpp"

//# Start User Globals Marker
#include <stdlib.h>
//# End User Globals Marker

// -------------------------------------------------------
// BUSINESS LOGIC OF THESE FUNCTIONS SUPPLIED BY DEVELOPER
// -------------------------------------------------------

// Init Function
//# Start Init Marker
void radial_actuator_controller::Init(const ros::TimerEvent& event)
{
    // Initialize Component
  paused = true;

  // THESE NEED TO BE UPDATED
  epsilon = 10;
  motorForwardPin = 87; //88;     // connected to GPIO2_24, pin P8_28
  motorBackwardPin = 86; //89;    // connected to GPIO2_25, pin P8_30

  // Limit Switch Pin GPIO P8_20 & P8_21
  
  
  adcPin = 0;  // connected to ADC0, pin P9_39

  // set up the pins to control the h-bridge
  gpio_export(motorForwardPin);
  gpio_export(motorBackwardPin);
  gpio_set_dir(motorForwardPin,OUTPUT_PIN);
  gpio_set_dir(motorBackwardPin,OUTPUT_PIN);
  // set up the encoder module
  rm_eqep_period = 1000000000L;
  radialMotoreQEP.initialize("/sys/devices/ocp.3/48304000.epwmss/48304180.eqep", eQEP::eQEP_Mode_Absolute);
  radialMotoreQEP.set_period(rm_eqep_period);
  // initialize the goal position to 0

  // Command line args for radial goal
  for (int i = 0; i < node_argc; i++) {
    if (!strcmp(node_argv[i], "-r")) {
      radialGoal = atoi(node_argv[i+1]);
    }
    if (!strcmp(node_argv[i], "-e")) {
      epsilon = atoi(node_argv[i+1]);
    }
  }

  ROS_INFO("RADIAL GOAL SET TO : %d",radialGoal);

  //  radialGoal = 1000;
    // Stop Init Timer
    initOneShotTimer.stop();
}
//# End Init Marker

// OnOneData Subscription handler for controlInputs_sub subscriber
//# Start controlInputs_sub_OnOneData Marker
void radial_actuator_controller::controlInputs_sub_OnOneData(const agse_package::controlInputs::ConstPtr& received_data)
{
    // Business Logic for controlInputs_sub subscriber subscribing to topic controlInputs callback 
  paused = received_data->paused;
  ROS_INFO( paused ? "Radial motor PAUSED!" : "Radial motor UNPAUSED!");
}
//# End controlInputs_sub_OnOneData Marker

// Component Service Callback
//# Start radialPosCallback Marker
bool radial_actuator_controller::radialPosCallback(agse_package::radialPos::Request  &req,
    agse_package::radialPos::Response &res)
{
    // Business Logic for radialPos_server Server providing radialPos Service
  if (req.update == true)
    {
      radialGoal = req.goal;
    }
  res.current = radialCurrent;
  return true;
}
//# End radialPosCallback Marker

// Callback for radialPosTimer timer
//# Start radialPosTimerCallback Marker
void radial_actuator_controller::radialPosTimerCallback(const ros::TimerEvent& event)
{
    // Business Logic for radialPosTimer 
  if (paused)
    {
      // read current value for radial position (encoder)
      radialCurrent = radialMotoreQEP.get_position();
      //ROS_INFO("Raidal Actuator Encoder Reading: %d",radialCurrent);

      //unsigned int adcVal = 0;
      //adc_get_value(adcPin,&adcVal);
      //radialCurrent = adcVal;
      //ROS_INFO("Got ADC %d value : %d",adcPin,adcVal);

      // update motor based on current value
      if ( abs(radialGoal-radialCurrent) > epsilon ) // if there's significant reason to move
	{
	  if (radialGoal > radialCurrent) 
	    {
	      gpio_set_value(motorBackwardPin,LOW);
	      gpio_set_value(motorForwardPin,HIGH);
	    }
	  else
	    {
	      gpio_set_value(motorForwardPin,LOW);
	      gpio_set_value(motorBackwardPin,HIGH);
	    }
	}
      else
	{
	  gpio_set_value(motorForwardPin,LOW);
	  gpio_set_value(motorBackwardPin,LOW);
	}
    }
}
//# End radialPosTimerCallback Marker

// ---------------------------------------------
// EVERYTHING BELOW HERE IS COMPLETELY GENERATED
// ---------------------------------------------

// Destructor - required for clean shutdown when process is killed
radial_actuator_controller::~radial_actuator_controller()
{
    radialPosTimer.stop();
    controlInputs_sub.shutdown();
    radialPos_server.shutdown();
//# Start Destructor Marker

//# End Destructor Marker
}

void radial_actuator_controller::startUp()
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
	     boost::bind(&radial_actuator_controller::controlInputs_sub_OnOneData, this, _1),
	     ros::VoidPtr(),
             &this->compQueue);
    this->controlInputs_sub = nh.subscribe(controlInputs_sub_options);

    // Configure all provided services associated with this component
    // server: radialPos_server
    advertiseName = "radialPos";
    if ( portGroupMap != NULL && portGroupMap->find("radialPos_server") != portGroupMap->end() )
        advertiseName += "_" + (*portGroupMap)["radialPos_server"];
    ros::AdvertiseServiceOptions radialPos_server_options;
    radialPos_server_options = 
	ros::AdvertiseServiceOptions::create<agse_package::radialPos>
	    (advertiseName.c_str(),
             boost::bind(&radial_actuator_controller::radialPosCallback, this, _1, _2),
	     ros::VoidPtr(),
             &this->compQueue);
    this->radialPos_server = nh.advertiseService(radialPos_server_options);
 
    // Create Init Timer
    ros::TimerOptions timer_options;
    timer_options = 
	ros::TimerOptions
	    (ros::Duration(-1),
	     boost::bind(&radial_actuator_controller::Init, this, _1),
	     &this->compQueue,
             true);
    this->initOneShotTimer = nh.createTimer(timer_options);  
  
    // Create all component timers
    // timer: timer.properties["name"]
    timer_options = 
	ros::TimerOptions
             (ros::Duration(0.01),
	     boost::bind(&radial_actuator_controller::radialPosTimerCallback, this, _1),
	     &this->compQueue);
    this->radialPosTimer = nh.createTimer(timer_options);


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
