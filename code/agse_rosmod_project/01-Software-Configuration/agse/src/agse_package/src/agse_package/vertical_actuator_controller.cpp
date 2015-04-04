#include "agse_package/vertical_actuator_controller.hpp"

//# Start User Globals Marker
#include <stdlib.h>
//# End User Globals Marker

// -------------------------------------------------------
// BUSINESS LOGIC OF THESE FUNCTIONS SUPPLIED BY DEVELOPER
// -------------------------------------------------------

// Init Function
//# Start Init Marker
void vertical_actuator_controller::Init(const ros::TimerEvent& event)
{
    // Initialize Component
  paused = true;
  lowerLimitReached = false;

  // THESE NEED TO BE UPDATED
  epsilon = 100;
  motorForwardPin = 89; //86;  // connected to GPIO2_22, pin P8_27
  motorBackwardPin = 88; //87; // connected to GPIO2_23, pin P8_29
  lowerLimitSwitchPin = 65;       // connected to GPIO2_01, pin P8_18
  
  adcPin = 1;  // connected to ADC1, pin P9_40

  // set up the pins to control the h-bridge
  gpio_export(motorForwardPin);
  gpio_export(motorBackwardPin);
  gpio_export(lowerLimitSwitchPin);
  gpio_set_dir(motorForwardPin,OUTPUT_PIN);
  gpio_set_dir(motorBackwardPin,OUTPUT_PIN);
  gpio_set_dir(lowerLimitSwitchPin,INPUT_PIN);
  // set up the encoder module
  vm_eqep_period = 1000000000L;
  verticalMotoreQEP.initialize("/sys/devices/ocp.3/48302000.epwmss/48302180.eqep", eQEP::eQEP_Mode_Absolute);
  verticalMotoreQEP.set_period(vm_eqep_period);

  // Command line args for radial goal
  for (int i = 0; i < node_argc; i++)
    {
      if (!strcmp(node_argv[i], "-unpaused"))
	{
	  paused = false;
	}
      if (!strcmp(node_argv[i], "-v"))
	{
	  verticalGoal = atoi(node_argv[i+1]);
	}
      if (!strcmp(node_argv[i], "-e"))
	{
	  epsilon = atoi(node_argv[i+1]);
	}
    }

  ROS_INFO("VERTICAL GOAL SET TO : %d",verticalGoal);

    // Stop Init Timer
    initOneShotTimer.stop();
}
//# End Init Marker

// OnOneData Subscription handler for controlInputs_sub subscriber
//# Start controlInputs_sub_OnOneData Marker
void vertical_actuator_controller::controlInputs_sub_OnOneData(const agse_package::controlInputs::ConstPtr& received_data)
{
    // Business Logic for controlInputs_sub subscriber subscribing to topic controlInputs callback 
  paused = received_data->paused;
  ROS_INFO( paused ? "Vertical motor PAUSED!" : "Vertical motor UNPAUSED!");
}
//# End controlInputs_sub_OnOneData Marker

// Component Service Callback
//# Start verticalPosCallback Marker
bool vertical_actuator_controller::verticalPosCallback(agse_package::verticalPos::Request  &req,
    agse_package::verticalPos::Response &res)
{
    // Business Logic for verticalPos_server Server providing verticalPos Service
  if (req.update == true)
    {
      ROS_INFO("GOT NEW HEIGHT GOAL: %d",(int)req.goal);
      ROS_INFO("CURRENT HEIGHT: %d",verticalCurrent);
      verticalGoal = req.goal;
    }
  if (req.setZeroPosition == true)
    {
      ROS_INFO("ZEROED HEIGHT ENCODER");
      verticalMotoreQEP.set_position(0);
    }
  res.lowerLimitReached = lowerLimitReached;
  res.upperLimitReached = false;
  res.current = verticalCurrent;
  return true;
}
//# End verticalPosCallback Marker

// Callback for verticalPosTimer timer
//# Start verticalPosTimerCallback Marker
void vertical_actuator_controller::verticalPosTimerCallback(const ros::TimerEvent& event)
{
    // Business Logic for verticalPosTimer 
  if (!paused)
    {
      // read current value for vertical position (encoder)
      verticalCurrent = verticalMotoreQEP.get_position();
      //ROS_INFO("Vertical Actuator Encoder Reading: %d",verticalCurrent);

      unsigned int limitSwitchState = 0;
      unsigned int backwardPinState = 0;
      gpio_get_value(lowerLimitSwitchPin,&limitSwitchState);
      gpio_get_value(motorBackwardPin,&backwardPinState);
      if (backwardPinState && !limitSwitchState)
	{
	  ROS_INFO("LOWER LIMIT REACHED: HEIGHT");
	  lowerLimitReached = true;
	}
      // update motor based on current value
      if ( abs(verticalGoal-verticalCurrent) > epsilon ) // if there's significant reason to move
	{
	  if (verticalGoal > verticalCurrent) 
	    {
	      lowerLimitReached = false;
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
  else 
    {
      gpio_set_value(motorForwardPin,LOW);
      gpio_set_value(motorBackwardPin,LOW);      
    }
}
//# End verticalPosTimerCallback Marker

// ---------------------------------------------
// EVERYTHING BELOW HERE IS COMPLETELY GENERATED
// ---------------------------------------------

// Destructor - required for clean shutdown when process is killed
vertical_actuator_controller::~vertical_actuator_controller()
{
    verticalPosTimer.stop();
    controlInputs_sub.shutdown();
    verticalPos_server.shutdown();
//# Start Destructor Marker

//# End Destructor Marker
}

void vertical_actuator_controller::startUp()
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
	     boost::bind(&vertical_actuator_controller::controlInputs_sub_OnOneData, this, _1),
	     ros::VoidPtr(),
             &this->compQueue);
    this->controlInputs_sub = nh.subscribe(controlInputs_sub_options);

    // Configure all provided services associated with this component
    // server: verticalPos_server
    advertiseName = "verticalPos";
    if ( portGroupMap != NULL && portGroupMap->find("verticalPos_server") != portGroupMap->end() )
        advertiseName += "_" + (*portGroupMap)["verticalPos_server"];
    ros::AdvertiseServiceOptions verticalPos_server_options;
    verticalPos_server_options = 
	ros::AdvertiseServiceOptions::create<agse_package::verticalPos>
	    (advertiseName.c_str(),
             boost::bind(&vertical_actuator_controller::verticalPosCallback, this, _1, _2),
	     ros::VoidPtr(),
             &this->compQueue);
    this->verticalPos_server = nh.advertiseService(verticalPos_server_options);
 
    // Create Init Timer
    ros::TimerOptions timer_options;
    timer_options = 
	ros::TimerOptions
	    (ros::Duration(-1),
	     boost::bind(&vertical_actuator_controller::Init, this, _1),
	     &this->compQueue,
             true);
    this->initOneShotTimer = nh.createTimer(timer_options);  
  
    // Create all component timers
    // timer: timer.properties["name"]
    timer_options = 
	ros::TimerOptions
             (ros::Duration(0.01),
	     boost::bind(&vertical_actuator_controller::verticalPosTimerCallback, this, _1),
	     &this->compQueue);
    this->verticalPosTimer = nh.createTimer(timer_options);


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
