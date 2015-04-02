#include "agse_package/user_input_controller.hpp"

//# Start User Globals Marker
#include "agse_package/uip.h"
//# End User Globals Marker

// -------------------------------------------------------
// BUSINESS LOGIC OF THESE FUNCTIONS SUPPLIED BY DEVELOPER
// -------------------------------------------------------

// Init Function
//# Start Init Marker
void user_input_controller::Init(const ros::TimerEvent& event)
{
    // Initialize Component
  paused = true;
  halted = false;
  manual = false;
 
  // PAUSE Switch - Amber
  pauseSwitchPin = 67;
  gpio_export(pauseSwitchPin);
  gpio_set_dir(pauseSwitchPin,INPUT_PIN);

  // MANUAL Switch - Blue
  manualSwitchPin = 68;
  gpio_export(manualSwitchPin);
  gpio_set_dir(manualSwitchPin,INPUT_PIN);

  // HALT Switch - Red
  haltSwitchPin = 69;
  gpio_export(haltSwitchPin);
  gpio_set_dir(haltSwitchPin,INPUT_PIN);

  // PAUSE MAIN LED - Needs to be updated
  pauseLED = 76;
  gpio_export(pauseLED);
  gpio_set_dir(pauseLED, OUTPUT_PIN);
  pauseLEDBlinkDelay = 2;

  // The Four Images to show in UIP
  img1 = cvLoadImage("/home/debian/Repositories/agse2015/code/agse_rosmod_project/01-Software-Configuration/agse/devel/lib/agse_package/01.png");
  img2 = cvLoadImage("/home/debian/Repositories/agse2015/code/agse_rosmod_project/01-Software-Configuration/agse/devel/lib/agse_package/02.png");
  img3 = cvLoadImage("/home/debian/Repositories/agse2015/code/agse_rosmod_project/01-Software-Configuration/agse/devel/lib/agse_package/03.png");
  img4 = cvLoadImage("/home/debian/Repositories/agse2015/code/agse_rosmod_project/01-Software-Configuration/agse/devel/lib/agse_package/04.png");

    // Stop Init Timer
    initOneShotTimer.stop();
}
//# End Init Marker

// OnOneData Subscription handler for sampleState_sub subscriber
//# Start sampleState_sub_OnOneData Marker
void user_input_controller::sampleState_sub_OnOneData(const agse_package::sampleState::ConstPtr& received_data)
{
    // Business Logic for sampleState_sub subscriber subscribing to topic sampleState callback 
  sample = *received_data;
}
//# End sampleState_sub_OnOneData Marker
// OnOneData Subscription handler for payloadBayState_sub subscriber
//# Start payloadBayState_sub_OnOneData Marker
void user_input_controller::payloadBayState_sub_OnOneData(const agse_package::payloadBayState::ConstPtr& received_data)
{
    // Business Logic for payloadBayState_sub subscriber subscribing to topic payloadBayState callback 
  payloadBay = *received_data;
}
//# End payloadBayState_sub_OnOneData Marker
// OnOneData Subscription handler for armState_sub subscriber
//# Start armState_sub_OnOneData Marker
void user_input_controller::armState_sub_OnOneData(const agse_package::armState::ConstPtr& received_data)
{
    // Business Logic for armState_sub subscriber subscribing to topic armState callback 
  arm = *received_data;
}
//# End armState_sub_OnOneData Marker

// Callback for userInputTimer timer
//# Start userInputTimerCallback Marker
void user_input_controller::userInputTimerCallback(const ros::TimerEvent& event)
{
  // Business Logic for userInputTimer 

  // SHOW IMAGES HERE
  //  gpio_set_value(pauseSwitch_LEDPin, HIGH);
  //  cvShowManyImages("UIP", 4, img1, img2, img3, img4);

  // DRAW TEXT FOR SAMPLE STATE HERE
  // DRAW TEXT FOR PAYLOAD BAY STATE HERE

  // HANDLE SCREEN BUTTONS HERE

  // HANDLE MISSILE SWITCHES HERE
  unsigned int previousSwitchState = pauseSwitchState;
  gpio_get_value(pauseSwitchPin, &pauseSwitchState);
  ROS_INFO("Pause Switch State: %d", pauseSwitchState);
  agse_package::controlInputs control;
  
  if ( previousSwitchState != pauseSwitchState )
    {
      paused = (pauseSwitchState == HIGH) ? true : false;
      agse_package::controlInputs control;
      control.paused = paused;
      if (paused) {
	ROS_INFO("Pausing the system!");
      }
      else {
	ROS_INFO("Unpausing the system!");
      }
    }
  previousSwitchState = haltSwitchState;
  gpio_get_value(haltSwitchPin, &haltSwitchState);
  ROS_INFO("Halt Switch State: %d", haltSwitchState);
  if ( previousSwitchState != haltSwitchState )
    {
      halted = (haltSwitchState == HIGH) ? true : false;
      control.stop = halted;
      if (halted) {
	ROS_INFO("Halting the system!");
      }
      else {
	ROS_INFO("Un-halting the system!");
      }
    }
  previousSwitchState = manualSwitchState;
  gpio_get_value(manualSwitchPin, &manualSwitchState);
  ROS_INFO("Manual Switch State: %d", manualSwitchState);
  if ( previousSwitchState != manualSwitchState )
    {
      manual= (manualSwitchState == HIGH) ? true : false;
      control.manual = manual;
      if (manual) {
	ROS_INFO("Switching the system to manual!");
      }
      else {
	ROS_INFO("Switching the system to automatic!");
      }
    }
  controlInputs_pub.publish(control);

  // HANDLE LED OUTPUTS HERE
  static int currentBlinkDelay = 0;
  if (paused)
    {
      gpio_set_value(pauseLED,HIGH);
    }
  else
    {
      if (currentBlinkDelay++ < pauseLEDBlinkDelay)
	gpio_set_value(pauseLED,LOW);
      else
	{
	  gpio_set_value(pauseLED,HIGH);
	  currentBlinkDelay = 0;
	}
    }
}
//# End userInputTimerCallback Marker

// ---------------------------------------------
// EVERYTHING BELOW HERE IS COMPLETELY GENERATED
// ---------------------------------------------

// Destructor - required for clean shutdown when process is killed
user_input_controller::~user_input_controller()
{
    userInputTimer.stop();
    controlInputs_pub.shutdown();
    sampleState_sub.shutdown();
    payloadBayState_sub.shutdown();
    armState_sub.shutdown();
    captureImage_client.shutdown();
//# Start Destructor Marker

//# End Destructor Marker
}

void user_input_controller::startUp()
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
    // subscriber: sampleState_sub
    advertiseName = "sampleState";
    if ( portGroupMap != NULL && portGroupMap->find("sampleState_sub") != portGroupMap->end() )
        advertiseName += "_" + (*portGroupMap)["sampleState_sub"];
    ros::SubscribeOptions sampleState_sub_options;
    sampleState_sub_options = 
	ros::SubscribeOptions::create<agse_package::sampleState>
	    (advertiseName.c_str(),
	     1000,
	     boost::bind(&user_input_controller::sampleState_sub_OnOneData, this, _1),
	     ros::VoidPtr(),
             &this->compQueue);
    this->sampleState_sub = nh.subscribe(sampleState_sub_options);
    // subscriber: payloadBayState_sub
    advertiseName = "payloadBayState";
    if ( portGroupMap != NULL && portGroupMap->find("payloadBayState_sub") != portGroupMap->end() )
        advertiseName += "_" + (*portGroupMap)["payloadBayState_sub"];
    ros::SubscribeOptions payloadBayState_sub_options;
    payloadBayState_sub_options = 
	ros::SubscribeOptions::create<agse_package::payloadBayState>
	    (advertiseName.c_str(),
	     1000,
	     boost::bind(&user_input_controller::payloadBayState_sub_OnOneData, this, _1),
	     ros::VoidPtr(),
             &this->compQueue);
    this->payloadBayState_sub = nh.subscribe(payloadBayState_sub_options);
    // subscriber: armState_sub
    advertiseName = "armState";
    if ( portGroupMap != NULL && portGroupMap->find("armState_sub") != portGroupMap->end() )
        advertiseName += "_" + (*portGroupMap)["armState_sub"];
    ros::SubscribeOptions armState_sub_options;
    armState_sub_options = 
	ros::SubscribeOptions::create<agse_package::armState>
	    (advertiseName.c_str(),
	     1000,
	     boost::bind(&user_input_controller::armState_sub_OnOneData, this, _1),
	     ros::VoidPtr(),
             &this->compQueue);
    this->armState_sub = nh.subscribe(armState_sub_options);

    // Configure all publishers associated with this component
    // publisher: controlInputs_pub
    advertiseName = "controlInputs";
    if ( portGroupMap != NULL && portGroupMap->find("controlInputs_pub") != portGroupMap->end() )
        advertiseName += "_" + (*portGroupMap)["controlInputs_pub"];
    this->controlInputs_pub = nh.advertise<agse_package::controlInputs>
	(advertiseName.c_str(), 1000);	

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
	     boost::bind(&user_input_controller::Init, this, _1),
	     &this->compQueue,
             true);
    this->initOneShotTimer = nh.createTimer(timer_options);  
  
    // Create all component timers
    // timer: timer.properties["name"]
    timer_options = 
	ros::TimerOptions
             (ros::Duration(0.2),
	     boost::bind(&user_input_controller::userInputTimerCallback, this, _1),
	     &this->compQueue);
    this->userInputTimer = nh.createTimer(timer_options);


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
