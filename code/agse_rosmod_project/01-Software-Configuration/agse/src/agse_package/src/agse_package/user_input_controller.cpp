#include "agse_package/user_input_controller.hpp"

//# Start User Globals Marker

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

  // THESE NEED TO BE UPDATED
  pauseSwitchPin = 57;

  // set up the pins to control the h-bridge
  gpio_export(pauseSwitchPin);
  gpio_set_dir(pauseSwitchPin,INPUT_PIN);
    // Stop Init Timer
    initOneShotTimer.stop();
}
//# End Init Marker

// OnOneData Subscription handler for sampleState_sub subscriber
//# Start sampleState_sub_OnOneData Marker
void user_input_controller::sampleState_sub_OnOneData(const agse_package::sampleState::ConstPtr& received_data)
{
    // Business Logic for sampleState_sub subscriber subscribing to topic sampleState callback 

}
//# End sampleState_sub_OnOneData Marker
// OnOneData Subscription handler for payloadBayState_sub subscriber
//# Start payloadBayState_sub_OnOneData Marker
void user_input_controller::payloadBayState_sub_OnOneData(const agse_package::payloadBayState::ConstPtr& received_data)
{
    // Business Logic for payloadBayState_sub subscriber subscribing to topic payloadBayState callback 

}
//# End payloadBayState_sub_OnOneData Marker

// Callback for userInputTimer timer
//# Start userInputTimerCallback Marker
void user_input_controller::userInputTimerCallback(const ros::TimerEvent& event)
{
    // Business Logic for userInputTimer 
  unsigned int previousSwitchState = pauseSwitchState;
  gpio_get_value(pauseSwitchPin, &pauseSwitchState);
  if ( previousSwitchState != pauseSwitchState )
    {
      paused = (pauseSwitchState == HIGH) ? true : false;
      agse_package::controlInputs control;
      control.paused = paused;
      controlInputs_pub.publish(control);
      if (paused)
	ROS_INFO("Pausing the system!");
      else
	ROS_INFO("Unpausing the system!");
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
    if ( portGroupMap != NULL && portGroupMap->find(advertiseName) != portGroupMap->end() )
        advertiseName += "_" + (*portGroupMap)[advertiseName];
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
    if ( portGroupMap != NULL && portGroupMap->find(advertiseName) != portGroupMap->end() )
        advertiseName += "_" + (*portGroupMap)[advertiseName];
    ros::SubscribeOptions payloadBayState_sub_options;
    payloadBayState_sub_options = 
	ros::SubscribeOptions::create<agse_package::payloadBayState>
	    (advertiseName.c_str(),
	     1000,
	     boost::bind(&user_input_controller::payloadBayState_sub_OnOneData, this, _1),
	     ros::VoidPtr(),
             &this->compQueue);
    this->payloadBayState_sub = nh.subscribe(payloadBayState_sub_options);

    // Configure all publishers associated with this component
    // publisher: controlInputs_pub
    advertiseName = "controlInputs";
    if ( portGroupMap != NULL && portGroupMap->find(advertiseName) != portGroupMap->end() )
        advertiseName += "_" + (*portGroupMap)[advertiseName];
    this->controlInputs_pub = nh.advertise<agse_package::controlInputs>
	(advertiseName.c_str(), 1000);	

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
             (ros::Duration(0.01),
	     boost::bind(&user_input_controller::userInputTimerCallback, this, _1),
	     &this->compQueue);
    this->userInputTimer = nh.createTimer(timer_options);

}
