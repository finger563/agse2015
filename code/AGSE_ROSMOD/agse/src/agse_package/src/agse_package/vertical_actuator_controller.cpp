#include "agse_package/vertical_actuator_controller.hpp"
#include "agse_package/gpio.h"

// -------------------------------------------------------
// BUSINESS LOGIC OF THESE FUNCTIONS SUPPLIED BY DEVELOPER
// -------------------------------------------------------

unsigned int LEDGPIO = 57; // GPIO1_25 = (1x32) + 25 = 57
unsigned int READGPIO = 15; // GPIO0_15 = (0x32) + 15 = 15
bool ledState = true;
const unsigned int onTime = 50;
unsigned int ledTime = 0;

// Init Function
void vertical_actuator_controller::Init(const ros::TimerEvent& event)
{
    // Initialize Component
  gpio_export(LEDGPIO);
  gpio_export(READGPIO);
  gpio_set_dir(LEDGPIO,OUTPUT_PIN);
  gpio_set_dir(READGPIO,INPUT_PIN);
    // Stop Init Timer
    initOneShotTimer.stop();
}

// OnOneData Subscription handler for controlInputs_sub subscriber
void vertical_actuator_controller::controlInputs_sub_OnOneData(const agse_package::controlInputs::ConstPtr& received_data)
{
    // Business Logic for controlInputs_sub subscriber callback 
}

// Component Service Callback
bool vertical_actuator_controller::verticalPos_serverCallback(agse_package::verticalPos::Request  &req,
    agse_package::verticalPos::Response &res)
{
    // Business Logic for <listener.ROS_Server instance at 0xb5417bc0> Service
}

// Callback for verticalPosTimer timer
void vertical_actuator_controller::verticalPosTimerCallback(const ros::TimerEvent& event)
{
    // Business Logic for verticalPosTimer 
  if (ledTime < onTime)
    ledTime++;
  else
    {
      ledTime=0;
      if (ledState)
	gpio_set_value(LEDGPIO,HIGH);
      else
	gpio_set_value(LEDGPIO,LOW);
      ledState = ledState ? false : true;
    }
  unsigned int value = LOW;
  gpio_get_value(READGPIO, &value);
}

// ---------------------------------------------
// EVERYTHING BELOW HERE IS COMPLETELY GENERATED
// ---------------------------------------------

// Destructor - required for clean shutdown when process is killed
vertical_actuator_controller::~vertical_actuator_controller()
{
    verticalPosTimer.stop();
    controlInputs_sub.shutdown();
    verticalPos_server_server.shutdown();
}

void vertical_actuator_controller::startUp()
{
    ros::NodeHandle nh;

    // Configure all subscribers associated with this component
    ros::SubscribeOptions controlInputs_sub_options;
    controlInputs_sub_options = 
	ros::SubscribeOptions::create<agse_package::controlInputs>
	    ("controlInputs",
	     1000,
	     boost::bind(&vertical_actuator_controller::controlInputs_sub_OnOneData, this, _1),
	     ros::VoidPtr(),
             &this->compQueue);
    this->controlInputs_sub = nh.subscribe(controlInputs_sub_options);

    // Configure all provided services associated with this component
    ros::AdvertiseServiceOptions verticalPos_server_server_options;
    verticalPos_server_server_options = 
	ros::AdvertiseServiceOptions::create<agse_package::verticalPos>
	    ("verticalPos",
             boost::bind(&vertical_actuator_controller::verticalPos_serverCallback, this, _1, _2),
	     ros::VoidPtr(),
             &this->compQueue);
    this->verticalPos_server_server = nh.advertiseService(verticalPos_server_server_options);
 
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
    timer_options = 
	ros::TimerOptions
             (ros::Duration(0.01),
	     boost::bind(&vertical_actuator_controller::verticalPosTimerCallback, this, _1),
	     &this->compQueue);
    this->verticalPosTimer = nh.createTimer(timer_options);

}