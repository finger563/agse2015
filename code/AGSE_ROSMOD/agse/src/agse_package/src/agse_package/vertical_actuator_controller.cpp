#include "agse_package/vertical_actuator_controller.hpp"

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

  paused = true;

  // THESE NEED TO BE UPDATED
  epsilon = 5;
  motorForwardPin = 57;  // GPIO1_25 = (1x32) + 25 = 57
  motorBackwardPin = 58;
  verticalEncoderPin0 = 59;
  verticalEncoderPin1 = 60;

  // set up the pins to control the h-bridge
  gpio_export(motorForwardPin);
  gpio_export(motorBackwardPin);
  gpio_set_dir(motorForwardPin,OUTPUT_PIN);
  gpio_set_dir(motorBackwardPin,OUTPUT_PIN);
  // set up the pins to read the encoder
  gpio_export(verticalEncoderPin0);
  gpio_export(verticalEncoderPin1);
  gpio_set_dir(verticalEncoderPin0,INPUT_PIN);
  gpio_set_dir(verticalEncoderPin1,INPUT_PIN);
  // initialize the goal position to 0
  verticalGoal = 0;
    // Stop Init Timer
    initOneShotTimer.stop();
}

// OnOneData Subscription handler for controlInputs_sub subscriber
void vertical_actuator_controller::controlInputs_sub_OnOneData(const agse_package::controlInputs::ConstPtr& received_data)
{
    // Business Logic for controlInputs_sub subscriber subscribing to topic controlInputs callback 
  paused = received_data->paused;
  ROS_INFO( paused ? "Vertical motor PAUSED!" : "Vertical motor UNPAUSED!");
}

// Component Service Callback
bool vertical_actuator_controller::verticalPos_serverCallback(agse_package::verticalPos::Request  &req,
    agse_package::verticalPos::Response &res)
{
    // Business Logic for verticalPos_server Server providing verticalPos Service
  if (req.update == true)
    {
      verticalGoal = req.goal;
    }
  res.current = verticalCurrent;
  return true;
}

// Callback for verticalPosTimer timer
void vertical_actuator_controller::verticalPosTimerCallback(const ros::TimerEvent& event)
{
  // Business Logic for verticalPosTimer 
  if (!paused)
    {
      // read current value for vertical position (encoder)
      // update motor based on current value
      if ( abs(verticalGoal-verticalCurrent) > epsilon ) // if there's significant reason to move
	{
	  if (verticalGoal > verticalCurrent) 
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
    // subscriber: controlInputs_sub
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
    // server: verticalPos_server
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
    // timer: timer.name
    timer_options = 
	ros::TimerOptions
             (ros::Duration(0.01),
	     boost::bind(&vertical_actuator_controller::verticalPosTimerCallback, this, _1),
	     &this->compQueue);
    this->verticalPosTimer = nh.createTimer(timer_options);

}
