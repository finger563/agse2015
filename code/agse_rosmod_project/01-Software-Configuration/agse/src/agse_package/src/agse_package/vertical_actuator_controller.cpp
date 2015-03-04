#include "agse_package/vertical_actuator_controller.hpp"

//# Start User Globals Marker

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

  // THESE NEED TO BE UPDATED
  epsilon = 5;
  motorForwardPin = 86;  // connected to GPIO2_22, pin P8_27
  motorBackwardPin = 87; // connected to GPIO2_23, pin P8_29
  
  adcPin = 1;  // connected to ADC1, pin P9_40

  // set up the pins to control the h-bridge
  gpio_export(motorForwardPin);
  gpio_export(motorBackwardPin);
  gpio_set_dir(motorForwardPin,OUTPUT_PIN);
  gpio_set_dir(motorBackwardPin,OUTPUT_PIN);
  // set up the encoder module
  vm_eqep_period = 1000000000L;
  verticalMotoreQEP.initialize("/sys/devices/ocp.3/48304000.epwmss/48304180.eqep", eQEP::eQEP_Mode_Absolute);
  verticalMotoreQEP.set_period(vm_eqep_period);
  // initialize the goal position to 0
  verticalGoal = 0;
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
      verticalGoal = req.goal;
    }
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
      ROS_INFO("Vertical Actuator Encoder Reading: %d",verticalCurrent);

      unsigned int adcVal = 0;
      adc_get_value(adcPin,&adcVal);
      ROS_INFO("Got ADC %d value : %d",adcPin,adcVal);

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
    ros::AdvertiseServiceOptions verticalPos_server_options;
    verticalPos_server_options = 
	ros::AdvertiseServiceOptions::create<agse_package::verticalPos>
	    ("verticalPos",
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

}
